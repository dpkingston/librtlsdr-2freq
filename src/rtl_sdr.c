/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * 2-frequency continuous alternating mode
 * ----------------------------------------
 * Based on DC9ST/librtlsdr-2freq (2017) and ported to osmocom rtl-sdr 2.0.2.
 *
 * When two -f arguments are given, the RTL-SDR alternates between two
 * frequencies indefinitely, outputting blocks of exactly -n samples at each
 * frequency in turn:
 *
 *   [freq1 block 0][freq2 block 0][freq1 block 1][freq2 block 1] ...
 *
 * The ADC sample clock runs continuously with no gaps.  The tuner switch
 * (I2C command to the R820T) happens at the start of each block.  The first
 * ~10-40 ms of each block contains settling artefacts; callers should discard
 * a configurable number of "settling samples" at the start of each block.
 *
 * This mode is designed for single-dongle TDOA direction finding where an
 * FM broadcast station (sync channel) and an LMR target channel are
 * monitored on a single RTL-SDR.  See TDOAv3 for the full system.
 *
 * Usage (2-frequency mode):
 *   rtl_sdr -f <freq1_hz> -f <freq2_hz> -s <rate> -g <gain> -n <samples_per_block> -
 *
 * Usage (standard single-frequency mode — identical to upstream rtl_sdr):
 *   rtl_sdr -f <freq_hz> [-s rate] [-g gain] [-n total_samples] <filename>
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define DEFAULT_SAMPLE_RATE		2048000
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)

static int do_exit = 0;
static uint32_t bytes_to_read = 0;
static rtlsdr_dev_t *dev = NULL;

/* 2-frequency alternating mode state */
static uint32_t freq_count = 0;          /* number of -f arguments seen (1 or 2) */
static uint32_t frequency1 = 100000000;  /* first frequency (sync / reference) */
static uint32_t frequency2 = 100000000;  /* second frequency (target) */
static uint32_t bytes_per_block = 0;     /* bytes per frequency block; 0 = single-freq mode */
static uint32_t bytes_in_block = 0;      /* bytes written into the current block */
static int current_freq_idx = 0;         /* 0 = frequency1, 1 = frequency2 */

void usage(void)
{
	fprintf(stderr,
		"rtl_sdr — I/Q recorder for RTL2832 based DVB-T receivers\n"
		"2-frequency continuous alternating mode for TDOA (osmocom 2.0.2 port)\n\n"
		"Single-frequency mode (standard):\n"
		"  rtl_sdr -f <freq_hz> [-s rate] [-g gain] [-n total_samples] <filename>\n\n"
		"2-frequency alternating mode (TDOA):\n"
		"  rtl_sdr -f <freq1_hz> -f <freq2_hz> -s <rate> -g <gain> -n <samples_per_block> -\n\n"
		"  Outputs blocks of <samples_per_block> IQ samples alternating between freq1 and\n"
		"  freq2 indefinitely: [freq1][freq2][freq1][freq2]...\n"
		"  The ADC clock runs continuously; no samples are dropped on tuner switches.\n"
		"  Discard the first N settling samples of each block in the caller.\n\n"
		"Options:\n"
		"\t-f frequency [Hz]  (specify twice for 2-frequency mode)\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-d device_index or serial (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-b output_block_size (default: auto in 2-freq mode, 16*16384 otherwise)]\n"
		"\t[-n number of samples: total count in single-freq, per-block in 2-freq]\n"
		"\t[-S force sync output (default: async)]\n"
		"\t[-D enable direct sampling (default: off)]\n"
		"\tfilename (use '-' to dump samples to stdout)\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	signal(SIGPIPE, SIG_IGN);
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}
#endif

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	if (ctx) {
		if (do_exit)
			return;

		if ((bytes_to_read > 0) && (bytes_to_read < len)) {
			len = bytes_to_read;
			do_exit = 1;
			rtlsdr_cancel_async(dev);
		}

		if (fwrite(buf, 1, len, (FILE*)ctx) != len) {
			fprintf(stderr, "Short write, samples lost, exiting!\n");
			rtlsdr_cancel_async(dev);
		}

		if (bytes_to_read > 0)
			bytes_to_read -= len;

		/*
		 * 2-frequency alternating: switch tuner at each block boundary.
		 *
		 * Because out_block_size is set equal to bytes_per_block (see main),
		 * each libusb callback delivers exactly bytes_per_block bytes, so
		 * bytes_in_block + len == bytes_per_block exactly on every call.
		 * The frequency switch fires after the full block has been written,
		 * so the data in this buffer is entirely at the old frequency.
		 * The *next* callback's data starts with tuner settling artefacts
		 * (callers discard settling_samples from the start of each block).
		 */
		if (bytes_per_block > 0) {
			bytes_in_block += len;
			if (bytes_in_block >= bytes_per_block) {
				bytes_in_block = 0;
				current_freq_idx ^= 1;
				verbose_set_frequency(dev,
					current_freq_idx ? frequency2 : frequency1);
			}
		}
	}
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
	int ppm_error = 0;
	int direct_sampling = 0;
	int sync_mode = 0;
	int blocksize_given = 0;
	FILE *file;
	uint8_t *buffer;
	int dev_index = 0;
	int dev_given = 0;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;

	while ((opt = getopt(argc, argv, "d:f:g:s:b:n:p:SD")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			if (freq_count == 0)
				frequency1 = (uint32_t)atofs(optarg);
			else if (freq_count == 1)
				frequency2 = (uint32_t)atofs(optarg);
			else {
				fprintf(stderr, "Error: at most two -f arguments are supported\n");
				usage();
			}
			freq_count++;
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'b':
			out_block_size = (uint32_t)atof(optarg);
			blocksize_given = 1;
			break;
		case 'n':
			bytes_to_read = (uint32_t)atof(optarg) * 2;
			break;
		case 'S':
			sync_mode = 1;
			break;
		case 'D':
			direct_sampling = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

	/*
	 * Mode selection: if two -f arguments were given, enter 2-frequency
	 * continuous alternating mode.
	 *
	 * In this mode:
	 *   - bytes_to_read (from -n) is repurposed as the block size in bytes.
	 *   - bytes_to_read is then cleared so the binary runs indefinitely.
	 *   - out_block_size is set equal to bytes_per_block so that each libusb
	 *     callback delivers exactly one block, giving clean block boundaries.
	 */
	if (freq_count >= 2) {
		if (bytes_to_read == 0) {
			fprintf(stderr,
				"Error: -n <samples_per_block> is required in 2-frequency mode\n");
			usage();
		}
		bytes_per_block = bytes_to_read;
		bytes_to_read = 0;  /* run indefinitely */

		/* Align libusb transfer size to block size for exact boundaries */
		if (!blocksize_given) {
			out_block_size = bytes_per_block;
		}

		fprintf(stderr,
			"2-frequency alternating mode:\n"
			"  Freq1 (sync):   %.6f MHz\n"
			"  Freq2 (target): %.6f MHz\n"
			"  Block size:     %u samples (%u bytes)\n"
			"  Running indefinitely — send SIGTERM or Ctrl-C to stop\n",
			frequency1 / 1e6, frequency2 / 1e6,
			bytes_per_block / 2, bytes_per_block);
	}

	if (out_block_size < MINIMAL_BUF_LENGTH ||
	    out_block_size > MAXIMAL_BUF_LENGTH) {
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}

	buffer = malloc(out_block_size * sizeof(uint8_t));

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	/* Set direct sampling */
	if (direct_sampling)
		verbose_direct_sampling(dev, 2);

	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set the initial frequency */
	verbose_set_frequency(dev, frequency1);

	if (0 == gain) {
		/* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		/* Enable manual gain */
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	verbose_ppm_set(dev, ppm_error);

	if (strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			goto out;
		}
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

	if (sync_mode) {
		fprintf(stderr, "Reading samples in sync mode...\n");
		while (!do_exit) {
			r = rtlsdr_read_sync(dev, buffer, out_block_size, &n_read);
			if (r < 0) {
				fprintf(stderr, "WARNING: sync read failed.\n");
				break;
			}

			if ((bytes_to_read > 0) && (bytes_to_read < (uint32_t)n_read)) {
				n_read = bytes_to_read;
				do_exit = 1;
			}

			if (fwrite(buffer, 1, n_read, file) != (size_t)n_read) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				break;
			}

			if ((uint32_t)n_read < out_block_size) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				break;
			}

			if (bytes_to_read > 0)
				bytes_to_read -= n_read;
		}
	} else {
		fprintf(stderr, "Reading samples in async mode...\n");
		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,
				      0, out_block_size);
	}

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (file != stdout)
		fclose(file);

	rtlsdr_close(dev);
	free(buffer);
out:
	return r >= 0 ? r : -r;
}
