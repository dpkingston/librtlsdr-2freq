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
 * frequencies indefinitely:
 *
 *   [freq1 block 0][freq2 block 0][freq1 block 1][freq2 block 1] ...
 *
 * Each -f argument has a corresponding -n argument for its block size.
 * One -n: both channels use the same size (symmetric, 50/50 duty cycle).
 * Two -n: channels use different sizes (asymmetric), e.g. a short sync block
 * and a long target block for higher target duty cycle.
 *
 * The libusb transfer size (out_block_size) is set to
 * GCD(GCD(block1, block2), 16384) bytes, which is the largest value that
 * divides both block sizes while staying ≤ 16 kB.  Keeping transfers small
 * reduces the USB pipeline depth after each hop (stale data from the previous
 * frequency), cutting per-hop settling from ~60 ms to ~16 ms.  buf_num is
 * reduced from the default 15 to 4 in 2-freq mode for the same reason.
 *
 * The ADC clock runs continuously; no samples are lost on tuner switches.
 * The first ~10-25 ms of each block contains R820T PLL settling artefacts;
 * callers must discard a configurable number of "settling samples" per block.
 *
 * Usage (symmetric 2-frequency mode):
 *   rtl_sdr -f <freq1_hz> -f <freq2_hz> -s <rate> -g <gain> \
 *           -n <samples_per_block> -
 *
 * Usage (asymmetric 2-frequency mode):
 *   rtl_sdr -f <freq1_hz> -f <freq2_hz> -s <rate> -g <gain> \
 *           -n <freq1_samples> -n <freq2_samples> -
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
#include <pthread.h>
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
static uint32_t freq_count = 0;              /* number of -f arguments seen (1 or 2) */
static uint32_t frequency1 = 100000000;      /* first frequency  (sync / reference) */
static uint32_t frequency2 = 100000000;      /* second frequency (target) */
static uint32_t n_count = 0;                 /* number of -n arguments seen (1 or 2) */
static uint32_t n_samples[2] = {0, 0};       /* stored -n values (samples, not bytes) */
static uint32_t bytes_per_block[2] = {0, 0}; /* bytes per block [freq1, freq2]; 0 = single-freq */
static uint32_t bytes_in_block = 0;          /* bytes accumulated in the current block */
static int current_freq_idx = 0;             /* 0 = frequency1, 1 = frequency2 */

/*
 * Retune worker thread — performs the actual rtlsdr_set_center_freq() call
 * outside of the libusb async callback context.
 *
 * Calling verbose_set_frequency() (which issues a synchronous USB control
 * transfer) from inside the libusb async bulk-transfer callback causes
 * LIBUSB_ERROR_BUSY (-6): the event loop is already running and cannot
 * service a nested synchronous transfer.  The fix is to signal a dedicated
 * thread that performs the retune after the callback returns.
 */
#ifndef _WIN32
static pthread_t           retune_thread;
static pthread_mutex_t     retune_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t      retune_cond  = PTHREAD_COND_INITIALIZER;
static volatile uint32_t   retune_freq  = 0;  /* 0 = no pending retune */

static void *retune_worker(void *arg)
{
	(void)arg;
	while (!do_exit) {
		pthread_mutex_lock(&retune_mutex);
		while (retune_freq == 0 && !do_exit)
			pthread_cond_wait(&retune_cond, &retune_mutex);
		uint32_t freq = retune_freq;
		retune_freq = 0;
		pthread_mutex_unlock(&retune_mutex);
		if (freq != 0)
			verbose_set_frequency(dev, freq);
	}
	return NULL;
}
#endif

void usage(void)
{
	fprintf(stderr,
		"rtl_sdr — I/Q recorder for RTL2832 based DVB-T receivers\n"
		"2-frequency continuous alternating mode for TDOA (osmocom 2.0.2 port)\n\n"
		"Single-frequency mode (standard):\n"
		"  rtl_sdr -f <freq_hz> [-s rate] [-g gain] [-n total_samples] <filename>\n\n"
		"2-frequency alternating mode — symmetric (same block size on both channels):\n"
		"  rtl_sdr -f <freq1_hz> -f <freq2_hz> -s <rate> -g <gain> \\\n"
		"          -n <samples_per_block> -\n\n"
		"2-frequency alternating mode — asymmetric (different block sizes):\n"
		"  rtl_sdr -f <freq1_hz> -f <freq2_hz> -s <rate> -g <gain> \\\n"
		"          -n <freq1_samples> -n <freq2_samples> -\n\n"
		"  -f is given twice: first value = freq1 (sync/FM), second = freq2 (target).\n"
		"  -n is given once for symmetric mode, or twice for asymmetric mode.\n"
		"  First -n matches first -f; second -n matches second -f.\n"
		"  The ADC clock runs continuously; no samples are dropped on tuner switches.\n"
		"  Discard the first N settling samples of each block in the caller.\n\n"
		"Options:\n"
		"\t-f frequency [Hz]          (specify twice for 2-frequency mode)\n"
		"\t-n samples                 (specify twice in 2-freq mode for asymmetric blocks)\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-d device_index or serial (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-b output_block_size (default: auto in 2-freq mode, 16*16384 otherwise)]\n"
		"\t[-S force sync output (default: async)]\n"
		"\t[-D enable direct sampling (default: off)]\n"
		"\tfilename (use '-' to dump samples to stdout)\n\n");
	exit(1);
}

/* Euclidean GCD for uint32_t — used to compute USB transfer size. */
static uint32_t gcd_u32(uint32_t a, uint32_t b)
{
	while (b) { uint32_t t = b; b = a % b; a = t; }
	return a;
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
		 * out_block_size = GCD(bytes_per_block[0], bytes_per_block[1]),
		 * so each libusb callback delivers an exact divisor of both block
		 * sizes.  bytes_in_block accumulates until it reaches the threshold
		 * for the current channel, at which point the tuner switches.
		 *
		 * For symmetric blocks GCD == block size, so the switch fires every
		 * callback (same as before).  For asymmetric blocks the switch fires
		 * after the appropriate number of callbacks for each channel.
		 *
		 * The frequency switch fires after the full block has been written,
		 * so all data in any given block is at a single frequency.
		 * The *next* block starts with tuner settling artefacts
		 * (callers discard settling_samples from the start of each block).
		 */
		if (bytes_per_block[0] > 0) {
			bytes_in_block += len;
			if (bytes_in_block >= bytes_per_block[current_freq_idx]) {
				bytes_in_block = 0;
				current_freq_idx ^= 1;
#ifndef _WIN32
				/* Signal the retune worker thread.  Cannot call
				 * verbose_set_frequency() here directly: it issues a
				 * synchronous libusb control transfer from inside the
				 * async bulk callback, which returns LIBUSB_ERROR_BUSY. */
				pthread_mutex_lock(&retune_mutex);
				retune_freq = current_freq_idx ? frequency2 : frequency1;
				pthread_cond_signal(&retune_cond);
				pthread_mutex_unlock(&retune_mutex);
#else
				verbose_set_frequency(dev,
					current_freq_idx ? frequency2 : frequency1);
#endif
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
			if (n_count == 0)
				n_samples[0] = (uint32_t)atof(optarg);
			else if (n_count == 1)
				n_samples[1] = (uint32_t)atof(optarg);
			else {
				fprintf(stderr, "Error: at most two -n arguments are supported\n");
				usage();
			}
			n_count++;
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
	 * Single-frequency mode: bytes_to_read comes from the first (only) -n.
	 * 2-frequency mode repurposes -n as per-channel block sizes below.
	 */
	if (n_count >= 1)
		bytes_to_read = n_samples[0] * 2;

	/*
	 * Mode selection: two -f arguments → 2-frequency continuous alternating.
	 *
	 * -n may be given once (symmetric: same block size for both channels) or
	 * twice (asymmetric: first -n for freq1/sync, second -n for freq2/target).
	 * bytes_to_read is cleared so the binary runs indefinitely.
	 *
	 * out_block_size is set to GCD(block0_bytes, block1_bytes) so that every
	 * libusb callback boundary is a block boundary for at least one channel.
	 * For symmetric blocks GCD == block size (identical to symmetric behaviour).
	 */
	if (freq_count >= 2) {
		if (n_count == 0) {
			fprintf(stderr,
				"Error: -n <samples_per_block> is required in 2-frequency mode\n"
				"       Use -n once for symmetric blocks, twice for asymmetric.\n");
			usage();
		}
		bytes_per_block[0] = n_samples[0] * 2;
		bytes_per_block[1] = (n_count >= 2 ? n_samples[1] : n_samples[0]) * 2;
		bytes_to_read = 0;  /* run indefinitely */

		if (!blocksize_given) {
			out_block_size = gcd_u32(bytes_per_block[0], bytes_per_block[1]);
			/*
			 * Cap the USB transfer size at 16 kB (= GCD with 16384).
			 * This preserves exact block-boundary alignment while keeping
			 * transfers small, which reduces the number of stale samples
			 * buffered in the USB pipeline after each frequency hop.
			 * Combined with buf_num=4 below, the pipeline stale-data window
			 * is ~4 * 8192 = 32768 samples (~16 ms at 2.048 MSPS), vs the
			 * default ~15 * 8192 = ~60 ms with the librtlsdr defaults.
			 */
			out_block_size = gcd_u32(out_block_size, 16384);
		}

		fprintf(stderr,
			"2-frequency alternating mode%s:\n"
			"  Freq1 (sync):   %.6f MHz  block %u samples (%u bytes)\n"
			"  Freq2 (target): %.6f MHz  block %u samples (%u bytes)\n"
			"  USB xfer size:  %u bytes, 4 buffers (~%u ms pipeline)\n"
			"  Running indefinitely — send SIGTERM or Ctrl-C to stop\n",
			(bytes_per_block[0] != bytes_per_block[1]) ? " [asymmetric]" : "",
			frequency1 / 1e6, bytes_per_block[0] / 2, bytes_per_block[0],
			frequency2 / 1e6, bytes_per_block[1] / 2, bytes_per_block[1],
			out_block_size,
			(4u * (out_block_size / 2u)) * 1000u / 2048000u);
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
#ifndef _WIN32
		if (freq_count >= 2)
			pthread_create(&retune_thread, NULL, retune_worker, NULL);
#endif
		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,
				      freq_count >= 2 ? 4 : 0, out_block_size);
#ifndef _WIN32
		if (freq_count >= 2) {
			pthread_mutex_lock(&retune_mutex);
			pthread_cond_signal(&retune_cond);  /* wake worker so it sees do_exit */
			pthread_mutex_unlock(&retune_mutex);
			pthread_join(retune_thread, NULL);
		}
#endif
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
