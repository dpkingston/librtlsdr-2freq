# 2-Frequency Continuous Alternating Mode

This fork of [osmocom/rtl-sdr](https://gitea.osmocom.org/sdr/rtl-sdr) (v2.0.2)
adds a continuous two-frequency alternating mode to `rtl_sdr` for single-dongle
TDOA radio direction finding.

## Background

The original [DC9ST/librtlsdr-2freq](https://github.com/DC9ST/librtlsdr-2freq)
patch (2017) demonstrated that `rtlsdr_set_center_freq()` can be safely called
from within the async callback to alternate the R820T tuner between two
frequencies while the RTL2832 ADC continues sampling without gaps.  DC9ST's
version was a one-shot capture (freq1 → freq2 → freq1, three blocks total) based
on a ~2014 vintage of the library.

This fork ports the approach to the current osmocom rtl-sdr 2.0.2 codebase and
extends it to **continuous indefinite alternating**, which is required for live
TDOA monitoring.

## How It Works

When two `-f` arguments are given, `rtl_sdr` enters 2-frequency mode:

1. Tunes to `freq1` and starts the async capture stream.
2. After exactly `-n` samples have been written, switches the tuner to `freq2`.
3. After the next `-n` samples, switches back to `freq1`.
4. Repeats indefinitely until SIGTERM / Ctrl-C.

The libusb transfer size is automatically set equal to the block byte length
(`-n` samples × 2 bytes/sample) so that each libusb callback delivers exactly
one complete block.  The tuner switch fires after the callback writes its data,
so all data in any given callback is at a single frequency.

The first portion of each block contains R820T PLL settling artefacts.  Callers
must discard a configurable number of **settling samples** at the start of each
block (typically 20–40 ms worth; measure with `scripts/measure_settling.py` in
TDOAv3).

Output stream layout:

```
[freq1 block 0 | freq2 block 0 | freq1 block 1 | freq2 block 1 | ...]
 <-- N samples --> <-- N samples --> <-- N samples --> ...
```

Each block: first ~20 ms settling artefacts, then stable signal at that
frequency.

## Changes from osmocom 2.0.2

Only `src/rtl_sdr.c` is modified (the library itself, `librtlsdr.c`, is
**unchanged**).  The patch adds:

- `freq_count`, `frequency1`, `frequency2` globals — track two `-f` arguments
- `bytes_per_block`, `bytes_in_block`, `current_freq_idx` globals — block state
- `blocksize_given` flag — avoid overriding an explicit `-b`
- `-f` option: accumulates up to two frequencies instead of overwriting
- Mode detection after getopt: if `freq_count >= 2`, repurpose `-n` as block
  size and clear `bytes_to_read` for infinite operation
- `out_block_size` auto-set to `bytes_per_block` in 2-freq mode (for exact
  block alignment); overridable with `-b`
- `rtlsdr_callback`: added block-boundary frequency switch
- Updated `usage()` documenting both modes

## Usage

### 2-frequency mode (TDOA)

```bash
# Alternate between KISW 99.9 MHz (sync) and 155.1 MHz (LMR target)
# Block size: 65536 samples (~32 ms at 2.048 MSPS)
rtl_sdr \
    -f 99900000 \
    -f 155100000 \
    -s 2048000 \
    -g 400 \
    -n 65536 \
    -
```

Output goes to stdout as a continuous stream of raw uint8 IQ pairs
(I, Q interleaved, 0–255, DC at 127.5), block-alternating between the
two frequencies.

### Single-frequency mode (unchanged from upstream)

```bash
rtl_sdr -f 99900000 -s 2048000 -g 0 -n 2048000 output.bin
```

## Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install    # installs to /usr/local/bin/rtl_sdr
```

Or to install only `rtl_sdr` without overwriting the system-wide binary:

```bash
cp rtl_sdr /usr/local/bin/rtl_sdr_2freq
```

Then set `rtl_sdr_binary: "/usr/local/bin/rtl_sdr_2freq"` in TDOAv3's
`config/node.yaml`.

## Dependencies

Same as upstream osmocom rtl-sdr:

```bash
# Debian/Ubuntu/Pi OS
sudo apt install libusb-1.0-0-dev cmake build-essential
```

## Differences from DC9ST/librtlsdr-2freq

| Feature | DC9ST (2017) | This fork |
|---------|-------------|-----------|
| Base library | ~2014 osmocom + async_rearrangement | osmocom 2.0.2 (Feb 2026) |
| Capture mode | One-shot: freq1 → freq2 → freq1 (3 blocks) | Continuous: alternates indefinitely |
| Second frequency flag | `-h <freq>` | Second `-f <freq>` (standard interface) |
| `-n` meaning | Samples per freq (total = 3×n) | Samples per block (runs forever) |
| Block size alignment | Caller-managed | Auto: `out_block_size = bytes_per_block` |
| Direct sampling (`-D`) | Not present | Inherited from upstream |
| Signal pipe handling | Missing `SIG_IGN` | Correct (`signal(SIGPIPE, SIG_IGN)`) |

## Integration with TDOAv3

TDOAv3's `FreqHopReceiver` already generates the correct command line:

```
rtl_sdr -f <sync_freq> -f <target_freq> -s <rate> -g <gain*10> -n <block_samples> -
```

No changes to TDOAv3 are required.  Set `freq_hop.rtl_sdr_binary` in
`config/node.yaml` to point to this binary.
