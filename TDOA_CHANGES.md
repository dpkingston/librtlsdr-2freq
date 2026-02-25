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
extends it with:

- **Continuous indefinite alternating** — required for live TDOA monitoring
- **Asymmetric block sizes** — different block sizes for each channel, enabling
  higher target duty cycle

## How It Works

When two `-f` arguments are given, `rtl_sdr` enters 2-frequency mode:

1. Tunes to `freq1` and starts the async capture stream.
2. After `freq1_block_samples` samples have been written, switches to `freq2`.
3. After `freq2_block_samples` samples, switches back to `freq1`.
4. Repeats indefinitely until SIGTERM / Ctrl-C.

The libusb transfer size (`out_block_size`) is set to
`GCD(freq1_block_bytes, freq2_block_bytes)` so that every callback boundary
aligns with a block boundary for at least one channel.  The tuner switch fires
after the last callback that completes a block.

The first portion of each block contains R820T PLL settling artefacts.  Callers
must discard a configurable number of **settling samples** at the start of each
block (typically 20–40 ms; measure empirically with
`scripts/measure_settling.py` in TDOAv3).

## Command-Line Interface

### `-f` argument

Specify twice.  The **first** `-f` is `freq1` (typically the FM sync station);
the **second** `-f` is `freq2` (the LMR target).

### `-n` argument

Specify **once** for symmetric mode (same block size on both channels), or
**twice** for asymmetric mode.  The **first** `-n` sets the block size for
`freq1`; the **second** `-n` sets the block size for `freq2`.

```
# Argument ordering:
-f <freq1>  -f <freq2>  -n <freq1_samples>  [-n <freq2_samples>]
```

This mirrors the natural pairing of `-f` and `-n` by position: the Nth `-n`
applies to the Nth `-f`.

### Output stream layout

**Symmetric** (`-n 65536`):

```
[freq1: 65536 samp | freq2: 65536 samp | freq1: 65536 samp | ...]
  ~32 ms             ~32 ms              ~32 ms
```

**Asymmetric** (`-n 16384 -n 65536`):

```
[freq1: 16384 | freq2: 65536 | freq1: 16384 | freq2: 65536 | ...]
   ~8 ms         ~32 ms          ~8 ms          ~32 ms
```

Asymmetric example gives ~80% target duty cycle vs 50% symmetric.

Each block: first ~20 ms settling artefacts, then stable signal at that
frequency.

## Usage Examples

### Symmetric 2-frequency mode (50/50 duty cycle)

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

### Asymmetric 2-frequency mode (higher target duty cycle)

```bash
# 16384 sync samples (~8 ms) + 65536 target samples (~32 ms)
# → ~80% of usable time on target channel
rtl_sdr \
    -f 99900000 \
    -f 155100000 \
    -s 2048000 \
    -g 400 \
    -n 16384 \
    -n 65536 \
    -
```

### Single-frequency mode (unchanged from upstream)

```bash
rtl_sdr -f 99900000 -s 2048000 -g 0 -n 2048000 output.bin
```

Output goes to stdout as a continuous stream of raw uint8 IQ pairs
(I, Q interleaved, 0–255, DC at 127.5), block-alternating between the
two frequencies.

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

## Changes from osmocom 2.0.2

Only `src/rtl_sdr.c` is modified (the library itself, `librtlsdr.c`, is
**unchanged**).  The patch adds:

- `freq_count`, `frequency1`, `frequency2` globals — track two `-f` arguments
- `n_count`, `n_samples[2]` globals — accumulate up to two `-n` arguments
- `bytes_per_block[2]`, `bytes_in_block`, `current_freq_idx` — block state
- `blocksize_given` flag — avoid overriding an explicit `-b`
- `gcd_u32()` helper — compute USB transfer size for asymmetric blocks
- `-f` option: accumulates up to two frequencies instead of overwriting
- `-n` option: accumulates up to two values (first → freq1, second → freq2)
- Mode detection after getopt: if `freq_count >= 2`, set per-channel block sizes
  from `-n` values and clear `bytes_to_read` for infinite operation
- `out_block_size` auto-set to `GCD(GCD(block1, block2), 16384)` in 2-freq
  mode — preserves exact block boundaries while capping each USB transfer at
  16 kB to reduce per-hop pipeline stale data; overridable with `-b`
- `buf_num` set to 4 (not the librtlsdr default 15) in 2-freq mode — with
  16 kB transfers, 4 × 8192 = 32768 samples ≈ 16 ms of pipeline lag after
  each hop, vs 15 × 8192 ≈ 60 ms with the defaults; settling_samples in
  TDOAv3 can be reduced from ~180 000 to ~30 000 accordingly
- `rtlsdr_callback`: block-boundary frequency switch using per-channel threshold
- Updated `usage()` documenting symmetric and asymmetric modes

## Differences from DC9ST/librtlsdr-2freq

| Feature | DC9ST (2017) | This fork |
|---------|-------------|-----------|
| Base library | ~2014 osmocom + async_rearrangement | osmocom 2.0.2 (Feb 2026) |
| Capture mode | One-shot: freq1 → freq2 → freq1 (3 blocks) | Continuous: alternates indefinitely |
| Second frequency flag | `-h <freq>` | Second `-f <freq>` (standard interface) |
| `-n` meaning | Samples per freq (total = 3×n) | Samples per block per channel (runs forever) |
| Asymmetric block sizes | Not supported | Two `-n` args: first → freq1, second → freq2 |
| Block size alignment | Caller-managed | Auto: `out_block_size = GCD(GCD(block1, block2), 16384)` |
| USB buffer count | Default (15) | 4 in 2-freq mode for lower hop latency |
| Direct sampling (`-D`) | Not present | Inherited from upstream |
| Signal pipe handling | Missing `SIG_IGN` | Correct (`signal(SIGPIPE, SIG_IGN)`) |

## Integration with TDOAv3

TDOAv3's `FreqHopReceiver` generates the correct command line automatically,
using two `-n` arguments when `target_samples_per_block` differs from
`samples_per_block`:

```
# Symmetric:
rtl_sdr -f <sync_freq> -f <target_freq> -s <rate> -g <gain*10> \
        -n <block_samples> -n <block_samples> -

# Asymmetric:
rtl_sdr -f <sync_freq> -f <target_freq> -s <rate> -g <gain*10> \
        -n <sync_samples> -n <target_samples> -
```

Configure in TDOAv3's `config/node.yaml`:

```yaml
freq_hop:
  rtl_sdr_binary: "/usr/local/bin/rtl_sdr_2freq"
  samples_per_block: 32768           # sync block (~16 ms)
  target_samples_per_block: 65536    # target block (~32 ms) — optional
  settling_samples: 24576            # ~12 ms; measure with measure_settling.py
```

With the reduced USB pipeline (4 × 8 kB = 16 ms vs former 15 × 8 kB = 60 ms),
the recommended `settling_samples` drops from ~180 000 to ~25 000, making
asymmetric mode with 32 768-sample sync blocks (>25 000 settling) viable.
