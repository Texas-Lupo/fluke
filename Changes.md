# fluke — Adaptive Link Controller for OpenIPC FPV

> Refined fork of [Texas-Lupo/fluke](https://github.com/Texas-Lupo/fluke)
> with seamless **waybeam_venc** integration.

---

## Stack overview

```
[camera SoC]
  └── waybeam_venc  (HTTP API :8888)   ← bitrate / GOP / ROI / IDR
        └── RTP/UDP stream
              └── wfb-ng (wfb_tx_cmd :8000)  ← MCS / FEC / BW / GI
                    └── wifi radio  (iw txpower)

[ground station]
  └── fluke (UDP :9999)   ← receives telemetry, drives all of the above
```

---

## What is fluke?

fluke is a closed-loop adaptive link controller. It receives per-packet
radio telemetry from the ground station (EVM, RSSI, SNR, lost packets,
recovered packets) and autonomously adjusts:

- **MCS index** (modulation / coding scheme) — higher = more throughput
- **FEC ratio** (wfb-ng k/n) — higher protection at bad links
- **TX power** (iw) — per-MCS power matrix
- **Encoder bitrate** (waybeam_venc) — tracks the safe bitrate for the current MCS
- **GOP size** (waybeam_venc) — long at good links, short at bad links
- **ROI QP delta** (waybeam_venc) — sharpens the center of frame when bandwidth is tight

---

## Changes from the original

### 1. waybeam_venc API endpoint (critical fix)

The original code called the **Majestic** streamer API:
```
curl 'http://127.0.0.1/api/v1/set?video0.bitrate=X'   # port 80 — Majestic
```

waybeam_venc runs on **port 8888** by default. All encoder calls are now
routed through dedicated helpers that hit the correct endpoint:

| Helper | Endpoint |
|---|---|
| `venc_set_bitrate(state, kbps)` | `GET /api/v1/set?video0.bitrate=<kbps>` |
| `venc_set_gop(state, seconds)` | `GET /api/v1/set?video0.gop_size=<s>` |
| `venc_set_roi_qp(state, delta)` | `GET /api/v1/set?fpv.roi_qp=<n>` |
| `venc_request_idr()` | `GET /request/idr` |

All four are **live** fields in waybeam_venc — no pipeline restart is triggered.

Each helper tracks its last-sent value and **skips the curl call when
nothing has changed**, avoiding pointless `system()` overhead on every
40 Hz tick.

---

### 2. GOP size adaptation

waybeam_venc exposes `video0.gop_size` as a live-tunable field. fluke
now adapts it based on link health:

| Link state | GOP | Reason |
|---|---|---|
| High MCS, no losses | `GOP_GOOD_LINK` (default **1.0 s**) | Efficient encoding, higher effective bitrate |
| Low MCS, losses, or recovering | `GOP_BAD_LINK` (default **0.25 s**) | Decoder recovers faster after packet loss |

A short GOP means the encoder emits I-frames more frequently. After a
burst of lost packets on a degraded link the ground decoder can resync
within `gop_bad_link` seconds instead of waiting up to a full second
for the next keyframe.

---

### 3. ROI QP adaptation

waybeam_venc's `fpv.roi_qp` applies a signed QP delta to the center
horizontal band. A more-negative value sharpens the center at the
expense of the outer bands (they get more compressed).

fluke now adapts `roi_qp` by MCS:

| MCS | roi_qp | Effect |
|---|---|---|
| `> ROI_MCS_THRESHOLD` (good link) | `ROI_QP_HIGH_MCS` (default **-4**) | Gentle sharpening, saves bitrate |
| `<= ROI_MCS_THRESHOLD` (marginal link) | `ROI_QP_LOW_MCS` (default **-18**) | Aggressive center sharpening — keep the FPV view sharp even when bandwidth is constrained |

This means on a weak link the edges of the frame will look blocky but
the center (where you're flying) stays crisp.

---

### 4. IDR keyframe on MCS downgrade

After any downward MCS transition, fluke waits one `cmd_delay_us` then
calls `venc_request_idr()`. This gives the ground decoder a clean resync
point immediately after the link quality change, minimising the
green-frame / corrupt-frame window that would otherwise last until the
next natural keyframe.

---

### 5. New configuration keys

All new keys are added to `/etc/fluke.conf` with sane defaults. The
existing auto-generation logic creates them automatically on first run.

```ini
# waybeam_venc connection
VENC_IP=127.0.0.1
VENC_PORT=8888

# GOP adaptation (seconds)
# Good link: long GOP = efficient encoding
# Bad link:  short GOP = fast decoder recovery after packet loss
GOP_GOOD_LINK=1.0
GOP_BAD_LINK=0.25

# ROI QP adaptation
# Negative = sharper center (better for FPV piloting at low MCS)
ROI_QP_HIGH_MCS=-4
ROI_QP_LOW_MCS=-18
ROI_MCS_THRESHOLD=2
```

`VENC_IP` / `VENC_PORT` let you point fluke at a venc instance on a
different IP (e.g. if the SoC and the wfb-ng host are separate boards).

---

### 6. OSD shows GOP and ROI QP

The OSD message now includes the current `GOP` and `ROI` values so you
can monitor encoder state directly in your goggles:

```
MCS:4 FEC:9/12 EVM:-3 RSSI:-68 SNR:22 NF:-90 LST:0 REC:0 GOP:1.00s ROI:-4 [RDY]
```

---

### 7. New FIFO runtime commands

Commands are sent by writing to `/tmp/qlink_cmd`:

```bash
echo "command" > /tmp/qlink_cmd
```

New commands added on top of the original set:

| Command | Effect |
|---|---|
| `request_idr` | Immediately request an IDR keyframe from venc |
| `set_gop_good <s>` | Change the good-link GOP size at runtime (e.g. `set_gop_good 0.5`) |
| `set_gop_bad <s>` | Change the bad-link GOP size at runtime (e.g. `set_gop_bad 0.1`) |

Original commands still work unchanged:

| Command | Effect |
|---|---|
| `delete_log` | Delete threshold log, enter failsafe |
| `predict` | Start the power calibration routine |
| `set_powerlevel <0-4>` | Change TX power level matrix selector |

---

## Configuration reference

Full `/etc/fluke.conf` with all keys and their defaults:

```ini
# Link quality offsets
OVERHEAD_RATIO=0.25
SAFETY_MARGIN=0.40
OFFSET_EVM1=2
OFFSET_EVM2=2
OFFSET_RSSI1=2
OFFSET_SNR1=2

# Timing (ticks = iterations of the 40 Hz main loop)
UPLINK_STABILITY_TICKS=60
CHANGE_COOLDOWN_TICKS=60
CMD_DELAY_US=9000
NO_TELEMETRY_TICKS_THRESH=40

# Downlink loss thresholds
DOWNLINK_LOST_PKTS_THRESH=2
DOWNLINK_LOST_PKTS67_THRESH=6
DOWNLINK_LOST_PKTS67_C_THRESH=3
UPLINK_LOST_PKTS_THRESH=0

# FEC
FEC_N_CONSTANT=12
FEC_K_PROBING_START=5
FEC_K_HIGH_PROTECTION=8
FEC_K_MED_PROTECTION=9
FEC_K_LOW_PROTECTION=10
FEC_K_FAILSAFE=5
FEC_RECOVERED_THRESH_HIGH=3
FEC_RECOVERED_THRESH_LOW=1

# MCS probing
PROBING_STABILITY_TICKS=120
PROBING_STEP_TICKS=10
MAX_PROBE_SUCCESS_COUNT=10

# Target SNR per MCS (MCS0 to MCS7)
SNR_TARGETS=6,8,10,12,17,21,24,29

# TX power level (0 = min, 4 = max)
POWER_LEVEL=4

# Power matrices (mBm per MCS, MCS0 to MCS7)
PWR_L0=100,100,100,100,100,100,100,100
PWR_L1=1000,800,600,400,200,100,100,100
PWR_L2=1500,1500,1500,1500,1500,1500,1500,1500
PWR_L3=2500,2500,2250,2000,1750,1750,1500,1250
PWR_L4=2900,2750,2500,2250,1900,1900,1900,1900

# Power calibration routine
PREDICT_TIME_S=20
PREDICT_DELTA=6
PREDICT_LOW_MCS_MAX=5
PREDICT_HIGH_MCS_MIN=6

# OSD
OSD_LEVEL=4
OSD_FONT_SIZE=20
OSD_COLOUR=7

# ── waybeam_venc ─────────────────────────────────────────────────────
VENC_IP=127.0.0.1
VENC_PORT=8888

# GOP adaptation (seconds)
GOP_GOOD_LINK=1.0
GOP_BAD_LINK=0.25

# ROI QP adaptation
ROI_QP_HIGH_MCS=-4
ROI_QP_LOW_MCS=-18
ROI_MCS_THRESHOLD=2
```

---

## Build

```bash
# Native (for testing)
make

# Cross-compile with Buildroot toolchain
make CC=$TARGET_CC
```

Requires: `gcc` (or cross-compiler), `pthread`, `libm`.

---

## Deploy

```bash
scp fluke root@192.168.2.13:/usr/bin/fluke
scp fluke.conf root@192.168.2.13:/etc/fluke.conf   # optional — auto-generated on first run
```

---

## Telemetry packet format

fluke listens on UDP port 9999. Each packet is a 4-byte big-endian
length prefix followed by a colon-separated ASCII payload:

```
<uint32_be length> <time>:<evm1>:<evm2>:<recovered>:<lost>:<rssi>:<snr>:<antennas>:<noise_penalty>:<fec_change>:<idr_code>
```

Special packets use the prefix `special:` followed by a command string
(e.g. `special:request_keyframe`).

---

## Architecture

```
main()
  ├── udp_listener_thread     — receives telemetry packets
  ├── fifo_listener_thread    — receives runtime commands from /tmp/qlink_cmd
  ├── periodic_update_osd     — writes OSD string every 1 s
  └── 40 Hz loop
        ├── handle_prediction()  — power calibration state machine
        └── g0ylink()            — core adaptive link logic
              ├── failsafe mode
              ├── probing mode (MCS uplink probe)
              ├── FEC adaptation
              ├── MCS downgrade on threshold breach
              └── venc_apply_for_mcs()
                    ├── venc_set_bitrate()
                    ├── venc_set_gop()
                    └── venc_set_roi_qp()
```

---

## License

GPL-3.0 — same as the original fluke.
