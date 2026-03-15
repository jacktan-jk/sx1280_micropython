# MicroPython SX1280 Bundle

This folder packages the current MicroPython `SX1280` driver from this repo as a small standalone bundle.

It is intended for direct use on ESP32-S3 boards without the CrowdGuard config-file layer.

## Files

- `sx128x.py`
  - port over from "https://github.com/maholli/CircuitPython_SX1280"
- `lilygo_t3s3_sx1280_pa.py`
  - helper with hardcoded LilyGo T3S3 SX1280 PA defaults
- `example_tx.py`
  - minimal transmit loop
- `example_rx.py`
  - minimal receive loop
- `LICENSE`

## Default board config

The helper module uses these defaults for LilyGo T3S3 SX1280 PA boards:

- SPI pins: `SCK=5`, `MISO=3`, `MOSI=6`, `CS=7`
- Control pins: `RESET=8`, `DIO1=9`, `BUSY=36`
- Optional RF-switch pins: `TX_EN=10`, `RX_EN=21`
- Radio profile:
  - `frequency_mhz = 2479.0`
  - `bandwidth_khz = 812.5`
  - `spreading_factor = 7`
  - `coding_rate = 6`
  - `tx_power_dbm = 5`
  - `preamble_length = 12`
  - `tx_timeout_ms = 1000`

`TX_EN` and `RX_EN` are included because some T3S3 SX1280 PA boards need the RF path driven explicitly.

## Quick start

Copy the files to a board with `mpremote`:

```bash
mpremote connect COMx fs cp sx128x.py :sx128x.py
mpremote connect COMx fs cp lilygo_t3s3_sx1280_pa.py :lilygo_t3s3_sx1280_pa.py
mpremote connect COMx fs cp example_tx.py :main.py
```

For a receiver:

```bash
mpremote connect COMx fs cp sx128x.py :sx128x.py
mpremote connect COMx fs cp lilygo_t3s3_sx1280_pa.py :lilygo_t3s3_sx1280_pa.py
mpremote connect COMx fs cp example_rx.py :main.py
```

## Basic use

```python
from lilygo_t3s3_sx1280_pa import build_radio

radio = build_radio()
radio.send(b"hello")
packet = radio.receive()
```

## Notes
- The helper tries SPI bus IDs `1`, `2`, then `0`
- TX power is capped in `sx128x.py` at `5 dBm`.
- The receive example keeps the radio in continuous RX mode and polls for packets.
