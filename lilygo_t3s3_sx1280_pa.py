try:
    import machine
except Exception as exc:
    raise RuntimeError("machine module is required") from exc

from sx128x import SX1280


DEFAULT_PINS = {
    "sck": 5,
    "miso": 3,
    "mosi": 6,
    "cs": 7,
    "reset": 8,
    "dio1": 9,
    "busy": 36,
    "tx_en": 10,
    "rx_en": 21,
}


DEFAULT_RADIO = {
    "frequency_mhz": 2479.0,
    "bandwidth_khz": 812.5,
    "spreading_factor": 7,
    "coding_rate": 6,
    "tx_power_dbm": 5,
    "preamble_length": 12,
    "tx_timeout_ms": 1000,
    "spi_baudrate": 8_000_000,
}


def _open_spi(baudrate, pins, spi_id=None):
    sck = machine.Pin(pins["sck"])
    miso = machine.Pin(pins["miso"])
    mosi = machine.Pin(pins["mosi"])
    bus_ids = (spi_id,) if spi_id is not None else (1, 2, 0)
    last_exc = None
    for bus_id in bus_ids:
        try:
            return machine.SPI(
                bus_id,
                baudrate=int(baudrate),
                polarity=0,
                phase=0,
                sck=sck,
                miso=miso,
                mosi=mosi,
            )
        except Exception as exc:
            last_exc = exc
    if last_exc is None:
        raise RuntimeError("SPI bus open failed")
    raise last_exc


def build_radio(*, pins=None, radio=None, spi_id=None):
    cfg_pins = dict(DEFAULT_PINS)
    if isinstance(pins, dict):
        cfg_pins.update(pins)

    cfg_radio = dict(DEFAULT_RADIO)
    if isinstance(radio, dict):
        cfg_radio.update(radio)

    spi = _open_spi(cfg_radio["spi_baudrate"], cfg_pins, spi_id=spi_id)
    cs = machine.Pin(cfg_pins["cs"], machine.Pin.OUT)
    cs.value(1)
    reset = machine.Pin(cfg_pins["reset"], machine.Pin.OUT)
    reset.value(1)
    busy = machine.Pin(cfg_pins["busy"], machine.Pin.IN)
    dio1 = machine.Pin(cfg_pins["dio1"], machine.Pin.IN)

    tx_en = None
    if cfg_pins.get("tx_en") is not None:
        tx_en = machine.Pin(cfg_pins["tx_en"], machine.Pin.OUT)
        tx_en.value(0)

    rx_en = None
    if cfg_pins.get("rx_en") is not None:
        rx_en = machine.Pin(cfg_pins["rx_en"], machine.Pin.OUT)
        rx_en.value(0)

    return SX1280(
        spi,
        cs,
        reset,
        busy,
        cfg_radio["frequency_mhz"],
        bandwidth_khz=cfg_radio["bandwidth_khz"],
        spreading_factor=cfg_radio["spreading_factor"],
        coding_rate=cfg_radio["coding_rate"],
        tx_power_dbm=cfg_radio["tx_power_dbm"],
        preamble_length=cfg_radio["preamble_length"],
        tx_timeout_ms=cfg_radio["tx_timeout_ms"],
        dio1=dio1,
        tx_en=tx_en,
        rx_en=rx_en,
    )


def radio_summary(radio):
    return {
        "frequency_mhz": getattr(radio, "_frequency_mhz", None),
        "bandwidth_khz": getattr(radio, "_bandwidth_khz", None),
        "spreading_factor": getattr(radio, "_spreading_factor", None),
        "coding_rate": getattr(radio, "_coding_rate", None),
        "tx_power_dbm": getattr(radio, "_tx_power_dbm", None),
        "preamble_length": getattr(radio, "_preamble_length", None),
        "rf_path": getattr(radio, "_rf_path", None),
        "irq_count": getattr(radio, "_dio1_irq_count", None),
    }
