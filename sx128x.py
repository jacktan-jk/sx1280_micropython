try:
    import utime as time
except Exception:
    import time

try:
    from micropython import const
except Exception:
    def const(value):
        return value


_RADIO_GET_STATUS = const(0xC0)
_RADIO_WRITE_REGISTER = const(0x18)
_RADIO_READ_REGISTER = const(0x19)
_RADIO_WRITE_BUFFER = const(0x1A)
_RADIO_READ_BUFFER = const(0x1B)
_RADIO_SET_SLEEP = const(0x84)
_RADIO_SET_STANDBY = const(0x80)
_RADIO_SET_TX = const(0x83)
_RADIO_SET_RX = const(0x82)
_RADIO_SET_PACKETTYPE = const(0x8A)
_RADIO_SET_RFFREQUENCY = const(0x86)
_RADIO_SET_TXPARAMS = const(0x8E)
_RADIO_SET_BUFFERBASEADDRESS = const(0x8F)
_RADIO_SET_MODULATIONPARAMS = const(0x8B)
_RADIO_SET_PACKETPARAMS = const(0x8C)
_RADIO_SET_DIOIRQPARAMS = const(0x8D)
_RADIO_GET_RXBUFFERSTATUS = const(0x17)
_RADIO_GET_IRQSTATUS = const(0x15)
_RADIO_GET_PACKETSTATUS = const(0x1D)
_RADIO_GET_RSSIINST = const(0x1F)
_RADIO_CLR_IRQSTATUS = const(0x97)
_RADIO_SET_REGULATORMODE = const(0x96)
_RADIO_SET_AUTOFS = const(0x9E)
_RADIO_PACKET_TYPE_LORA = const(0x01)

_PACKET_HEADER_EXPLICIT = const(0x00)
_PACKET_CRC_MODE_ON = const(0x20)
_PACKET_IQ_NORMAL = const(0x40)

_XTAL_FREQ = const(52_000_000)
_FREQ_STEP = _XTAL_FREQ / 262144
_TX_DONE_IRQ_MASK = const(0x0001)
_RX_DONE_IRQ_MASK = const(0x0002)
_HEADER_ERR_IRQ_MASK = const(0x0020)
_CRC_ERR_IRQ_MASK = const(0x0040)
_RX_TX_TIMEOUT_IRQ_MASK = const(0x4000)
_TX_TIMEOUT_IRQ_MASK = _RX_TX_TIMEOUT_IRQ_MASK
_DEFAULT_IRQ_MASK = const(
    _TX_DONE_IRQ_MASK | _RX_DONE_IRQ_MASK | _HEADER_ERR_IRQ_MASK | _CRC_ERR_IRQ_MASK | _RX_TX_TIMEOUT_IRQ_MASK
)
_MAX_SAFE_TX_POWER_DBM = const(5)

_SF_MAP = {
    5: 0x50,
    6: 0x60,
    7: 0x70,
    8: 0x80,
    9: 0x90,
    10: 0xA0,
    11: 0xB0,
    12: 0xC0,
}

_BW_MAP = {
    203.125: 0x34,
    406.25: 0x26,
    812.5: 0x18,
    1625.0: 0x0A,
}

_CR_MAP = {
    5: 0x01,
    6: 0x02,
    7: 0x03,
    8: 0x04,
}


def _sleep_ms(value):
    value = max(0, int(value))
    sleep_ms = getattr(time, "sleep_ms", None)
    if callable(sleep_ms):
        sleep_ms(value)
    else:
        time.sleep(value / 1000.0)


def _ticks_ms():
    fn = getattr(time, "ticks_ms", None)
    if callable(fn):
        return int(fn())
    monotonic = getattr(time, "monotonic", None)
    if callable(monotonic):
        return int(monotonic() * 1000)
    return 0


def _ticks_diff(now_ms, started_ms):
    fn = getattr(time, "ticks_diff", None)
    if callable(fn):
        return int(fn(now_ms, started_ms))
    return int(now_ms) - int(started_ms)


def _pin_set_output(pin, value=1):
    init = getattr(pin, "init", None)
    mode = getattr(pin, "OUT", None)
    if callable(init) and mode is not None:
        init(mode=mode, value=int(bool(value)))
        return
    setter = getattr(pin, "value", None)
    if callable(setter):
        setter(int(bool(value)))


def _pin_set_input(pin, pull_up=False):
    init = getattr(pin, "init", None)
    mode = getattr(pin, "IN", None)
    pull = getattr(pin, "PULL_UP", None) if pull_up else None
    if callable(init) and mode is not None:
        kwargs = {"mode": mode}
        if pull is not None:
            kwargs["pull"] = pull
        init(**kwargs)


def _pin_value(pin):
    getter = getattr(pin, "value", None)
    if callable(getter):
        return int(getter())
    return int(getter or 0)


def _counter_read(counter):
    value = 0
    for idx in range(len(counter) - 1, -1, -1):
        value = (value << 8) | int(counter[idx])
    return int(value)


def _counter_inc(counter):
    for idx in range(len(counter)):
        value = (int(counter[idx]) + 1) & 0xFF
        counter[idx] = value
        if value != 0:
            break


class SX1280:
    def __init__(
        self,
        spi,
        cs,
        reset,
        busy,
        frequency_mhz,
        *,
        bandwidth_khz=812.5,
        spreading_factor=7,
        coding_rate=6,
        tx_power_dbm=5,
        preamble_length=12,
        tx_timeout_ms=1000,
        dio1=None,
        tx_en=None,
        rx_en=None,
        debug=False,
    ):
        if spi is None or cs is None or reset is None or busy is None:
            raise ValueError("SX1280 requires spi, cs, reset, and busy")
        self._spi = spi
        self._cs = cs
        self._reset = reset
        self._busy = busy
        self._dio1 = dio1
        self._tx_en = tx_en
        self._rx_en = rx_en
        self._debug = bool(debug)
        self._buffer = bytearray(260)
        self._payload_length = 1
        self._tx_timeout_ms = max(100, int(tx_timeout_ms))
        self._frequency_mhz = float(frequency_mhz)
        self._bandwidth_khz = self._normalize_bandwidth(bandwidth_khz)
        self._spreading_factor = self._normalize_sf(spreading_factor)
        self._coding_rate = self._normalize_cr(coding_rate)
        self._preamble_length = max(1, min(255, int(preamble_length)))
        self._tx_power_dbm = self._normalize_tx_power(tx_power_dbm)
        self._listen = False
        self._last_irq_mask = 0
        self._last_status = 0
        self._packet_len = 0
        self._packet_pointer = 0
        self._dio1_irq_counter = bytearray(4)
        self._dio1_irq_supported = False
        self._dio1_irq_error = ""
        self._rf_path = "idle"

        _pin_set_output(self._cs, 1)
        _pin_set_output(self._reset, 1)
        _pin_set_input(self._busy)
        if self._dio1 is not None:
            _pin_set_input(self._dio1)
            self._register_dio1_irq()
        if self._tx_en is not None:
            _pin_set_output(self._tx_en, 0)
        if self._rx_en is not None:
            _pin_set_output(self._rx_en, 0)

        self.reset()
        self.default_config()

    @property
    def _dio1_irq_count(self):
        return _counter_read(self._dio1_irq_counter)

    def _normalize_sf(self, value):
        value = int(value)
        if value not in _SF_MAP:
            raise ValueError("unsupported SX1280 spreading factor: %s" % value)
        return value

    def _normalize_bandwidth(self, value):
        rounded = round(float(value), 3)
        if rounded not in _BW_MAP:
            raise ValueError("unsupported SX1280 bandwidth: %s" % value)
        return rounded

    def _normalize_cr(self, value):
        value = int(value)
        if value not in _CR_MAP:
            raise ValueError("unsupported SX1280 coding rate denominator: %s" % value)
        return value

    def _normalize_tx_power(self, value):
        value = int(value)
        if value > _MAX_SAFE_TX_POWER_DBM:
            value = _MAX_SAFE_TX_POWER_DBM
        if value < -18:
            value = -18
        return value

    def _select(self):
        _pin_set_output(self._cs, 0)

    def _deselect(self):
        _pin_set_output(self._cs, 1)

    def _transfer(self, out_buf):
        in_buf = bytearray(len(out_buf))
        self._select()
        try:
            self._spi.write_readinto(out_buf, in_buf)
        finally:
            self._deselect()
        return in_buf

    def _write_only(self, out_buf):
        self._select()
        try:
            self._spi.write(out_buf)
        finally:
            self._deselect()

    def _register_dio1_irq(self):
        irq_fn = getattr(self._dio1, "irq", None)
        irq_rising = getattr(self._dio1, "IRQ_RISING", None)
        if not callable(irq_fn) or irq_rising is None:
            return
        try:
            irq_fn(handler=self._handle_dio1_irq, trigger=irq_rising)
            self._dio1_irq_supported = True
            self._dio1_irq_error = ""
        except Exception as exc:
            self._dio1_irq_supported = False
            self._dio1_irq_error = str(exc)

    def _handle_dio1_irq(self, _pin):
        _counter_inc(self._dio1_irq_counter)

    def _set_rf_path(self, tx=False, rx=False):
        tx = bool(tx)
        rx = bool(rx)
        if self._tx_en is not None:
            _pin_set_output(self._tx_en, 1 if tx else 0)
        if self._rx_en is not None:
            _pin_set_output(self._rx_en, 1 if rx else 0)
        if tx and rx:
            self._rf_path = "both"
        elif tx:
            self._rf_path = "tx"
        elif rx:
            self._rf_path = "rx"
        else:
            self._rf_path = "idle"
        if self._tx_en is not None or self._rx_en is not None:
            _sleep_ms(1)

    def _busywait(self, timeout_ms=200):
        started_ms = _ticks_ms()
        while _pin_value(self._busy):
            if _ticks_diff(_ticks_ms(), started_ms) >= timeout_ms:
                raise RuntimeError("SX1280 busy timeout")
            _sleep_ms(1)

    def _send_command(self, command):
        self._busywait()
        response = self._transfer(command)
        if response:
            self._last_status = int(response[0])
        self._busywait()
        return response

    def _write_register(self, address, value):
        address = int(address) & 0xFFFF
        value = int(value) & 0xFF
        self._send_command(
            bytes(
                [
                    _RADIO_WRITE_REGISTER,
                    (address >> 8) & 0xFF,
                    address & 0xFF,
                    value,
                ]
            )
        )

    def _read_register(self, address):
        address = int(address) & 0xFFFF
        response = self._send_command(
            bytes(
                [
                    _RADIO_READ_REGISTER,
                    (address >> 8) & 0xFF,
                    address & 0xFF,
                    0x00,
                    0x00,
                ]
            )
        )
        return int(response[4])

    def reset(self):
        _pin_set_output(self._reset, 0)
        _sleep_ms(50)
        _pin_set_output(self._reset, 1)
        _sleep_ms(30)

    def default_config(self):
        self._listen = False
        self.set_standby()
        self.clear_irq_status()
        self.set_regulator_mode()
        self.set_packet_type()
        self.set_rf_frequency(self._frequency_mhz)
        self.set_modulation_params(
            spreading_factor=self._spreading_factor,
            bandwidth_khz=self._bandwidth_khz,
            coding_rate=self._coding_rate,
        )
        self.set_packet_params(payload_length=self._payload_length)
        self.set_buffer_base_address()
        self.set_tx_params(self._tx_power_dbm)
        self.high_sensitivity_lna(True)
        self.set_dio_irq_params()
        self.set_auto_fs(False)
        self._send_command(bytes([0x9A, 0x00]))
        self._send_command(bytes([0xD5]))

    def set_regulator_mode(self, mode=0x01):
        self._send_command(bytes([_RADIO_SET_REGULATORMODE, int(mode) & 0xFF]))

    def set_standby(self, state=0x00):
        self._set_rf_path(tx=False, rx=False)
        self._send_command(bytes([_RADIO_SET_STANDBY, int(state) & 0xFF]))

    def set_packet_type(self):
        self._send_command(bytes([_RADIO_SET_PACKETTYPE, _RADIO_PACKET_TYPE_LORA]))

    def set_rf_frequency(self, frequency_mhz):
        hz = int(round(float(frequency_mhz) * 1_000_000))
        pll_steps = int(hz / _FREQ_STEP)
        self._send_command(
            bytes(
                [
                    _RADIO_SET_RFFREQUENCY,
                    (pll_steps >> 16) & 0xFF,
                    (pll_steps >> 8) & 0xFF,
                    pll_steps & 0xFF,
                ]
            )
        )

    def set_modulation_params(self, *, spreading_factor, bandwidth_khz, coding_rate):
        sf_code = _SF_MAP[int(spreading_factor)]
        bw_code = _BW_MAP[round(float(bandwidth_khz), 3)]
        cr_code = _CR_MAP[int(coding_rate)]
        self._send_command(bytes([_RADIO_SET_MODULATIONPARAMS, sf_code, bw_code, cr_code]))
        if sf_code in (0x50, 0x60):
            self._write_register(0x0925, 0x1E)
        elif sf_code in (0x70, 0x80):
            self._write_register(0x0925, 0x37)
        else:
            self._write_register(0x0925, 0x32)

    def set_packet_params(
        self,
        *,
        payload_length,
        preamble_length=None,
        header_type=_PACKET_HEADER_EXPLICIT,
        crc_mode=_PACKET_CRC_MODE_ON,
        invert_iq=_PACKET_IQ_NORMAL,
    ):
        payload_length = max(1, min(252, int(payload_length)))
        if preamble_length is None:
            preamble_length = self._preamble_length
        preamble_length = max(1, min(255, int(preamble_length)))
        self._payload_length = payload_length
        self._send_command(
            bytes(
                [
                    _RADIO_SET_PACKETPARAMS,
                    preamble_length,
                    int(header_type) & 0xFF,
                    payload_length,
                    int(crc_mode) & 0xFF,
                    int(invert_iq) & 0xFF,
                    0x00,
                    0x00,
                ]
            )
        )

    def set_buffer_base_address(self, tx_base_address=0x00, rx_base_address=0x00):
        self._send_command(
            bytes(
                [
                    _RADIO_SET_BUFFERBASEADDRESS,
                    int(tx_base_address) & 0xFF,
                    int(rx_base_address) & 0xFF,
                ]
            )
        )

    def set_tx_params(self, power_dbm, ramp_time=0xE0):
        power_dbm = self._normalize_tx_power(power_dbm)
        power_code = max(0, min(0x1F, power_dbm + 18))
        self._send_command(bytes([_RADIO_SET_TXPARAMS, power_code, int(ramp_time) & 0xFF]))

    def set_dio_irq_params(
        self,
        irq_mask=_DEFAULT_IRQ_MASK,
        dio1_mask=_DEFAULT_IRQ_MASK,
        dio2_mask=0x0000,
        dio3_mask=0x0000,
    ):
        self._send_command(
            bytes(
                [
                    _RADIO_SET_DIOIRQPARAMS,
                    (irq_mask >> 8) & 0xFF,
                    irq_mask & 0xFF,
                    (dio1_mask >> 8) & 0xFF,
                    dio1_mask & 0xFF,
                    (dio2_mask >> 8) & 0xFF,
                    dio2_mask & 0xFF,
                    (dio3_mask >> 8) & 0xFF,
                    dio3_mask & 0xFF,
                ]
            )
        )

    def clear_irq_status(self, irq_mask=0xFFFF):
        self._send_command(
            bytes(
                [
                    _RADIO_CLR_IRQSTATUS,
                    (int(irq_mask) >> 8) & 0xFF,
                    int(irq_mask) & 0xFF,
                ]
            )
        )

    def get_irq_status(self, *, clear=False):
        response = self._send_command(bytes([_RADIO_GET_IRQSTATUS, 0x00, 0x00, 0x00]))
        irq_mask = (int(response[2]) << 8) | int(response[3])
        if clear:
            self.clear_irq_status(irq_mask if irq_mask else 0xFFFF)
        return irq_mask

    def set_auto_fs(self, enabled):
        self._send_command(bytes([_RADIO_SET_AUTOFS, 1 if enabled else 0]))

    def high_sensitivity_lna(self, enabled=True):
        reg = self._read_register(0x0891)
        if enabled:
            reg |= 0xC0
        else:
            reg &= 0x3F
        self._write_register(0x0891, reg)

    def write_buffer(self, data):
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        if not data:
            raise ValueError("SX1280 payload must not be empty")
        if len(data) > 252:
            raise ValueError("SX1280 payload exceeds FIFO limit")
        self._busywait()
        self._write_only(bytes([_RADIO_WRITE_BUFFER, 0x00]) + bytes(data))
        self._busywait()

    def read_buffer(self, offset, payload_length):
        payload_length = max(0, int(payload_length))
        if payload_length <= 0:
            return b""
        self._busywait()
        response = self._transfer(
            bytes([_RADIO_READ_BUFFER, int(offset) & 0xFF, 0x00]) + (b"\x00" * payload_length)
        )
        self._busywait()
        return bytes(response[3:3 + payload_length])

    def get_rx_buffer_status(self):
        response = self._send_command(bytes([_RADIO_GET_RXBUFFERSTATUS, 0x00, 0x00, 0x00]))
        return bytes(response[:4])

    def get_packet_status(self):
        response = self._send_command(bytes([_RADIO_GET_PACKETSTATUS, 0x00, 0x00, 0x00, 0x00]))
        return bytes(response[2:5])

    def get_rssi_inst(self):
        response = self._send_command(bytes([_RADIO_GET_RSSIINST, 0x00, 0x00]))
        return int(response[2])

    def set_tx(self, period_base=0x02, period_count=0x0000):
        period_count = int(period_count) & 0xFFFF
        self._set_rf_path(tx=True, rx=False)
        self.clear_irq_status()
        self._send_command(
            bytes(
                [
                    _RADIO_SET_TX,
                    int(period_base) & 0xFF,
                    (period_count >> 8) & 0xFF,
                    period_count & 0xFF,
                ]
            )
        )
        self._listen = False

    def set_rx(self, period_base=0x02, period_count=0xFFFF):
        period_count = int(period_count) & 0xFFFF
        self._set_rf_path(tx=False, rx=True)
        self.clear_irq_status()
        self._send_command(
            bytes(
                [
                    _RADIO_SET_RX,
                    int(period_base) & 0xFF,
                    (period_count >> 8) & 0xFF,
                    period_count & 0xFF,
                ]
            )
        )
        self._listen = True

    def start_listening(self, continuous=True):
        period_count = 0xFFFF if continuous else 0x0000
        self.set_rx(period_count=period_count)

    def stop_listening(self):
        self.set_standby()
        self._listen = False

    def wait_tx_done(self, timeout_ms=None):
        if timeout_ms is None:
            timeout_ms = self._tx_timeout_ms
        started_ms = _ticks_ms()
        while _ticks_diff(_ticks_ms(), started_ms) < int(timeout_ms):
            irq_mask = self.get_irq_status(clear=False)
            self._last_irq_mask = int(irq_mask)
            if irq_mask & _TX_DONE_IRQ_MASK:
                self.clear_irq_status(_TX_DONE_IRQ_MASK)
                return True
            if irq_mask & _TX_TIMEOUT_IRQ_MASK:
                self.clear_irq_status(_TX_TIMEOUT_IRQ_MASK)
                return False
            _sleep_ms(2)
        return False

    def send(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        was_listening = bool(self._listen)
        self.set_packet_params(payload_length=len(data))
        self.write_buffer(data)
        self.set_tx()
        sent = self.wait_tx_done()
        if was_listening:
            self.start_listening()
        else:
            self.set_standby()
        return sent

    def receive(self, continuous=True, keep_listening=True):
        if not self._listen:
            self.start_listening(continuous=continuous)

        irq_mask = self.get_irq_status(clear=False)
        self._last_irq_mask = int(irq_mask)
        if irq_mask & (_HEADER_ERR_IRQ_MASK | _CRC_ERR_IRQ_MASK | _RX_TX_TIMEOUT_IRQ_MASK):
            self.clear_irq_status(irq_mask)
            if not keep_listening:
                self.stop_listening()
            return None

        if (irq_mask & _RX_DONE_IRQ_MASK) == 0 and self._dio1 is not None and not _pin_value(self._dio1):
            return None

        buf_status = self.get_rx_buffer_status()
        packet_len = int(buf_status[2])
        packet_pointer = int(buf_status[3])
        self._packet_len = packet_len
        self._packet_pointer = packet_pointer
        if packet_len <= 0:
            return None

        packet = self.read_buffer(packet_pointer, packet_len)
        self.clear_irq_status(irq_mask if irq_mask else 0xFFFF)
        if not keep_listening:
            self.stop_listening()
        return bytes(packet)

    def status(self):
        response = self._send_command(bytes([_RADIO_GET_STATUS]))
        self._last_status = int(response[0])
        return self._last_status

    def sleep(self):
        self._send_command(bytes([_RADIO_SET_SLEEP, 0x07]))
