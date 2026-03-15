try:
    import utime as time
except Exception:
    import time

from lilygo_t3s3_sx1280_pa import build_radio, radio_summary


def sleep_ms(value):
    fn = getattr(time, "sleep_ms", None)
    if callable(fn):
        fn(int(value))
    else:
        time.sleep(float(value) / 1000.0)


radio = build_radio()
radio.start_listening()
print("SX1280 RX ready:", radio_summary(radio))

while True:
    packet = radio.receive()
    if packet:
        try:
            decoded = packet.decode("utf-8")
        except Exception:
            decoded = None
        print("RX len=", len(packet), "text=", decoded, "hex=", packet.hex())
    sleep_ms(100)
