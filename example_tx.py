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
print("SX1280 TX ready:", radio_summary(radio))

counter = 0
while True:
    payload = ("hello-%d" % counter).encode("utf-8")
    ok = radio.send(payload)
    print("TX", counter, ok, payload)
    counter += 1
    sleep_ms(1000)
