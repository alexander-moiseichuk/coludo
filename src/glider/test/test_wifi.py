# On-board (MicroPython) test for the Wi-Fi station config parsing (wifi.py).
# Does NOT connect (that needs the panda AP up) -- only checks construction is correct.
# Run by `make test`.

import config_default
import wifi


def main():
    w = wifi.Wifi(config_default.default())
    assert w.ssid == 'panda', w.ssid
    assert w.tx_power == 11
    assert w.isconnected() is False  # WLAN not created until connect()
    assert w.ifconfig() is None
    print('ok: wifi config parsed (ssid=%s tx=%s)' % (w.ssid, w.tx_power))


main()
