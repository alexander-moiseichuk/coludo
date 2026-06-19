# On-board test for the latest-value blackboard (blackboard.py): declare / write / read / value,
# latest-wins, auto-declare, inspect, and Inspector registration. Run by `make test`.

import inspector
from blackboard import Blackboard


def main():
    # declaring a quantity registers the blackboard with the Inspector
    Blackboard.declare('accel')
    assert 'blackboard' in inspector.Inspector.names()
    assert Blackboard.value('accel') is None  # declared but unwritten

    # write/read; read() exposes value/source/timestamp
    Blackboard.write('accel', (1.0, 2.0, 3.0), 'adxl375')
    slot = Blackboard.read('accel')
    assert slot.value == (1.0, 2.0, 3.0) and slot.source == 'adxl375' and slot.timestamp > 0

    # latest-wins
    Blackboard.write('accel', (4.0, 5.0, 6.0), 'adxl375')
    assert Blackboard.value('accel') == (4.0, 5.0, 6.0)

    # write auto-declares an undeclared quantity
    Blackboard.write('altitude', 123.4, 'baro_icp10111')
    assert Blackboard.value('altitude') == 123.4

    # unknown quantity -> None (no slot)
    assert Blackboard.read('missing') is None and Blackboard.value('missing') is None

    # inspect surfaces the live quantities
    snapshot = Blackboard.inspect()
    assert set(snapshot.keys()) == {'accel', 'altitude'}
    assert snapshot['accel']['value'] == (4.0, 5.0, 6.0) and snapshot['accel']['source'] == 'adxl375'
    assert Blackboard.stats()['quantities'] == ['accel', 'altitude']

    print('ok: blackboard declare/write/read/value, latest-wins, auto-declare, inspect')


main()
