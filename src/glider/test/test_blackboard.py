# On-board test for the latest-value blackboard (blackboard.py): the raw layer (per quantity+source)
# and the fused layer (publish/read), providers(), Inspector registration. Run by `make test`.

import inspector
from blackboard import Blackboard


def main():
    # declaring a quantity registers the blackboard with the Inspector
    Blackboard.declare('altitude')
    assert 'blackboard' in inspector.Inspector.names()
    assert Blackboard.value('altitude') is None  # nothing fused yet

    # raw layer: several providers of one quantity coexist (no clobber)
    Blackboard.write('altitude', 10.0, 'icp')
    Blackboard.write('altitude', 20.0, 'bmp')
    assert Blackboard.raw('altitude', 'icp').value == 10.0
    assert Blackboard.raw('altitude', 'bmp').value == 20.0
    assert set(Blackboard.providers('altitude').keys()) == {'icp', 'bmp'}

    # latest-wins per source; timestamp/source recorded
    Blackboard.write('altitude', 11.0, 'icp')
    icp = Blackboard.raw('altitude', 'icp')
    assert icp.value == 11.0 and icp.source == 'icp' and icp.timestamp > 0

    # fused layer: publish (by fusion) -> read/value (by consumers)
    assert Blackboard.value('altitude') is None  # raw written, but nothing fused
    Blackboard.publish('altitude', 11.0, 'icp')
    assert Blackboard.value('altitude') == 11.0 and Blackboard.read('altitude').source == 'icp'

    # write auto-declares; unknown -> None
    Blackboard.write('accel', (1.0, 2.0, 3.0), 'adxl')
    assert Blackboard.raw('accel', 'adxl').value == (1.0, 2.0, 3.0)
    assert Blackboard.raw('altitude', 'missing') is None and Blackboard.value('missing') is None

    # inspect shows the fused values; stats lists raw providers per quantity
    snap = Blackboard.inspect()
    assert snap['altitude']['value'] == 11.0 and snap['altitude']['source'] == 'icp'
    assert sorted(Blackboard.stats()['altitude']) == ['bmp', 'icp']

    print('ok: blackboard raw(per source)/providers + fused publish/read, inspect')


main()
