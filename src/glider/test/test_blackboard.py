# On-board test for the blackboard (blackboard.py): provide/write/raw, value() rank-preference,
# the expiry handover (rank 0 expires -> rank 1 takes over), and stale extrapolation. Run by
# `make test`.

import time

import inspector
from blackboard import Blackboard


def main():
    # provide registers channels with rank(=priority) + expiry; registers with the Inspector
    Blackboard.provide('icp', {'altitude': {'priority': 0, 'timeout_ms': 50}})
    Blackboard.provide('bmp', {'altitude': {'priority': 1, 'timeout_ms': 500}})
    assert 'blackboard' in inspector.Inspector.names()
    assert Blackboard.value('altitude') is None  # nothing written yet

    # both fresh -> rank 0 (icp) wins; per-source latest still visible via raw()
    Blackboard.write('altitude', 100.0, 'icp')
    Blackboard.write('altitude', 200.0, 'bmp')
    assert Blackboard.value('altitude') == 100.0
    assert Blackboard.read('altitude')[1] == 'icp'
    assert Blackboard.raw('altitude', 'bmp') == 200.0

    # rank 0 (icp) goes stale past its 50 ms window while rank 1 (bmp) stays fresh -> bmp takes over
    time.sleep_ms(70)
    Blackboard.write('altitude', 201.0, 'bmp')  # bmp fresh, icp now stale
    assert Blackboard.value('altitude') == 201.0 and Blackboard.read('altitude')[1] == 'bmp'

    # single-provider passthrough, vector value
    Blackboard.provide('adxl', {'accel': {'priority': 0, 'timeout_ms': 100}})
    Blackboard.write('accel', (1.0, 2.0, 3.0), 'adxl')
    assert Blackboard.value('accel') == (1.0, 2.0, 3.0)

    # all stale -> linearly extrapolate the newest source's last two points forward to now
    Blackboard.provide('s', {'h': {'priority': 0, 'timeout_ms': 10}})
    Blackboard.write('h', 10.0, 's')
    time.sleep_ms(5)
    Blackboard.write('h', 12.0, 's')  # rising +2 over ~5 ms
    time.sleep_ms(40)  # both points now older than the 10 ms window -> extrapolate
    value, source, age = Blackboard.read('h')
    assert source is None and age is None  # no fresh provider -> extrapolated
    assert value > 12.0, value  # projected forward beyond the last reading

    # inspect shows the fused value per param; stats lists each param's providers
    snap = Blackboard.inspect()
    assert snap['altitude']['value'] == 201.0 and snap['altitude']['source'] == 'bmp'
    assert sorted(Blackboard.stats()['altitude']) == ['bmp', 'icp']

    # provide() hands back named write-channel(s): one name -> the channel, several -> a tuple, none
    # -> the dict; the clumsy `channels = provide(...); self._x = channels['x']` is gone
    one = Blackboard.provide('s1', {'p1': {'priority': 0, 'timeout_ms': 100}}, 'p1')
    one.push(5.0)  # the returned object is the _Channel, push()-able directly
    assert Blackboard.value('p1') == 5.0
    pa_ch, pb_ch = Blackboard.provide('s2', {'pa': {'priority': 0, 'timeout_ms': 100},
                                             'pb': {'priority': 0, 'timeout_ms': 100}}, 'pa', 'pb')
    pa_ch.push(1.0)
    pb_ch.push(2.0)
    assert Blackboard.value('pa') == 1.0 and Blackboard.value('pb') == 2.0
    assert 'pc' in Blackboard.provide('s3', {'pc': {'priority': 0, 'timeout_ms': 100}})  # no want -> dict

    # parameter() is the dependency accessor: get-or-create read handles, order-independent of setup
    dep = Blackboard.parameter('pa')  # an existing param
    pending = Blackboard.parameter('not_yet')  # created on first touch though no source exists yet
    assert dep.value() == 1.0 and pending.value() is None
    handle_a, handle_b = Blackboard.parameter('pa', 'pb')  # several -> a tuple in order
    assert handle_a.value() == 1.0 and handle_b.value() == 2.0

    print('ok: blackboard provide/parameter ergonomics, rank-preference, expiry handover, extrapolation')


main()
