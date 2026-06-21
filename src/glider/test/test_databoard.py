# On-board test for the databoard (databoard.py): provide/write/raw, value() rank-preference,
# the expiry handover (rank 0 expires -> rank 1 takes over), and stale extrapolation. Run by
# `make test`.

import time

import inspector
from databoard import Databoard


def main():
    # provide registers channels with rank(=priority) + expiry; registers with the Inspector
    Databoard.provide('icp', {'altitude': {'priority': 0, 'timeout_ms': 50}})
    Databoard.provide('bmp', {'altitude': {'priority': 1, 'timeout_ms': 500}})
    assert 'databoard' in inspector.Inspector.names()
    assert Databoard.value('altitude') is None  # nothing written yet

    # both fresh -> rank 0 (icp) wins; per-source latest still visible via raw()
    Databoard.write('altitude', 100.0, 'icp')
    Databoard.write('altitude', 200.0, 'bmp')
    assert Databoard.value('altitude') == 100.0
    assert Databoard.read('altitude')[1] == 'icp'
    assert Databoard.raw('altitude', 'bmp') == 200.0

    # rank 0 (icp) goes stale past its 50 ms window while rank 1 (bmp) stays fresh -> bmp takes over
    time.sleep_ms(70)
    Databoard.write('altitude', 201.0, 'bmp')  # bmp fresh, icp now stale
    assert Databoard.value('altitude') == 201.0 and Databoard.read('altitude')[1] == 'bmp'

    # single-provider passthrough, vector value
    Databoard.provide('adxl', {'accel': {'priority': 0, 'timeout_ms': 100}})
    Databoard.write('accel', (1.0, 2.0, 3.0), 'adxl')
    assert Databoard.value('accel') == (1.0, 2.0, 3.0)

    # all stale -> linearly extrapolate the newest source's last two points forward to now
    Databoard.provide('s', {'h': {'priority': 0, 'timeout_ms': 10}})
    Databoard.write('h', 10.0, 's')
    time.sleep_ms(5)
    Databoard.write('h', 12.0, 's')  # rising +2 over ~5 ms
    time.sleep_ms(40)  # both points now older than the 10 ms window -> extrapolate
    value, source, age = Databoard.read('h')
    assert source is None and age is None  # no fresh provider -> extrapolated
    assert value > 12.0, value  # projected forward beyond the last reading

    # inspect shows the fused value per param; stats lists each param's providers
    snap = Databoard.inspect()
    assert snap['altitude']['value'] == 201.0 and snap['altitude']['source'] == 'bmp'
    assert sorted(Databoard.stats()['altitude']) == ['bmp', 'icp']

    # provide() hands back named write-channel(s): one name -> the channel, several -> a tuple, none
    # -> the dict; the clumsy `channels = provide(...); self._x = channels['x']` is gone
    one = Databoard.provide('s1', {'p1': {'priority': 0, 'timeout_ms': 100}}, 'p1')
    one.push(5.0)  # the returned object is the _Channel, push()-able directly
    assert Databoard.value('p1') == 5.0
    pa_ch, pb_ch = Databoard.provide('s2', {'pa': {'priority': 0, 'timeout_ms': 100},
                                             'pb': {'priority': 0, 'timeout_ms': 100}}, 'pa', 'pb')
    pa_ch.push(1.0)
    pb_ch.push(2.0)
    assert Databoard.value('pa') == 1.0 and Databoard.value('pb') == 2.0
    assert 'pc' in Databoard.provide('s3', {'pc': {'priority': 0, 'timeout_ms': 100}})  # no want -> dict

    # parameter() is the dependency accessor: get-or-create read handles, order-independent of setup
    dep = Databoard.parameter('pa')  # an existing param
    pending = Databoard.parameter('not_yet')  # created on first touch though no source exists yet
    assert dep.value() == 1.0 and pending.value() is None
    handle_a, handle_b = Databoard.parameter('pa', 'pb')  # several -> a tuple in order
    assert handle_a.value() == 1.0 and handle_b.value() == 2.0

    # shared freshness window: the primary (lowest-rank) timeout governs EVERY channel. A backup is
    # used only while itself that fresh; when it lapses too, the PRIMARY is extrapolated (not the
    # backup's stale value, which would carry its bias).
    Databoard.provide('pri', {'w': {'priority': 0, 'timeout_ms': 20}})    # primary: tight 20 ms
    Databoard.provide('sec', {'w': {'priority': 1, 'timeout_ms': 5000}})  # backup declares 5 s...
    Databoard.write('w', 10.0, 'pri')
    time.sleep_ms(5)
    Databoard.write('w', 12.0, 'pri')  # primary: two points ~5 ms apart, rising +2
    Databoard.write('w', 5.0, 'sec')   # backup is the newest channel, but only rank 1
    assert Databoard.value('w') == 12.0  # both fresh -> rank-0 primary wins
    # primary goes stale; the backup keeps pushing within the 20 ms window -> backup is used
    time.sleep_ms(30)
    Databoard.write('w', 6.0, 'sec')
    assert Databoard.read('w')[1] == 'sec' and Databoard.value('w') == 6.0
    # backup lapses too (its own 5 s timeout is ignored) -> extrapolate the PRIMARY, not sec's 6.0
    time.sleep_ms(30)
    value, source, age = Databoard.read('w')
    assert source is None and age is None  # nobody fresh in the shared 20 ms window
    assert value > 12.0, value  # primary's rising trajectory projected forward, not the stale backup

    # two rank-0 sources -> the shared window is the MIN of their timeouts (the tighter one)
    Databoard.provide('a0', {'m': {'priority': 0, 'timeout_ms': 40}})
    Databoard.provide('b0', {'m': {'priority': 0, 'timeout_ms': 15}})  # tighter -> sets the window
    Databoard.write('m', 1.0, 'a0')
    time.sleep_ms(25)  # 25 ms > 15 ms window though < a0's own 40 ms -> a0 stale: min() applied
    assert Databoard.read('m')[1] is None  # nobody fresh in the 15 ms window

    # offset reconciliation layered on the shared window: while the primary is fresh the backup
    # learns its bias; on handover (primary stale, backup fresh in the window) it is bias-corrected
    Databoard.provide('icp3', {'alt3': {'priority': 0, 'timeout_ms': 50, 'reconcile': True}})
    Databoard.provide('bmp3', {'alt3': {'priority': 1, 'timeout_ms': 5000}})
    for _ in range(5):  # icp 100, bmp 102.5 (+2.5 bias); both fresh -> bmp learns once per icp sample
        Databoard.write('alt3', 100.0, 'icp3')
        Databoard.write('alt3', 102.5, 'bmp3')
        Databoard.value('alt3')  # a read drives the learn step
        time.sleep_ms(2)
    assert abs(Databoard.value('alt3') - 100.0) < 1e-6  # primary fresh -> icp raw 100, no correction
    learned = Databoard.parameter('alt3').offsets()
    assert abs(learned['bmp3'] + 2.5) < 0.1, learned  # bmp bias learned ~ icp - bmp = -2.5
    time.sleep_ms(60)  # icp's 50 ms window lapses
    Databoard.write('alt3', 102.5, 'bmp3')  # bmp fresh within the 50 ms window -> it takes over
    value, source, _age = Databoard.read('alt3')
    assert source == 'bmp3' and abs(value - 100.0) < 0.1, (value, source)  # corrected, not raw 102.5
    # a non-reconciled param never learns an offset (the shared-window 'pri'/'sec' pair above)
    assert Databoard.parameter('w').offsets() == {}

    print('ok: databoard provide/parameter ergonomics, rank-preference, shared-window handover, '
          'primary extrapolation, offset reconciliation')


main()
