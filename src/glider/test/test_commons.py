# On-board test for the shared primitives bundle (commons.py, g13): between() in-range pass-through,
# both bounds, the symmetric +/-x usage and open (inf) sides. Also the validation/measurement anchor for
# the future native/viper versions (g14/g15). Run by `make test`.

import math

from commons import between


def test_between():
    # in range -> unchanged; below -> low; above -> high; on the bounds -> the bound
    assert between(0, 5, 10) == 5
    assert between(0, -3, 10) == 0
    assert between(0, 99, 10) == 10
    assert between(0, 0, 10) == 0 and between(0, 10, 10) == 10

    # symmetric +/- limit (how mixer/pid/navigation use it)
    assert between(-45, 60, 45) == 45
    assert between(-45, -60, 45) == -45
    assert between(-45, 12, 45) == 12

    # floats + open (inf) sides -- an unbounded limit is a no-op
    assert between(-1.5, 0.25, 1.5) == 0.25
    assert between(-math.inf, 123.0, math.inf) == 123.0
    assert between(0.0, -math.inf, math.inf) == 0.0


test_between()
print('ok: commons -- between() clamp pass-through, both bounds, symmetric, inf-open sides')
