# On-board test for the shared primitives bundle (commons.py, g13/g15): between() in-range pass-through,
# both bounds, the symmetric +/-x usage and open (inf) sides; plus the g15 viper bundle (clamp_int,
# wrap180) -- correctness of BOTH the @micropython.viper variant and the bytecode fallback (they must
# agree), and select() swapping between them. Run by `make test`.

import math

import commons
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


def test_clamp_int():
    # the viper variant and the bytecode fallback must agree across the range + bounds
    for low, value, high in ((0, 5, 10), (0, -3, 10), (0, 99, 10), (-45, -60, 45), (-45, 60, 45),
                             (-45, 12, 45), (0, 0, 10), (0, 10, 10)):
        expected = commons._clamp_int_upy(low, value, high)
        assert commons._clamp_int_viper(low, value, high) == expected, (low, value, high)
        assert expected == max(low, min(value, high))  # matches the plain min/max it replaced (sg90)


def test_wrap180():
    # wrap to (-180, 180]; viper variant == bytecode fallback; in-range pass-through; 350->10 etc.
    for degrees in (0, 20, 180, -180, 181, -181, 270, -270, 359, -359, 540, -540):
        expected = commons._wrap180_upy(degrees)
        assert commons._wrap180_viper(degrees) == expected, degrees
        assert -180 <= expected <= 180
    assert commons._wrap180_upy(200) == -160 and commons._wrap180_upy(-200) == 160


def test_select():
    # select() rebinds the public names; both modes stay correct
    commons.select(False)  # bytecode
    assert commons.clamp_int is commons._clamp_int_upy and commons.wrap180 is commons._wrap180_upy
    assert commons.clamp_int(0, 99, 10) == 10 and commons.wrap180(200) == -160
    commons.select(True)  # viper (the default)
    assert commons.clamp_int is commons._clamp_int_viper and commons.wrap180 is commons._wrap180_viper
    assert commons.clamp_int(0, 99, 10) == 10 and commons.wrap180(200) == -160


test_between()
test_clamp_int()
test_wrap180()
test_select()
print('ok: commons -- between, clamp_int + wrap180 (viper == bytecode), select() swap')
