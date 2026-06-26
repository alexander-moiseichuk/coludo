# On-board test for the shared primitives bundle (commons.py). For each function with an
# optimised variant, the _opt build (@micropython.viper for ints, @micropython.native for floats) must
# return EXACTLY what the _upy bytecode reference does -- across ranges, bounds, and (for between) the
# inf-open sides. Also: the plain name binds the _opt variant. Run by `make test`.

import math

import commons


def test_between():
    # _opt (native) must match _upy: in/at/below/above bounds, symmetric +/-, floats, inf-open sides
    for low, value, high in ((0, 5, 10), (0, -3, 10), (0, 99, 10), (0, 0, 10), (0, 10, 10),
                             (-45, 60, 45), (-45, -60, 45), (-45, 12, 45), (-1.5, 0.25, 1.5),
                             (-math.inf, 123.0, math.inf), (0.0, -math.inf, math.inf)):
        expected = commons.between_upy(low, value, high)
        assert commons.between_opt(low, value, high) == expected, (low, value, high)


def test_magnitude_sq():
    # _opt (native) == _upy bytecode; |(x,y,z)|^2 with no sqrt
    for x, y, z in ((3.0, 4.0, 12.0), (0.0, 0.0, 0.0), (-2.0, 1.0, -2.0), (10.0, 0.0, 0.0)):
        assert commons.magnitude_sq_opt(x, y, z) == commons.magnitude_sq_upy(x, y, z) == x * x + y * y + z * z


def test_bank_demand():
    # bank-to-turn: proportional + symmetric hard clamp (gain 0 -> no bank). Both variants kept (the alias
    # is _upy -- native measured 1.03x), so the _opt build must still match the _upy reference exactly.
    for error, gain, limit in ((10.0, 1.5, 30.0), (40.0, 1.5, 30.0), (-40.0, 1.5, 30.0),
                               (0.0, 1.5, 30.0), (25.0, 0.0, 30.0)):
        assert commons.bank_demand_opt(error, gain, limit) == commons.bank_demand_upy(error, gain, limit)
    assert commons.bank_demand(10.0, 1.5, 30.0) == 15.0 and commons.bank_demand(40.0, 1.5, 30.0) == 30.0


def test_clamp_int():
    # the _opt (viper) variant and the _upy bytecode reference must agree across the range + bounds
    for low, value, high in ((0, 5, 10), (0, -3, 10), (0, 99, 10), (-45, -60, 45), (-45, 60, 45),
                             (-45, 12, 45), (0, 0, 10), (0, 10, 10)):
        expected = commons.clamp_int_upy(low, value, high)
        assert commons.clamp_int_opt(low, value, high) == expected, (low, value, high)
        assert expected == max(low, min(value, high))  # matches the plain min/max it replaced (sg90)


def test_wrap180():
    # wrap to (-180, 180]; _opt (viper) == _upy bytecode; in-range pass-through; 350->10 etc.
    for degrees in (0, 20, 180, -180, 181, -181, 270, -270, 359, -359, 540, -540):
        expected = commons.wrap180_upy(degrees)
        assert commons.wrap180_opt(degrees) == expected, degrees
        assert -180 <= expected <= 180
    assert commons.wrap180_upy(200) == -160 and commons.wrap180_upy(-200) == 160


def test_fin_deflection_limit():
    # dynamic-pressure governor (coludo.md): full 45 at low speed, 5 floor at high, clamped + monotonic
    assert commons.fin_deflection_limit(0) == 45 and commons.fin_deflection_limit(15) == 45
    assert commons.fin_deflection_limit(30) == 14 and commons.fin_deflection_limit(50) == 5
    assert commons.fin_deflection_limit(200) == 5 and commons.fin_deflection_limit(-5) == 45  # saturate/guard
    previous = 45
    for speed in range(81):
        limit = commons.fin_deflection_limit(speed)
        assert 5 <= limit <= 45 and limit <= previous  # in range + non-increasing with speed
        previous = limit


def test_alias():
    # each plain name binds its optimised variant (viper for ints, native for floats)
    assert commons.clamp_int is commons.clamp_int_opt and commons.wrap180 is commons.wrap180_opt
    assert commons.between is commons.between_opt and commons.magnitude_sq is commons.magnitude_sq_opt
    assert commons.bank_demand is commons.bank_demand_upy  # native 1.03x -> alias stays on the bytecode


test_between()
test_magnitude_sq()
test_bank_demand()
test_clamp_int()
test_wrap180()
test_fin_deflection_limit()
test_alias()
print('ok: commons -- between, magnitude_sq, bank_demand (@native), clamp_int, wrap180 (@viper); _opt==_upy')
