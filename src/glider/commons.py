# commons.py — small, dependency-free primitives shared across the control-math modules (mixer / pid /
# navigation / sequencer). Today just the clamp `between()` (6/24 g13), but this is deliberately the
# bundle module for the g14/g15 plan: the hot, isolated operations gathered here can later be compiled
# to viper/native and selected via board.config WITHOUT touching any caller (see doc/plan.md / wishes
# g15). Keep everything here pure and host-testable so the native and pure versions can be validated and
# benchmarked against each other.


def between(low, value, high):
    """Clamp `value` to the inclusive range [low, high]: `low` if below, `high` if above, else `value`.
    With low=-x, high=+x it is a symmetric +/-x clamp; either bound may be math.inf for an open side
    (between(-inf, v, inf) == v). Assumes low <= high (the caller's responsibility)."""
    return low if value < low else (high if value > high else value)
