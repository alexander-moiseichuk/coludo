# `calibrate <board> <i2c|spi> <id> [margin-steps]` -- find a sensor bus's max stable frequency.
#
# Drives the board's `bustune` primitive (retune-in-place + per-device health) UP a frequency ladder,
# stopping at the first step any device fails. Reports the ceiling (highest all-healthy step), the
# LIMITING device (first to drop out -> the one to rewire / move off the shared bus), and a `chosen`
# freq backed off `margin` ladder steps for headroom (default 1 -- your MAX-1 rule). Restores the bus
# to its configured freq afterwards; it does NOT persist. To apply, the operator runs the printed
# `set-config board ... + reboot` (the immutable-config activation path). The sweep lives here on CC,
# the board only executes one retune-and-test step at a time.

import json

from . import command

# per-kind ladders (Hz). i2c tops at the ESP32-P4 Fast-mode-Plus ceiling (1 MHz); spi steps up from the
# configured 5 MHz. Short PCB wires usually clear datasheet -- the sweep finds the real per-board limit.
_LADDER = {
    'i2c': [100000, 400000, 1000000],
    'spi': [5000000, 8000000, 10000000, 16000000, 20000000],
}
_FREQ_KEY = {'i2c': 'freq', 'spi': 'baud'}  # the bus-spec field each kind tunes


@command('calibrate', 'sweep a sensor bus (i2c/spi) to its max stable frequency via bustune')
async def calibrate_command(hub, tokens, session) -> list:
    if len(tokens) < 4 or tokens[2] not in _LADDER:
        return ['from cc err badargs calibrate <board> <i2c|spi> <id> [margin-steps]']
    target, kind, ident = tokens[1], tokens[2], tokens[3]
    margin = int(tokens[4]) if len(tokens) >= 5 and tokens[4].isdigit() else 1
    board = hub.boards.get(target)
    if board is None or not board.online:
        return ['from cc err noboard %s' % target]

    # the configured freq -> restore to it after the sweep (never leave the bus overclocked)
    original = None
    cfg = await board.command('get-config', 'board')
    if cfg is not None and cfg.command == 'ok':
        spec = json.loads(cfg.args[0]).get('buses', {}).get(kind, {}).get(str(ident), {})
        original = spec.get(_FREQ_KEY[kind])

    rungs, ceiling, limiter = [], None, None
    for freq in _LADDER[kind]:
        resp = await board.command('bustune', kind, str(ident), str(freq))
        if resp is None or resp.command != 'ok':
            return ['from cc err bustune %s' % (' '.join(map(str, resp.args)) if resp else 'offline')]
        report = json.loads(resp.args[0])
        rungs.append({'freq': freq, 'all_ok': report.get('all_ok')})
        if report.get('all_ok'):
            ceiling = freq
        else:  # first failing step -> the limiting device(s): what to rewire or split off the bus
            limiter = {'freq': freq, 'failed': [n for n, v in report.get('devices', {}).items() if v != 'ok']}
            break

    chosen, note = None, None
    if ceiling is not None and limiter is None:  # swept clean -> the top rung is proven; nothing to back off from
        chosen = ceiling
        note = 'no device limit found within the ladder (raise the ladder to probe higher)'
    elif ceiling is not None:  # a rung failed -> keep `margin` step(s) below the known-bad point for headroom
        idx = _LADDER[kind].index(ceiling)
        chosen = _LADDER[kind][max(0, idx - margin)]
        note = 'limited by %s at %d Hz; chosen is %d step(s) below the %d Hz ceiling' % (
            ', '.join(limiter['failed']), limiter['freq'], margin, ceiling)

    if original is not None:  # put the bus back where it was; the chosen freq is applied via set-config
        await board.command('bustune', kind, str(ident), str(original))

    apply = None if chosen is None else '%s set-config board (buses.%s.%s.%s = %d) + reboot' % (
        target, kind, ident, _FREQ_KEY[kind], chosen)
    out = {'bus': '%s:%s' % (kind, ident), 'configured': original, 'ceiling': ceiling, 'chosen': chosen,
           'margin_steps': margin, 'limiter': limiter, 'note': note, 'ladder': rungs, 'apply': apply}
    return ['from cc ok %s' % json.dumps(out)]
