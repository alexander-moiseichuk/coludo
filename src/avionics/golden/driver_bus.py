"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The bus the driver builders (build_drivers_sdp810.py, build_drivers_ina226.py, build_drivers_vl53.py) run plan B's
drivers over: a stand-in for plan B's i2cbus.Bus that answers from a scripted part and records, in order, every I2C
transaction, every wait and every pin level the driver produced -- its TRANSCRIPT.

The transcript is written at the I2C level, the level the Rust port talks at (embedded-hal), so the Rust tests
replay it step for step through crates/drivers' scripted mock bus and the driver must produce the same sequence:
  {"op": "write", "address": a, "bytes": hex}                     writeto(), or write() of a register: the
                                                                  register address (1 or 2 bytes, big-endian) then
                                                                  the data, in one transaction;
  {"op": "read", "address": a, "bytes": hex}                      readfrom(): the part's answer;
  {"op": "write_read", "address": a, "write": hex, "read": hex}   read() of a register: its address, a repeated
                                                                  start, the part's answer;
  {"op": "delay_ms", "ms": n}                                     asyncio.sleep_ms(n);
  {"op": "pin", "high": bool}                                     a machine.Pin output driven (XSHUT).
A transaction the part refuses carries "fails": true (plan B saw OSError(19), a NAK); a refused read's bytes are
zeros of the asked length, which is all the replay checks of it.

The part is a callable part(op, address, register, count, data) -> bytes | None that raises OSError to refuse:
`op` is 'write', 'read' or 'write_read', `register` is the register address for a register access (None for a
plain one), `count` the bytes asked for, `data` the bytes written. A builder writes it as a small register model
(the INA226's words), or takes sequence(): the answers in transaction order, NAK to refuse one, which also says
whether every scripted answer was used.

Waits never sleep: run() (below) replaces asyncio.sleep_ms with one that records the wait and moves the shims'
virtual clock, so a driver's polling loop costs no real time and the clock stays a function of the script alone.

Imported by the builders, which generate.py runs with the shims installed.
"""

import asyncio
import contextlib

import i2cbus
import shims

_NAK_ERRNO: int = 19  # plan B's OSError(19), ENODEV: what machine.I2C raises when nothing acks


def hex_bytes(data: bytes) -> str:
    """Bytes as the transcript writes them: lowercase hex digit pairs."""
    return bytes(data).hex()


def register_bytes(register: int, addrsize: int) -> bytes:
    """A register address as it goes on the wire: one byte, or two big-endian for the 16-bit VL53 map."""
    return register.to_bytes(addrsize // 8, 'big')


class Bus:
    """
    plan B's i2cbus.Bus, as its drivers call it, over a scripted part; the transcript accumulates in `transcript`.
    """

    def __init__(self, part) -> None:
        """
        A bus with one scripted part on it.

        Args:
            part - the part's answer function (see the module notes).
        """
        self._part = part
        self.transcript: list = []

    def _call(self, step: dict, op: str, address: int, register, count: int, data: bytes):
        """Ask the part, record the step (a refusal marked), and re-raise a refusal as plan B's OSError."""
        try:
            answer = self._part(op, address, register, count, data)
        except OSError:
            if count:
                step['read' if op == 'write_read' else 'bytes'] = hex_bytes(bytes(count))
            step['fails'] = True
            self.transcript.append(step)
            raise
        if count:
            if answer is None or len(answer) != count:
                raise ValueError('the scripted part answered %r to a %d-byte %s' % (answer, count, op))
            step['read' if op == 'write_read' else 'bytes'] = hex_bytes(answer)
        self.transcript.append(step)
        return answer

    async def read(self, addr: int, reg: int, count: int, addrsize: int = 8) -> bytes:
        """readfrom_mem: the register address, a repeated start, `count` bytes."""
        step = {'op': 'write_read', 'address': addr, 'write': hex_bytes(register_bytes(reg, addrsize))}
        return bytes(self._call(step, 'write_read', addr, reg, count, b''))

    async def write(self, addr: int, reg: int, data: bytes, addrsize: int = 8) -> None:
        """writeto_mem: the register address and the data, one transaction."""
        step = {'op': 'write', 'address': addr, 'bytes': hex_bytes(register_bytes(reg, addrsize) + bytes(data))}
        self._call(step, 'write', addr, reg, 0, bytes(data))

    async def writeto(self, addr: int, data: bytes) -> None:
        """A plain write."""
        step = {'op': 'write', 'address': addr, 'bytes': hex_bytes(data)}
        self._call(step, 'write', addr, None, 0, bytes(data))

    async def readfrom(self, addr: int, count: int) -> bytes:
        """A plain read of `count` bytes."""
        step = {'op': 'read', 'address': addr}
        return bytes(self._call(step, 'read', addr, None, count, b''))

    def device(self, addr: int):
        """plan B's per-address window (diagnose reads through it): the real i2cbus._Device over this bus."""
        return i2cbus._Device(self, addr)

    def pin_class(self) -> type:
        """
        A machine.Pin stand-in whose output levels go into this transcript: constructed with `value=` it drives that
        level, and value(level) drives another; the input-side names a driver touches exist and do nothing.
        """
        transcript = self.transcript

        class _Pin:
            OUT: int = 1
            IN: int = 0
            PULL_UP: int = 1
            PULL_DOWN: int = 2
            IRQ_FALLING: int = 1
            IRQ_RISING: int = 2

            def __init__(self, gpio: int, mode: int = 0, pull: int = None, value: int = None) -> None:
                self.gpio = gpio
                if value is not None:
                    transcript.append({'op': 'pin', 'high': bool(value)})

            def value(self, level: int = None):
                if level is None:
                    return 1
                transcript.append({'op': 'pin', 'high': bool(level)})
                return None

            def irq(self, handler, trigger: int) -> None:
                pass

        return _Pin


def nak() -> OSError:
    """The refusal a scripted part raises: plan B's OSError(19)."""
    return OSError(_NAK_ERRNO, 'ENODEV')


NAK: str = 'nak'  # in a sequence() answer list: refuse this transaction


def sequence(answers: list):
    """
    A scripted part that answers transactions in order from `answers`: None acks a write, bytes answer a read, NAK
    refuses either. Check `exhausted()` on the returned part afterwards: an answer left over is a builder mistake.

    Args:
        answers - one entry per transaction the driver will make.

    Returns:
        The part callable, with an `exhausted()` attribute.
    """
    queue = list(answers)

    def part(op: str, address: int, register, count: int, data: bytes):
        if not queue:
            raise ValueError('the scripted part ran out of answers at %s 0x%02x' % (op, address))
        answer = queue.pop(0)
        if answer == NAK:
            raise nak()
        if (answer is None) != (count == 0):
            raise ValueError('the scripted part answered %r to a %d-byte %s' % (answer, count, op))
        return answer

    part.exhausted = lambda: not queue
    return part


@contextlib.contextmanager
def waits_into(bus: Bus):
    """
    Within the block, asyncio.sleep_ms records a {"op": "delay_ms"} step in `bus`'s transcript and moves the virtual
    clock instead of sleeping.
    """
    original = asyncio.sleep_ms

    async def sleep_ms(milliseconds: int) -> None:
        bus.transcript.append({'op': 'delay_ms', 'ms': milliseconds})
        shims.advance(milliseconds)
        await asyncio.sleep(0)

    asyncio.sleep_ms = sleep_ms
    try:
        yield
    finally:
        asyncio.sleep_ms = original


def run(bus: Bus, coroutine):
    """
    Run one plan B coroutine to its end over `bus`, its waits recorded (see waits_into).

    Args:
        bus - the bus whose transcript records the waits.
        coroutine - the coroutine (setup(), _read(), ...).

    Returns:
        What the coroutine returned.
    """
    with waits_into(bus):
        return asyncio.run(coroutine)
