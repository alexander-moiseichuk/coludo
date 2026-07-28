"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Inspector -- the registry of Inspectable objects and the operator-facing introspection surface.
Control's inspect/update/stats commands resolve an object by name through the Inspector
(doc/specs/cc-protocol.md). Any object an operator should see or tweak registers itself here.
"""


class Inspectable:
    """
    Mixin for an operator-inspectable object.

    `name` is the registry key; `kind` is a category. inspect() returns a json-able dict of
    properties; update(props) applies the supported, changed ones and returns the names actually
    changed; stats() returns interesting runtime numbers. The defaults read `_inspect` (readable
    property names) and write `_writable` (the subset settable via update()); override any of the
    three for computed values.
    """

    name: str = 'object'
    kind: str = 'object'
    _inspect: tuple = ()
    _writable: tuple = ()

    def inspect(self) -> dict:
        return {prop: getattr(self, prop) for prop in self._inspect}

    def update(self, props: dict) -> list:
        changed = []
        for key, value in props.items():
            if key in self._writable and getattr(self, key, None) != value:
                setattr(self, key, value)
                changed.append(key)
        return changed

    def stats(self) -> dict:
        return {}


class Inspector:
    _objects: dict = {}

    @classmethod
    def register(cls, obj) -> None:
        cls._objects[obj.name] = obj

    @classmethod
    def unregister(cls, name: str) -> None:
        cls._objects.pop(name, None)

    @classmethod
    def names(cls) -> list:
        return sorted(cls._objects.keys())

    @classmethod
    def get(cls, name: str):
        return cls._objects.get(name)

    @classmethod
    async def probe_all(cls) -> dict:
        """
        Run probe() on every registered inspectable that implements it.

        The shared probe-all sweep: cc arm/verify keep the non-None failures, `probe all` reports every
        result.

        Returns:
            {name: result} for each inspectable with a probe(); a None result means it passed.
        """
        results = {}
        for name in cls.names():
            run = getattr(cls.get(name), 'probe', None)
            if run is not None:
                results[name] = await run()
        return results

    @classmethod
    def calibration_all(cls) -> dict:
        """
        Calibration requirement + state for every registered device that declares one.

        The sweep behind the CC `calibrate` status query and the dashboard: devices that need no
        calibration are simply absent, so the operator sees a short list of what actually wants doing
        rather than a wall of "n/a".

        Returns:
            {name: instruction} for devices with OUTSTANDING calibration; empty when all are satisfied.
        """
        states = {}
        for name in cls.names():
            report = getattr(cls.get(name), 'calibration', None)
            if report is not None:
                instruction = report()
                if instruction:  # '' = nothing to do, so it never reaches the operator
                    states[name] = instruction
        return states

    @classmethod
    def inspect(cls, name: str) -> dict:
        return cls._require(name).inspect()

    @classmethod
    def update(cls, name: str, props: dict) -> list:
        return cls._require(name).update(props)

    @classmethod
    def stats(cls, name: str) -> dict:
        return cls._require(name).stats()

    @classmethod
    def _require(cls, name: str):
        obj = cls._objects.get(name)
        if obj is None:
            raise KeyError(name)
        return obj
