# Operator Control commands (specs/cc-protocol.md "Operator commands") as a drop-in registry.
#
# Each command is a small module in this package that registers itself with @command('name'); the
# hub calls load() once at start to import them all, so a new operator command is added by dropping
# a file here -- server.py never changes. (Board-facing commands live on the board, in cc_client.)
#
# A handler is `handler(hub, tokens, session) -> list[str]` (the reply lines, each `from cc ...`).
# `hub` is the Server (for the board registry), `tokens` the split operator line, `session` the
# per-connection state (e.g. the sticky `selected` board). A handler may be async if it needs to
# query boards; the dispatcher awaits it.

import importlib
import pkgutil

REGISTRY = {}  # name -> _Command


class _Command:
    """A registered operator command: its name, handler, and one-line help."""

    def __init__(self, name: str, handler, help_text: str):
        self.name: str = name
        self.handler = handler
        self.help: str = help_text


def command(name: str, help_text: str = ''):
    """Decorator: register a handler under an operator command name."""

    def register(handler):
        REGISTRY[name] = _Command(name, handler, help_text)
        return handler

    return register


def load() -> dict:
    """Import every command module in this package so each self-registers; return the registry."""
    for module in pkgutil.iter_modules(__path__):
        importlib.import_module('%s.%s' % (__name__, module.name))
    return REGISTRY
