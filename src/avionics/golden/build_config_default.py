"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture config_default.json: plan B's built-in board config, and the config_id plan B gives it.

The Rust firmware embeds this file (include_str!) and serves it for every board-config answer (get-config
board/running/saved/default, and the whoami config_id) until it has a config store of its own. The id is
config.config_id(), which hashes the Python repr of the document and leaves board.firmware_version out.

The document's firmware_version is the frozen 'dev' (generate.py blocks the deploy-stamped version.py, so the
fixture does not move with every deploy). Plan B never serves that: config_default stamps the running build
there, and config.load() restamps a saved file, so get-config and whoami always name the same build. A
consumer serving the document must restamp board.firmware_version with its running build first, which
leaves config_id unchanged.

`mcu` repeats document.board.mcu because the firmware reads it there (firmware/src/commands/config.rs).

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import config as config_module
import config_default

"""What a consumer must do with the document before serving it (see the module docstring)."""
_NOTE: str = ('document.board.firmware_version is the frozen dev: restamp it with the running build before '
              'serving the document, as plan B does (config_id leaves it out, so the id stands)')


def build() -> dict:
    """
    The config_default fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, note, document, config_id, mcu}: the default config exactly as
        config_default.default() builds it, its id, and its MCU.
    """
    document = config_default.default()
    return {
        'meta': {'source': 'src/glider/config_default.py, src/glider/config.py'},
        'compare': {'kind': 'exact'},
        'note': _NOTE,
        'document': document,
        'config_id': config_module.config_id(document),
        'mcu': document['board']['mcu'],
    }
