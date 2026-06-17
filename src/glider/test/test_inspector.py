# On-board (MicroPython) test for the Inspectable mixin + Inspector registry (inspector.py).
# Run by `make test`. Raises (-> runner reports FAIL) on any failed assertion.

from inspector import Inspectable, Inspector


class _Widget(Inspectable):
    name = 'widget'
    kind = 'widget'
    _inspect = ('color', 'size')
    _writable = ('color',)

    def __init__(self):
        self.color = 'red'
        self.size = 3


def main():
    Inspector.register(_Widget())
    assert 'widget' in Inspector.names()

    # default inspect() reads _inspect
    assert Inspector.inspect('widget') == {'color': 'red', 'size': 3}

    # update applies only writable + changed properties and returns their names
    assert Inspector.update('widget', {'color': 'blue', 'size': 9}) == ['color']
    assert Inspector.inspect('widget')['color'] == 'blue'
    assert Inspector.inspect('widget')['size'] == 3  # size not writable -> unchanged
    assert Inspector.update('widget', {'color': 'blue'}) == []  # same value -> no change

    assert Inspector.stats('widget') == {}

    # unknown object raises KeyError
    raised = False
    try:
        Inspector.inspect('nope')
    except KeyError:
        raised = True
    assert raised

    print('ok: inspector register/names/inspect/update(changed)/stats, unknown raises')


main()
