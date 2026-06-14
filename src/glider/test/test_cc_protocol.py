# On-board (MicroPython) test for the CC line protocol (cc_protocol.py).
# Run by `make test`. Raises (-> runner reports FAIL) on any failed assertion.

import cc_protocol as cc


def main():
    # bare command + board-id
    m = cc.parse('ping glider1')
    assert m.command == 'ping' and m.board == 'glider1' and m.params == [] and m.named == {}

    # positional params (bare tokens are plain strings; receiver converts numerics)
    m = cc.parse('log glider1 3000')
    assert m.board == 'glider1' and m.params == ['3000']

    # named params
    m = cc.parse('tel glider1 ms=3000')
    assert m.board == 'glider1' and m.named == {'ms': '3000'} and m.params == []

    # simple strings with safe punctuation stay bare
    assert cc.encode('192.168.10.1') == '192.168.10.1'
    assert cc.encode('glider7a') == 'glider7a'
    assert cc.encode(3000) == '3000'

    # values with spaces / specials are base64 with a readable prefix
    enc = cc.encode('pad 7, gusty')
    assert enc.startswith('base64:') and cc.decode(enc) == 'pad 7, gusty'

    # a value containing '=' must not look like a named param -> base64
    enc = cc.encode('a=b')
    assert enc.startswith('base64:') and cc.decode(enc) == 'a=b'

    # JSON rides as one base64 value (no rest-of-line special case)
    js = '{"board": {"id": "g7a"}, "n": 2}'
    line = cc.build('save-config', ['glider1', js])
    m = cc.parse(line)
    assert m.board == 'glider1' and m.params == [js], m.params

    # named value with spaces round-trips
    line = cc.build('note', ['glider1'], {'msg': 'pad 7, gusty'})
    m = cc.parse(line)
    assert m.command == 'note' and m.board == 'glider1' and m.named == {'msg': 'pad 7, gusty'}

    # response with a JSON payload
    line = cc.build('ok', ['glider1', '{"temp": 54}'])
    m = cc.parse(line)
    assert m.command == 'ok' and m.board == 'glider1' and m.params == ['{"temp": 54}']

    # command is lowercased; values keep case
    m = cc.parse('SELECT Glider1')
    assert m.command == 'select' and m.args == ['Glider1']

    # whoami / operator commands have no board-id
    assert cc.parse('whoami').board is None
    m = cc.parse('help log')
    assert m.command == 'help' and m.args == ['log']

    # empty line
    m = cc.parse('   ')
    assert m.command is None and m.args == []

    # mixed positional + named, with an encoded positional that contains '='
    line = cc.build('cmd', ['glider1', 'a=b'], {'k': 'v', 'two': 'x y'})
    m = cc.parse(line)
    assert m.args == ['glider1', 'a=b'], m.args
    assert m.named == {'k': 'v', 'two': 'x y'}, m.named

    print('ok: cc_protocol parse/build/encode/decode (bare + base64, no quoting)')


main()
