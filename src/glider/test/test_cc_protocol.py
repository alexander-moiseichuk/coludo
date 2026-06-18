# On-board (MicroPython) test for the CC line protocol (cc_protocol.py). Board-first: a board
# socket sees `command params` (no id), so parse() is command-first. Run by `make test`.

import cc_protocol as cc


def main():
    # bare command, no params
    m = cc.parse('ping')
    assert m.command == 'ping' and m.args == [] and m.named == {}

    # positional params (bare tokens are strings; the receiver converts numerics)
    m = cc.parse('log 3000')
    assert m.command == 'log' and m.args == ['3000']

    # named params
    m = cc.parse('tel ms=3000')
    assert m.command == 'tel' and m.named == {'ms': '3000'} and m.args == []

    # simple values stay bare; spaces/specials ride as base64
    assert cc.encode('192.168.10.1') == '192.168.10.1'
    assert cc.encode(3000) == '3000'
    enc = cc.encode('pad 7, gusty')
    assert enc.startswith('base64:') and cc.decode(enc) == 'pad 7, gusty'
    enc = cc.encode('a=b')  # a value with '=' must not look like a named param
    assert enc.startswith('base64:') and cc.decode(enc) == 'a=b'

    # JSON rides as one base64 value (no special case)
    payload = '{"board": {"id": "g7a"}, "n": 2}'
    m = cc.parse(cc.build('save-config', [payload]))
    assert m.command == 'save-config' and m.args == [payload]

    # an encoded positional containing '=' stays positional
    m = cc.parse(cc.build('inspect', ['wifi', 'a=b']))
    assert m.args == ['wifi', 'a=b']

    # command lowercased; values keep case
    m = cc.parse('STAGE Glider1')
    assert m.command == 'stage' and m.args == ['Glider1']

    # response forms parse too (status first); iam carries the board id
    m = cc.parse('iam glider1 base64:eyJhIjogMX0=')
    assert m.command == 'iam' and m.args[0] == 'glider1' and m.args[1] == '{"a": 1}'
    assert cc.parse('pong').command == 'pong'
    m = cc.parse('err badcmd nope')
    assert m.command == 'err' and m.args == ['badcmd', 'nope']

    # empty line
    m = cc.parse('   ')
    assert m.command is None and m.args == []

    # build round-trips named + positional through parse
    line = cc.build('note', ['glider1'], {'msg': 'pad 7, gusty'})
    m = cc.parse(line)
    assert m.command == 'note' and m.args == ['glider1'] and m.named == {'msg': 'pad 7, gusty'}

    print('ok: cc_protocol parse/build/encode/decode (board-first, base64, no quoting)')


main()
