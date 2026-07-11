"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board (MicroPython) test for the CC line protocol (cc_protocol.py). Board-first: a board socket
sees `command params` (no id), so parse() is command-first. Run by `make test`.
"""

import cc_protocol as cc


def main():
    # bare command, no params
    msg = cc.parse('ping')
    assert msg.command == 'ping' and msg.args == [] and msg.named == {}

    # positional params (bare tokens are strings; the receiver converts numerics)
    msg = cc.parse('log 3000')
    assert msg.command == 'log' and msg.args == ['3000']

    # named params
    msg = cc.parse('tel ms=3000')
    assert msg.command == 'tel' and msg.named == {'ms': '3000'} and msg.args == []

    # simple values stay bare; spaces/specials ride as base64
    assert cc.encode('192.168.10.1') == '192.168.10.1'
    assert cc.encode(3000) == '3000'
    enc = cc.encode('pad 7, gusty')
    assert enc.startswith('base64:') and cc.decode(enc) == 'pad 7, gusty'
    enc = cc.encode('a=b')  # a value with '=' must not look like a named param
    assert enc.startswith('base64:') and cc.decode(enc) == 'a=b'

    # JSON rides as one base64 value (no special case)
    payload = '{"board": {"id": "g7a"}, "n": 2}'
    msg = cc.parse(cc.build('set-config', ['board', payload]))
    assert msg.command == 'set-config' and msg.args == ['board', payload]

    # an encoded positional containing '=' stays positional
    msg = cc.parse(cc.build('inspect', ['wifi', 'a=b']))
    assert msg.args == ['wifi', 'a=b']

    # command lowercased; values keep case
    msg = cc.parse('STAGE Glider1')
    assert msg.command == 'stage' and msg.args == ['Glider1']

    # response forms parse too (status first); iam carries the board id
    msg = cc.parse('iam taster base64:eyJhIjogMX0=')
    assert msg.command == 'iam' and msg.args[0] == 'taster' and msg.args[1] == '{"a": 1}'
    assert cc.parse('pong').command == 'pong'
    msg = cc.parse('err badcmd nope')
    assert msg.command == 'err' and msg.args == ['badcmd', 'nope']

    # empty line
    msg = cc.parse('   ')
    assert msg.command is None and msg.args == []

    # build round-trips named + positional through parse
    line = cc.build('note', ['taster'], {'msg': 'pad 7, gusty'})
    msg = cc.parse(line)
    assert msg.command == 'note' and msg.args == ['taster'] and msg.named == {'msg': 'pad 7, gusty'}

    print('ok: cc_protocol parse/build/encode/decode (board-first, base64, no quoting)')


main()
