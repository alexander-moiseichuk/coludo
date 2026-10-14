"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The CC contract runner: checks a board's answers to the CC protocol (doc/specs/cc-protocol.md) request by
request, so the Rust port (src/avionics) can be held to what plan B (src/glider/cc_client.py) answers.

  cases       the tiered cases, the envelope rules, and run() -- the engine that sends and judges them
  transports  how a board is reached: plan B in-process (fake), the live hub's /api/cmd, the serial console
  normalize   masking of the volatile fields, and the --diff comparison of two boards' answers

The command line is tools/cc_contract.py; src/control/test/test_contract.py runs it against plan B.
"""
