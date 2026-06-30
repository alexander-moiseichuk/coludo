# flight_telemetry.py — parse a Coludo recorder capture (the UART stream to the Luckfox) into aligned
# telemetry streams + log lines, for offline analysis. The recorder interleaves two record kinds on
# uart:1 (recorder.py):
#   @<session>_<file>@<row>              telemetry; first row per file is `uptime;<field>;...`, then
#                                        each data row is `<uptime_us>;<v>;<v>;...`  (';'-separated)
#   <ticks_us> <descriptor> :: <message> best-effort log line
# parse() reads a raw capture (both kinds interleaved) and returns the streams + logs. Stdlib only, so
# it stays importable in the test suite; the plotly rendering lives in flight_report.py.

import re

# the session prefix on each telemetry @tag: YYYYMMDD_HHMMSS, optionally with a _<rand> disambiguator
# (recorder.session()); both shapes strip down to the bare file name.
_SESSION = re.compile(r'^\d{8}_\d{6}(?:_\d+)?_')


class Stream:
    """One telemetry file: its field names and numeric rows (uptime first)."""

    def __init__(self, name: str):
        self.name: str = name  # the file, e.g. 'adxl375.csv' (session prefix stripped)
        self.fields: list = []  # column names after the leading 'uptime'
        self.rows: list = []  # [uptime_us, v1, v2, ...] per row (floats; '' for a missing/blank cell)

    def column(self, field: str):
        """The (time_seconds, value) series for one field name — blanks skipped. Empty if absent."""
        if field not in self.fields:
            return [], []
        index = self.fields.index(field) + 1  # +1 past the uptime column
        times, values = [], []
        for row in self.rows:
            if len(row) > index and row[index] != '':
                times.append(row[0] / 1e6)
                values.append(row[index])
        return times, values


def _number(token: str):
    """A telemetry cell -> float when it parses, else the raw string ('' stays '')."""
    try:
        return float(token)
    except ValueError:
        return token


def parse(text: str):
    """text -> ({file -> Stream}, logs), where logs is a list of (uptime_us | None, line)."""
    streams = {}
    logs = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith('@'):
            tag, _, row = line[1:].partition('@')
            if not row:
                continue
            name = _SESSION.sub('', tag)  # 'YYYYMMDD_HHMMSS_imu.csv' -> 'imu.csv'
            stream = streams.get(name)
            if stream is None:
                stream = streams[name] = Stream(name)
            cells = row.split(';')
            if not stream.fields and cells[0] == 'uptime':
                stream.fields = cells[1:]  # the header row
            else:
                values = [_number(cell) for cell in cells]
                try:
                    values[0] = int(float(values[0]))  # uptime as integer microseconds
                except (ValueError, TypeError, IndexError):
                    continue  # skip the row -- bad uptime would crash column() downstream
                stream.rows.append(values)
        else:  # a log line: '<ticks_us> <descriptor> :: <message>'
            first = line.split(' ', 1)[0]
            logs.append((int(first) if first.isdigit() else None, line))
    return streams, logs
