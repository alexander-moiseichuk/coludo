import sys, os, math
sys.path.insert(0, 'tools')
sys.path.insert(0, 'src/glider')
import flight_telemetry as ft
import plotly.graph_objects as go
import plotly.io as pio

SP = os.environ['SP']
tl = (25.514944, -80.392972); br = (25.514583, -80.391111)
cx, cy = (tl[0] + br[0]) / 2, (tl[1] + br[1]) / 2
K = 111320.0


def EN(lat, lon):
    return ((lon - cy) * K * math.cos(math.radians(cx)), (lat - cx) * K)


def nearest(times, values, targets):
    out = []; j = 0; n = len(times)
    for t in targets:
        while j + 1 < n and abs(times[j + 1] - t) < abs(times[j] - t):
            j += 1
        out.append(values[j] if n else 0.0)
    return out


def track(path):
    streams, _ = ft.parse(open(path).read())
    gnss = next((s for s in streams.values() if 'lat' in s.fields), None)
    if gnss is None:
        return None
    baro = next((s for s in streams.values() if 'elevation' in s.fields), None) or \
        next((s for s in streams.values() if 'altitude' in s.fields), None)
    t, lat = gnss.column('lat'); _, lon = gnss.column('lon')
    field = 'elevation' if (baro and 'elevation' in baro.fields) else 'altitude'
    alt = nearest(*baro.column(field), targets=t) if baro else [0.0] * len(t)
    E, N = [], []
    for a, b in zip(lat, lon):
        e, nn = EN(a, b); E.append(e); N.append(nn)
    return E, N, alt, [x / 1e6 for x in t]


colors = {'o': '#1f77b4', 'ov': '#2ca02c', 'oo': '#d62728'}
fig = go.Figure()
for q, dash in ((2, 'solid'), (5, 'dash')):
    for patt in ('o', 'ov', 'oo'):
        tr = track('%s/q_%s_%d_1.txt' % (SP, patt, q))
        if tr is None:
            continue
        E, N, alt, tt = tr
        fig.add_trace(go.Scatter3d(x=E, y=N, z=alt, mode='lines', name='%s q%d' % (patt, q),
                      line=dict(width=4, color=colors[patt], dash=dash),
                      text=['%s q%d  t=%.0fs' % (patt, q, x) for x in tt], hovertemplate='%{text}<extra></extra>'))
        fig.add_trace(go.Scatter3d(x=[E[-1]], y=[N[-1]], z=[alt[-1]], mode='markers',
                      marker=dict(size=4, color=colors[patt], symbol='x'), showlegend=False))
zc = [EN(*tl), EN(br[0], tl[1]), EN(*br), EN(tl[0], br[1]), EN(*tl)]
fig.add_trace(go.Scatter3d(x=[p[0] for p in zc], y=[p[1] for p in zc], z=[0] * 5, mode='lines',
              name='zone (187x40 m)', line=dict(width=8, color='black')))
fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode='markers', name='zone centre',
              marker=dict(size=5, color='black', symbol='diamond')))
fig.update_layout(title='HPRC endgame - o / ov / oo at air-quality 2 (solid) vs 5 (dashed), seed 1  '
                        '(x = touchdown; black = zone strip)',
                  scene=dict(xaxis_title='East (m)', yaxis_title='North (m)', zaxis_title='Altitude (m)',
                             aspectmode='manual', aspectratio=dict(x=2, y=1, z=0.8)))
out = '%s/endgame_3d.html' % SP
pio.write_html(fig, out, include_plotlyjs='cdn', full_html=True)
print('wrote', out, os.path.getsize(out) // 1024, 'KB', '| traces', len(fig.data))
