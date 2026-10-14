#!/usr/bin/env python3
# Coludo project, copyright under MIT license, Alexander Moiseichuk
"""
Render an airframe's STL meshes to a picture for the docs: one PNG, or with --frames a turntable as an animated
PNG (GitHub plays both in a README).

Orthographic, flat-shaded (two-sided: printed-part meshes carry unreliable normals), painted back to front, drawn at
--supersample times the size and scaled down for smooth edges, on a transparent background so it reads on GitHub's
light and dark themes alike. numpy + PIL only (no matplotlib, no GPU).

  tools/render_model.py models/TMS-7/glider.stl --up=-y --fins models/TMS-7/glider/Fin.stl --out tms7.png
  tools/render_model.py models/TMS-7/glider.stl --up=-y --fins models/TMS-7/glider/Fin.stl --frames 36 --width 480 \
      --elevation 25 --out doc/photos/TMS-7_glider_turntable.png

--forward and --up name the mesh axes the airframe's nose and top lie along (TMS-7's meshes: z forward, -y up: the
wings and the camera hole are on the belly); the view then turns --azimuth degrees about the up axis from the right
wing's side and looks down --elevation degrees.

--fins adds a printed fin part, which the assembly mesh leaves out, as copies around the body tube: the fin mesh's own
z is its span (z 0 at the body's surface, the root tab below it), its y the chord with the trailing edge at its
largest y, its x the thickness (TMS-7's Fin.stl). Each copy stands at one of --fin-angles (degrees from the top,
toward the left wing), its z 0 at --fin-root mm from the tube's axis, its trailing edge at --fin-trailing mm along the
nose axis (the mesh's own coordinate). TMS-7's defaults: three at 120 degrees, the yaw fin on top and the two elevons
below the wings' plane (doc/photos/TMS-7_glider_static_burn.jpg), their trailing edges flush with the tail.
"""
import argparse
import math
import re
import struct

import numpy
from PIL import Image, ImageDraw

_AXES: dict = {'x': 0, 'y': 1, 'z': 2, '-x': 0, '-y': 1, '-z': 2}
_LIGHT: tuple = (-0.4, 0.5, 0.75)  # toward the light, in the screen frame (right, up, toward the viewer)
_AMBIENT: float = 0.35  # the share of the base colour a face turned away from the light still shows


def load(path: str) -> numpy.ndarray:
    """The facets of one STL file, ASCII or binary, as an (n, 3, 3) array of vertices."""
    with open(path, 'rb') as handle:
        data = handle.read()
    count = struct.unpack('<I', data[80:84])[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * count:  # binary: the header, the count, 50 bytes a facet
        records = numpy.frombuffer(data, dtype=numpy.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)),
                                                            ('attribute', '<u2')]), count=count, offset=84)
        return records['vertices'].astype(float)
    numbers = re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)
    return numpy.array(numbers, dtype=float).reshape(-1, 3, 3)


def airframe(facets: numpy.ndarray, forward: str, up: str) -> numpy.ndarray:
    """The facets in the airframe's frame (x the nose, y the left wing, z the top), not yet centred."""
    def unit(name: str) -> numpy.ndarray:
        vector = numpy.zeros(3)
        vector[_AXES[name]] = -1.0 if name.startswith('-') else 1.0
        return vector
    nose, top = unit(forward), unit(up)
    basis = numpy.stack([nose, numpy.cross(top, nose), top])  # left = top x nose: (nose, left, top) right-handed
    return facets @ basis.T


def centred(frame: numpy.ndarray) -> numpy.ndarray:
    """The facets moved so their bounding box is centred on the origin."""
    flat = frame.reshape(-1, 3)
    return frame - (flat.min(0) + flat.max(0)) / 2.0


def fins(fin: numpy.ndarray, angles: list, root: float, trailing: float) -> numpy.ndarray:
    """Copies of one fin mesh around the body tube in the airframe frame (see the module notes for its axes)."""
    span, chord, thickness = fin[..., 2], fin[..., 1], fin[..., 0]
    copies = []
    for angle in angles:
        turn = math.radians(angle)
        outward = numpy.array([0.0, math.sin(turn), math.cos(turn)])  # from the axis, through the fin's span
        across = numpy.array([0.0, math.cos(turn), -math.sin(turn)])
        along = trailing + (chord.max() - chord)  # the trailing edge aft, the leading edge toward the nose
        copies.append(along[..., None] * numpy.array([1.0, 0.0, 0.0]) + (root + span)[..., None] * outward
                      + thickness[..., None] * across)
    return numpy.concatenate(copies)


def view(azimuth: float, elevation: float) -> numpy.ndarray:
    """The rotation from the airframe frame to the screen's (right, up, toward the viewer)."""
    turn, tilt = math.radians(azimuth), math.radians(elevation)
    # seen from the right wing's side (the viewer on -y), the nose to the screen's right and the top up, then turned
    # about the up axis and tilted to look down
    side = numpy.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
    cos_turn, sin_turn, cos_tilt, sin_tilt = math.cos(turn), math.sin(turn), math.cos(tilt), math.sin(tilt)
    spin = numpy.array([[cos_turn, 0.0, sin_turn], [0.0, 1.0, 0.0], [-sin_turn, 0.0, cos_turn]])
    pitch = numpy.array([[1.0, 0.0, 0.0], [0.0, cos_tilt, -sin_tilt], [0.0, sin_tilt, cos_tilt]])
    return pitch @ spin @ side


def render(frame: numpy.ndarray, rotation: numpy.ndarray, width: int, height: int, colour: tuple,
           supersample: int, extent: float | None) -> Image.Image:
    """
    One picture: the facets turned to the screen, painted far to near; `extent` mm across the width (a turntable's
    one scale, about the frame's centre), or None to fit this view's own outline into the picture.
    """
    screen = frame @ rotation.T
    flat = screen.reshape(-1, 3)
    if extent is None:
        low, high = flat.min(0), flat.max(0)
        middle = (low + high) / 2.0
        screen = screen - numpy.array([middle[0], middle[1], 0.0])
        fit = min(width / (high[0] - low[0]), height / (high[1] - low[1]))
    else:
        fit = width / extent
    normals = numpy.cross(screen[:, 1] - screen[:, 0], screen[:, 2] - screen[:, 0])
    lengths = numpy.linalg.norm(normals, axis=1)
    keep = lengths > 1e-12
    screen, normals = screen[keep], normals[keep] / lengths[keep, None]
    light = numpy.array(_LIGHT) / numpy.linalg.norm(_LIGHT)
    shade = _AMBIENT + (1.0 - _AMBIENT) * numpy.abs(normals @ light)
    big_width, big_height = width * supersample, height * supersample
    scale = 0.92 * fit * supersample
    centre = numpy.array([big_width / 2.0, big_height / 2.0])
    image = Image.new('RGBA', (big_width, big_height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    for index in numpy.argsort(screen[:, :, 2].mean(axis=1)):
        points = screen[index, :, :2] * numpy.array([scale, -scale]) + centre
        tone = tuple(int(min(255, channel * shade[index])) for channel in colour)
        draw.polygon([tuple(point) for point in points], fill=tone + (255,))
    return image.resize((width, height), Image.LANCZOS)


def main() -> None:
    """Parse the arguments, render, save."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('stl', nargs='+', help='the meshes, drawn together')
    parser.add_argument('--out', required=True, help='the PNG to write (an animated one with --frames)')
    parser.add_argument('--forward', default='z', choices=sorted(_AXES), help="the mesh axis the nose points along")
    parser.add_argument('--up', default='y', choices=sorted(_AXES), help='the mesh axis the top points along')
    parser.add_argument('--azimuth', type=float, default=35.0, help='degrees the view turns about the up axis')
    parser.add_argument('--elevation', type=float, default=30.0, help='degrees the view looks down')
    parser.add_argument('--width', type=int, default=800)
    parser.add_argument('--height', type=int, help='default: half the width')
    parser.add_argument('--colour', default='150,170,200', help='the base colour, R,G,B')
    parser.add_argument('--supersample', type=int, default=3)
    parser.add_argument('--frames', type=int, default=0, help='a turntable of this many frames (an animated PNG)')
    parser.add_argument('--frame-ms', type=int, default=80)
    parser.add_argument('--fins', help='a fin part to add around the body tube (see the notes)')
    parser.add_argument('--fin-angles', default='0,120,240', help='degrees from the top, toward the left wing')
    parser.add_argument('--fin-root', type=float, default=24.0, help="mm from the tube's axis to the fin's z 0")
    parser.add_argument('--fin-trailing', type=float, default=-7.0, help='the trailing edge, mm along the nose axis')
    args = parser.parse_args()
    facets = numpy.concatenate([load(path) for path in args.stl])
    frame = airframe(facets, args.forward, args.up)
    if args.fins:
        angles = [float(angle) for angle in args.fin_angles.split(',')]
        frame = numpy.concatenate([frame, fins(load(args.fins), angles, args.fin_root, args.fin_trailing)])
    frame = centred(frame)
    colour = tuple(int(part) for part in args.colour.split(','))
    height = args.height or args.width // 2
    # the largest span any view shows, so a turntable keeps one scale
    extent = 2.0 * float(numpy.linalg.norm(frame.reshape(-1, 3), axis=1).max())
    if args.frames:
        pictures = [render(frame, view(args.azimuth + 360.0 * step / args.frames, args.elevation), args.width,
                           height, colour, args.supersample, extent) for step in range(args.frames)]
        pictures[0].save(args.out, save_all=True, append_images=pictures[1:], duration=args.frame_ms, loop=0,
                         disposal=1, optimize=True)
    else:
        render(frame, view(args.azimuth, args.elevation), args.width, height, colour, args.supersample,
               None).save(args.out, optimize=True)


if __name__ == '__main__':
    main()
