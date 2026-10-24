## =========================================================================== ##
# MIT License
# Copyright (c) 2026 JIC Smart Factory
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
## =========================================================================== ##
# Author   : Roman Parak
# File Name: ../Example/Media/make_logo_variants.py
## =========================================================================== ##

# System (Default)
import sys
# OS (Operating system interfaces)
import os
# The repository root, built from this file's location, so the script runs from any working directory.
#   Unit [string]
CONST_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
#   Add access if it is not in the system path.
if os.path.join(CONST_REPO_ROOT, 'src') not in sys.path:
    sys.path.append(os.path.join(CONST_REPO_ROOT, 'src'))
# Argparse (Command-line parsing)
import argparse
# Regular expressions (Default)
import re
# XML (ElementTree XML API)
import xml.etree.ElementTree as ET
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core - the script log
from Configuration.Core import Script_Log_Cls

"""
Description:
    Initialization of constants.

    The black and white SVG versions of the project's logos (design-decisions: the brand), made from the
    owner's originals in Media/Original/ by recolouring every fill and stroke (the paths, the viewBox and the
    transparency kept; styles, classes, editor metadata, gradients and background rectangles removed). Only
    the standard library writes the SVGs; matplotlib (pinned) draws the check image. A raster original is
    never wrapped into an SVG: a logo without a vector original is not made here (KNOWN_ISSUES.md).
"""
# The owner's originals and the folder the variants are written into.
#   Unit [string]
CONST_ORIGINALS = os.path.join(CONST_REPO_ROOT, 'Media', 'Original')
CONST_MEDIA = os.path.join(CONST_REPO_ROOT, 'Media')
# The check image (design-decisions: generated artefacts in Data/Generated/<Kind>/).
#   Unit [string]
CONST_CHECK_IMAGE = os.path.join(CONST_REPO_ROOT, 'Data', 'Generated', 'Images', 'logo_check.png')
# The logos with a vector original: output name -> original file in Media/Original/.
#   Unit [Dict<string, string>]
CONST_LOGOS = {'JIC_Smart_Factory': 'jic-smart-factory.svg', 'KINALI': 'kinali.svg'}
# The variants: suffix -> the one colour of every fill and stroke.
#   Unit [Dict<string, string>]
CONST_VARIANTS = {'Black': '#000000', 'White': '#FFFFFF'}
# The check image's grounds: white for the black logos, GitHub's dark ground for the white ones.
#   Unit [Dict<string, string>]
CONST_GROUNDS = {'Black': '#FFFFFF', 'White': '#0D1117'}
# The SVG namespace.
#   Unit [string]
CONST_SVG_NS = 'http://www.w3.org/2000/svg'
# Shape elements kept, and their geometry and painting attributes kept (everything else is dropped).
#   Unit [Set<string>]
CONST_SHAPES = {'path', 'polygon', 'polyline', 'rect', 'circle', 'ellipse', 'line'}
CONST_GEOMETRY = {'d', 'points', 'x', 'y', 'width', 'height', 'rx', 'ry', 'cx', 'cy', 'r', 'x1', 'y1', 'x2', 'y2', 'transform',
                  'fill-rule', 'clip-rule', 'stroke-width', 'stroke-linecap', 'stroke-linejoin', 'stroke-miterlimit'}
# Elements dropped with their content: styles, definitions (gradients), editor metadata, descriptions.
#   Unit [Set<string>]
CONST_DROPPED = {'style', 'defs', 'metadata', 'title', 'desc', 'linearGradient', 'radialGradient'}
# Elements that make a file unusable as a vector logo: refused, never dropped silently.
#   Unit [Set<string>]
CONST_REFUSED = {'image', 'script', 'foreignObject', 'use', 'text', 'clipPath', 'mask', 'pattern', 'filter'}


class Logo_Error_Cls(ValueError):
    """
    Description:
        A logo original cannot be converted to a plain one-colour vector logo. The message names the file and
        the reason; the script prints it as an [ERROR] line.
    """


def _Local(tag: str) -> str:
    """
    Description:
        The local name of an element tag or attribute ('{ns}path' -> 'path').

    Args:
        (1) tag [string]: The tag or attribute name, with or without a namespace.

    Returns:
        (1) parameter [string]: The local name.
    """

    return tag.rsplit('}', 1)[-1]


def Read_Class_Paint(root: ET.Element) -> tp.Dict[str, tp.Dict[str, str]]:
    """
    Description:
        The fill and stroke of each CSS class in the file's <style> blocks (Illustrator's '.cls-1 { fill: ... }').

    Args:
        (1) root [xml.etree.ElementTree.Element]: The parsed SVG.

    Returns:
        (1) parameter [Dict<string, Dict<string, string>>]: class -> {'fill' / 'stroke': value}.
    """

    paint = {}
    for style in root.iter('{%s}style' % CONST_SVG_NS):
        for names, body in re.findall(r'([^{}]+)\{([^}]*)\}', style.text or ''):
            declarations = dict((k.strip(), v.strip()) for k, v in (d.split(':', 1) for d in body.split(';') if ':' in d))
            for name in re.findall(r'\.([\w-]+)', names):
                paint.setdefault(name, {}).update({k: v for k, v in declarations.items() if k in ('fill', 'stroke')})
    return paint


def _Paint(element: ET.Element, classes: tp.Dict[str, tp.Dict[str, str]], inherited: tp.Dict[str, str]) -> tp.Dict[str, str]:
    """
    Description:
        The effective fill and stroke of an element: inherited, then its classes, then its attributes, then its
        inline style (SVG's precedence).

    Args:
        (1) element [xml.etree.ElementTree.Element]: The element.
        (2) classes [Dict<string, Dict<string, string>>]: The class paint (Read_Class_Paint).
        (3) inherited [Dict<string, string>]: The paint of its parent.

    Returns:
        (1) parameter [Dict<string, string>]: 'fill' / 'stroke' -> value.
    """

    paint = dict(inherited)
    for name in (element.get('class') or '').split():
        paint.update(classes.get(name, {}))
    paint.update({k: element.get(k) for k in ('fill', 'stroke') if element.get(k) is not None})
    for declaration in (element.get('style') or '').split(';'):
        if ':' in declaration:
            k, v = (x.strip() for x in declaration.split(':', 1))
            if k in ('fill', 'stroke'):
                paint[k] = v
    return paint


def Recolour_Logo(source: str, colour: str) -> ET.Element:
    """
    Description:
        A clean one-colour copy of an SVG logo: every visible fill and stroke set to colour (gradients flattened
        to it), 'none' kept; the shapes' geometry and the viewBox kept; styles, classes, ids, editor attributes,
        definitions, metadata, comments, width and height removed; a background rectangle covering the whole
        viewBox removed. An element that is not a plain shape (an embedded image, text, a script, a clip or a
        reference) is refused.

    Args:
        (1) source [string]: The original SVG.
        (2) colour [string]: The colour, '#RRGGBB'.

    Returns:
        (1) parameter [xml.etree.ElementTree.Element]: The new SVG root.
    """

    try:
        original = ET.parse(source).getroot()
    except (OSError, ET.ParseError) as error:
        raise Logo_Error_Cls('The logo %s cannot be read as SVG (%s). Restore it in Media/Original/.' % (source, error)) from error
    if _Local(original.tag) != 'svg' or not original.get('viewBox'):
        raise Logo_Error_Cls('The logo %s has no <svg> root with a viewBox; a viewBox is required to scale it.' % source)
    view_box = [float(v) for v in original.get('viewBox').replace(',', ' ').split()]
    refused = sorted({_Local(e.tag) for e in original.iter() if _Local(e.tag) in CONST_REFUSED})
    if refused:
        raise Logo_Error_Cls('The logo %s contains %s; it is not a plain vector logo and is not converted.' % (source, ', '.join(refused)))
    classes = Read_Class_Paint(original)
    root = ET.Element('{%s}svg' % CONST_SVG_NS, {'viewBox': original.get('viewBox')})

    def Copy(parent_in: ET.Element, parent_out: ET.Element, inherited: tp.Dict[str, str]) -> None:
        """
        Description:
            Copy the children of an original element into the new tree, recoloured (recursive for groups).

        Args:
            (1 - 2) parent_in, parent_out [xml.etree.ElementTree.Element]: The original parent and its copy.
            (3) inherited [Dict<string, string>]: The paint the parent passes down.
        """

        for child in parent_in:
            if not isinstance(child.tag, str):
                continue
            name = _Local(child.tag)
            if not child.tag.startswith('{%s}' % CONST_SVG_NS) or name in CONST_DROPPED:
                continue
            paint = _Paint(child, classes, inherited)
            if name == 'g':
                group = ET.SubElement(parent_out, '{%s}g' % CONST_SVG_NS, {k: v for k, v in child.attrib.items() if k == 'transform'})
                Copy(child, group, paint)
                continue
            if name not in CONST_SHAPES:
                raise Logo_Error_Cls('The logo %s contains <%s>, which this script does not convert.' % (source, name))
            # A rectangle covering the whole viewBox is a background, not part of the logo.
            if name == 'rect' and [float(child.get(k, 0)) for k in ('x', 'y', 'width', 'height')] == view_box:
                continue
            attributes = {k: v for k, v in child.attrib.items() if k in CONST_GEOMETRY}
            # SVG paints an unset fill black; 'none' stays none; any other paint (a colour, a gradient) becomes colour.
            attributes['fill'] = 'none' if paint.get('fill', '#000000') == 'none' else colour
            if paint.get('stroke', 'none') != 'none':
                attributes['stroke'] = colour
            ET.SubElement(parent_out, '{%s}%s' % (CONST_SVG_NS, name), attributes)

    Copy(original, root, {})
    return root


def Write_Logo(root: ET.Element, path: str) -> str:
    """
    Description:
        Write an SVG root deterministically (two-space indent, no declaration, a final newline).

    Args:
        (1) root [xml.etree.ElementTree.Element]: The SVG.
        (2) path [string]: The file.

    Returns:
        (1) parameter [string]: The absolute path written.
    """

    ET.register_namespace('', CONST_SVG_NS)
    tree = ET.ElementTree(root)
    ET.indent(tree, '  ')
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(ET.tostring(root, encoding='unicode') + '\n')
    return path


def Check_Logo(path: str, colour: str) -> tp.List[str]:
    """
    Description:
        Every problem of a written logo: not valid XML, no viewBox, a width or height, a forbidden element, a
        fill or stroke other than colour or 'none', a style, class or namespaced (editor) attribute, an
        external reference.

    Args:
        (1) path [string]: The SVG.
        (2) colour [string]: Its one colour.

    Returns:
        (1) parameter [List<string>]: The problems (empty: the file is clean).
    """

    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as error:
        return ['not valid XML: %s' % error]
    problems = []
    if not root.get('viewBox'):
        problems.append('no viewBox')
    if root.get('width') or root.get('height'):
        problems.append('a fixed width or height')
    for e in root.iter():
        name = _Local(e.tag)
        if name in CONST_REFUSED or name in CONST_DROPPED:
            problems.append('a <%s> element' % name)
        for k, v in e.attrib.items():
            if k in ('fill', 'stroke') and v not in (colour, 'none'):
                problems.append('%s="%s" on <%s>' % (k, v, name))
            if k in ('style', 'class') or k.startswith('{') or 'href' in k or 'url(' in v:
                problems.append('the attribute %s on <%s>' % (_Local(k), name))
    return problems


def Shape_Path(element: ET.Element) -> tp.Any:
    """
    Description:
        A shape element as a matplotlib Path in SVG user units (y down): path data with M, L, H, V, C, S, Q, T, Z
        (absolute and relative), polygons, polylines, rectangles, circles, ellipses.

    Args:
        (1) element [xml.etree.ElementTree.Element]: The shape (no transform).

    Returns:
        (1) parameter [matplotlib.path.Path]: The outline.
    """

    from matplotlib.path import Path
    name = _Local(element.tag)
    if element.get('transform'):
        raise Logo_Error_Cls('The check image does not draw transformed shapes (<%s transform=...>).' % name)
    f = lambda k: float(element.get(k, 0))
    if name in ('polygon', 'polyline'):
        v = [float(x) for x in element.get('points').replace(',', ' ').split()]
        points = list(zip(v[0::2], v[1::2]))
        return Path(points + ([points[0]] if name == 'polygon' else []), closed=name == 'polygon')
    if name == 'rect':
        x, y, w, h = f('x'), f('y'), f('width'), f('height')
        return Path([(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)], closed=True)
    if name in ('circle', 'ellipse'):
        rx, ry = (f('r'), f('r')) if name == 'circle' else (f('rx'), f('ry'))
        unit = Path.unit_circle()
        return Path(unit.vertices * [rx, ry] + [f('cx'), f('cy')], unit.codes)
    if name != 'path':
        raise Logo_Error_Cls('The check image does not draw <%s>.' % name)
    tokens = re.findall(r'[A-Za-z]|[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?', element.get('d'))
    vertices, codes, i = [], [], 0
    command, pos, start, last_control = None, (0.0, 0.0), (0.0, 0.0), None
    numbers = lambda n: [float(t) for t in tokens[i:i + n]]
    while i < len(tokens):
        if tokens[i].isalpha():
            command = tokens[i]
            i += 1
            if command in 'Zz':
                vertices.append(start)
                codes.append(Path.CLOSEPOLY)
                pos, last_control = start, None
                continue
        if command is None:
            raise Logo_Error_Cls('The path data does not start with a command.')
        relative, c = command.islower(), command.upper()
        offset = pos if relative else (0.0, 0.0)
        if c in 'ML':
            x, y = numbers(2)
            i += 2
            pos = (offset[0] + x, offset[1] + y)
            vertices.append(pos)
            codes.append(Path.MOVETO if c == 'M' else Path.LINETO)
            if c == 'M':
                start = pos
                command = 'l' if relative else 'L'
            last_control = None
        elif c in 'HV':
            (value,) = numbers(1)
            i += 1
            pos = (offset[0] + value, pos[1]) if c == 'H' else (pos[0], offset[1] + value)
            vertices.append(pos)
            codes.append(Path.LINETO)
            last_control = None
        elif c in 'CS':
            if c == 'C':
                x1, y1, x2, y2, x, y = numbers(6)
                i += 6
                c1 = (offset[0] + x1, offset[1] + y1)
            else:
                x2, y2, x, y = numbers(4)
                i += 4
                c1 = pos if last_control is None else (2 * pos[0] - last_control[0], 2 * pos[1] - last_control[1])
            c2 = (offset[0] + x2, offset[1] + y2)
            pos = (offset[0] + x, offset[1] + y)
            vertices += [c1, c2, pos]
            codes += [Path.CURVE4] * 3
            last_control = c2
        elif c in 'QT':
            if c == 'Q':
                x1, y1, x, y = numbers(4)
                i += 4
                c1 = (offset[0] + x1, offset[1] + y1)
            else:
                x, y = numbers(2)
                i += 2
                c1 = pos if last_control is None else (2 * pos[0] - last_control[0], 2 * pos[1] - last_control[1])
            pos = (offset[0] + x, offset[1] + y)
            vertices += [c1, pos]
            codes += [Path.CURVE3] * 2
            last_control = c1
        else:
            raise Logo_Error_Cls("The check image does not draw the path command '%s'." % command)
    return Path(vertices, codes)


def Render_Check(logos: tp.Dict[str, tp.Dict[str, str]], path: str) -> str:
    """
    Description:
        The check image: each black logo on white (top row) and each white logo on GitHub's dark ground (bottom
        row), drawn from the SVGs' shapes with matplotlib.

    Args:
        (1) logos [Dict<string, Dict<string, string>>]: logo -> {variant: SVG path}.
        (2) path [string]: The PNG.

    Returns:
        (1) parameter [string]: The absolute path written.
    """

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import PathPatch
    names = sorted(logos)
    figure, axes = plt.subplots(len(CONST_VARIANTS), len(names), figsize=(4.0 * len(names), 2.0 * len(CONST_VARIANTS)), dpi=150,
                                squeeze=False)
    for row, (variant, colour) in enumerate(CONST_VARIANTS.items()):
        for column, name in enumerate(names):
            ax = axes[row][column]
            root = ET.parse(logos[name][variant]).getroot()
            x0, y0, width, height = [float(v) for v in root.get('viewBox').split()]
            for e in root.iter():
                if _Local(e.tag) in CONST_SHAPES and e.get('fill') != 'none':
                    ax.add_patch(PathPatch(Shape_Path(e), facecolor=colour, edgecolor='none'))
            margin = 0.08 * max(width, height)
            ax.set_xlim(x0 - margin, x0 + width + margin)
            ax.set_ylim(y0 + height + margin, y0 - margin)
            ax.set_aspect('equal')
            ax.set_facecolor(CONST_GROUNDS[variant])
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_title('%s_%s.svg' % (name, variant), fontsize=8)
    figure.tight_layout()
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    figure.savefig(path)
    plt.close(figure)
    return path


def main():
    """
    Description:
        Write Media/<Logo>_Black.svg and Media/<Logo>_White.svg from the vector originals in Media/Original/,
        check each written file, and draw the check image (Data/Generated/Images/logo_check.png). Deterministic:
        the same originals give the same bytes. Exit 1 if an original cannot be converted or a written file fails
        its check.

            .venv/bin/python Example/Media/make_logo_variants.py [--out DIR] [--check-image FILE]
    """

    parser = argparse.ArgumentParser(description='Black and white SVG logo variants from the originals.')
    parser.add_argument('--out', default=CONST_MEDIA, help='write the SVGs into this directory (default: Media/)')
    parser.add_argument('--check-image', default=CONST_CHECK_IMAGE, help='the check image (default: Data/Generated/Images/logo_check.png)')
    args = parser.parse_args()

    log = Script_Log_Cls('make_logo_variants')
    written, failed = {}, False
    for name, original in CONST_LOGOS.items():
        for variant, colour in CONST_VARIANTS.items():
            try:
                path = Write_Logo(Recolour_Logo(os.path.join(CONST_ORIGINALS, original), colour),
                                  os.path.join(args.out, '%s_%s.svg' % (name, variant)))
            except Logo_Error_Cls as error:
                print(log.Line('[ERROR] %s' % error))
                failed = True
                continue
            problems = Check_Logo(path, colour)
            failed = failed or bool(problems)
            written.setdefault(name, {})[variant] = path
            print(log.Line('[%s] %s: %d bytes%s' % ('ERROR' if problems else 'INFO', path, os.path.getsize(path),
                                                    '; ' + '; '.join(problems) if problems else ', clean')))
    if not failed:
        print(log.Line('[INFO] Check image: %s' % Render_Check(written, args.check_image)))
    log.Close()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
