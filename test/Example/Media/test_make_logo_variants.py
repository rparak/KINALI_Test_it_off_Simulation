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
# File Name: ../test/Example/Media/test_make_logo_variants.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Hashlib (Secure hashes)
import hashlib
# Importlib (Import machinery)
import importlib.util
# XML (ElementTree XML API)
import xml.etree.ElementTree as ET
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    Example/Media/make_logo_variants.py, run as a person runs it and imported for its functions.
"""
# The variants the script writes into Media/.
#   Unit [List<string>]
CONST_OUTPUTS = ['JIC_Smart_Factory_Black.svg', 'JIC_Smart_Factory_White.svg', 'KINALI_Black.svg', 'KINALI_White.svg']
# The owner's originals, pinned (2026-10-24): the JIC file is byte-identical to jic-sf-ui-style's
# assets/logos/jic-smart-factory.svg, the brand's source of truth (design-decisions: the brand).
#   Unit [Dict<string, string>]
CONST_ORIGINALS = {'jic-smart-factory.svg': '05cc6e18366cb91b4316b7d06fbcaf6290ac98526720dc019dcb7e2a1c75c9e1',
                   'kinali.svg': '676152befa0ec31abd6e9dbe403fb0ec5ebaf5175e14b9a358054776e20d2a23'}
# The SVG namespace.
#   Unit [string]
CONST_NS = '{http://www.w3.org/2000/svg}'


@pytest.fixture(scope='module')
def logos(root):
    path = os.path.join(root, 'Example', 'Media', 'make_logo_variants.py')
    sys.path.insert(0, os.path.dirname(path))
    spec = importlib.util.spec_from_file_location('make_logo_variants', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _Geometry(path):
    """
    Description:
        The shapes of an SVG in document order: (element, its geometry attributes).
    """

    keep = ('d', 'points', 'x', 'y', 'width', 'height', 'cx', 'cy', 'r')
    return [(e.tag.rsplit('}', 1)[-1], tuple((k, e.get(k)) for k in keep if e.get(k) is not None))
            for e in ET.parse(path).getroot().iter() if e.tag.rsplit('}', 1)[-1] in ('path', 'polygon', 'rect', 'circle')]


@pytest.mark.parametrize('name', sorted(CONST_ORIGINALS))
def test_the_originals_are_the_pinned_files(root, name):
    """
    Description:
        The owner's originals in Media/Original/ are the files received (sha256), kept untouched for regeneration.
    """

    with open(os.path.join(root, 'Media', 'Original', name), 'rb') as handle:
        assert hashlib.sha256(handle.read()).hexdigest() == CONST_ORIGINALS[name]


def test_the_script_regenerates_the_committed_variants_byte_for_byte(root, tmp_path, run_script):
    """
    Description:
        From an unrelated directory, --out writes the four variants, each identical to the committed one in
        Media/ (deterministic), the check image is drawn, and the working directory stays empty.
    """

    work = tmp_path / 'work'
    work.mkdir()
    result = run_script('Media/make_logo_variants.py', ['--out', str(tmp_path / 'out'), '--check-image', str(tmp_path / 'check.png')], work)
    assert result.returncode == 0, result.stdout + result.stderr
    for name in CONST_OUTPUTS:
        assert (tmp_path / 'out' / name).read_bytes() == open(os.path.join(root, 'Media', name), 'rb').read(), name
    assert (tmp_path / 'check.png').stat().st_size > 0 and os.listdir(work) == []


@pytest.mark.parametrize('name', CONST_OUTPUTS)
def test_each_variant_is_one_colour_with_the_original_geometry(root, logos, name):
    """
    Description:
        Each committed variant passes the check (only its colour or none, a viewBox, no width or height, no
        image, style, class or editor attribute), has the original's viewBox, and keeps every shape's geometry.
    """

    colour = '#000000' if name.endswith('_Black.svg') else '#FFFFFF'
    path = os.path.join(root, 'Media', name)
    assert logos.Check_Logo(path, colour) == []
    original = os.path.join(root, 'Media', 'Original', 'jic-smart-factory.svg' if name.startswith('JIC') else 'kinali.svg')
    assert ET.parse(path).getroot().get('viewBox') == ET.parse(original).getroot().get('viewBox')
    assert _Geometry(path) == _Geometry(original)
    fills = {e.get('fill') for e in ET.parse(path).getroot().iter() if e.get('fill')}
    assert fills == {colour}


def _Write(tmp_path, body, attributes='viewBox="0 0 10 10"'):
    path = tmp_path / 'logo.svg'
    path.write_text('<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" %s>%s</svg>' % (attributes, body))
    return str(path)


def test_classes_gradients_backgrounds_and_metadata_are_resolved_or_removed(tmp_path, logos):
    """
    Description:
        A class fill and a gradient fill become the colour, a full-viewBox background rectangle, the style, the
        defs, the metadata, ids and width / height are removed, fill='none' stays none.
    """

    body = ('<style>.a { fill: #640aba; }</style><defs><linearGradient id="g"><stop offset="0" stop-color="red"/></linearGradient></defs>'
            '<metadata>editor</metadata><rect x="0" y="0" width="10" height="10" fill="#fff"/>'
            '<path id="p1" class="a" d="M1 1h2v2z"/><circle cx="5" cy="5" r="1" style="fill:url(#g)"/>'
            '<rect x="1" y="6" width="2" height="2" fill="none" stroke="#123456"/>')
    root = logos.Recolour_Logo(_Write(tmp_path, body, 'width="100" height="100" viewBox="0 0 10 10"'), '#000000')
    out = logos.Write_Logo(root, str(tmp_path / 'out.svg'))
    assert logos.Check_Logo(out, '#000000') == []
    shapes = [(e.tag.rsplit('}', 1)[-1], e.get('fill'), e.get('stroke')) for e in ET.parse(out).getroot()]
    assert shapes == [('path', '#000000', None), ('circle', '#000000', None), ('rect', 'none', '#000000')]


@pytest.mark.parametrize('body, word', [('<image href="logo.png" width="10" height="10"/>', 'image'),
                                        ('<text x="1" y="1">KINALI</text>', 'text'),
                                        ('<use href="#a"/>', 'use')])
def test_a_logo_that_is_not_plain_vector_is_refused(tmp_path, logos, body, word):
    """
    Description:
        An embedded image (a raster wrapped in SVG), text or a reference is refused with a sentence naming it,
        never dropped silently.
    """

    with pytest.raises(logos.Logo_Error_Cls, match='contains %s; it is not a plain vector logo' % word):
        logos.Recolour_Logo(_Write(tmp_path, body), '#000000')


def test_a_logo_without_a_viewbox_is_refused(tmp_path, logos):
    """
    Description:
        A viewBox is required (the variants scale without a fixed size).
    """

    with pytest.raises(logos.Logo_Error_Cls, match='no <svg> root with a viewBox'):
        logos.Recolour_Logo(_Write(tmp_path, '<path d="M0 0h1v1z"/>', 'width="10" height="10"'), '#000000')


def test_the_check_finds_a_foreign_colour_and_an_image(tmp_path, logos):
    """
    Description:
        Check_Logo names a fill other than the variant's colour, a <image> element and a fixed width.
    """

    path = _Write(tmp_path, '<path d="M0 0h1v1z" fill="#640aba"/><image href="x.png"/>', 'viewBox="0 0 10 10" width="10"')
    problems = logos.Check_Logo(path, '#000000')
    assert 'fill="#640aba" on <path>' in problems and 'a <image> element' in problems and 'a fixed width or height' in problems


def test_the_path_reader_draws_relative_and_smooth_commands(logos):
    """
    Description:
        Path data with relative lines, H / V, a cubic and its smooth continuation reaches the expected points.
    """

    element = ET.Element(CONST_NS + 'path', {'d': 'M1,1h2v2l-1,1c1,0 2,1 2,2s1,2 2,2Z'})
    vertices = logos.Shape_Path(element).vertices.tolist()
    assert vertices[:4] == [[1, 1], [3, 1], [3, 3], [2, 4]]
    assert vertices[4:7] == [[3, 4], [4, 5], [4, 6]] and vertices[7:10] == [[4, 7], [5, 8], [6, 8]]
