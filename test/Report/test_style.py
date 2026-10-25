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
# File Name: ../test/Report/test_style.py
## =========================================================================== ##

# OS (Operating system interfaces)
import os
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Report/Style
from Report.Style import (Contrast, Label, Parameters, Panel_Letter, Save_Figure, CONST_THEMES, CONST_PROGRAM_NAMES, CONST_JIC_PURPLE,
                          CONST_JIC_PURPLE_TINT, CONST_MIN_CONTRAST, CONST_DPI)

"""
Description:
    The comparison's figure style (owner, 2026-10-25): the themes' colours and their contrast, the labels, the files a figure is
    saved as.
"""


def test_contrast_is_the_wcag_ratio():
    """
    Description:
        Black on white is 21:1, a colour on itself 1:1, the order does not matter.
    """

    assert Contrast('#000000', '#FFFFFF') == pytest.approx(21.0) and Contrast('#640ABA', '#640ABA') == pytest.approx(1.0)
    assert Contrast('#0072B2', '#FFFFFF') == pytest.approx(Contrast('#FFFFFF', '#0072B2'))


@pytest.mark.parametrize('theme', sorted(CONST_THEMES))
def test_each_program_colour_reads_on_its_theme(theme):
    """
    Description:
        Both programs' colours keep at least 4.5:1 on their theme's background and differ from each other; the ink too.
    """

    t = CONST_THEMES[theme]
    for colour in list(t.Program.values()) + [t.Ink]:
        assert Contrast(colour, t.Background) >= CONST_MIN_CONTRAST, (theme, colour)
    assert t.Program['taught'] != t.Program['optimized']


def test_the_jic_purple_is_tinted_only_on_the_dark_theme():
    """
    Description:
        The owner's condition: the JIC purple itself is below 4.5:1 on the dark background (2.07:1), so the dark theme uses its
        40 % tint (5.09:1); the light theme the purple itself (9.15:1).
    """

    dark, light = CONST_THEMES['dark'], CONST_THEMES['light']
    assert Contrast(CONST_JIC_PURPLE, dark.Background) < CONST_MIN_CONTRAST
    assert dark.Program['optimized'] == CONST_JIC_PURPLE_TINT and light.Program['optimized'] == CONST_JIC_PURPLE
    assert Contrast(CONST_JIC_PURPLE_TINT, dark.Background) == pytest.approx(5.09, abs=0.01)


def test_a_label_with_an_underscore_is_set_in_math_roman():
    """
    Description:
        cmr10 has no underscore: a name with one is set as mathtext roman with the underscores and spaces escaped; other text stays.
    """

    assert Label('J_RETURN_Y') == r'$\mathrm{J\_RETURN\_Y}$'
    assert Label('Loader_Free (item 16)') == r'$\mathrm{Loader\_Free\ (item\ 16)}$'
    assert Label('robot motion') == 'robot motion' and CONST_PROGRAM_NAMES == {'taught': 'Default program', 'optimized': 'Optimized program'}


def test_a_figure_is_saved_as_png_and_svg_per_theme(tmp_path):
    """
    Description:
        Save_Figure writes <stem>_<theme>.png at 300 dpi and .svg, in the theme's background; the settings use Computer Modern
        (mathtext 'cm', serif cmr10).
    """

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    theme = CONST_THEMES['dark']
    with plt.rc_context(Parameters(theme)):
        figure, axis = plt.subplots(figsize=(1.0, 1.0))
        axis.plot([0, 1], [0, 1])
        Panel_Letter(axis, 0, theme)
        files = Save_Figure(figure, str(tmp_path), 'probe', theme)
    assert [os.path.basename(f) for f in files] == ['probe_dark.png', 'probe_dark.svg']
    image = Image.open(files[0])
    assert image.info['dpi'][0] == pytest.approx(CONST_DPI, abs=1)
    assert image.convert('RGB').getpixel((1, 1)) == (13, 17, 23)
    assert '<svg' in open(files[1], encoding='utf-8').read()[:500]
    assert Parameters(theme)['mathtext.fontset'] == 'cm' and Parameters(theme)['font.serif'][0] == 'cmr10'
