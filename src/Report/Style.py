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
# File Name: ../Report/Style.py
## =========================================================================== ##

# OS (Operating system interfaces)
import os
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field

"""
Description:
    Initialization of constants.

    The comparison's figure style (owner, 2026-10-25; design-decisions): one definition, two themes - dark (near-black, for the
    screen) and light (white, for print) - with LaTeX-like typesetting from matplotlib's own Computer Modern fonts (mathtext
    'cm', no TeX, no vendored fonts), a colourblind-safe categorical palette (Okabe & Ito, 2008), one colour per program in
    every figure, panel letters, and both a 300 dpi PNG and an SVG per figure and theme. matplotlib is imported only where a
    figure is made. The programs' display names live here too (owner: "Default program", "Optimized program").
"""
# The programs' names on figures, labels and headings (the code keeps 'taught' / 'optimized').
#   Unit [Dict<string, string>]
CONST_PROGRAM_NAMES = {'taught': 'Default program', 'optimized': 'Optimized program'}
# The Okabe-Ito palette (Color Universal Design): orange, sky blue, bluish green, yellow, blue, vermillion, reddish purple, grey.
#   Unit [Dict<string, string>] (hex)
CONST_OKABE_ITO = {'orange': '#E69F00', 'sky': '#56B4E9', 'green': '#009E73', 'yellow': '#F0E442', 'blue': '#0072B2',
                   'vermillion': '#D55E00', 'purple': '#CC79A7', 'grey': '#999999'}
# The JIC Smart Factory purple, and the tint of it (40 % towards white) the dark theme uses: 5.09:1 on its background (the
# owner's condition: at least 4.5:1); the purple itself is 2.07:1 there, 9.15:1 on white.
#   Unit [string] (hex)
CONST_JIC_PURPLE, CONST_JIC_PURPLE_TINT = '#640ABA', '#A26CD6'
# The contrast a program's colour keeps on its theme's background (WCAG 2 for graphics and text).
#   Unit [float] (-)
CONST_MIN_CONTRAST = 4.5
# The figures' sizes: a single column, a double column (inches), the export resolution.
#   Unit [float] (in), [int] (dpi)
CONST_SINGLE, CONST_DOUBLE, CONST_DPI = 3.5, 7.2, 300
# The line width of a series and of a bound (pt).
#   Unit [float] (pt)
CONST_LINE, CONST_BOUND = 1.6, 0.9


@dataclass
class Theme_Str:
    """
    Description:
        One theme of the figure style: its name, the background, the inks, the grid, a colour per program and per moving
        object.
    """

    # The theme's name ('dark' or 'light').
    #   Unit [string]
    Name: str = ''
    # The figure background, the primary and the secondary ink, the grid and the bound lines.
    #   Unit [string] (hex)
    Background: str = ''
    Ink: str = ''
    Ink_2: str = ''
    Grid: str = ''
    Bound: str = ''
    # Per program ('taught', 'optimized') its colour.
    #   Unit [Dict<string, string>] (hex)
    Program: tp.Dict[str, str] = field(default_factory=dict)
    # Per moving object its colour (fills, lanes).
    #   Unit [Dict<string, string>] (hex)
    Object: tp.Dict[str, str] = field(default_factory=dict)


CONST_OBJECTS = {'robot': CONST_OKABE_ITO['orange'], 'loader Y': CONST_OKABE_ITO['vermillion'], 'loader Z': CONST_OKABE_ITO['yellow'],
                 'loader jaws': CONST_OKABE_ITO['green'], 'MG06 lid/handle': CONST_OKABE_ITO['purple'], 'PCB': CONST_OKABE_ITO['grey']}
CONST_THEMES = {'dark': Theme_Str(Name='dark', Background='#0D1117', Ink='#E6EDF3', Ink_2='#9DA7B3', Grid='#2A313C', Bound='#8B949E',
                                  Program={'taught': CONST_OKABE_ITO['sky'], 'optimized': CONST_JIC_PURPLE_TINT}, Object=dict(CONST_OBJECTS)),
                'light': Theme_Str(Name='light', Background='#FFFFFF', Ink='#111111', Ink_2='#4A4A4A', Grid='#E3E3E3', Bound='#6E6E6E',
                                   Program={'taught': CONST_OKABE_ITO['blue'], 'optimized': CONST_JIC_PURPLE}, Object=dict(CONST_OBJECTS))}


def Contrast(colour_1: str, colour_2: str) -> float:
    """
    Description:
        The contrast ratio of two colours (WCAG 2: the relative luminances' (L1 + 0.05) / (L2 + 0.05), the lighter first).

    Args:
        (1 - 2) colour_1, colour_2 [string]: The colours ('#RRGGBB').

    Returns:
        (1) parameter [float]: The ratio, 1 to 21.
    """

    def Luminance(colour: str) -> float:
        """
        Description:
            A colour's relative luminance (sRGB).

        Args:
            (1) colour [string]: '#RRGGBB'.

        Returns:
            (1) parameter [float]: 0 to 1.
        """

        c = [int(colour.lstrip('#')[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
        c = [x / 12.92 if x <= 0.03928 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
        return 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]

    a, b = sorted((Luminance(colour_1), Luminance(colour_2)), reverse=True)
    return (a + 0.05) / (b + 0.05)


def Parameters(theme: Theme_Str) -> tp.Dict[str, tp.Any]:
    """
    Description:
        matplotlib's settings of a theme: serif Computer Modern text and math (matplotlib's own cmr10 and mathtext 'cm'), the
        theme's colours, a light grid below the data, no top and right spines, thin ticks, the legend without a frame.

    Args:
        (1) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [Dict<string, Any>]: rcParams.
    """

    return {'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm', 'axes.formatter.use_mathtext': True,
            'axes.unicode_minus': False, 'font.size': 8.5, 'axes.titlesize': 9.0, 'axes.labelsize': 8.5, 'legend.fontsize': 7.5,
            'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5, 'figure.facecolor': theme.Background, 'axes.facecolor': theme.Background,
            'savefig.facecolor': theme.Background, 'text.color': theme.Ink, 'axes.labelcolor': theme.Ink, 'axes.edgecolor': theme.Ink_2,
            'axes.titlecolor': theme.Ink, 'xtick.color': theme.Ink_2, 'ytick.color': theme.Ink_2, 'grid.color': theme.Grid,
            'grid.linewidth': 0.5, 'axes.grid': True, 'axes.axisbelow': True, 'axes.spines.top': False, 'axes.spines.right': False,
            'axes.linewidth': 0.6, 'xtick.major.width': 0.6, 'ytick.major.width': 0.6, 'xtick.major.size': 2.5, 'ytick.major.size': 2.5,
            'lines.linewidth': CONST_LINE, 'legend.frameon': False, 'svg.fonttype': 'path', 'savefig.dpi': CONST_DPI,
            'savefig.bbox': 'tight', 'savefig.pad_inches': 0.04}


def Label(text: str) -> str:
    """
    Description:
        A label that holds an underscore (a job or a configuration name: J_RETURN_Y, Loader_Free): cmr10 has no underscore
        glyph, so the text is set in mathtext's roman with the underscores and spaces escaped; any other text as it is.

    Args:
        (1) text [string]: The text (no '$').

    Returns:
        (1) parameter [string]: The label.
    """

    if '_' not in text:
        return text
    return r'$\mathrm{%s}$' % text.replace('_', r'\_').replace(' ', r'\ ').replace('%', r'\%')


def Panel_Letter(axis: tp.Any, index: int, theme: Theme_Str) -> None:
    """
    Description:
        The panel letter (a), (b), ... at an axis' top left corner, outside the data.

    Args:
        (1) axis [matplotlib Axes]: The panel.
        (2) index [int]: Its index (0 -> (a)).
        (3) theme [Theme_Str(object)]: The theme.
    """

    # cmr10 has no bold face: the letter is set in mathtext's bold Computer Modern.
    axis.text(-0.02, 1.02, r'$\mathbf{(%s)}$' % 'abcdefghijklmnopqrstuvwxyz'[index], transform=axis.transAxes, ha='right', va='bottom',
              fontsize=9.5, color=theme.Ink)


def Save_Figure(figure: tp.Any, folder: str, stem: str, theme: Theme_Str) -> tp.List[str]:
    """
    Description:
        A figure as <stem>_<theme>.png (CONST_DPI) and <stem>_<theme>.svg (text as paths: the same glyphs everywhere), then
        closed.

    Args:
        (1) figure [matplotlib Figure]: The figure.
        (2) folder [string]: The folder (created).
        (3) stem [string]: The file name without the theme and extension.
        (4) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [List<string>]: The files written (PNG, SVG).
    """

    import matplotlib.pyplot as plt
    os.makedirs(folder, exist_ok=True)
    out = []
    for extension in ('png', 'svg'):
        path = os.path.abspath(os.path.join(folder, '%s_%s.%s' % (stem, theme.Name, extension)))
        figure.savefig(path, dpi=CONST_DPI, metadata={'Date': None} if extension == 'svg' else {'Software': None})
        out.append(path)
    plt.close(figure)
    return out
