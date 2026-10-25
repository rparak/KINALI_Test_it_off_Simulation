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
# File Name: ../test/Example/Report/test_make_media.py
## =========================================================================== ##

# OS (Operating system interfaces)
import os
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# imageio-ffmpeg (Reading the video back) [pip3 install imageio-ffmpeg]
import imageio_ffmpeg
# Pillow (Reading the images back) [pip3 install pillow]
from PIL import Image

"""
Description:
    Example/Report/make_media.py run as a person runs it (fresh interpreter, unrelated working directory), on the optimized
    trajectory built into a temporary folder; the videos cut short with --until.
"""


@pytest.mark.slow
def test_the_media_are_written(tmp_path, run_script, built_trajectory):
    """
    Description:
        F14: exit 0; the first second of the optimized program's physics run as 1920 x 1080 and 2560 x 1440 H.264 videos at
        60 fps (61 frames each), its GIF preview from the first one's frames (960 px wide, at most 10 MB), the 25 point images
        and their contact sheet, the collision image; nothing in the working directory.
    """

    folder = os.path.dirname(str(built_trajectory('optimized')))
    built_trajectory('taught')
    out, media, work = tmp_path / 'out', tmp_path / 'media', tmp_path / 'work'
    work.mkdir()
    arguments = ['--out', str(out), '--media', str(media), '--trajectories', folder, '--program', 'optimized', '--until', '1.0',
                 '--resolution', '1080p', '1440p']
    result = run_script('Report/make_media.py', arguments, work)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    assert os.listdir(work) == []
    for name in ('1080p', '1440p'):
        frames, seconds = imageio_ffmpeg.count_frames_and_secs(str(out / ('Optimized_Program_%s.mp4' % name)))
        assert frames == 61 and seconds == pytest.approx(61 / 60.0, abs=0.02)
    assert sorted(os.listdir(media)) == ['Cycle_Optimized.gif', 'Key_Points.png']
    gif = Image.open(media / 'Cycle_Optimized.gif')
    assert gif.size == (960, 540) and os.path.getsize(media / 'Cycle_Optimized.gif') <= 10 * 1000 * 1000
    assert len(os.listdir(out / 'Points')) == 25 and Image.open(out / 'Collision_Structure.png').size == (1920, 1080)
