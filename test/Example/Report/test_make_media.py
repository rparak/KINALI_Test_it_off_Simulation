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


def _Video(path):
    meta = next(imageio_ffmpeg.read_frames(str(path)))
    frames, _ = imageio_ffmpeg.count_frames_and_secs(str(path))
    return tuple(meta['size']), frames


@pytest.mark.slow
def test_the_media_are_written(tmp_path, run_script, built_trajectory):
    """
    Description:
        F14 (owner, 2026-10-25): exit 0; the first second of both programs' physics runs in Overview_Right at 1920 x 1080 and
        2560 x 1440 (61 frames at 60 fps), the 2 x 2 mosaics and the side by side (2560 x 1440, 2560 x 720), the eight event
        clips per program (1080p, 0.2 s either side); the GIF previews (960 px wide, at most 10 MB); the view images of
        Overview_Right at the seven moments of each program and the collision images (2560 x 1440); the curated README images,
        the key points' sheet and the collision overlays at most 1.5 MB each; the 25 point images; nothing in the working
        directory.
    """

    folder = os.path.dirname(str(built_trajectory('optimized')))
    assert os.path.dirname(str(built_trajectory('taught'))) == folder
    out, media, work = tmp_path / 'out', tmp_path / 'media', tmp_path / 'work'
    work.mkdir()
    arguments = ['--out', str(out), '--media', str(media), '--trajectories', folder, '--until', '1.0', '--clip', '0.2', '--views',
                 'Overview_Right']
    result = run_script('Report/make_media.py', arguments, work)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    assert os.listdir(work) == []
    for stem in ('Default', 'Optimized'):
        assert _Video(out / ('%s_Overview_Right_1080p.mp4' % stem)) == ((1920, 1080), 61)
        assert _Video(out / ('%s_Overview_Right_1440p.mp4' % stem)) == ((2560, 1440), 61)
        assert _Video(out / ('%s_Mosaic_1440p.mp4' % stem)) == ((2560, 1440), 61)
    assert _Video(out / 'Default_vs_Optimized_Side_by_Side.mp4') == ((2560, 720), 61)
    events = ('fetch', 'fork_close', 'fork_open', 'handover', 'insert', 'pick', 'place', 'take')
    assert sorted(os.listdir(out / 'Clips')) == sorted('%s_%s.mp4' % (s, e) for s in ('Default', 'Optimized') for e in events)
    assert _Video(out / 'Clips' / 'Optimized_insert.mp4') == ((1920, 1080), 25)
    moments = ('MG06_closed', 'handover', 'insert', 'pick', 'place', 'take', 'test')
    collision = ['Collision_%s.png' % v for v in ('Input_Crate', 'Loader_Handover', 'MG06_Fork', 'Only_Overview_Right', 'Overview_Right')]
    assert sorted(os.listdir(out / 'Views')) == sorted(['%s_Overview_Right_%s.png' % (s, m) for s in ('Default', 'Optimized') for m in moments]
                                                       + collision)
    assert all(Image.open(out / 'Views' / f).size == (2560, 1440) for f in os.listdir(out / 'Views'))
    curated = ['View_%s.png' % v for v in ('Input_Crate', 'Loader_Handover', 'MG06_Fork', 'MG06_Insert', 'Output_Crate', 'Overview_Right')]
    gifs = ['Cycle_Optimized.gif', 'Cycle_Side_by_Side.gif']
    assert sorted(os.listdir(media)) == sorted(curated + gifs + ['Collision_Loader_Handover.png', 'Collision_Overview_Right.png', 'Key_Points.png'])
    assert all(os.path.getsize(media / f) <= 1500 * 1000 for f in os.listdir(media) if f.endswith('.png'))
    for name in gifs:
        assert Image.open(media / name).size[0] == 960 and os.path.getsize(media / name) <= 10 * 1000 * 1000
    assert len(os.listdir(out / 'Points')) == 25
