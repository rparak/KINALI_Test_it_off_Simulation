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
# File Name: ../test/Report/test_media.py
## =========================================================================== ##

# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Pillow (Reading the GIF back) [pip3 install pillow]
from PIL import Image
# imageio-ffmpeg (Reading the video back) [pip3 install imageio-ffmpeg]
import imageio_ffmpeg
# Custom Lib.: the module under test
#   ../Report/Media
from Report.Media import Frame_Renderer_Cls, Label_Frame, Frame_Samples, Video_Writer_Cls, Write_Gif, Contact_Sheet, Media_Error_Cls

"""
Description:
    The comparison's media (FEATURES.md F14): the labelled frames, the H.264 video, the GIF kept under a size, the contact
    sheet, the frames of the cell; on small inputs, read back.
"""


def _Frames(n, width=160, height=96, seed=0):
    generator = np.random.default_rng(seed)
    return [generator.integers(0, 256, (height, width, 3), dtype=np.uint8) for _ in range(n)]


def test_a_label_darkens_the_top_left_corner_only():
    """
    Description:
        The label's band and text change only the top left corner: the frame keeps its size, its bottom right half is
        unchanged; a '§' (no glyph in the font) is dropped, not drawn as a box.
    """

    image = np.full((360, 640, 3), 200, np.uint8)
    labelled = Label_Frame(image, ['Optimized program  |  S11_To_Take', 'Wait(1.648)'])
    assert labelled.shape == image.shape and np.array_equal(labelled[180:, 320:], image[180:, 320:])
    assert labelled[2, 2].sum() < image[2, 2].sum()
    assert np.array_equal(Label_Frame(image, ['SPEC §4.7']), Label_Frame(image, ['SPEC 4.7']))


def test_the_frames_follow_the_run_at_a_speed():
    """
    Description:
        1 s of a run sampled every 4 ms at 60 frames per second in real time: 61 frames from sample 0 to sample 250, each the
        nearest sample (frame 5 at 83.3 ms: sample 21); at 4 x
        speed and 15 frames per second, every frame 0.2667 s of the run apart.
    """

    real = Frame_Samples(1.0, 0.004, 60.0, 1.0)
    assert len(real) == 61 and real[0] == 0 and real[-1] == 250 and real[1] == 4 and real[5] == 21
    fast = Frame_Samples(8.0, 0.004, 15.0, 4.0)
    assert len(fast) == 31 and fast[3] == 200


def test_a_video_is_written_and_read_back(tmp_path):
    """
    Description:
        Ten 160 x 96 frames at 10 fps: an H.264 MP4 of 10 frames and 1.0 s; a frame size that is not a multiple of 8 is refused
        with its size named.
    """

    path = str(tmp_path / 'clip.mp4')
    writer = Video_Writer_Cls(path, 160, 96, 10.0)
    for frame in _Frames(10):
        writer.Write(frame)
    writer.Close()
    frames, seconds = imageio_ffmpeg.count_frames_and_secs(path)
    assert frames == 10 and seconds == pytest.approx(1.0, abs=0.01)
    with pytest.raises(Media_Error_Cls, match='162x96 px is not a multiple of 8'):
        Video_Writer_Cls(str(tmp_path / 'bad.mp4'), 162, 96, 10.0)


def test_a_gif_keeps_its_rate_and_fits_its_size(tmp_path):
    """
    Description:
        A GIF at 15 fps stores 10 ms frame times that average 1000 / 15 ms (70, 70, 60 ...); a limit the full palette misses
        is met with fewer colours or every other frame; a limit nothing meets is refused and leaves no file.
    """

    frames = _Frames(30, seed=1)
    path = str(tmp_path / 'a.gif')
    size, colours, kept = Write_Gif(frames, path, 15.0, 160, 10 ** 8)
    assert (colours, kept) == (256, 30) and size == os.path.getsize(path)
    image = Image.open(path)
    durations = []
    for k in range(image.n_frames):
        image.seek(k)
        durations.append(image.info['duration'])
    assert sum(durations) == 2000 and set(durations) == {60, 70}
    smaller, colours, kept = Write_Gif(frames, path, 15.0, 160, size - 1)
    assert smaller <= size - 1 and (colours < 256 or kept == 15)
    with pytest.raises(Media_Error_Cls, match='does not fit'):
        Write_Gif(frames, str(tmp_path / 'b.gif'), 15.0, 160, 100)
    assert not os.path.exists(str(tmp_path / 'b.gif'))


def test_a_contact_sheet_is_a_grid_with_gaps():
    """
    Description:
        Five 32 x 18 tiles in three columns: two rows, 8 px gaps around every tile.
    """

    sheet = Contact_Sheet([np.zeros((18, 32, 3), np.uint8)] * 5, ['P_%d' % k for k in range(5)], 3)
    assert sheet.shape == (2 * 18 + 3 * 8, 3 * 32 + 4 * 8, 3)
    assert tuple(sheet[0, 0]) == (252, 252, 251)


def test_the_cell_renders_from_a_fixed_camera(cell_model):
    """
    Description:
        A frame of the cell from the fixed camera; only the collision groups shown it differs; a frame beyond the offscreen
        buffer is refused with both sizes named.
    """

    data = mujoco.MjData(cell_model)
    mujoco.mj_forward(cell_model, data)
    camera = (0.10, -0.34, 0.40, 2.0, 78.0, -34.0)
    renderer = Frame_Renderer_Cls(cell_model, 160, 96, camera)
    full = renderer.Render(data)
    renderer.Close()
    renderer = Frame_Renderer_Cls(cell_model, 160, 96, camera, groups=[1, 3])
    collision = renderer.Render(data)
    renderer.Close()
    assert full.shape == (96, 160, 3) and full.std() > 0 and not np.array_equal(full, collision)
    with pytest.raises(Media_Error_Cls, match='3000x1440 px exceeds the model.s offscreen buffer 2560x1440 px'):
        Frame_Renderer_Cls(cell_model, 3000, 1440, camera)
