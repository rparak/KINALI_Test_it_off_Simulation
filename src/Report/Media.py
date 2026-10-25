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
# File Name: ../Report/Media.py
## =========================================================================== ##

# OS (Operating system interfaces)
import os
# Copy (The render-only model copy)
import copy
# IO (A PNG measured in memory)
import io
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Pillow (Images: the label, the GIF, the contact sheet) [pip3 install pillow]
from PIL import Image, ImageDraw, ImageFont
# imageio-ffmpeg (a pinned ffmpeg binary: the H.264 video) [pip3 install imageio-ffmpeg]
import imageio_ffmpeg
# Custom Lib.:
#   ../Render/Core - the studio look's render flags
from Render.Core import CONST_LOOK_FLAGS

"""
Description:
    Initialization of constants.

    The media of the comparison (FEATURES.md F14): frames of the cell from named free-camera views with the studio look (on a
    render-only copy of the model: the views' field of view, the aluminium toned down - the model itself is never changed),
    a step label on them, an H.264 video (imageio-ffmpeg's ffmpeg), a GIF preview kept under a size, a contact sheet, a PNG
    kept under a size, the collision geoms laid over the visual cell. It knows nothing of the program: it takes a model, its
    data and images.
"""
# The label: its text height as a fraction of the frame's height, its band's colour and opacity, the text colour, the
# margin around it as a fraction of the text height.
#   Unit [float] (-), [Tuple<int>] (RGB), [int] (0-255), [Tuple<int>] (RGB), [float] (-)
CONST_LABEL_HEIGHT, CONST_LABEL_BAND, CONST_LABEL_ALPHA, CONST_LABEL_INK, CONST_LABEL_PAD = 1.0 / 40.0, (11, 11, 11), 170, (240, 239, 236), 0.5
# A character the scalable default font has no glyph for, and its replacement ('SPEC §4.7' -> 'SPEC 4.7').
#   Unit [Tuple<string>]
CONST_LABEL_REPLACE = ('§', '')
# The video: H.264 (libx264) at a constant rate factor and preset, yuv420p (plays everywhere), the moov atom first (streams),
# frame sizes rounded to a multiple of 8.
#   Unit [string], [int], [string], [int]
CONST_CODEC, CONST_CRF, CONST_PRESET, CONST_MACRO_BLOCK = 'libx264', 18, 'slow', 8
# The render-only tone-down (owner, 2026-10-25: visual only): the 86 geoms with the studio look's aluminium material (the
# table's slats, the MG06 press plate and base, the loader's carriages; their own rgba 0.85-0.93, specular 0.75, shininess 0.65)
# saturated to white under the key light (2.81 % of the Input_Crate view's pixels above 245); with their rgb scaled by 0.75 and
# the material's specular 0.3 / shininess 0.5, 0.01 % (measured 2026-10-25). The model's files and its fingerprint unchanged.
#   Unit [Dict<string, Tuple<float>>] (rgb scale, specular, shininess)
CONST_TONE = {'look_alu': (0.75, 0.30, 0.50)}
# The collision overlay: its opacity over the visual cell, the visual cell's dimming, the depth within which a collision geom
# counts as visible (it lies on or before the visual surface; 12 mm: the offscreen depth is coarse without ARB_clip_control on
# macOS, and the planning-only table boxes lie within millimetres of the slats), the outline's darkening.
#   Unit [float] (-), [float] (-), [float] (m), [float] (-)
CONST_OVERLAY_ALPHA, CONST_OVERLAY_DIM, CONST_OVERLAY_DEPTH, CONST_OVERLAY_EDGE = 0.38, 0.88, 0.012, 0.55
# The PNGs kept under a size: the widths tried, largest first.
#   Unit [Tuple<int>] (px)
CONST_PNG_WIDTHS = (2560, 2240, 1920, 1600, 1440, 1280, 1120, 960)
# The GIF's palettes tried, largest first, then every other frame dropped, until the file fits.
#   Unit [Tuple<int>] (colours)
CONST_GIF_COLOURS = (256, 192, 128, 96, 64)
# The contact sheet: the gap between its tiles, its background, a tile's label height as a fraction of the tile's height.
#   Unit [int] (px), [Tuple<int>] (RGB), [float] (-)
CONST_SHEET_GAP, CONST_SHEET_BACKGROUND, CONST_SHEET_LABEL = 8, (252, 252, 251), 1.0 / 14.0


@dataclass
class View_Str:
    """
    Description:
        A named free-camera view: the point looked at, the distance, the two angles (as the viewer's mouse camera) and the
        vertical field of view.
    """

    # The point looked at.
    #   Unit [Tuple<float>] (m)
    Lookat: tp.Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # The distance, the azimuth, the elevation (negative: from above), the vertical field of view.
    #   Unit [float] (m), [float] (deg), [float] (deg), [float] (deg)
    Distance: float = 1.0
    Azimuth: float = 90.0
    Elevation: float = -30.0
    Fovy: float = 45.0


# The named views (owner, 2026-10-25; free cameras: no camera is added to the model). Overview_Right matches the owner's viewer
# screenshot (2026-10-25, rendered side by side with it); Overview_Left mirrors it about the cell's centre plane x = 0.13 m;
# the close views were chosen from rendered candidates so their events are not occluded.
#   Unit [Dict<string, View_Str>]
CONST_VIEWS = {'Overview_Right': View_Str((0.33, -0.08, 0.06), 3.15, 127.0, -23.0, 45.0),
               'Overview_Left': View_Str((-0.07, -0.08, 0.06), 3.15, 53.0, -23.0, 45.0),
               'Top': View_Str((0.13, -0.36, 0.10), 2.0, 90.0, -89.0, 45.0),
               'Input_Crate': View_Str((0.157, -0.70, 0.06), 0.9, 60.0, -40.0, 40.0),
               'Output_Crate': View_Str((0.406, -0.72, 0.05), 0.85, 60.0, -45.0, 40.0),
               'Loader_Handover': View_Str((0.01, -0.33, 0.18), 0.75, 120.0, -25.0, 40.0),
               'MG06_Insert': View_Str((0.0, -0.05, 0.22), 0.95, 120.0, -35.0, 40.0),
               'MG06_Fork': View_Str((0.03, -0.22, 0.32), 0.9, 40.0, -20.0, 40.0)}


def Render_Model(model: mujoco.MjModel) -> mujoco.MjModel:
    """
    Description:
        A render-only copy of a model with the tone-down of CONST_TONE (the rgb of every geom with the material scaled - a geom's
        own rgba is what is drawn -, the material's specular and shininess set); the model itself is not changed.

    Args:
        (1) model [mujoco.MjModel]: The model.

    Returns:
        (1) parameter [mujoco.MjModel]: The copy.
    """

    out = copy.deepcopy(model)
    for name, (scale, specular, shininess) in CONST_TONE.items():
        k = mujoco.mj_name2id(out, mujoco.mjtObj.mjOBJ_MATERIAL, name)
        if k >= 0:
            out.geom_rgba[out.geom_matid == k, :3] *= scale
            out.mat_specular[k], out.mat_shininess[k] = specular, shininess
    return out


class Media_Error_Cls(RuntimeError):
    """
    Description:
        A medium cannot be made: a frame size beyond the model's offscreen buffer, or a GIF that does not fit its size
        however it is reduced. The message names the value and the limit.
    """


class Frame_Renderer_Cls(object):
    """
    Description:
        Frames of a model from a named view (View_Str) with the studio look's render flags (Render.Core), on a render-only copy
        (Render_Model: the tone-down, the view's field of view), from one offscreen renderer kept open; optionally only some
        geom groups shown; RGB, depth or the geom of each pixel.

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The model (not changed).
            (2) width, height [int]: The frame size (px), within the model's offscreen buffer.
            (3) view [View_Str(object)]: The view.
            (4) groups [List<int>]: The geom groups shown, or None for the model's defaults.
            (5) tone [bool]: The tone-down of CONST_TONE (False: the model's look as it is).

        Example:
            Initialization:
                Cls = Frame_Renderer_Cls(model, 1920, 1080, CONST_VIEWS['Overview_Right'])

            Features:
                # Functions of the class.
                Cls.Render(data); Cls.Depth(data); Cls.Geoms(data); Cls.Close()
    """

    def __init__(self, model: mujoco.MjModel, width: int, height: int, view: View_Str, groups: tp.Optional[tp.Sequence[int]] = None,
                 tone: bool = True) -> None:
        if width > model.vis.global_.offwidth or height > model.vis.global_.offheight:
            raise Media_Error_Cls('A frame of %dx%d px exceeds the model\'s offscreen buffer %dx%d px (models/scene.xml is read with '
                                  'Scene.Core\'s buffer).' % (width, height, model.vis.global_.offwidth, model.vis.global_.offheight))
        self.Model = Render_Model(model) if tone else copy.deepcopy(model)
        self.Model.vis.global_.fovy = view.Fovy
        self.__renderer = mujoco.Renderer(self.Model, height, width)
        self.__camera = mujoco.MjvCamera()
        self.__camera.lookat[:] = view.Lookat
        self.__camera.distance, self.__camera.azimuth, self.__camera.elevation = view.Distance, view.Azimuth, view.Elevation
        self.__option = mujoco.MjvOption()
        if groups is not None:
            self.__option.geomgroup[:] = [1 if g in groups else 0 for g in range(len(self.__option.geomgroup))]

    def __Update(self, data: mujoco.MjData) -> None:
        """
        Description:
            The scene of the data's state with the look's flags.

        Args:
            (1) data [mujoco.MjData]: The state.
        """

        self.__renderer.update_scene(data, camera=self.__camera, scene_option=self.__option)
        for flag, on in CONST_LOOK_FLAGS.items():
            self.__renderer.scene.flags[flag] = on

    def Render(self, data: mujoco.MjData) -> np.ndarray:
        """
        Description:
            A frame of the data's current state (after mj_forward or mj_kinematics).

        Args:
            (1) data [mujoco.MjData]: The state.

        Returns:
            (1) parameter [Matrix<uint8> HxWx3]: The RGB frame.
        """

        self.__Update(data)
        return self.__renderer.render().copy()

    def Depth(self, data: mujoco.MjData) -> np.ndarray:
        """
        Description:
            The depth of each pixel of the data's current state.

        Args:
            (1) data [mujoco.MjData]: The state.

        Returns:
            (1) parameter [Matrix<float> HxW]: The depth (m).
        """

        self.__renderer.enable_depth_rendering()
        try:
            self.__Update(data)
            return self.__renderer.render().copy()
        finally:
            self.__renderer.disable_depth_rendering()

    def Geoms(self, data: mujoco.MjData) -> np.ndarray:
        """
        Description:
            The geom drawn at each pixel of the data's current state.

        Args:
            (1) data [mujoco.MjData]: The state.

        Returns:
            (1) parameter [Matrix<int> HxW]: The geom id (-1 where no geom).
        """

        self.__renderer.enable_segmentation_rendering()
        try:
            self.__Update(data)
            ids = self.__renderer.render().copy()
        finally:
            self.__renderer.disable_segmentation_rendering()
        return np.where(ids[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), ids[..., 0], -1)

    def Close(self) -> None:
        """
        Description:
            Release the renderer.
        """

        self.__renderer.close()


def Collision_Overlay(image: np.ndarray, depth: np.ndarray, geoms: np.ndarray, geom_depth: np.ndarray,
                      colours: tp.Dict[int, tp.Tuple[int, int, int]]) -> np.ndarray:
    """
    Description:
        The collision geoms laid over the visual cell in image space (owner, 2026-10-25): the visual frame dimmed
        (CONST_OVERLAY_DIM), each pixel of a coloured collision geom that lies on or before the visual surface (its depth at
        most CONST_OVERLAY_DEPTH behind the visual one: coincident surfaces count, hidden ones do not) blended in its colour at
        CONST_OVERLAY_ALPHA, the outlines between geoms darker - no transparency sorting, so no z-fighting.

    Args:
        (1) image [Matrix<uint8> HxWx3]: The visual frame.
        (2) depth [Matrix<float> HxW]: Its depth (m).
        (3) geoms [Matrix<int> HxW]: The collision-only frame's geom per pixel (-1 none).
        (4) geom_depth [Matrix<float> HxW]: The collision-only frame's depth (m).
        (5) colours [Dict<int, Tuple<int>>]: Per collision geom its colour (RGB); a geom without one is left out.

    Returns:
        (1) parameter [Matrix<uint8> HxWx3]: The overlaid frame.
    """

    lut = np.zeros((max(colours) + 2, 3), float)
    known = np.zeros(max(colours) + 2, bool)
    for g, c in colours.items():
        lut[g], known[g] = c, True
    ids = np.where(geoms >= 0, geoms, len(lut) - 1)
    ids = np.minimum(ids, len(lut) - 1)
    visible = known[ids] & (geom_depth <= depth + CONST_OVERLAY_DEPTH)
    out = image.astype(float) * CONST_OVERLAY_DIM
    out[visible] = out[visible] * (1.0 - CONST_OVERLAY_ALPHA) + lut[ids][visible] * CONST_OVERLAY_ALPHA
    edge = np.zeros_like(visible)
    edge[1:, :] |= ids[1:, :] != ids[:-1, :]
    edge[:, 1:] |= ids[:, 1:] != ids[:, :-1]
    edge &= visible
    out[edge] = lut[ids][edge] * CONST_OVERLAY_EDGE
    return np.clip(out, 0, 255).astype(np.uint8)


def Write_Png(image: np.ndarray, path: str, max_bytes: tp.Optional[int] = None) -> tp.Tuple[int, int]:
    """
    Description:
        A PNG of an image (optimized); with a size cap, the largest width of CONST_PNG_WIDTHS (not above the image's) whose
        file fits (the aspect kept).

    Args:
        (1) image [Matrix<uint8> HxWx3]: The image.
        (2) path [string]: The file.
        (3) max_bytes [int]: The largest file allowed (bytes), or None.

    Returns:
        (1) parameter [int]: The file's size (bytes).
        (2) parameter [int]: Its width (px).
    """

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    source = Image.fromarray(image)
    widths = [source.size[0]] + [w for w in CONST_PNG_WIDTHS if w < source.size[0]] if max_bytes else [source.size[0]]
    for width in widths:
        picture = source if width == source.size[0] else source.resize((width, int(round(source.size[1] * width / source.size[0]))),
                                                                      Image.Resampling.LANCZOS)
        buffer = io.BytesIO()
        picture.save(buffer, format='PNG', optimize=True)
        if max_bytes is None or buffer.tell() <= max_bytes:
            with open(path, 'wb') as handle:
                handle.write(buffer.getvalue())
            return buffer.tell(), width
    raise Media_Error_Cls('The PNG %s does not fit %d bytes even %d px wide (%d bytes); allow more or crop it.' % (path, max_bytes, widths[-1],
                                                                                                             buffer.tell()))


def Label_Frame(image: np.ndarray, lines: tp.Sequence[str], height: float = CONST_LABEL_HEIGHT) -> np.ndarray:
    """
    Description:
        A frame with text lines on a translucent dark band in its top left corner (Pillow's scalable default font).

    Args:
        (1) image [Matrix<uint8> HxWx3]: The frame.
        (2) lines [List<string>]: The lines.
        (3) height [float]: The text height as a fraction of the frame's height.

    Returns:
        (1) parameter [Matrix<uint8> HxWx3]: The labelled frame (a copy).
    """

    size = max(10, int(round(image.shape[0] * height)))
    font = ImageFont.load_default(size=size)
    pad = int(round(size * CONST_LABEL_PAD))
    base = Image.fromarray(image).convert('RGBA')
    band = Image.new('RGBA', base.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(band)
    step = int(round(size * 1.35))
    lines = [line.replace(a, b) for line in lines for a, b in [CONST_LABEL_REPLACE]]
    width = max(int(draw.textlength(line, font=font)) for line in lines)
    draw.rectangle([0, 0, width + 2 * pad, len(lines) * step + 2 * pad - (step - size)], fill=CONST_LABEL_BAND + (CONST_LABEL_ALPHA,))
    for k, line in enumerate(lines):
        draw.text((pad, pad + k * step), line, font=font, fill=CONST_LABEL_INK + (255,))
    return np.asarray(Image.alpha_composite(base, band).convert('RGB'))


def Frame_Samples(t_end: float, dt: float, fps: float, speed: float) -> np.ndarray:
    """
    Description:
        The sample shown in each frame of a video of a run sampled every dt: the frame k shows the time k * speed / fps,
        rounded to the nearest sample, up to the run's end.

    Args:
        (1) t_end [float]: The run's last sample time (s).
        (2) dt [float]: The sample time (s).
        (3) fps [float]: The frame rate (1/s).
        (4) speed [float]: The playback speed (1 = real time).

    Returns:
        (1) parameter [Vector<int> K]: The sample per frame.
    """

    times = np.arange(0.0, t_end + 1e-9, speed / fps)
    return np.minimum(np.round(times / dt).astype(int), int(round(t_end / dt)))


class Video_Writer_Cls(object):
    """
    Description:
        An H.264 MP4 written frame by frame through imageio-ffmpeg's ffmpeg (CONST_CODEC, CONST_CRF, CONST_PRESET, yuv420p,
        +faststart).

    Initialization of the Class:
        Args:
            (1) path [string]: The file.
            (2) width, height [int]: The frame size (px; multiples of CONST_MACRO_BLOCK).
            (3) fps [float]: The frame rate (1/s).

        Example:
            Initialization:
                Cls = Video_Writer_Cls('a.mp4', 1920, 1080, 60.0)

            Features:
                # Functions of the class.
                Cls.Write(frame); Cls.Close()
    """

    def __init__(self, path: str, width: int, height: int, fps: float) -> None:
        if width % CONST_MACRO_BLOCK or height % CONST_MACRO_BLOCK:
            raise Media_Error_Cls('A video frame of %dx%d px is not a multiple of %d px; choose another size.' % (width, height, CONST_MACRO_BLOCK))
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.__writer = imageio_ffmpeg.write_frames(path, (width, height), fps=fps, codec=CONST_CODEC, quality=None, pix_fmt_in='rgb24',
                                                    pix_fmt_out='yuv420p', macro_block_size=CONST_MACRO_BLOCK, ffmpeg_log_level='error',
                                                    output_params=['-crf', str(CONST_CRF), '-preset', CONST_PRESET, '-movflags', '+faststart'])
        self.__writer.send(None)

    def Write(self, frame: np.ndarray) -> None:
        """
        Description:
            Append a frame.

        Args:
            (1) frame [Matrix<uint8> HxWx3]: The RGB frame.
        """

        self.__writer.send(np.ascontiguousarray(frame))

    def Close(self) -> None:
        """
        Description:
            Finish the file (ffmpeg flushes and exits).
        """

        self.__writer.close()


def Write_Gif(frames: tp.Sequence[np.ndarray], path: str, fps: float, width: int, max_bytes: int) -> tp.Tuple[int, int, int]:
    """
    Description:
        A looping GIF of frames scaled to a width, with one palette for all frames (taken from a strip of every frame, no
        dithering: no flicker), kept under a size: each of CONST_GIF_COLOURS in turn, then the same with every other frame
        dropped (the frame time doubled, so the playback speed stays). A GIF stores frame times in 10 ms: each frame's time
        is its end rounded to 10 ms less the previous one's, so the rate is exact on average (15 fps: 70, 70, 60 ms ...).

    Args:
        (1) frames [List<Matrix<uint8> HxWx3>]: The frames.
        (2) path [string]: The file.
        (3) fps [float]: The frame rate (1/s).
        (4) width [int]: The width (px); the height keeps the aspect.
        (5) max_bytes [int]: The largest file allowed (bytes).

    Returns:
        (1) parameter [int]: The file's size (bytes).
        (2) parameter [int]: The colours used.
        (3) parameter [int]: The frames kept.
    """

    height = int(round(frames[0].shape[0] * width / frames[0].shape[1]))
    scaled = [Image.fromarray(f).resize((width, height), Image.Resampling.LANCZOS) for f in frames]
    strip = Image.new('RGB', (width, height * min(len(scaled), 16)))
    for k, frame in enumerate(scaled[::max(1, len(scaled) // 16)][:16]):
        strip.paste(frame, (0, k * height))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    for step in (1, 2):
        for colours in CONST_GIF_COLOURS:
            palette = strip.quantize(colors=colours, method=Image.Quantize.MEDIANCUT)
            kept = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in scaled[::step]]
            ends = np.round(np.arange(1, len(kept) + 1) * 100.0 * step / fps) * 10
            kept[0].save(path, save_all=True, append_images=kept[1:], duration=[int(d) for d in np.diff(np.r_[0, ends])], loop=0,
                         optimize=False, disposal=1)
            size = os.path.getsize(path)
            if size <= max_bytes:
                return size, colours, len(kept)
    os.remove(path)
    raise Media_Error_Cls('The GIF %s does not fit %d bytes even with %d colours and every other frame (%d bytes); shorten it or reduce '
                          'its width.' % (path, max_bytes, CONST_GIF_COLOURS[-1], size))


def Contact_Sheet(images: tp.Sequence[np.ndarray], labels: tp.Sequence[str], columns: int) -> np.ndarray:
    """
    Description:
        The images in a grid, row by row, each with its label on it (Label_Frame), CONST_SHEET_GAP apart.

    Args:
        (1) images [List<Matrix<uint8> HxWx3>]: The images, all of one size.
        (2) labels [List<string>]: One label per image.
        (3) columns [int]: The grid's columns.

    Returns:
        (1) parameter [Matrix<uint8> HxWx3]: The sheet.
    """

    h, w = images[0].shape[:2]
    rows = (len(images) + columns - 1) // columns
    gap = CONST_SHEET_GAP
    sheet = np.empty((rows * h + (rows + 1) * gap, columns * w + (columns + 1) * gap, 3), np.uint8)
    sheet[:] = CONST_SHEET_BACKGROUND
    for k, (image, label) in enumerate(zip(images, labels)):
        r, c = divmod(k, columns)
        y, x = gap + r * (h + gap), gap + c * (w + gap)
        sheet[y:y + h, x:x + w] = Label_Frame(image, [label], CONST_SHEET_LABEL)
    return sheet
