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
# File Name: ../test/Planner/test_core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Planner/Core
from Planner.Core import (Topp, Grid_Times, Spline_Path_Cls, Time_Path, Rrt_Connect, Shortcut, Prune, Smooth, Edge_Valid, Planner_Error_Cls)

"""
Description:
    The planner (FEATURES.md F9): TOPP against the closed form (B3), the timed spline within the limits at every sample, the
    seeded planner's determinism, the spline's refit. Expected values are the closed-form ones, written here.
"""
# The limits of the tests: the AUBO i5's at 60 % of the datasheet speeds and the assumed accelerations (Cell.yaml).
#   Unit [Vector<float> 1x6] (rad/s), (rad/s^2)
CONST_V = 0.6 * np.radians([220.0, 220.0, 220.0, 220.0, 230.0, 230.0])
CONST_A = np.array([3.0, 3.0, 3.0, 6.0, 6.0, 6.0])
# An obstacle in joint space (rad): a box the configurations must keep out of.
#   Unit [Vector<float> 1x6] (rad)
CONST_LOW, CONST_HIGH = np.array([-0.4, -0.4, -3.0, -3.0, -3.0, -3.0]), np.array([0.4, 0.4, 3.0, 3.0, 3.0, 3.0])


def _Analytic(dq, v, a):
    """
    Description:
        The time-optimal time of a straight joint move from rest to rest under box limits: the path speed and acceleration
        limits are the tightest joint's, a trapezoid when the cruise speed is reached, else a triangle.
    """

    d = np.abs(dq)
    moving = d > 0
    v_s, a_s = float(np.min(v[moving] / d[moving])), float(np.min(a[moving] / d[moving]))
    return (v_s / a_s + 1.0 / v_s, 'trapezoid') if v_s ** 2 / a_s < 1.0 else (2.0 * math.sqrt(1.0 / a_s), 'triangle')


@pytest.mark.parametrize('dq', [[2.5, 1.0, 0.5, 0.3, 0.2, 0.1], [0.2, 0.1, 0.05, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 3.0, 1.0, 0.5], [6.0, 0, 0, 0,
                                                                                                                                  0, 0]])
def test_topp_matches_the_closed_form_of_a_straight_move(dq):
    """
    Description:
        B3: the TOPP time of a straight joint move (q' constant, q'' = 0, 1000 grid intervals) equals the closed-form
        trapezoid or triangle within a relative 1e-3 (measured 2026-10-24: at most 1.9e-7), at rest at both ends, and no
        grid point exceeds a limit.
    """

    dq, n = np.array(dq, dtype=float), 1000
    x, u = Topp(np.tile(dq, (n + 1, 1)), np.zeros((n + 1, 6)), 1.0 / n, CONST_V, CONST_A)
    expected, _ = _Analytic(dq, CONST_V, CONST_A)
    assert abs(Grid_Times(x, u, 1.0 / n)[-1] - expected) / expected <= 1e-3
    assert x[0] == 0.0 and x[-1] == 0.0
    assert np.all(np.sqrt(x)[:, None] * np.abs(dq) <= CONST_V * (1 + 1e-9))
    assert np.all(np.abs(u)[:, None] * np.abs(dq) <= CONST_A * (1 + 1e-9))


def test_both_closed_form_cases_are_covered():
    """
    Description:
        The cases above include a trapezoid and a triangle (else the B3 test would not test both).
    """

    kinds = {_Analytic(np.array(dq, dtype=float), CONST_V, CONST_A)[1] for dq in ([2.5, 1.0, 0.5, 0.3, 0.2, 0.1], [0.2, 0.1, 0.05, 0.0, 0.0, 0.0])}
    assert kinds == {'trapezoid', 'triangle'}


def test_a_timed_spline_keeps_the_limits_at_every_sample():
    """
    Description:
        A curved spline through four waypoints timed on the 4 ms grid: every sampled joint speed and acceleration (exact,
        and by finite differences of the samples) within the limits, at rest at both ends, the ends exactly the waypoints,
        the duration whole samples and at most 0.5 % above the time-optimal one.
    """

    points = np.radians([[0, 0, 0, 0, 0, 0], [40, -20, 30, 10, 0, 5], [60, 10, 70, -20, 30, 0], [90, 30, 90, 0, 60, -30]])
    timed = Time_Path(Spline_Path_Cls(points), CONST_V, CONST_A, 1000, 0.004)
    assert np.all(np.abs(timed.QD) <= CONST_V * (1 + 1e-9)) and np.all(np.abs(timed.QDD) <= CONST_A * (1 + 1e-9))
    q = np.vstack([timed.Q[0], timed.Q, timed.Q[-1]])
    qd = np.diff(q, axis=0) / 0.004
    assert np.all(np.abs(qd) <= CONST_V * 1.001) and np.all(np.abs(np.diff(qd, axis=0) / 0.004) <= CONST_A * 1.001)
    assert np.array_equal(timed.Q[0], points[0]) and np.array_equal(timed.Q[-1], points[-1])
    assert np.all(timed.QD[[0, -1]] == 0.0)
    assert abs(timed.Duration / 0.004 - round(timed.Duration / 0.004)) < 1e-9 and timed.Optimal_Time <= timed.Duration <= 1.005 * timed.Optimal_Time


def _Valid(q):
    """
    Description:
        Outside the joint-space box (the test obstacle).
    """

    return not bool(np.all((q > CONST_LOW) & (q < CONST_HIGH)))


def _Plan(seed):
    """
    Description:
        A plan around the obstacle: RRT-Connect, the shortcut, the prune, the spline.
    """

    rng = np.random.default_rng(seed)
    start, goal = np.array([-1.0, 0, 0, 0, 0, 0]), np.array([1.0, 0, 0, 0, 0, 0])
    lower, upper = -np.pi * np.ones(6), np.pi * np.ones(6)
    path = Rrt_Connect(start, goal, _Valid, lower, upper, rng, 5000, 0.3, 0.02, 0.1).Path
    short = Prune(Shortcut(path, _Valid, rng, 100, 0.02), _Valid, 0.02)
    return path, short, Smooth(short, _Valid, 0.01, 50)


def test_the_planner_goes_around_an_obstacle_and_is_deterministic_with_its_seed():
    """
    Description:
        RRT-Connect, the shortcut, the prune and the spline find a valid path around a box in joint space whose straight
        edge is blocked; the same seed gives the same arrays (B5), another seed another path.
    """

    assert not Edge_Valid(np.array([-1.0, 0, 0, 0, 0, 0]), np.array([1.0, 0, 0, 0, 0, 0]), _Valid, 0.02)
    path, short, smooth = _Plan(7)
    # Valid at the check step (0.01 rad), so between the checks within a hair of the box: dense samples clear a box 2 mrad
    # smaller (the planning margin's job in the cell).
    s = np.linspace(0, smooth.Path.Length, 4001)
    assert all(not np.all((q > CONST_LOW + 0.002) & (q < CONST_HIGH - 0.002)) for q in smooth.Path.Q(s))
    again = _Plan(7)
    assert np.asarray(path).tobytes() == np.asarray(again[0]).tobytes() and smooth.Waypoints.tobytes() == again[2].Waypoints.tobytes()
    assert np.asarray(_Plan(8)[0]).tobytes() != np.asarray(path).tobytes()
    assert len(short) <= len(path)


def _Corridor(q):
    """
    Description:
        Within 0.05 rad of an L-shaped polyline in joints 1-2 (the others at 0): a spline through its corner leaves it.
    """

    a = q[:2]
    near = min(math.hypot(a[0] - min(max(a[0], 0.0), 1.0), a[1]), math.hypot(a[0] - 1.0, a[1] - min(max(a[1], 0.0), 1.0)))
    return near <= 0.05 and bool(np.all(np.abs(q[2:]) < 1e-12))


def test_the_spline_is_refitted_where_it_leaves_the_valid_set():
    """
    Description:
        A valid L-shaped polyline in a 0.05 rad corridor: the spline through its corner leaves the corridor, so polyline
        midpoints are inserted until it is valid (2 refits, measured 2026-10-24); with no refit allowed it fails, naming
        the setting.
    """

    corner = [np.zeros(6), np.r_[1.0, 0, 0, 0, 0, 0], np.r_[1.0, 1.0, 0, 0, 0, 0]]
    smooth = Smooth(corner, _Corridor, 0.01, 50)
    assert smooth.Refits == 2 and len(smooth.Waypoints) == len(corner) + 2
    assert all(_Corridor(q) for q in smooth.Path.Q(np.linspace(0, smooth.Path.Length, 401)))
    with pytest.raises(Planner_Error_Cls, match='Max_Refits'):
        Smooth(corner, _Corridor, 0.01, 0)


def test_prune_keeps_the_vertices_that_are_needed():
    """
    Description:
        Prune removes a vertex whose neighbours see each other and keeps the one that turns around the obstacle.
    """

    path = [np.array([-1.0, 0.5, 0, 0, 0, 0]), np.array([-0.5, 0.5, 0, 0, 0, 0]), np.array([0.45, 0.45, 0, 0, 0, 0]), np.array([0.5, -1.0, 0, 0,
                                                                                                                                0, 0])]
    pruned = Prune(path, _Valid, 0.01)
    assert len(pruned) == 3 and np.array_equal(pruned[1], path[2])


def test_an_unconnected_plan_names_the_parameter():
    """
    Description:
        A wall across the whole joint space (joint 1 between 0.5 and 0.9 rad) between the start and the goal: RRT-Connect
        fails after its budget, naming Rrt.Max_Iterations.
    """

    with pytest.raises(Planner_Error_Cls, match='Rrt.Max_Iterations'):
        Rrt_Connect(np.array([-1.0, 0, 0, 0, 0, 0]), np.array([1.0, 0, 0, 0, 0, 0]), lambda q: not 0.5 < q[0] < 0.9, -np.pi * np.ones(6),
                    np.pi * np.ones(6), np.random.default_rng(0), 50, 0.3, 0.02, 0.1)
