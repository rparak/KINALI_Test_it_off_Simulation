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
# File Name: ../Planner/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# SciPy (Scientific computing) [pip3 install scipy]
from scipy.interpolate import CubicSpline

"""
Description:
    Initialization of constants.

    Joint-space motion planning for the optimized program (FEATURES.md F9), independent of the cell: a configuration's
    validity is a function the caller gives. A path through waypoints as a cubic spline over its chord length; its
    time-optimal timing within joint speed and acceleration limits (TOPP by reachability, Pham & Pham 2018: per grid
    point a 2-D linear program, solved in closed form by its vertices - no LP solver); RRT-Connect with a seeded
    generator, the shortcut and the spline's refit until it is valid. Deterministic: the same inputs and seed give the
    same bytes. It prints nothing.
"""
# A joint whose path derivative is smaller than this does not bound the path speed (its constraint is on the
# acceleration term only).
#   Unit [float] (rad per unit of path)
CONST_TINY = 1e-12
# The bound of the TOPP's (x = sdot^2, u = sddot) polygon where a constraint leaves it open (a joint at rest on the path).
#   Unit [float]
CONST_OPEN = 1e9
# The sampled timing's tolerance on the limits, and the stretch beyond the exceeded ratio (TOPP meets the limits at its grid
# points; between them the path's curvature may exceed them by a fraction of a percent).
#   Unit [float] (-)
CONST_LIMIT_TOLERANCE, CONST_STRETCH_MARGIN = 1e-9, 1.0001


class Planner_Error_Cls(ValueError):
    """
    Description:
        A path cannot be planned (RRT-Connect without a connection, a spline that stays invalid after its refits). The
        message names what failed and the parameter to change.
    """


class Spline_Path_Cls(object):
    """
    Description:
        A joint path through waypoints: a cubic spline (scipy CubicSpline, clamped: zero path derivative at both ends)
        over the cumulative chord length s in [0, L] (the joint-space distance along the polyline).

    Initialization of the Class:
        Args:
            (1) waypoints [Matrix<float> nx6]: The waypoints (rad), first and last the ends; at least 2, no two equal in a row.

        Example:
            Initialization:
                # Assignment of the variables.
                path = Spline_Path_Cls(np.array([q_a, q_mid, q_b]))

            Features:
                # Properties of the class.
                path.Length, path.Knots

                # Functions of the class.
                path.Q(s), path.D1(s), path.D2(s)
    """

    def __init__(self, waypoints: np.ndarray) -> None:
        points = np.asarray(waypoints, dtype=float)
        chords = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if len(points) < 2 or np.any(chords <= 0.0):
            raise Planner_Error_Cls('a spline needs at least 2 waypoints, no two equal in a row')
        self.Waypoints = points
        self.Knots = np.r_[0.0, np.cumsum(chords)]
        self.Length = float(self.Knots[-1])
        self._Spline = CubicSpline(self.Knots, points, bc_type='clamped', axis=0)

    def Q(self, s: np.ndarray) -> np.ndarray:
        """
        Description:
            The joints at path parameters s.

        Args:
            (1) s [Vector<float>]: The path parameters, 0 .. Length.

        Returns:
            (1) parameter [Matrix<float> nx6]: The joints (rad).
        """

        return self._Spline(s)

    def D1(self, s: np.ndarray) -> np.ndarray:
        """
        Description:
            dq/ds at s.

        Args:
            (1) s [Vector<float>]: The path parameters.

        Returns:
            (1) parameter [Matrix<float> nx6]: The first derivatives.
        """

        return self._Spline(s, 1)

    def D2(self, s: np.ndarray) -> np.ndarray:
        """
        Description:
            d2q/ds2 at s.

        Args:
            (1) s [Vector<float>]: The path parameters.

        Returns:
            (1) parameter [Matrix<float> nx6]: The second derivatives.
        """

        return self._Spline(s, 2)


def _Bounds(d1: np.ndarray, d2: np.ndarray, d1_next: np.ndarray, d2_next: np.ndarray, ds: float, v_max: np.ndarray,
            a_max: np.ndarray) -> tp.Tuple[np.ndarray, np.ndarray, float]:
    """
    Description:
        One grid interval's constraints on u = sddot as linear functions of x = sdot^2 at its start (TOPP-RA's first-order
        interpolation): |q'_j u + q''_j x| <= a_j at the start point, and at the end point with x + 2 ds u substituted
        (coefficients q'_j + 2 ds q''_j on u, q''_j on x), so u is bounded where q' vanishes at one end. Each gives a lower
        and an upper bound on u, (c_0 + c_1 x); a joint at rest at the start (|q'_j| <= CONST_TINY) and the joint speeds
        (x q'_j^2 <= v_j^2) bound x alone.

    Args:
        (1 - 2) d1, d2 [Vector<float> 1x6]: q' and q'' at the interval's start.
        (3 - 4) d1_next, d2_next [Vector<float> 1x6]: q' and q'' at its end.
        (5) ds [float]: The grid step.
        (6 - 7) v_max, a_max [Vector<float> 1x6]: The limits (rad/s, rad/s^2).

    Returns:
        (1) parameter [Matrix<float> kx2]: The lower bounds on u, rows (c_0, c_1).
        (2) parameter [Matrix<float> kx2]: The upper bounds on u, rows (c_0, c_1).
        (3) parameter [float]: The largest x the speeds and the joints at rest allow.
    """

    lower, upper, x_max = [], [], CONST_OPEN
    for j in range(len(d1)):
        if abs(d1[j]) > CONST_TINY:
            x_max = min(x_max, v_max[j] ** 2 / d1[j] ** 2)
        elif abs(d2[j]) > CONST_TINY:
            x_max = min(x_max, a_max[j] / abs(d2[j]))
        for on_u, on_x in ((d1[j], d2[j]), (d1_next[j] + 2.0 * ds * d2_next[j], d2_next[j])):
            if abs(on_u) > CONST_TINY:
                low, high = [-a_max[j] / on_u, -on_x / on_u], [a_max[j] / on_u, -on_x / on_u]
                if on_u < 0:
                    low, high = high, low
                lower.append(low)
                upper.append(high)
    return np.array(lower).reshape(-1, 2), np.array(upper).reshape(-1, 2), x_max


def _Reach(lower: np.ndarray, upper: np.ndarray, x_max: float, ds: float, high: float) -> float:
    """
    Description:
        The largest x at a grid point from which some admissible u reaches the next point's interval [0, high]
        (0 <= x + 2 ds u <= high): every pair of a lower and an upper bound on u, both linear in x, must keep
        lower <= upper - a half-line in x each; their intersection with [0, x_max] is an interval.

    Args:
        (1 - 2) lower, upper [Matrix<float> kx2]: The bounds on u (_Bounds).
        (3) x_max [float]: The bound on x alone.
        (4) ds [float]: The grid step.
        (5) high [float]: The next point's largest controllable x.

    Returns:
        (1) parameter [float]: The largest x (0 when only rest is controllable).
    """

    lows = np.vstack([lower, [[0.0, -1.0 / (2.0 * ds)]]])
    highs = np.vstack([upper, [[high / (2.0 * ds), -1.0 / (2.0 * ds)]]])
    gap_0 = highs[None, :, 0] - lows[:, None, 0]
    gap_1 = highs[None, :, 1] - lows[:, None, 1]
    x_low, x_high = 0.0, x_max
    falling = gap_1 < -CONST_TINY
    if np.any(falling):
        x_high = min(x_high, float(np.min(gap_0[falling] / -gap_1[falling])))
    rising = gap_1 > CONST_TINY
    if np.any(rising):
        x_low = max(x_low, float(np.max(-gap_0[rising] / gap_1[rising])))
    flat = ~falling & ~rising
    if np.any(gap_0[flat] < -1e-12) or x_high < x_low:
        return 0.0
    return max(0.0, x_high)


def Topp(d1: np.ndarray, d2: np.ndarray, ds: float, v_max: np.ndarray, a_max: np.ndarray) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        Time-optimal path parametrization by reachability (TOPP-RA, Pham & Pham 2018) on a uniform grid s_0 .. s_N with
        x = sdot^2, u = sddot constant on each interval (x_{i+1} = x_i + 2 ds u_i), at rest at both ends: the backward pass
        finds each point's largest controllable x (_Reach), the forward pass takes the largest admissible u that stays
        controllable.

    Args:
        (1 - 2) d1, d2 [Matrix<float> (N+1)x6]: q' and q'' at the grid points.
        (3) ds [float]: The grid step.
        (4 - 5) v_max, a_max [Vector<float> 1x6]: The joint limits (rad/s, rad/s^2).

    Returns:
        (1) parameter [Vector<float> N+1]: x at the grid points (x_0 = x_N = 0).
        (2) parameter [Vector<float> N]: u on each interval.
    """

    n = len(d1) - 1
    bounds = [_Bounds(d1[i], d2[i], d1[i + 1], d2[i + 1], ds, v_max, a_max) for i in range(n)]
    controllable = np.zeros(n + 1)
    for i in range(n - 1, -1, -1):
        controllable[i] = _Reach(*bounds[i], ds, controllable[i + 1])
    x, u = np.zeros(n + 1), np.zeros(n)
    for i in range(n):
        lower, upper, _ = bounds[i]
        low = max([-x[i] / (2.0 * ds)] + [c_0 + c_1 * x[i] for c_0, c_1 in lower])
        high = min([(controllable[i + 1] - x[i]) / (2.0 * ds)] + [c_0 + c_1 * x[i] for c_0, c_1 in upper])
        u[i] = high if high >= low else low
        x[i + 1] = min(max(0.0, x[i] + 2.0 * ds * u[i]), controllable[i + 1])
        u[i] = (x[i + 1] - x[i]) / (2.0 * ds)
    return x, u


def Grid_Times(x: np.ndarray, u: np.ndarray, ds: float) -> np.ndarray:
    """
    Description:
        The times of the grid points: on each interval sddot = u is constant, so its duration is
        (sqrt(x_{i+1}) - sqrt(x_i)) / u, or ds / sqrt(x_i) when u = 0.

    Args:
        (1) x [Vector<float> N+1]: sdot^2 at the grid points.
        (2) u [Vector<float> N]: sddot on the intervals.
        (3) ds [float]: The grid step.

    Returns:
        (1) parameter [Vector<float> N+1]: The times (s), from 0.
    """

    root = np.sqrt(np.maximum(x, 0.0))
    durations = np.empty(len(u))
    for i in range(len(u)):
        if abs(u[i]) > CONST_TINY:
            durations[i] = (root[i + 1] - root[i]) / u[i]
        else:
            durations[i] = ds / root[i] if root[i] > 0 else 0.0
    return np.r_[0.0, np.cumsum(durations)]


@dataclass
class Timed_Path_Str:
    """
    Description:
        A path timed on the 4 ms grid: the joints and their exact speeds and accelerations per sample (the start
        included), the stretch beyond the time-optimal duration, and the peaks as fractions of the limits.
    """

    # The joints, speeds, accelerations, sample 0 the start.
    #   Unit [Matrix<float> (n+1)x6] (rad, rad/s, rad/s^2)
    Q: np.ndarray
    QD: np.ndarray
    QDD: np.ndarray
    # The path parameter and its rate per sample.
    #   Unit [Vector<float> n+1], [Vector<float> n+1]
    S: np.ndarray
    SD: np.ndarray
    # The time-optimal duration and the duration on the grid.
    #   Unit [float] (s)
    Optimal_Time: float
    Duration: float
    # The peak joint speed and acceleration, fractions of the limits.
    #   Unit [float] (-)
    Peak_Speed: float
    Peak_Acceleration: float


def Time_Path(path: Spline_Path_Cls, v_max: np.ndarray, a_max: np.ndarray, grid_points: int, dt: float) -> Timed_Path_Str:
    """
    Description:
        The time-optimal timing of a path (Topp on grid_points intervals) sampled every dt from rest to rest: the duration
        rounded up to whole samples (a uniform time stretch k >= 1, which divides speeds by k and accelerations by k^2),
        the samples exact (sddot constant per grid interval: s quadratic in time); stretched further while a sampled joint
        speed or acceleration exceeds its limit between the grid points. The last sample is the path's end exactly.

    Args:
        (1) path [Spline_Path_Cls(object)]: The path.
        (2 - 3) v_max, a_max [Vector<float> 1x6]: The joint limits (rad/s, rad/s^2).
        (4) grid_points [int]: The TOPP grid's intervals.
        (5) dt [float]: The sample time (s).

    Returns:
        (1) parameter [Timed_Path_Str(object)]: The timed samples.
    """

    v_max, a_max = np.asarray(v_max, dtype=float), np.asarray(a_max, dtype=float)
    grid = np.linspace(0.0, path.Length, grid_points + 1)
    ds = path.Length / grid_points
    x, u = Topp(path.D1(grid), path.D2(grid), ds, v_max, a_max)
    times = Grid_Times(x, u, ds)
    optimal = float(times[-1])
    stretch = 1.0
    for _ in range(20):
        n = max(1, int(math.ceil(optimal * stretch / dt - 1e-9)))
        k = n * dt / optimal
        tau = np.arange(n + 1) * dt / k
        j = np.clip(np.searchsorted(times, tau, side='right') - 1, 0, grid_points - 1)
        h = tau - times[j]
        sd0 = np.sqrt(np.maximum(x[j], 0.0))
        s = np.clip(grid[j] + sd0 * h + 0.5 * u[j] * h * h, 0.0, path.Length)
        sd, sdd = np.maximum(sd0 + u[j] * h, 0.0) / k, u[j] / k ** 2
        s[-1], sd[0], sd[-1] = path.Length, 0.0, 0.0
        d1 = path.D1(s)
        q, qd, qdd = path.Q(s), d1 * sd[:, None], path.D2(s) * (sd ** 2)[:, None] + d1 * sdd[:, None]
        qd[0], qd[-1] = 0.0, 0.0
        speed, acceleration = float(np.max(np.abs(qd) / v_max)), float(np.max(np.abs(qdd) / a_max))
        if speed <= 1.0 + CONST_LIMIT_TOLERANCE and acceleration <= 1.0 + CONST_LIMIT_TOLERANCE:
            q[-1] = path.Waypoints[-1]
            q[0] = path.Waypoints[0]
            return Timed_Path_Str(Q=q, QD=qd, QDD=qdd, S=s, SD=sd, Optimal_Time=optimal, Duration=n * dt, Peak_Speed=speed,
                                  Peak_Acceleration=acceleration)
        stretch = k * max(speed, math.sqrt(acceleration)) * CONST_STRETCH_MARGIN
    raise Planner_Error_Cls('the sampled timing still exceeds the limits after 20 stretches (speed %.4f, acceleration %.4f of the limits)'
                            % (speed, acceleration))


def Edge_Valid(q_0: np.ndarray, q_1: np.ndarray, valid: tp.Callable[[np.ndarray], bool], step: float) -> bool:
    """
    Description:
        A straight joint-space edge checked at configurations no farther apart than step in any joint (both ends
        included).

    Args:
        (1 - 2) q_0, q_1 [Vector<float> 1x6]: The ends (rad).
        (3) valid [function]: A configuration's validity.
        (4) step [float]: The largest joint change between checks (rad).

    Returns:
        (1) parameter [bool]: True when every checked configuration is valid.
    """

    n = max(1, int(math.ceil(float(np.max(np.abs(q_1 - q_0))) / step)))
    return all(valid(q_0 + (q_1 - q_0) * (i / n)) for i in range(n + 1))


@dataclass
class Rrt_Result_Str:
    """
    Description:
        RRT-Connect's path and its effort.
    """

    # The path's configurations, start to goal.
    #   Unit [List<Vector<float> 1x6>] (rad)
    Path: tp.List[np.ndarray]
    # The iterations and the nodes of both trees.
    #   Unit [int]
    Iterations: int
    Nodes: int


def Rrt_Connect(start: np.ndarray, goal: np.ndarray, valid: tp.Callable[[np.ndarray], bool], lower: np.ndarray, upper: np.ndarray,
                rng: np.random.Generator, max_iterations: int, extend_step: float, edge_step: float, goal_bias: float) -> Rrt_Result_Str:
    """
    Description:
        RRT-Connect (Kuffner & LaValle 2000) in joint space: two trees from the start and the goal take turns - one extends
        one step (extend_step, joint-space distance) towards a random configuration within the limits (or, with
        probability goal_bias, towards the other tree's root), the other connects towards the new node step by step until
        blocked or joined. Edges are checked at edge_step. The nearest node is the first of the smallest distance.

    Args:
        (1 - 2) start, goal [Vector<float> 1x6]: The ends (rad), both valid.
        (3) valid [function]: A configuration's validity.
        (4 - 5) lower, upper [Vector<float> 1x6]: The joint limits (rad).
        (6) rng [numpy.random.Generator]: The seeded generator.
        (7) max_iterations [int]: The iteration budget.
        (8 - 9) extend_step, edge_step [float]: The tree step and the edge check step (rad).
        (10) goal_bias [float]: The probability of steering at the other tree's root.

    Returns:
        (1) parameter [Rrt_Result_Str(object)]: The path and the effort.
    """

    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if Edge_Valid(start, goal, valid, edge_step):
        return Rrt_Result_Str(Path=[start, goal], Iterations=0, Nodes=2)
    trees = [([start], [-1]), ([goal], [-1])]

    def Extend(tree: tp.Tuple[tp.List[np.ndarray], tp.List[int]], target: np.ndarray) -> tp.Tuple[str, int]:
        """
        Description:
            One step of a tree towards a target: 'reached', 'advanced' or 'trapped', and the new node's index.

        Args:
            (1) tree [Tuple]: The nodes and their parents.
            (2) target [Vector<float> 1x6]: The target (rad).

        Returns:
            (1) parameter [string]: The outcome.
            (2) parameter [int]: The new node (or -1).
        """

        nodes, parents = tree
        distances = np.linalg.norm(np.asarray(nodes) - target, axis=1)
        near = int(np.argmin(distances))
        if distances[near] <= extend_step:
            new, outcome = target.copy(), 'reached'
        else:
            new, outcome = nodes[near] + (target - nodes[near]) * (extend_step / distances[near]), 'advanced'
        if not Edge_Valid(nodes[near], new, valid, edge_step):
            return 'trapped', -1
        nodes.append(new)
        parents.append(near)
        return outcome, len(nodes) - 1

    def Branch(tree: tp.Tuple[tp.List[np.ndarray], tp.List[int]], index: int) -> tp.List[np.ndarray]:
        """
        Description:
            The nodes from a tree's root to a node.

        Args:
            (1) tree [Tuple]: The nodes and their parents.
            (2) index [int]: The node.

        Returns:
            (1) parameter [List<Vector<float> 1x6>]: Root to node.
        """

        nodes, parents, out = tree[0], tree[1], []
        while index >= 0:
            out.append(nodes[index])
            index = parents[index]
        return out[::-1]

    for iteration in range(1, max_iterations + 1):
        a, b = trees[(iteration - 1) % 2], trees[iteration % 2]
        target = b[0][0] if rng.random() < goal_bias else lower + (upper - lower) * rng.random(len(lower))
        outcome, new = Extend(a, target)
        if outcome == 'trapped':
            continue
        while True:
            outcome, joined = Extend(b, a[0][new])
            if outcome != 'advanced':
                break
        if outcome == 'reached':
            path = Branch(a, new) + Branch(b, joined)[::-1][1:]
            if a is not trees[0]:
                path = path[::-1]
            return Rrt_Result_Str(Path=path, Iterations=iteration, Nodes=len(trees[0][0]) + len(trees[1][0]))
    raise Planner_Error_Cls('RRT-Connect found no path in %d iterations (%d nodes); raise Rrt.Max_Iterations in Configuration/Optimization.yaml'
                            % (max_iterations, len(trees[0][0]) + len(trees[1][0])))


def Shortcut(path: tp.List[np.ndarray], valid: tp.Callable[[np.ndarray], bool], rng: np.random.Generator, iterations: int,
             edge_step: float) -> tp.List[np.ndarray]:
    """
    Description:
        Random shortcuts of a polyline: two points on it (uniform in its length, from the seeded generator) joined by a
        straight edge when the edge is valid, the polyline between them replaced.

    Args:
        (1) path [List<Vector<float> 1x6>]: The polyline (rad).
        (2) valid [function]: A configuration's validity.
        (3) rng [numpy.random.Generator]: The seeded generator.
        (4) iterations [int]: The tries.
        (5) edge_step [float]: The edge check step (rad).

    Returns:
        (1) parameter [List<Vector<float> 1x6>]: The shorter polyline, same ends.
    """

    points = [np.asarray(p, dtype=float) for p in path]
    for _ in range(iterations):
        if len(points) < 3:
            break
        lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1))]
        t_1, t_2 = sorted(rng.random(2) * lengths[-1])
        i, j = int(np.searchsorted(lengths, t_1, side='right') - 1), int(np.searchsorted(lengths, t_2, side='right') - 1)
        if i >= j:
            continue

        def At(t: float, k: int) -> np.ndarray:
            """
            Description:
                The polyline's point at length t on its edge k.

            Args:
                (1) t [float]: The length along the polyline.
                (2) k [int]: The edge.

            Returns:
                (1) parameter [Vector<float> 1x6]: The point (rad).
            """

            k = min(k, len(points) - 2)
            span = lengths[k + 1] - lengths[k]
            return points[k] + (points[k + 1] - points[k]) * ((t - lengths[k]) / span if span > 0 else 0.0)

        p_1, p_2 = At(t_1, i), At(t_2, j)
        if Edge_Valid(p_1, p_2, valid, edge_step):
            points = points[:i + 1] + [p_1, p_2] + points[j + 1:]
            points = [points[0]] + [p for k, p in enumerate(points[1:], 1) if np.linalg.norm(p - points[k - 1]) > 1e-9]
    return points


def Prune(path: tp.List[np.ndarray], valid: tp.Callable[[np.ndarray], bool], edge_step: float) -> tp.List[np.ndarray]:
    """
    Description:
        The polyline with every interior vertex removed whose neighbours join with a valid straight edge, in order from the
        start (deterministic): the shortcut's near-coincident vertices would make the spline through them oscillate.

    Args:
        (1) path [List<Vector<float> 1x6>]: The polyline (rad).
        (2) valid [function]: A configuration's validity.
        (3) edge_step [float]: The edge check step (rad).

    Returns:
        (1) parameter [List<Vector<float> 1x6>]: The polyline, same ends, no more vertices.
    """

    points, i = [np.asarray(p, dtype=float) for p in path], 1
    while i < len(points) - 1:
        if Edge_Valid(points[i - 1], points[i + 1], valid, edge_step):
            del points[i]
        else:
            i += 1
    return points


@dataclass
class Smooth_Result_Str:
    """
    Description:
        A valid spline path and the refits it took.
    """

    # The waypoints and the spline through them.
    #   Unit [Matrix<float> nx6] (rad), [Spline_Path_Cls(object)]
    Waypoints: np.ndarray
    Path: Spline_Path_Cls
    # The midpoints inserted.
    #   Unit [int]
    Refits: int


def Smooth(polyline: tp.List[np.ndarray], valid: tp.Callable[[np.ndarray], bool], check_step: float, max_refits: int) -> Smooth_Result_Str:
    """
    Description:
        The spline through a valid polyline, checked at configurations no farther apart than check_step in any joint: where
        a configuration fails, the midpoint of that span's polyline edge (valid, being on a valid edge) is inserted and the
        spline refitted, up to max_refits times.

    Args:
        (1) polyline [List<Vector<float> 1x6>]: The valid polyline (rad).
        (2) valid [function]: A configuration's validity.
        (3) check_step [float]: The largest joint change between checks (rad).
        (4) max_refits [int]: The midpoints allowed.

    Returns:
        (1) parameter [Smooth_Result_Str(object)]: The waypoints, the path, the refits.
    """

    points = np.asarray(polyline, dtype=float)
    for refit in range(max_refits + 1):
        path = Spline_Path_Cls(points)
        dense = np.linspace(0.0, path.Length, 4 * len(points) + 1)
        variation = float(np.max(np.sum(np.abs(np.diff(path.Q(dense), axis=0)), axis=0)))
        n = max(2, int(math.ceil(variation / check_step)) + 1)
        s = np.linspace(0.0, path.Length, n)
        q = path.Q(s)
        while float(np.max(np.abs(np.diff(q, axis=0)))) > check_step:
            n = 2 * n
            s = np.linspace(0.0, path.Length, n)
            q = path.Q(s)
        failed = next((i for i in range(n) if not valid(q[i])), None)
        if failed is None:
            return Smooth_Result_Str(Waypoints=points, Path=path, Refits=refit)
        if refit == max_refits:
            break
        k = int(np.clip(np.searchsorted(path.Knots, s[failed], side='right') - 1, 0, len(points) - 2))
        points = np.insert(points, k + 1, 0.5 * (points[k] + points[k + 1]), axis=0)
    raise Planner_Error_Cls('the spline is still invalid after %d refits; raise Spline.Max_Refits or the planner margin' % max_refits)
