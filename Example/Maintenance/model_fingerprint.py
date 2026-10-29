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
# File Name: ../Example/Maintenance/model_fingerprint.py
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
# JSON (JSON encoder and decoder)
import json
# Hashlib (Secure hashes)
import hashlib
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the logs
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Script_Log_Cls
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model

"""
Description:
    Initialization of constants.

    The fingerprint of the compiled cell (SPEC §10.2): sizes, solver options and per-name hashes of every
    physical and visual property, so that a restructure proves nothing in the model changed. Floats are
    rounded to 1e-9 before hashing (-0.0 folded into 0.0). Mesh file paths are not part of the compiled
    model; the mesh data (vertices, faces) is. This script is the one reader and writer of the format.
"""
# The baseline the cell is compared with by default.
#   Unit [string]
CONST_BASELINE = os.path.join(CONST_REPO_ROOT, 'test', 'Scene', 'data', 'baseline_model_fingerprint.json')
# Hash digits kept, and the rounding of floats before hashing.
#   Unit [int]
CONST_HASH_DIGITS, CONST_DECIMALS = 16, 9
# The model sizes recorded.
#   Unit [List<string>]
CONST_SIZES = ['nq', 'nv', 'nu', 'na', 'nbody', 'njnt', 'ngeom', 'nsite', 'nmesh', 'nmat', 'ntex', 'neq', 'nexclude', 'npair', 'ncam',
               'nlight', 'nkey']


def Hash_Value(x: tp.Any) -> str:
    """
    Description:
        The hash of a value: floats rounded to 1e-9 (and -0.0 made 0.0), integers and strings as they are.

    Args:
        (1) x [Any]: A scalar or an array.

    Returns:
        (1) parameter [string]: The first 16 hex digits of its SHA-1.
    """

    a = np.asarray(x)
    if a.dtype.kind == 'f':
        a = np.round(a.astype(float), CONST_DECIMALS) + 0.0
    return hashlib.sha1(repr(a.tolist()).encode()).hexdigest()[:CONST_HASH_DIGITS]


def Element_Names(m: mujoco.MjModel, objtype: int, n: int) -> tp.List[str]:
    """
    Description:
        A unique key per element: its name, or '<body>#<k>' for an unnamed geom or site (the k-th unnamed
        one of that body), '#<k>' for other unnamed elements.

    Args:
        (1) m [mujoco.MjModel]: The model.
        (2) objtype [mujoco.mjtObj]: The element type.
        (3) n [int]: The number of elements.

    Returns:
        (1) parameter [List<string>]: The keys, by element id.
    """

    out, seen = [], {}
    for i in range(n):
        name = mujoco.mj_id2name(m, objtype, i)
        if not name:
            if objtype == mujoco.mjtObj.mjOBJ_GEOM:
                base = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[i]) or 'world'
            elif objtype == mujoco.mjtObj.mjOBJ_SITE:
                base = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.site_bodyid[i]) or 'world'
            else:
                base = ''
            k = seen.get(base, 0)
            seen[base] = k + 1
            name = f'{base}#{k}'
        out.append(name)
    return out


def Field_Table(m: mujoco.MjModel, objtype: int, n: int, fields: tp.List[str]) -> tp.Dict[str, tp.Dict[str, str]]:
    """
    Description:
        {element key: {field: hash}} for model array fields.

    Args:
        (1) m [mujoco.MjModel]: The model.
        (2) objtype [mujoco.mjtObj]: The element type.
        (3) n [int]: The number of elements.
        (4) fields [List<string>]: The MjModel array names (e.g. 'geom_rgba').

    Returns:
        (1) parameter [Dict<string, Dict<string, string>>]: The table.
    """

    names = Element_Names(m, objtype, n)
    return {name: {f: Hash_Value(getattr(m, f)[i]) for f in fields} for i, name in enumerate(names)}


def Element_Reference(m: mujoco.MjModel, objtype: int, i: int) -> tp.Optional[str]:
    """
    Description:
        The name of a referenced element, '#<id>' if unnamed, None for no reference.

    Args:
        (1) m [mujoco.MjModel]: The model.
        (2) objtype [mujoco.mjtObj]: The element type.
        (3) i [int]: The element id (negative: none).

    Returns:
        (1) parameter [string]: The reference.
    """

    return None if i < 0 else (mujoco.mj_id2name(m, objtype, i) or f'#{i}')


def Model_Fingerprint(m: mujoco.MjModel) -> tp.Dict[str, tp.Any]:
    """
    Description:
        The fingerprint of a compiled model (the format of test/Scene/data/baseline_model_fingerprint.json).

    Args:
        (1) m [mujoco.MjModel]: The model.

    Returns:
        (1) parameter [Dict<string, Any>]: The fingerprint.
    """

    o = m.opt
    opt = {k: (np.asarray(getattr(o, k)).tolist()) for k in dir(o) if not k.startswith('_')
           and isinstance(getattr(o, k), (int, float, np.ndarray)) and not callable(getattr(o, k))}
    fp = {'sizes': {k: int(getattr(m, k)) for k in CONST_SIZES}, 'timestep': float(o.timestep), 'opt': opt}
    fp['bodies'] = Field_Table(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody,
                               ['body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_parentid',
                                'body_gravcomp'])
    fp['geoms'] = Field_Table(m, mujoco.mjtObj.mjOBJ_GEOM, m.ngeom,
                              ['geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_contype', 'geom_conaffinity', 'geom_condim',
                               'geom_solref', 'geom_solimp', 'geom_solmix', 'geom_friction', 'geom_margin', 'geom_gap', 'geom_rgba',
                               'geom_matid', 'geom_group', 'geom_bodyid', 'geom_dataid', 'geom_priority'])
    fp['meshes'] = {}
    for i, name in enumerate(Element_Names(m, mujoco.mjtObj.mjOBJ_MESH, m.nmesh)):
        va, vn = m.mesh_vertadr[i], m.mesh_vertnum[i]
        fa, fn = m.mesh_faceadr[i], m.mesh_facenum[i]
        fp['meshes'][name] = {'nvert': int(vn), 'nface': int(fn), 'vert': Hash_Value(m.mesh_vert[va:va + vn]),
                              'face': Hash_Value(m.mesh_face[fa:fa + fn]), 'pos': Hash_Value(m.mesh_pos[i]), 'quat': Hash_Value(m.mesh_quat[i])}
    fp['materials'] = Field_Table(m, mujoco.mjtObj.mjOBJ_MATERIAL, m.nmat,
                                  ['mat_rgba', 'mat_specular', 'mat_shininess', 'mat_reflectance', 'mat_emission', 'mat_texrepeat'])
    fp['actuators'] = Field_Table(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu,
                                  ['actuator_trntype', 'actuator_trnid', 'actuator_dyntype', 'actuator_gaintype', 'actuator_biastype',
                                   'actuator_dynprm', 'actuator_gainprm', 'actuator_biasprm', 'actuator_ctrlrange', 'actuator_ctrllimited',
                                   'actuator_forcerange', 'actuator_forcelimited', 'actuator_gear', 'actuator_actadr'])
    fp['joints'] = Field_Table(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt,
                               ['jnt_type', 'jnt_range', 'jnt_limited', 'jnt_stiffness', 'jnt_pos', 'jnt_axis', 'jnt_qposadr',
                                'jnt_bodyid', 'jnt_solref', 'jnt_solimp'])
    # The dof properties of each joint's first dof.
    for i, name in enumerate(Element_Names(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt)):
        d = m.jnt_dofadr[i]
        fp['joints'][name].update(damping=Hash_Value(m.dof_damping[d]), armature=Hash_Value(m.dof_armature[d]),
                                  frictionloss=Hash_Value(m.dof_frictionloss[d]), springref=Hash_Value(m.qpos_spring[m.jnt_qposadr[i]]))
    fp['equality'] = {}
    for i, name in enumerate(Element_Names(m, mujoco.mjtObj.mjOBJ_EQUALITY, m.neq)):
        ot = {mujoco.mjtEq.mjEQ_JOINT: mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtEq.mjEQ_WELD: mujoco.mjtObj.mjOBJ_BODY,
              mujoco.mjtEq.mjEQ_CONNECT: mujoco.mjtObj.mjOBJ_BODY}.get(m.eq_type[i], mujoco.mjtObj.mjOBJ_UNKNOWN)
        fp['equality'][name] = {'type': int(m.eq_type[i]), 'obj1': Element_Reference(m, ot, m.eq_obj1id[i]),
                                'obj2': Element_Reference(m, ot, m.eq_obj2id[i]), 'data': Hash_Value(m.eq_data[i]),
                                'solref': Hash_Value(m.eq_solref[i]), 'solimp': Hash_Value(m.eq_solimp[i]),
                                'active0': bool(m.eq_active0[i])}
    fp['sites'] = Field_Table(m, mujoco.mjtObj.mjOBJ_SITE, m.nsite, ['site_pos', 'site_quat', 'site_bodyid'])
    fp['excludes'] = Hash_Value(m.exclude_signature)
    return fp


def Compare_Fingerprints(current: tp.Any, reference: tp.Any, path: str = '') -> tp.List[str]:
    """
    Description:
        Every leaf that differs between two fingerprints, as 'path: reference -> current'.

    Args:
        (1 - 2) current, reference [Dict<string, Any>]: The fingerprints (or sub-trees).
        (3) path [string]: The path of the sub-tree.

    Returns:
        (1) parameter [List<string>]: The differences, sorted by path.
    """

    out = []
    if isinstance(current, dict) and isinstance(reference, dict):
        for k in sorted(set(current) | set(reference)):
            p = f'{path}/{k}' if path else k
            if k not in reference:
                out.append(f'{p}: only in the current model')
            elif k not in current:
                out.append(f'{p}: only in the reference')
            else:
                out += Compare_Fingerprints(current[k], reference[k], p)
    elif current != reference:
        out.append(f'{path}: {reference} -> {current}')
    return out


def main():
    """
    Description:
        Compare the cell's fingerprint with the baseline (default) or another file, listing every difference
        (exit 1 if there is one), or write it with --out.

            .venv/bin/python Example/Maintenance/model_fingerprint.py [--compare FILE | --out FILE]
    """

    parser = argparse.ArgumentParser(description='Fingerprint of the compiled cell: compare or write.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--compare', default=CONST_BASELINE, help='compare with this file (default: the baseline in test/Scene/data/)')
    group.add_argument('--out', default=None, help='write the fingerprint to this file instead of comparing')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('model_fingerprint')
    # The same types as a file read back.
    fp = json.loads(json.dumps(Model_Fingerprint(Build_Cell_Model(Load_Cell_Configuration()))))
    if args.out is not None:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', encoding='utf-8') as handle:
            handle.write(json.dumps(fp, indent=1, sort_keys=True) + '\n')
        print(log.Line('[INFO] Wrote %s: %d entries.' % (args.out, sum(len(v) for v in fp.values() if isinstance(v, dict)))))
        log.Close()
        return 0
    with open(args.compare, encoding='utf-8') as handle:
        differences = Compare_Fingerprints(fp, json.load(handle))
    print(log.Line('[%s] %d difference(s) against %s' % ('INFO' if not differences else 'WARNING', len(differences), args.compare)))
    for line in differences:
        print(log.Line('  ' + line))
    log.Close()
    return 1 if differences else 0


if __name__ == '__main__':
    sys.exit(main())
