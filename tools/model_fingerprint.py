"""Fingerprint of the compiled cell model: sizes, solver options and per-name hashes of every physical / visual
property, so a restructure can prove that nothing in the model changed.

usage:  python tools/model_fingerprint.py                       # write docs/baseline_model_fingerprint.json
        python tools/model_fingerprint.py --out FILE            # write FILE
        python tools/model_fingerprint.py --compare FILE        # compare the current model with FILE (exit 1 on a difference)

Floats are rounded to 1e-9 before hashing (-0.0 folded into 0.0). Mesh file paths are not part of the compiled model,
so moving the assets does not change the fingerprint; the mesh data (vertices, faces) does.
"""
import sys, json, hashlib, argparse
from pathlib import Path
import numpy as np, mujoco

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / 'docs' / 'baseline_model_fingerprint.json'


def build_model():
    sys.path.insert(0, str(ROOT))
    import cell
    return cell.build_model()


def h(x):
    """hash of a value: floats rounded to 1e-9, ints / strings as they are"""
    a = np.asarray(x)
    if a.dtype.kind == 'f': a = np.round(a.astype(float), 9) + 0.0
    return hashlib.sha1(repr(a.tolist()).encode()).hexdigest()[:16]


def _names(m, objtype, n):
    """unique key per element: its name, or '<body>#<k>' / '#<i>' for unnamed ones"""
    out, seen = [], {}
    for i in range(n):
        nm = mujoco.mj_id2name(m, objtype, i)
        if not nm:
            if objtype == mujoco.mjtObj.mjOBJ_GEOM: base = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[i]) or 'world'
            elif objtype == mujoco.mjtObj.mjOBJ_SITE: base = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.site_bodyid[i]) or 'world'
            else: base = ''
            k = seen.get(base, 0); seen[base] = k + 1; nm = f'{base}#{k}'
        out.append(nm)
    return out


def _table(m, objtype, n, fields):
    """{element name: {field: hash}} for the given model array fields (array prefix + field)"""
    names = _names(m, objtype, n)
    return {nm: {f: h(getattr(m, f)[i]) for f in fields} for i, nm in enumerate(names)}


def _ref(m, objtype, i):
    return None if i < 0 else (mujoco.mj_id2name(m, objtype, i) or f'#{i}')


def fingerprint(m):
    o = m.opt
    opt = {k: (np.asarray(getattr(o, k)).tolist()) for k in dir(o) if not k.startswith('_')
           and isinstance(getattr(o, k), (int, float, np.ndarray)) and not callable(getattr(o, k))}
    fp = {'sizes': {k: int(getattr(m, k)) for k in ['nq', 'nv', 'nu', 'na', 'nbody', 'njnt', 'ngeom', 'nsite', 'nmesh',
                                                     'nmat', 'ntex', 'neq', 'nexclude', 'npair', 'ncam', 'nlight', 'nkey']},
          'timestep': float(o.timestep), 'opt': opt}
    fp['bodies'] = _table(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody,
                          ['body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_parentid',
                           'body_gravcomp'])
    fp['geoms'] = _table(m, mujoco.mjtObj.mjOBJ_GEOM, m.ngeom,
                         ['geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_contype', 'geom_conaffinity', 'geom_condim',
                          'geom_solref', 'geom_solimp', 'geom_solmix', 'geom_friction', 'geom_margin', 'geom_gap', 'geom_rgba',
                          'geom_matid', 'geom_group', 'geom_bodyid', 'geom_dataid', 'geom_priority'])
    fp['meshes'] = {}
    for i, nm in enumerate(_names(m, mujoco.mjtObj.mjOBJ_MESH, m.nmesh)):
        va, vn = m.mesh_vertadr[i], m.mesh_vertnum[i]; fa, fn = m.mesh_faceadr[i], m.mesh_facenum[i]
        fp['meshes'][nm] = {'nvert': int(vn), 'nface': int(fn), 'vert': h(m.mesh_vert[va:va + vn]),
                            'face': h(m.mesh_face[fa:fa + fn]), 'pos': h(m.mesh_pos[i]), 'quat': h(m.mesh_quat[i])}
    fp['materials'] = _table(m, mujoco.mjtObj.mjOBJ_MATERIAL, m.nmat,
                             ['mat_rgba', 'mat_specular', 'mat_shininess', 'mat_reflectance', 'mat_emission', 'mat_texrepeat'])
    fp['actuators'] = _table(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu,
                             ['actuator_trntype', 'actuator_trnid', 'actuator_dyntype', 'actuator_gaintype', 'actuator_biastype',
                              'actuator_dynprm', 'actuator_gainprm', 'actuator_biasprm', 'actuator_ctrlrange', 'actuator_ctrllimited',
                              'actuator_forcerange', 'actuator_forcelimited', 'actuator_gear', 'actuator_actadr'])
    fp['joints'] = _table(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt,
                          ['jnt_type', 'jnt_range', 'jnt_limited', 'jnt_stiffness', 'jnt_pos', 'jnt_axis', 'jnt_qposadr',
                           'jnt_bodyid', 'jnt_solref', 'jnt_solimp'])
    for i, nm in enumerate(_names(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt)):       # dof properties of the joint's first dof
        d = m.jnt_dofadr[i]
        fp['joints'][nm].update(damping=h(m.dof_damping[d]), armature=h(m.dof_armature[d]),
                                frictionloss=h(m.dof_frictionloss[d]), springref=h(m.qpos_spring[m.jnt_qposadr[i]]))
    fp['equality'] = {}
    for i, nm in enumerate(_names(m, mujoco.mjtObj.mjOBJ_EQUALITY, m.neq)):
        ot = {mujoco.mjtEq.mjEQ_JOINT: mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtEq.mjEQ_WELD: mujoco.mjtObj.mjOBJ_BODY,
              mujoco.mjtEq.mjEQ_CONNECT: mujoco.mjtObj.mjOBJ_BODY}.get(m.eq_type[i], mujoco.mjtObj.mjOBJ_UNKNOWN)
        fp['equality'][nm] = {'type': int(m.eq_type[i]), 'obj1': _ref(m, ot, m.eq_obj1id[i]), 'obj2': _ref(m, ot, m.eq_obj2id[i]),
                              'data': h(m.eq_data[i]), 'solref': h(m.eq_solref[i]), 'solimp': h(m.eq_solimp[i]),
                              'active0': bool(m.eq_active0[i])}
    fp['sites'] = _table(m, mujoco.mjtObj.mjOBJ_SITE, m.nsite, ['site_pos', 'site_quat', 'site_bodyid'])
    fp['excludes'] = h(m.exclude_signature)
    return fp


def compare(cur, ref, path=''):
    """list of 'path: ref -> cur' strings for every leaf that differs"""
    out = []
    if isinstance(cur, dict) and isinstance(ref, dict):
        for k in sorted(set(cur) | set(ref)):
            p = f'{path}/{k}' if path else k
            if k not in ref: out.append(f'{p}: only in the current model')
            elif k not in cur: out.append(f'{p}: only in the reference')
            else: out += compare(cur[k], ref[k], p)
    elif cur != ref: out.append(f'{path}: {ref} -> {cur}')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', type=Path, default=None, help=f'write the fingerprint here (default {DEFAULT_OUT.relative_to(ROOT)})')
    ap.add_argument('--compare', type=Path, help='compare with this fingerprint instead of writing one')
    a = ap.parse_args()
    fp = json.loads(json.dumps(fingerprint(build_model())))      # same types as a file read back
    if a.compare:
        diff = compare(fp, json.loads(a.compare.read_text()))
        print(f'{len(diff)} difference(s) against {a.compare}')
        for l in diff: print('  ' + l)
        return 1 if diff else 0
    out = a.out or DEFAULT_OUT
    out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(fp, indent=1, sort_keys=True) + '\n')
    print(f'wrote {out}: {sum(len(v) for v in fp.values() if isinstance(v, dict))} entries')
    return 0


if __name__ == '__main__':
    sys.exit(main())
