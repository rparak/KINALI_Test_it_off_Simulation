"""MG06 cell: workstation + MG06 + 2-axis loader (SCHUNK EGI 040) + AUBO i5 on the robot pedestal with the
OnRobot VGC10 + fork (the tool that opens / closes the MG06).  Builds the MuJoCo model with MjSpec.

Frames: everything is in the MG06 fixture frame (origin = centre of the MG06 footprint on the table, Z up,
front of MG06 = -Y).  The robot base stands on the robot plate (deska_robot) = the CAD origin of the cell.
"""
import os, json, numpy as np, mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(os.path.join(HERE, 'pcb_meta.json')))

ROBOT_BASE = np.array([0.42072, -0.33497, 0.16200])   # top of deska_robot (= CAD origin of the cell .blend)
ROBOT_YAW = np.radians(-135.0)                       # all tasks lie within +-60 deg of the pan-joint zero
ROBOT_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
KP = [5000, 5000, 4000, 2000, 2000, 1500]
KV = [150, 150, 100, 25, 25, 15]
FMAX = [180, 180, 150, 60, 60, 60]                    # Nm (estimate; AUBO does not publish joint torques)
ARMATURE = [0.2, 0.2, 0.15, 0.04, 0.04, 0.04]         # kg m^2, reflected motor + gear inertia (estimate, ratio ~100)
# AUBO i5 datasheet: joint speed 220 deg/s (J1-J4), 230 deg/s (J5-J6), payload 5 kg, reach 886.5 mm
VMAX_DATASHEET = np.radians([220, 220, 220, 220, 230, 230])

# OnRobot VGC10 with 4 silicone cups D15 mm at 40 kPa (datasheet: 20-60 % vacuum recommended):
# theoretical holding force = 4 * pi * 0.0075^2 * 40e3 = 28 N
CUP_R, CUP_N, VACUUM_PA = 0.0075, 4, 40e3
VACUUM_FORCE = CUP_N * np.pi * CUP_R**2 * VACUUM_PA
CUP_XY = [(0.035, 0.018), (-0.035, 0.018), (0.035, -0.018), (-0.035, -0.018)]
LOADER_TAU = 0.03                                     # s, first-order lag of the loader axis drives

# blue crates (Box_Type_A): outer and inner dimensions from the CAD mesh
# crate_pick = left crate (Box_Type_A_ID_2, seen from the front of the cell), crate_place = right crate (Box_Type_A_ID_1)
CRATES = {'crate_pick': dict(outer=((0.0587, -0.9001), (0.2557, -0.6031)), inner=((0.0737, -0.8771), (0.2407, -0.6261))),
          'crate_place': dict(outer=((0.3077, -0.9001), (0.5047, -0.6031)), inner=((0.3227, -0.8771), (0.4897, -0.6261)))}
CRATE_FLOOR, CRATE_TOP = 0.004, 0.149

# PCB body frame = plate axes, origin = board centre; board half thickness, top of the component area
PCB_HALF_T = 0.0009
SUCTION_IN_PCB = np.array([-0.00125, 0.0046, 0.0075])   # cups on the top of the component area (box)
R_TOOL_IN_PCB = np.diag([1.0, -1.0, -1.0])               # tool +Z = -PCB normal, tool +X = PCB +X


def crate_center(name):
    (x0, y0), (x1, y1) = CRATES[name]['inner']; return np.array([(x0 + x1) / 2, (y0 + y1) / 2])


def pcb_in_crate(name):
    c = crate_center(name); return np.array([c[0], c[1], CRATE_FLOOR + PCB_HALF_T]), np.array([1.0, 0, 0, 0])


def build_spec():
    sc = mujoco.MjSpec.from_file(os.path.join(HERE, 'scene.xml'))
    for k in list(sc.keys): sc.delete(k)
    # crates: replace the filled convex hulls by floor + 4 walls (the robot reaches into them)
    for g in list(sc.geoms):
        if g.name.startswith('ws_col_Box_Type_A'): sc.delete(g)
    ws = sc.body('workstation')
    for n, c in CRATES.items():
        (ox0, oy0), (ox1, oy1) = c['outer']; (ix0, iy0), (ix1, iy1) = c['inner']; zt = CRATE_TOP
        boxes = [((ox0, oy0, 0.002), (ox1, oy1, CRATE_FLOOR)), ((ox0, oy0, 0), (ix0, oy1, zt)), ((ix1, oy0, 0), (ox1, oy1, zt)),
                 ((ix0, oy0, 0), (ix1, iy0, zt)), ((ix0, iy1, 0), (ix1, oy1, zt))]
        for i, (lo, hi) in enumerate(boxes):
            lo, hi = np.array(lo), np.array(hi)
            ws.add_geom(name=f'{n}_{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=(lo + hi) / 2, size=(hi - lo) / 2,
                        contype=2, conaffinity=1, group=3, rgba=[.3, .5, .9, .3])   # loader, robot and PCB
    # robot + tool
    rb = mujoco.MjSpec.from_file(os.path.join(HERE, 'robot', 'aubo_i5', 'aubo_i5.xml'))
    tl = mujoco.MjSpec.from_file(os.path.join(HERE, 'robot_tool', 'vgc10_opener_free.xml'))
    tool = tl.body('ee_tool')
    for j in list(tool.joints): tl.delete(j)                           # rigidly mounted on the flange
    tool.pos = [0, 0, 0]; tool.gravcomp = 1
    # suction cups: own body (rigid), so the adhesion actuator acts only on the cups, not on the fork
    for g in tool.geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_BOX and abs(g.size[0] - 0.0525) < 1e-4:   # cup block: 2 mm shorter, cups stick out
            g.size = [0.0525, 0.0293, 0.038]; g.pos = [0, 0, 0.2205]
    for i, (x, y) in enumerate(CUP_XY):      # each cup its own (rigid) body + adhesion actuator: a cup holds only if it touches
        cb = tool.add_body(name=f'vgc10_cup{i}')
        # bellows of the cup: 3 mm spring travel along the tool axis (7 N compresses it ~2 mm)
        cb.add_joint(name=f'cup{i}_bellows', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1], range=[-0.003, 0.0005],
                     stiffness=3000, springref=0, damping=20, armature=0.005)
        cb.add_geom(name=f'cup{i}_mass', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.004, 0, 0], pos=[x, y, 0.25], mass=0.02,
                    contype=0, conaffinity=0, group=4)
        # collision as a square pad of the same area (flat face contact: clean normals for the adhesion force)
        a = np.sqrt(np.pi) * CUP_R / 2
        cb.add_geom(name=f'cup{i}', type=mujoco.mjtGeom.mjGEOM_BOX, size=[a, a, 0.00155], pos=[x, y, 0.2606 - 0.00155],
                    rgba=[0.15, 0.15, 0.15, 1], friction=[2.0, 0.02, 0.001], condim=4, group=1,
                    solref=[0.002, 1], solimp=[0.99, 0.999, 0.0005, 0.5, 2], solmix=100,   # silicone cup
                    contype=1, conaffinity=1)
    rb.site('attachment_site').attach_body(tool, 'tool/', '')
    fr = sc.worldbody.add_frame(pos=ROBOT_BASE, quat=[np.cos(ROBOT_YAW / 2), 0, 0, np.sin(ROBOT_YAW / 2)])
    fr.attach_body(rb.body('base_link'), 'aubo/', '')
    for b in sc.bodies:
        if b.name.startswith('aubo/'):
            b.gravcomp = 1
            for g in b.geoms:                 # robot + tool touch MG06, loader, PCB, workstation (bit 2), crates
                if g.contype or g.conaffinity: g.contype, g.conaffinity = 1, 3
    # parts mounted together never collide (MuJoCo excludes only direct parent/child pairs)
    for b1, b2 in [('tool/ee_tool', 'wrist_3_link'), ('tool/ee_tool', 'wrist_2_link'), ('wrist_3_link', 'wrist_1_link')]:
        sc.add_exclude(bodyname1='aubo/' + b1, bodyname2='aubo/' + b2)
    for jn, kp, kv, fm, arm in zip(ROBOT_JOINTS, KP, KV, FMAX, ARMATURE):
        j = sc.joint('aubo/' + jn); j.armature = arm; j.damping = np.array([2.0, 0, 0])
        a = sc.add_actuator(name='robot_' + jn, target='aubo/' + jn, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_position(kp=kp, kv=kv)
        a.forcerange = [-fm, fm]; a.forcelimited = True; a.ctrlrange = [-3.04, 3.04]; a.ctrllimited = True
    # vacuum: adhesion actuator on the cups - pulls only what the cups actually touch (ctrl 0..1 = off..on)
    for i in range(CUP_N):
        va = sc.add_actuator(name=f'vacuum{i}', target=f'aubo/tool/vgc10_cup{i}', trntype=mujoco.mjtTrn.mjTRN_BODY)
        va.set_to_adhesion(gain=VACUUM_FORCE / CUP_N); va.ctrlrange = [0, 1]; va.ctrllimited = True
    # loader axis drives: first-order lag between command and servo set-point
    for n in ('y_servo', 'z_servo'):
        a = sc.actuator(n); a.dyntype = mujoco.mjtDyn.mjDYN_FILTEREXACT; a.dynprm[0] = LOADER_TAU
    style(sc)
    return sc


def style(sc):
    """look: neutral floor, three lights with shadows, metal / plastic materials, cameras on the PCB and the tool"""
    sc.visual.quality.shadowsize = 8192
    sc.visual.headlight.ambient = [0.25, 0.25, 0.25]; sc.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    t = sc.texture('grid'); t.builtin = mujoco.mjtBuiltin.mjBUILTIN_FLAT; t.mark = mujoco.mjtMark.mjMARK_RANDOM
    t.random = 0.25; t.rgb1 = [0.62, 0.62, 0.60]; t.rgb2 = [0.62, 0.62, 0.60]; t.markrgb = [0.55, 0.55, 0.53]; t.width = t.height = 1024
    mf = sc.material('grid'); mf.texrepeat = [6, 6]; mf.reflectance = 0.0; mf.specular = 0.1
    for l in list(sc.worldbody.lights): sc.delete(l)
    for pos, d, dif, sh in [([1.2, -2.0, 2.6], [-0.4, 0.7, -1.0], 0.55, True), ([-1.5, -1.0, 2.0], [0.6, 0.4, -1.0], 0.30, True),
                            ([0.0, 1.5, 2.2], [0.0, -0.6, -1.0], 0.20, False)]:
        sc.worldbody.add_light(pos=pos, dir=d / np.linalg.norm(d), diffuse=[dif] * 3, specular=[0.25] * 3, castshadow=sh,
                               type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    mats = {'alu': dict(specular=0.75, shininess=0.65), 'plastic': dict(specular=0.25, shininess=0.3),
            'dark': dict(specular=0.15, shininess=0.2)}
    for n, kw in mats.items(): sc.add_material(name='look_' + n, rgba=[1, 1, 1, 1], **kw)
    for g in sc.geoms:
        if g.group != 2 or g.material or g.name.startswith('pcb'): continue
        r, gg, b = g.rgba[:3]; mx, mn = max(r, gg, b), min(r, gg, b)
        g.material = 'look_dark' if mx < 0.25 else ('look_alu' if mx - mn < 0.18 else 'look_plastic')
    sc.worldbody.add_camera(name='cam_pcb', pos=[-0.55, -0.85, 0.75], mode=mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY,
                            targetbody='pcb', fovy=40)
    sc.worldbody.add_camera(name='cam_tool', pos=[-0.30, 0.35, 0.80], mode=mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY,
                            targetbody='aubo/tool/ee_tool', fovy=45)


def build_model():
    sc = build_spec()
    return sc.compile()


def export_xml(path=os.path.join(HERE, 'cell_scene.xml')):
    """write the assembled cell as one MJCF (for the MuJoCo viewer); mesh paths are made relative to this folder"""
    import xml.etree.ElementTree as ET
    sc = build_spec(); sc.compile(); r = ET.fromstring(sc.to_xml())
    for c in r.iter('compiler'): c.set('meshdir', '.')
    for me in r.iter('mesh'):
        n, f = me.get('name', ''), me.get('file', '')
        if n.startswith('aubo/tool/'): me.set('file', 'robot_tool/assets/' + f)
        elif n.startswith('aubo/'): me.set('file', 'robot/aubo_i5/meshes/' + f)
    ET.ElementTree(r).write(path)


if __name__ == '__main__':
    export_xml()
    m = build_model()
    print('nq', m.nq, 'nu', m.nu, 'ngeom', m.ngeom, 'neq', m.neq)
    print([m.body(i).name for i in range(m.nbody) if m.body(i).name.startswith('aubo/')])
    print([m.actuator(i).name for i in range(m.nu)])
