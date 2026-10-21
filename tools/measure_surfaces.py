#!/usr/bin/env python3
"""Do do cao be mat dat vat trong AWS Small House (he toa do world, don vi m).
Dung: python3 measure_surfaces.py <thu_muc_aws_repo_hoac_share> [out.yaml]
Can: pip3 install trimesh pycollada numpy pyyaml"""
import sys, re, math, numpy as np, trimesh, xml.etree.ElementTree as ET
ROOT = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else None
WANT = ['Bed_01', 'SofaC_01', 'ChairA_01', 'ChairD_01', 'CoffeeTable_01', 'KitchenTable_01',
        'NightStand_01', 'TVCabinet_01', 'ReadingDesk_01', 'BalconyTable_01', 'ShoeRack_01']
def unit(p):
    m = re.search(r'<unit[^>]*meter="([\d.eE+-]+)"', open(p, errors='ignore').read(5000))
    return float(m.group(1)) if m else 1.0
def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
world = ET.parse(ROOT + '/worlds/small_house.world').getroot().find('world')
meshes, rows = {}, []
for m in world.findall('model'):
    uri = m.find('include/uri')
    if uri is None: continue
    base = uri.text.strip().replace('model://aws_robomaker_residential_', '')
    if base not in WANT: continue
    dae = f"{ROOT}/models/aws_robomaker_residential_{base}/meshes/aws_{base}_visual.DAE"
    if dae not in meshes:
        mesh = trimesh.util.concatenate(trimesh.load(dae, force='scene').dump())
        mesh.apply_scale(unit(dae))          # trimesh khong tu ap dung <unit> cua COLLADA
        meshes[dae] = mesh
    mesh = meshes[dae]
    pose = [float(v) for v in m.find('pose').text.split()]
    R = rot(*pose[3:]); t = np.array(pose[:3])
    n = mesh.face_normals @ R.T
    c = mesh.triangles_center @ R.T + t
    up = (n[:, 2] > 0.95) & (c[:, 2] > 0.15)        # mat ngang huong len, cao hon 15 cm
    if not up.any(): continue
    z = np.round(c[up, 2], 2); a = mesh.area_faces[up]
    zs = {}
    for zz, aa in zip(z, a): zs[zz] = zs.get(zz, 0.0) + aa
    amax = max(zs.values())
    big = {k: v for k, v in zs.items() if v >= 0.4 * amax}   # bo qua mat nho (dinh tua lung, vien)
    ztop = max(big); area = zs[ztop]                        # mat ngang CAO NHAT trong cac mat lon
    others = sorted(float(k) for k, v in zs.items() if v >= 0.05 and abs(k - ztop) > 0.03)
    sel = up.copy(); sel[up] = np.abs(z - ztop) < 0.015
    tri = mesh.triangles[sel].reshape(-1, 3) @ R.T + t
    lo, hi = tri.min(0), tri.max(0)
    rows.append(dict(name=m.get('name'), type=base, z=round(float(ztop), 2),
                     x=[round(float(lo[0]), 2), round(float(hi[0]), 2)],
                     y=[round(float(lo[1]), 2), round(float(hi[1]), 2)], area_m2=round(float(area), 2),
                     other_z=[round(v, 2) for v in others]))
rows.sort(key=lambda r: (r['type'], r['name']))
for r in rows:
    print(f"{r['name']:22s} z={r['z']:.2f} m  x{r['x']}  y{r['y']}  dien tich ~{r['area_m2']} m2  mat khac {r['other_z']}")
if OUT:
    import yaml
    yaml.safe_dump({'frame': 'world', 'note': 'z = mat ngang cao nhat co dien tich >= 40% mat lon nhat (visual mesh); x,y = bao cua mat do; other_z = cac mat ngang khac >= 0.05 m2',
                    'surfaces': rows}, open(OUT, 'w'), allow_unicode=True, sort_keys=False)
    print('->', OUT)
