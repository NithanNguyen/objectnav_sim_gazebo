#!/usr/bin/env python3
"""Tải model Google Scanned Objects (GSO) từ Gazebo Fuel và tạo model "bọc" chuẩn cho Gazebo Classic 11.

Model bọc gso_<alias>:
  * static, gốc = TÂM ĐÁY vật (pose z = độ cao bề mặt là vật nằm đúng trên mặt),
  * visual = mesh OBJ gốc của GSO (kèm .mtl + texture),
  * collision = HỘP bao (AABB) thay cho mesh va chạm,
  * tự kiểm tra: OBJ có mtllib không, map_Kd trong .mtl có trỏ đúng file texture không (sửa nếu sai),
  * in kích thước để bạn đối chiếu với vật thật (cốc ~0.1 m).
Chỉ dùng thư viện chuẩn của Python 3.8 (Ubuntu 20.04), không cần numpy.

Dùng:
  python3 fetch_gso.py <thu_muc_models> <alias>=<TenModelGSO> [<alias>=<TenModelGSO> ...]
Ví dụ:
  python3 fetch_gso.py ~/sim_ws/src/home_sim_world/models mug=Room_Essentials_Mug_White_Yellow
Nếu máy không tải được (mạng chặn), tải zip bằng trình duyệt từ trang model trên app.gazebosim.org rồi:
  python3 fetch_gso.py <thu_muc_models> mug=Room_Essentials_Mug_White_Yellow --zip ~/Downloads/xxx.zip
"""
import io
import os
import re
import shutil
import sys
import tempfile
import urllib.parse
import urllib.request
import zipfile

OWNER = 'GoogleResearch'
BASE = 'https://fuel.gazebosim.org/1.0'


def candidate_urls(name):
    q = urllib.parse.quote(name)
    # Mẫu 1 theo tài liệu fuel_tools (".../worlds/Empty.zip"); mẫu 2 là đường dẫn có số phiên bản.
    return ['%s/%s/models/%s.zip' % (BASE, OWNER, q),
            '%s/%s/models/%s/1/%s.zip' % (BASE, OWNER, q, q)]


def download(name):
    errs = []
    for url in candidate_urls(name):
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'home_sim_world/0.1'})
            data = urllib.request.urlopen(req, timeout=120).read()
            zipfile.ZipFile(io.BytesIO(data))          # kiểm tra đúng là file zip
            print('  tải xong %s (%.1f MB)' % (url, len(data) / 1e6))
            return data
        except Exception as e:                          # noqa: BLE001
            errs.append('%s -> %s' % (url, e))
    raise SystemExit('Không tải được %s:\n  %s' % (name, '\n  '.join(errs)))


def obj_bounds(path):
    lo = [float('inf')] * 3
    hi = [float('-inf')] * 3
    n = 0
    with open(path, errors='ignore') as f:
        for line in f:
            if line.startswith('v '):
                v = [float(t) for t in line.split()[1:4]]
                for i in range(3):
                    lo[i] = min(lo[i], v[i])
                    hi[i] = max(hi[i], v[i])
                n += 1
    if n == 0:
        raise SystemExit('OBJ không có đỉnh: %s' % path)
    return lo, hi, n


def fix_material(obj_path, all_files):
    """Bảo đảm OBJ -> mtllib -> map_Kd -> texture đều tồn tại; trả về danh sách cảnh báo."""
    notes = []
    obj_dir = os.path.dirname(obj_path)
    txt = open(obj_path, errors='ignore').read()
    m = re.search(r'^mtllib\s+(.+)$', txt, re.M)
    if not m:
        notes.append('OBJ không khai báo mtllib -> Gazebo sẽ hiển thị màu xám (không có texture).')
        return notes
    mtl = os.path.normpath(os.path.join(obj_dir, m.group(1).strip()))
    if not os.path.exists(mtl):
        found = [f for f in all_files if f.lower().endswith('.mtl')]
        if not found:
            notes.append('Không tìm thấy file .mtl -> không có texture.')
            return notes
        shutil.copy(found[0], mtl)
        notes.append('Đã chép %s vào cạnh OBJ.' % os.path.basename(found[0]))
    lines = open(mtl, errors='ignore').read().splitlines()
    pngs = [f for f in all_files if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
    changed = False
    for i, line in enumerate(lines):
        mm = re.match(r'^(\s*map_Kd\s+)(.+)$', line)
        if not mm:
            continue
        mdir = os.path.dirname(mtl)
        tex = os.path.normpath(os.path.join(mdir, mm.group(2).strip()))
        if not os.path.exists(tex):
            if not pngs:
                notes.append('map_Kd trỏ tới %s nhưng không có ảnh texture nào.' % mm.group(2).strip())
                continue
            tex = pngs[0]
        # Chép texture vào cạnh .mtl và trỏ bằng tên file trần: tránh phụ thuộc cách loader
        # của Gazebo xử lý đường dẫn tương đối kiểu "../materials/...".
        local = os.path.join(mdir, os.path.basename(tex))
        if os.path.abspath(tex) != os.path.abspath(local):
            shutil.copy(tex, local)
            notes.append('Đã chép texture %s vào cạnh .mtl' % os.path.basename(tex))
        if mm.group(2).strip() != os.path.basename(tex):
            lines[i] = mm.group(1) + os.path.basename(tex)
            changed = True
            notes.append('Sửa map_Kd -> %s' % os.path.basename(tex))
    if changed:
        open(mtl, 'w').write('\n'.join(lines) + '\n')
    if not any(re.match(r'^\s*map_Kd', l) for l in lines):
        notes.append('File .mtl không có map_Kd -> không có texture.')
    return notes


SDF = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <!-- mesh shifted so the model origin is the bottom centre -->
      <visual name="visual">
        <pose>{ox:.5f} {oy:.5f} {oz:.5f} 0 0 0</pose>
        <geometry><mesh><uri>model://{name}/{obj}</uri></mesh></geometry>
      </visual>
      <collision name="collision">
        <pose>0 0 {cz:.5f} 0 0 0</pose>
        <geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
'''


def build(models_dir, alias, gso_name, zip_path=None):
    name = 'gso_' + alias
    print('[%s] <- %s/%s' % (name, OWNER, gso_name))
    data = open(zip_path, 'rb').read() if zip_path else download(gso_name)
    raw = tempfile.mkdtemp(prefix='gso_')
    zipfile.ZipFile(io.BytesIO(data)).extractall(raw)
    files = [os.path.join(dp, f) for dp, _, fs in os.walk(raw) for f in fs]
    objs = [f for f in files if f.lower().endswith('.obj')]
    if not objs:
        raise SystemExit('  Không có file .obj trong zip. Các file: %s' % [os.path.relpath(f, raw) for f in files])
    od = os.path.dirname(objs[0])
    src_root = os.path.dirname(od) if os.path.basename(od) == 'meshes' else od   # thư mục gốc của model
    dst = os.path.join(models_dir, name)
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src_root, dst, ignore=shutil.ignore_patterns('model.sdf', 'model.config', 'thumbnails'))
    obj = os.path.join(dst, os.path.relpath(objs[0], src_root))
    notes = fix_material(obj, [os.path.join(dst, os.path.relpath(f, src_root)) for f in files
                               if os.path.exists(os.path.join(dst, os.path.relpath(f, src_root)))])
    lo, hi, n = obj_bounds(obj)
    size = [hi[i] - lo[i] for i in range(3)]
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    open(os.path.join(dst, 'model.sdf'), 'w').write(SDF.format(
        name=name, obj=os.path.relpath(obj, dst),
        ox=-cx, oy=-cy, oz=-lo[2], cz=size[2] / 2, sx=size[0], sy=size[1], sz=size[2]))
    open(os.path.join(dst, 'model.config'), 'w').write(
        '<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.6">model.sdf</sdf>\n  <author><name>Google Research (GSO, CC BY 4.0)</name></author>\n'
        '  <description>%s/%s, wrapped for Gazebo Classic</description>\n</model>\n' % (name, OWNER, gso_name))
    print('  %d đỉnh, kích thước %.3f x %.3f x %.3f m' % (n, *size))
    if max(size) > 2.0 or max(size) < 0.01:
        print('  CẢNH BÁO: kích thước bất thường -> kiểm tra đơn vị của mesh.')
    for s in notes:
        print('  ' + s)
    print('  -> %s' % dst)


if __name__ == '__main__':
    args = sys.argv[1:]
    zp = None
    if '--zip' in args:
        i = args.index('--zip')
        zp = os.path.expanduser(args[i + 1])
        del args[i:i + 2]
    if len(args) < 2:
        raise SystemExit(__doc__)
    mdir = os.path.expanduser(args[0])
    pairs = [a.split('=', 1) for a in args[1:]]
    if zp and len(pairs) != 1:
        raise SystemExit('--zip chỉ dùng với đúng một model.')
    for alias, gso in pairs:
        build(mdir, alias, gso, zp)
