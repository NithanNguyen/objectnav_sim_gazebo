"""Lõi thuật toán khám phá frontier, không phụ thuộc ROS (chỉ numpy + scipy) để kiểm thử được ngoài ROS.

Thuật toán: chọn frontier gần nhất theo khoảng cách ĐƯỜNG ĐI (geodesic) trên vùng đã biết là trống
(Yamauchi 1997) + kiểm tra lại mục tiêu liên tục — repetitive re-checking (Holz et al. 2010)
+ blacklist theo bán kính. Node ROS 2 dùng lõi này: scripts/frontier_explorer.py.

Quy ước lưới: giống nav_msgs/OccupancyGrid — data[row, col], row tăng theo +y, col tăng theo +x;
-1 = chưa biết, 0..100 = xác suất chiếm chỗ. Tương thích Python 3.8, numpy 1.17, scipy 1.3 (Ubuntu 20.04).
"""
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

UNKNOWN = -1
_CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
_SQUARE = np.ones((3, 3), dtype=bool)


class Grid(object):
    """Ảnh chụp một OccupancyGrid: mảng (H, W) int16 + độ phân giải + gốc (không hỗ trợ gốc bị quay)."""

    def __init__(self, data, resolution, origin_x, origin_y):
        self.data = np.asarray(data, dtype=np.int16)
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape((info.height, info.width))
        return cls(data, info.resolution, info.origin.position.x, info.origin.position.y)

    @property
    def shape(self):
        return self.data.shape

    def to_cell(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def to_world(self, r, c):
        return c * self.res + self.ox + 0.5 * self.res, r * self.res + self.oy + 0.5 * self.res

    def inside(self, r, c):
        return 0 <= r < self.data.shape[0] and 0 <= c < self.data.shape[1]

    def free_mask(self, free_thresh):
        return (self.data >= 0) & (self.data <= free_thresh)

    def occupied_mask(self, occ_thresh):
        return self.data >= occ_thresh


class Frontiers(object):
    """Kết quả phát hiện frontier: mask, nhãn cụm, và danh sách cụm đạt kích thước tối thiểu."""

    def __init__(self, mask, labels, clusters):
        self.mask = mask          # (H, W) bool: ô frontier
        self.labels = labels      # (H, W) int: nhãn cụm (0 = không phải frontier)
        self.clusters = clusters  # list of dict: label, size, cells (N,2), centroid (x, y)


class Target(object):
    def __init__(self, goal_xy, yaw, centroid_xy, cells_xy, cost, size):
        self.goal_xy = goal_xy        # điểm tiếp cận (world) gửi cho Nav2
        self.yaw = yaw                # hướng nhìn về tâm cụm frontier
        self.centroid_xy = centroid_xy
        self.cells_xy = cells_xy      # (N, 2) tọa độ world các ô frontier của cụm — dùng để re-check
        self.cost = cost              # khoảng cách đường đi (m)
        self.size = size              # số ô của cụm


def detect_frontiers(grid, free_thresh=20, min_size=8):
    """Frontier = ô trống có ít nhất một lân cận 4 là ô chưa biết; gom cụm theo lân cận 8."""
    free = grid.free_mask(free_thresh)
    unknown = grid.data == UNKNOWN
    near_unknown = ndimage.binary_dilation(unknown, structure=_CROSS)
    mask = free & near_unknown
    mask[0, :] = mask[-1, :] = mask[:, 0] = mask[:, -1] = False
    labels, n = ndimage.label(mask, structure=_SQUARE)
    clusters = []
    if n:
        sizes = np.bincount(labels.ravel(), minlength=n + 1)
        for lab in np.nonzero(sizes >= min_size)[0]:
            if lab == 0:
                continue
            cells = np.argwhere(labels == lab)
            cr, cc = cells.mean(axis=0)
            clusters.append({'label': int(lab), 'size': int(sizes[lab]), 'cells': cells,
                             'centroid': grid.to_world(cr, cc)})
    return Frontiers(mask, labels, clusters)


def traversable(grid, robot_cell, robot_radius, free_thresh=20, occ_thresh=65,
                escape_radius=0.6, escape_clearance=0.10, return_strict=False):
    """Ô đi được = ô đã biết là trống và cách ô chiếm chỗ đã biết >= robot_radius.

    Vùng thoát quanh robot: trong escape_radius chấp nhận clearance >= escape_clearance, vì tường mới thấy sau
    khi tới nơi có thể làm C-space quanh robot co lại (Nav2 vẫn lập được đường ra khỏi vùng inflation).
    """
    free = grid.free_mask(free_thresh)
    clearance = ndimage.distance_transform_edt(~grid.occupied_mask(occ_thresh)) * grid.res
    strict = free & (clearance >= robot_radius)
    trav = strict.copy()
    r0, c0 = robot_cell
    if grid.inside(r0, c0) and not strict[r0, c0]:   # chỉ cần vùng thoát khi robot đang ở sát vật cản
        k = int(math.ceil(escape_radius / grid.res))
        rs, re = max(r0 - k, 0), min(r0 + k + 1, grid.shape[0])
        cs, ce = max(c0 - k, 0), min(c0 + k + 1, grid.shape[1])
        rr, cc = np.mgrid[rs:re, cs:ce]
        disk = (rr - r0) ** 2 + (cc - c0) ** 2 <= k * k
        sub = free[rs:re, cs:ce] & (clearance[rs:re, cs:ce] >= escape_clearance) & disk
        trav[rs:re, cs:ce] |= sub
        trav[r0, c0] = True
    return (trav, strict, clearance) if return_strict else trav


def geodesic_distance(trav, source_cell, res):
    """Khoảng cách đường đi (m) từ source_cell tới mọi ô trong trav, lân cận 8 (Dijkstra, scipy.csgraph)."""
    h, w = trav.shape
    idx = -np.ones(trav.shape, dtype=np.int64)
    n = int(trav.sum())
    idx[trav] = np.arange(n)
    rows, cols, wts = [], [], []
    for dr, dc, cost in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0))):
        # cặp ô (r, c) -> (r + dr, c + dc), dr >= 0
        a = idx[0:h - dr, max(0, -dc):w - max(0, dc)]
        b = idx[dr:h, max(0, dc):w - max(0, -dc)]
        ok = (a >= 0) & (b >= 0)
        rows.append(a[ok])
        cols.append(b[ok])
        wts.append(np.full(int(ok.sum()), cost))
    out = np.full(trav.shape, np.inf)
    r0, c0 = source_cell
    if n == 0 or idx[r0, c0] < 0:
        return out
    g = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n)).tocsr()
    d = dijkstra(g, directed=False, indices=int(idx[r0, c0]))
    out[trav] = d * res
    return out


class Blacklist(object):
    """Đếm thất bại và blacklist theo BÁN KÍNH quanh tâm cụm (tâm cụm dịch chuyển khi /map cập nhật,
    nên không dùng khóa là tọa độ làm tròn)."""

    def __init__(self, radius=0.6, max_failures=2):
        self.radius = float(radius)
        self.max_failures = int(max_failures)
        self.failures = []   # [x, y, count]
        self.points = []     # [(x, y)] đã blacklist

    def clear(self):
        self.failures = []
        self.points = []

    def _near(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= self.radius

    def contains(self, xy):
        return any(self._near(xy, p) for p in self.points)

    def record_failure(self, xy):
        """Tăng bộ đếm thất bại gần xy; trả về True nếu điểm này vừa bị blacklist."""
        for f in self.failures:
            if self._near(xy, f):
                f[2] += 1
                f[0], f[1] = xy  # bám theo tâm cụm mới nhất
                break
        else:
            f = [xy[0], xy[1], 1]
            self.failures.append(f)
        if f[2] >= self.max_failures and not self.contains(xy):
            self.points.append((xy[0], xy[1]))
            return True
        return False


def select_target(grid, frontiers, robot_xy, blacklist, robot_radius=0.30, approach_radius=0.6,
                  min_goal_distance=0.4, free_thresh=20, occ_thresh=65, goal_clearance=None, cost_ok=None):
    """Chọn cụm frontier có điểm tiếp cận gần nhất THEO ĐƯỜNG ĐI.

    Ứng viên = ô đủ clearance (robot_radius), cách một ô của cụm <= approach_radius, cách robot (theo đường đi)
    >= min_goal_distance; điểm tiếp cận của cụm = ứng viên gần tâm cụm nhất; chi phí = khoảng cách đường đi
    tới điểm đó. Trả về (Target hoặc None, số cụm tới được, số cụm bị blacklist).

    goal_clearance: clearance tối thiểu (m) của ĐIỂM DỪNG, lớn hơn robot_radius của đường đi — để robot dừng
    và xoay được tại đích (với footprint đa giác, nên >= bán kính ngoại tiếp). cost_ok: mask bool cùng kích
    thước grid (costmap_ok_mask) — chỉ nhận điểm dừng mà costmap của Nav2 cũng coi là thoáng.
    """
    robot_cell = grid.to_cell(*robot_xy)
    trav, strict, clearance = traversable(grid, robot_cell, robot_radius, free_thresh, occ_thresh,
                                          return_strict=True)
    stop_ok = strict & (clearance >= (robot_radius if goal_clearance is None else goal_clearance))
    if cost_ok is not None:
        stop_ok &= cost_ok
    dist = geodesic_distance(trav, robot_cell, grid.res) if grid.inside(*robot_cell) else np.full(grid.shape, np.inf)
    k = int(math.ceil(approach_radius / grid.res))
    h, w = grid.shape
    best, n_reachable, n_black = None, 0, 0
    for cl in frontiers.clusters:
        if blacklist.contains(cl['centroid']):
            n_black += 1
            continue
        cells = cl['cells']
        r0, c0 = max(int(cells[:, 0].min()) - k, 0), max(int(cells[:, 1].min()) - k, 0)
        r1, c1 = min(int(cells[:, 0].max()) + k + 1, h), min(int(cells[:, 1].max()) + k + 1, w)
        sub = np.zeros((r1 - r0, c1 - c0), dtype=bool)
        sub[cells[:, 0] - r0, cells[:, 1] - c0] = True
        near = ndimage.distance_transform_edt(~sub) <= k
        d_sub = dist[r0:r1, c0:c1]
        # điểm dừng phải đủ clearance thật (không lấy ô của vùng thoát quanh robot)
        cand = near & stop_ok[r0:r1, c0:c1] & np.isfinite(d_sub) & (d_sub >= min_goal_distance)
        if not cand.any():
            continue
        n_reachable += 1
        # Điểm tiếp cận = ô ứng viên gần TÂM cụm nhất (Euclid): với cụm dài, đi tới giữa cụm thay vì nhích
        # từng đoạn ở đầu gần; chi phí của cụm = khoảng cách đường đi tới điểm này.
        cx, cy = cl['centroid']
        crow, ccol = (cy - grid.oy) / grid.res - 0.5 - r0, (cx - grid.ox) / grid.res - 0.5 - c0
        rr_all, cc_all = np.nonzero(cand)
        k_best = int(np.argmin((rr_all - crow) ** 2 + (cc_all - ccol) ** 2))
        rr, cc = int(rr_all[k_best]), int(cc_all[k_best])
        cost = float(d_sub[rr, cc])
        if best is None or cost < best.cost:
            gx, gy = grid.to_world(rr + r0, cc + c0)
            cells_xy = np.column_stack([cells[:, 1] * grid.res + grid.ox + 0.5 * grid.res,
                                        cells[:, 0] * grid.res + grid.oy + 0.5 * grid.res])
            best = Target((gx, gy), math.atan2(cy - gy, cx - gx), (cx, cy), cells_xy, cost, cl['size'])
    return best, n_reachable, n_black


def frontier_alive(grid, frontiers, cells_xy, radius=0.5):
    """Repetitive re-checking: mục tiêu còn sống nếu vẫn có ô frontier trong bán kính radius quanh các ô
    frontier ban đầu của cụm (tọa độ world, nên chịu được việc /map đổi kích thước/gốc)."""
    if cells_xy is None or len(cells_xy) == 0 or not frontiers.mask.any():
        return False
    cols = np.floor((cells_xy[:, 0] - grid.ox) / grid.res).astype(int)
    rows = np.floor((cells_xy[:, 1] - grid.oy) / grid.res).astype(int)
    ok = (rows >= 0) & (rows < grid.shape[0]) & (cols >= 0) & (cols < grid.shape[1])
    if not ok.any():
        return False
    dist_f = ndimage.distance_transform_edt(~frontiers.mask) * grid.res
    return bool(dist_f[rows[ok], cols[ok]].min() <= radius)


def known_free_area(grid, free_thresh=20):
    return float(grid.free_mask(free_thresh).sum()) * grid.res * grid.res


def known_cells_near(grid, cells_xy, margin=1.0):
    """Số ô đã biết (trống hoặc chiếm chỗ) trong hộp bao các ô frontier của cụm, nới thêm margin (m).
    Dùng để đo lượng thông tin thu được quanh mục tiêu: tới nơi mà con số này không tăng -> thất bại."""
    if cells_xy is None or len(cells_xy) == 0:
        return 0
    x0, y0 = cells_xy.min(axis=0) - margin
    x1, y1 = cells_xy.max(axis=0) + margin
    h, w = grid.shape
    c0 = min(max(int(math.floor((x0 - grid.ox) / grid.res)), 0), w)
    c1 = min(max(int(math.ceil((x1 - grid.ox) / grid.res)), 0), w)
    r0 = min(max(int(math.floor((y0 - grid.oy) / grid.res)), 0), h)
    r1 = min(max(int(math.ceil((y1 - grid.oy) / grid.res)), 0), h)
    return int((grid.data[r0:r1, c0:c1] >= 0).sum())


def costmap_ok_mask(grid, costmap, max_cost):
    """Mask bool theo lưới `grid`: True nếu ô tương ứng trên costmap Nav2 (nav_msgs/OccupancyGrid đã dịch giá trị:
    0 trống, 1..98 vùng phồng, 99 inscribed, 100 lethal, -1 chưa biết) có giá trị 0..max_cost. Ô nằm ngoài
    costmap được coi là True (costmap đổi kích thước chậm hơn /map một nhịp)."""
    h, w = grid.shape
    ys = grid.oy + (np.arange(h) + 0.5) * grid.res
    xs = grid.ox + (np.arange(w) + 0.5) * grid.res
    rows = np.floor((ys - costmap.oy) / costmap.res).astype(int)
    cols = np.floor((xs - costmap.ox) / costmap.res).astype(int)
    rin = (rows >= 0) & (rows < costmap.shape[0])
    cin = (cols >= 0) & (cols < costmap.shape[1])
    vals = costmap.data[np.ix_(np.clip(rows, 0, costmap.shape[0] - 1), np.clip(cols, 0, costmap.shape[1] - 1))]
    ok = (vals >= 0) & (vals <= max_cost)
    inside = np.outer(rin, cin)
    return np.where(inside, ok, True)
