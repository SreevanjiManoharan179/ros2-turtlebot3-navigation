"""Occupancy-map helpers: load Nav2 map files, convert coordinates, check goals.

Works on map YAML + PGM files (same semantics as nav2_map_server) and on live
nav_msgs/OccupancyGrid messages.  Pure Python apart from the optional PyYAML.
"""
import math
import os

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


def _read_pgm(path):
    """Return (width, height, maxval, pixels) for a binary (P5) or ASCII (P2) PGM.

    pixels is a list of rows, row 0 = top of the image.
    """
    with open(path, 'rb') as f:
        data = f.read()
    tokens = []
    i = 0
    n = len(data)
    while len(tokens) < 4:
        while i < n and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < n and data[i:i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        j = i
        while j < n and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic = tokens[0]
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b'P5':
        i += 1  # exactly one whitespace byte after maxval
        if maxval > 255:
            raise ValueError('16-bit PGM maps are not supported')
        raw = data[i:i + width * height]
        if len(raw) != width * height:
            raise ValueError(f'{path}: truncated PGM data')
        rows = [list(raw[r * width:(r + 1) * width]) for r in range(height)]
    elif magic == b'P2':
        values = [int(v) for v in data[i:].split()]
        rows = [values[r * width:(r + 1) * width] for r in range(height)]
    else:
        raise ValueError(f'{path}: unsupported image format {magic!r} (use PGM)')
    return width, height, maxval, rows


def _load_yaml(path):
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f)
    except ImportError:  # minimal fallback for flat map files
        out = {}
        with open(path) as f:
            for line in f:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                key, val = [s.strip() for s in line.split(':', 1)]
                if val.startswith('['):
                    out[key] = [float(v) for v in val.strip('[]').split(',')]
                else:
                    try:
                        out[key] = float(val) if '.' in val else int(val)
                    except ValueError:
                        out[key] = val
        return out


class OccupancyMap:
    """2-D occupancy grid in ROS convention (row 0 = bottom, values -1/0..100)."""

    def __init__(self, width, height, resolution, origin_x, origin_y, cells, image_rows=None):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cells = cells              # flat list, index = row * width + col
        self.image_rows = image_rows    # original greyscale rows (top first), if loaded from file

    # ----------------------------------------------------------------- loading
    @classmethod
    def from_yaml(cls, yaml_path):
        meta = _load_yaml(yaml_path)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
        width, height, maxval, rows = _read_pgm(image)
        negate = int(meta.get('negate', 0))
        occ_t = float(meta.get('occupied_thresh', 0.65))
        free_t = float(meta.get('free_thresh', 0.25))
        mode = str(meta.get('mode', 'trinary'))
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        cells = [UNKNOWN] * (width * height)
        for img_r, row in enumerate(rows):
            grid_r = height - 1 - img_r
            base = grid_r * width
            for c, pix in enumerate(row):
                p = pix / float(maxval)
                occ = p if negate else 1.0 - p
                if occ > occ_t:
                    val = OCCUPIED
                elif occ < free_t:
                    val = FREE
                elif mode == 'scale':
                    val = int(round(99.0 * (occ - free_t) / (occ_t - free_t)))
                else:
                    val = UNKNOWN
                cells[base + c] = val
        return cls(width, height, meta['resolution'], origin[0], origin[1], cells, rows)

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        return cls(info.width, info.height, info.resolution,
                   info.origin.position.x, info.origin.position.y, list(msg.data))

    # ------------------------------------------------------------- coordinates
    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return col, row

    def cell_to_world(self, col, row):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def value_at(self, x, y):
        col, row = self.world_to_cell(x, y)
        if not self.in_bounds(col, row):
            return None
        return self.cells[row * self.width + col]

    def extent(self):
        """(x_min, y_min, x_max, y_max) of the whole grid."""
        return (self.origin_x, self.origin_y,
                self.origin_x + self.width * self.resolution,
                self.origin_y + self.height * self.resolution)

    # ------------------------------------------------------------------ checks
    def is_obstacle(self, value, unknown_is_obstacle=False, occupied_threshold=65):
        if value is None:
            return True
        if value == UNKNOWN:
            return unknown_is_obstacle
        return value >= occupied_threshold

    def clearance(self, x, y, max_distance=1.0, unknown_is_obstacle=False):
        """Distance (m) from (x, y) to the nearest obstacle cell, capped at max_distance."""
        col, row = self.world_to_cell(x, y)
        reach = int(math.ceil(max_distance / self.resolution))
        best = max_distance
        for dr in range(-reach, reach + 1):
            r = row + dr
            for dc in range(-reach, reach + 1):
                c = col + dc
                if not self.in_bounds(c, r):
                    continue
                if not self.is_obstacle(self.cells[r * self.width + c], unknown_is_obstacle):
                    continue
                cx, cy = self.cell_to_world(c, r)
                # distance to the cell edge, not its centre
                d = math.hypot(cx - x, cy - y) - self.resolution / 2.0
                if d < best:
                    best = max(d, 0.0)
        return best

    def check_goal(self, x, y, robot_radius=0.105, min_clearance=0.12):
        """Validate a goal position before sending it to Nav2.

        Returns (ok, reason, clearance_m).  The goal must lie inside the map, on a
        known free cell, with at least robot_radius + min_clearance to obstacles.
        """
        value = self.value_at(x, y)
        if value is None:
            x0, y0, x1, y1 = self.extent()
            return False, (f'({x:.2f}, {y:.2f}) is outside the map '
                           f'[x {x0:.1f}..{x1:.1f}, y {y0:.1f}..{y1:.1f}]'), 0.0
        if value == UNKNOWN:
            return False, f'({x:.2f}, {y:.2f}) is in unexplored (unknown) space', 0.0
        if value >= 65:
            return False, f'({x:.2f}, {y:.2f}) is inside an obstacle or wall', 0.0
        need = robot_radius + min_clearance
        clear = self.clearance(x, y, max_distance=need + 0.5, unknown_is_obstacle=True)
        if clear < need:
            return False, (f'({x:.2f}, {y:.2f}) is only {clear:.2f} m from an obstacle '
                           f'(needs >= {need:.2f} m for the robot to fit)'), clear
        return True, 'ok', clear

    # ------------------------------------------------------------- comparison
    def agreement_with(self, other):
        """Cell-wise comparison with another map over this map's known cells.

        Returns a dict with the share of free/occupied cells of *self* that
        *other* labels the same way (occupied cells may be matched within one cell).
        """
        free_total = free_match = occ_total = occ_match = 0
        for row in range(self.height):
            for col in range(self.width):
                v = self.cells[row * self.width + col]
                if v == UNKNOWN:
                    continue
                x, y = self.cell_to_world(col, row)
                if v >= 65:
                    occ_total += 1
                    hit = False
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            o = other.value_at(x + dx * self.resolution, y + dy * self.resolution)
                            if o is not None and o >= 65:
                                hit = True
                    occ_match += int(hit)
                else:
                    free_total += 1
                    o = other.value_at(x, y)
                    free_match += int(o is not None and o != UNKNOWN and o < 65)
        return {
            'free_cells': free_total,
            'free_agreement_pct': 100.0 * free_match / free_total if free_total else 0.0,
            'occupied_cells': occ_total,
            'occupied_agreement_pct': 100.0 * occ_match / occ_total if occ_total else 0.0,
        }


# ------------------------------------------------------------- floor-plan alignment
def _occupied_points(m, threshold=65):
    pts = []
    for row in range(m.height):
        for col in range(m.width):
            v = m.cells[row * m.width + col]
            if v != UNKNOWN and v >= threshold:
                pts.append(m.cell_to_world(col, row))
    return pts


def distance_field(m, threshold=65):
    """Distance [m] from every cell centre of *m* to the nearest occupied cell (two-pass chamfer)."""
    inf = 1e9
    w, h = m.width, m.height
    d = [0.0 if (m.cells[i] != UNKNOWN and m.cells[i] >= threshold) else inf for i in range(w * h)]
    diag = math.sqrt(2.0)
    for row in range(h):                       # forward pass
        for col in range(w):
            i = row * w + col
            best = d[i]
            if col > 0:
                best = min(best, d[i - 1] + 1.0)
            if row > 0:
                best = min(best, d[i - w] + 1.0)
                if col > 0:
                    best = min(best, d[i - w - 1] + diag)
                if col < w - 1:
                    best = min(best, d[i - w + 1] + diag)
            d[i] = best
    for row in range(h - 1, -1, -1):           # backward pass
        for col in range(w - 1, -1, -1):
            i = row * w + col
            best = d[i]
            if col < w - 1:
                best = min(best, d[i + 1] + 1.0)
            if row < h - 1:
                best = min(best, d[i + w] + 1.0)
                if col < w - 1:
                    best = min(best, d[i + w + 1] + diag)
                if col > 0:
                    best = min(best, d[i + w - 1] + diag)
            d[i] = best
    return [v * m.resolution for v in d]


def wall_accuracy(slam, reference, dx=0.0, dy=0.0, field=None, points=None, cap=0.3):
    """How far the occupied cells of *slam* (shifted by dx, dy) are from the walls of *reference*.

    Returns {'mean_m', 'within_5cm_pct', 'within_10cm_pct', 'cells'}; distances are capped at *cap*.
    """
    field = field if field is not None else distance_field(reference)
    points = points if points is not None else _occupied_points(slam)
    ds = []
    w, h, res = reference.width, reference.height, reference.resolution
    for x, y in points:
        # bilinear sample between cell centres, so that sub-cell shifts change the result
        fx = (x + dx - reference.origin_x) / res - 0.5
        fy = (y + dy - reference.origin_y) / res - 0.5
        c0, r0 = int(math.floor(fx)), int(math.floor(fy))
        if 0 <= c0 < w - 1 and 0 <= r0 < h - 1:
            tx, ty = fx - c0, fy - r0
            i = r0 * w + c0
            v = ((1 - tx) * (1 - ty) * field[i] + tx * (1 - ty) * field[i + 1] +
                 (1 - tx) * ty * field[i + w] + tx * ty * field[i + w + 1])
            ds.append(min(v, cap))
        else:
            ds.append(cap)
    n = len(ds)
    if not n:
        return {'mean_m': float('nan'), 'within_5cm_pct': 0.0, 'within_10cm_pct': 0.0, 'cells': 0}
    return {'mean_m': sum(ds) / n,
            'within_5cm_pct': 100.0 * sum(1 for v in ds if v <= 0.05 + 1e-9) / n,
            'within_10cm_pct': 100.0 * sum(1 for v in ds if v <= 0.10 + 1e-9) / n,
            'cells': n}


def align_to_reference(slam, reference, search=0.15, step=0.01):
    """Best (dx, dy) that moves the walls of *slam* onto the walls of *reference* (a floor plan).

    Grid search over +-search metres; returns (dx, dy, accuracy_before, accuracy_after).
    Shift the map origin by (dx, dy) to apply it.
    """
    field = distance_field(reference)
    points = _occupied_points(slam)
    before = wall_accuracy(slam, reference, field=field, points=points)
    best = (before['mean_m'], 0.0, 0.0)
    n = int(round(search / step))
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            dx, dy = i * step, j * step
            acc = wall_accuracy(slam, reference, dx, dy, field=field, points=points)
            if acc['mean_m'] < best[0] - 1e-9:
                best = (acc['mean_m'], dx, dy)
    _, dx, dy = best
    after = wall_accuracy(slam, reference, dx, dy, field=field, points=points)
    return dx, dy, before, after
