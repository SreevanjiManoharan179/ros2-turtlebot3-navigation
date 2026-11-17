"""Named locations (rooms) and scenario files.

A goal can be given as:
  * a location name from config/locations.yaml, e.g. "kitchen"
  * "x,y" or "x,y,yaw_deg" (map frame, metres / degrees)
  * a dict {x: .., y: .., yaw_deg: ..} or a list [x, y, yaw_deg] (inside YAML files)
"""
import math
import os
from collections import namedtuple

import yaml

PACKAGE = 'tb3_point_to_point'

Location = namedtuple('Location', ['name', 'x', 'y', 'yaw'])  # yaw in radians


def share_dir():
    """Installed share directory of this package (falls back to the source tree)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PACKAGE)
    except Exception:  # running from the source tree (unit tests, tools)
        return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def default_path(*parts):
    return os.path.join(share_dir(), *parts)


def _location_from(name, value):
    if isinstance(value, dict):
        return Location(name, float(value['x']), float(value['y']),
                        math.radians(float(value.get('yaw_deg', 0.0))))
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        yaw = float(value[2]) if len(value) > 2 else 0.0
        return Location(name, float(value[0]), float(value[1]), math.radians(yaw))
    raise ValueError(f'cannot read location {name!r}: {value!r}')


class LocationBook:
    def __init__(self, path=None):
        self.path = path or default_path('config', 'locations.yaml')
        with open(self.path) as f:
            data = yaml.safe_load(f) or {}
        self.locations = {name: _location_from(name, val)
                          for name, val in (data.get('locations') or {}).items()}
        self.rooms = {name: [float(v) for v in rect] for name, rect in (data.get('rooms') or {}).items()}
        self.spawn_name = data.get('spawn')

    @property
    def spawn(self):
        return self.locations.get(self.spawn_name) if self.spawn_name else None

    def names(self):
        return list(self.locations)

    def room_of(self, x, y):
        """Name of the room containing (x, y); doorways count as the corridor/room they open to."""
        best = None
        for name, (x0, y0, x1, y1) in self.rooms.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                if best is None or name != 'Corridor':
                    best = name
        return best

    def resolve(self, spec, name=None):
        """Turn a goal spec (name, 'x,y[,yaw]', dict or list) into a Location."""
        if isinstance(spec, Location):
            return spec
        if isinstance(spec, (dict, list, tuple)):
            return _location_from(name or 'custom', spec)
        text = str(spec).strip()
        key = text.lower().replace(' ', '_').replace('-', '_')
        if key in self.locations:
            return self.locations[key]
        parts = [p for p in text.replace(';', ',').split(',') if p.strip()]
        if len(parts) in (2, 3):
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                vals = None
            if vals is not None:
                yaw = math.radians(vals[2]) if len(vals) == 3 else 0.0
                return Location(name or f'({vals[0]:.2f}, {vals[1]:.2f})', vals[0], vals[1], yaw)
        raise KeyError(f'unknown location {spec!r}. Known: {", ".join(self.locations)}; '
                       'or give coordinates as "x,y" or "x,y,yaw_deg"')


def load_scenarios(path=None):
    path = path or default_path('config', 'scenarios.yaml')
    with open(path) as f:
        return yaml.safe_load(f) or {}, path
