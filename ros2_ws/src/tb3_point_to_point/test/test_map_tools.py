import os

from tb3_point_to_point.locations import LocationBook
from tb3_point_to_point.map_tools import FREE, OCCUPIED, UNKNOWN, OccupancyMap, align_to_reference

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
GT_MAP = os.path.join(PKG, 'maps', 'indoor_office_ground_truth.yaml')
LOCATIONS = os.path.join(PKG, 'config', 'locations.yaml')


def _tiny_map(tmp_path):
    # 4x3 image, row 0 = top. black = occupied, white = free, grey = unknown
    pixels = bytes([0, 254, 254, 205,
                    254, 254, 254, 205,
                    254, 254, 0, 205])
    (tmp_path / 'm.pgm').write_bytes(b'P5\n4 3\n255\n' + pixels)
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return OccupancyMap.from_yaml(str(tmp_path / 'm.yaml'))


def test_pgm_orientation_and_values(tmp_path):
    m = _tiny_map(tmp_path)
    assert (m.width, m.height) == (4, 3)
    assert m.value_at(0.5, 2.5) == OCCUPIED     # top-left pixel is the highest row in the grid
    assert m.value_at(2.5, 0.5) == OCCUPIED     # bottom row, third pixel
    assert m.value_at(1.5, 1.5) == FREE
    assert m.value_at(3.5, 1.5) == UNKNOWN
    assert m.value_at(10.0, 10.0) is None


def test_goal_check_on_ground_truth_map():
    m = OccupancyMap.from_yaml(GT_MAP)
    book = LocationBook(LOCATIONS)
    for name, loc in book.locations.items():
        ok, reason, clearance = m.check_goal(loc.x, loc.y)
        assert ok, f'{name}: {reason}'
        assert clearance > 0.3
    ok, reason, _ = m.check_goal(-6.0, 0.0)        # inside the west wall
    assert not ok
    ok, reason, _ = m.check_goal(20.0, 0.0)        # outside the map
    assert not ok and 'outside' in reason
    ok, reason, _ = m.check_goal(-4.4, 2.6)        # inside the lab bench
    assert not ok


def test_room_lookup_and_resolve():
    book = LocationBook(LOCATIONS)
    assert book.room_of(-3.0, 1.6) == 'Lab'
    assert book.room_of(0.0, 0.0) == 'Corridor'
    assert book.resolve('Meeting Room').name == 'meeting_room'
    loc = book.resolve('1.5,-2.0,90')
    assert (loc.x, loc.y) == (1.5, -2.0)


def test_alignment_recovers_a_known_map_offset():
    ref = OccupancyMap.from_yaml(GT_MAP)
    shifted = OccupancyMap(ref.width, ref.height, ref.resolution, ref.origin_x + 0.07, ref.origin_y - 0.04,
                           list(ref.cells))
    dx, dy, before, after = align_to_reference(shifted, ref, search=0.12, step=0.01)
    assert abs(dx + 0.07) < 0.011 and abs(dy - 0.04) < 0.011
    assert after['mean_m'] < before['mean_m']
    assert after['within_10cm_pct'] > 99.0
