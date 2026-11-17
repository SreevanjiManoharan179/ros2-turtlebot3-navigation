#!/usr/bin/env python3
"""Generate the custom indoor environment used by this project.

One layout definition (below) produces four consistent outputs:

  1. worlds/indoor_office.world             Gazebo Classic 11 world (ROS 2 Humble)
  2. worlds/indoor_office_gz.sdf            Gazebo Sim / Harmonic world (ROS 2 Jazzy)
  3. maps/indoor_office_ground_truth.pgm/.yaml   exact occupancy map rasterised
                                            from the same geometry (reference map)
  4. config/locations.yaml                  named rooms / goal poses + spawn pose

Run it again after editing LAYOUT to rebuild all three.  Standard library only.

    python3 tools/generate_indoor_office.py            # writes into the package
    python3 tools/generate_indoor_office.py --png x.png  # also draws a floor plan
"""
import argparse
import math
import os
import sys

# ---------------------------------------------------------------------------
# Layout definition (metres, world frame).  The building interior spans
# x = -6 .. 6 and y = -4 .. 4.  A corridor runs west-east through the middle
# (y = -0.7 .. 0.7) with three rooms on each side.
# ---------------------------------------------------------------------------
WALL_THICKNESS = 0.15
WALL_HEIGHT = 1.0

# Walls as centre-line segments (x1, y1, x2, y2).  Door openings are simply
# gaps between segments (all doors are 1.0 m wide; the lobby opening is 1.6 m).
WALLS = [
    # exterior
    (-6.075, -4.0, 6.075, -4.0),
    (-6.075, 4.0, 6.075, 4.0),
    (-6.0, -4.0, -6.0, 4.0),
    (6.0, -4.0, 6.0, 4.0),
    # corridor north wall: doors to Lab [-3.4,-2.4], Office [0.6,1.6], Meeting [2.6,3.6]
    (-6.0, 0.7, -3.4, 0.7),
    (-2.4, 0.7, 0.6, 0.7),
    (1.6, 0.7, 2.6, 0.7),
    (3.6, 0.7, 6.0, 0.7),
    # corridor south wall: Lobby opening [-4.6,-3.0], Kitchen [0.2,1.2], Storage [4.6,5.6]
    (-6.0, -0.7, -4.6, -0.7),
    (-3.0, -0.7, 0.2, -0.7),
    (1.2, -0.7, 4.6, -0.7),
    (5.6, -0.7, 6.0, -0.7),
    # north dividers: Lab | Office has an internal door [1.9, 2.9]; Office | Meeting is solid
    (-2.0, 0.7, -2.0, 1.9),
    (-2.0, 2.9, -2.0, 4.0),
    (2.0, 0.7, 2.0, 4.0),
    # south dividers: Lobby | Kitchen has an internal door [-3.2, -2.2]; Kitchen | Storage solid
    (-1.5, -4.0, -1.5, -3.2),
    (-1.5, -2.2, -1.5, -0.7),
    (2.5, -4.0, 2.5, -0.7),
]

# Furniture: ("box", name, cx, cy, size_x, size_y, height, rgb)
#            ("cyl", name, cx, cy, radius, height, rgb)
WOOD = (0.58, 0.40, 0.23)
DARK_WOOD = (0.42, 0.28, 0.17)
FURNITURE = [
    # Lab
    ("box", "lab_bench", -4.4, 2.6, 2.2, 0.7, 0.9, (0.90, 0.91, 0.94)),
    ("box", "lab_cabinet", -5.6, 3.55, 0.6, 0.7, 1.8, (0.33, 0.43, 0.58)),
    ("cyl", "robot_arm_base", -5.0, 1.35, 0.3, 0.6, (0.95, 0.55, 0.10)),
    # Office
    ("box", "desk_1", -1.0, 3.35, 1.4, 0.7, 0.75, WOOD),
    ("box", "desk_2", 1.0, 3.35, 1.4, 0.7, 0.75, WOOD),
    ("cyl", "round_table", -0.5, 1.75, 0.45, 0.75, (0.70, 0.52, 0.32)),
    ("cyl", "office_plant", -1.65, 1.1, 0.2, 0.9, (0.20, 0.60, 0.25)),
    # Meeting room
    ("box", "meeting_table", 4.4, 2.5, 2.2, 1.0, 0.75, DARK_WOOD),
    ("box", "chair_1", 3.85, 1.7, 0.45, 0.45, 0.9, (0.25, 0.25, 0.30)),
    ("box", "chair_2", 4.95, 1.7, 0.45, 0.45, 0.9, (0.25, 0.25, 0.30)),
    ("box", "chair_3", 3.85, 3.3, 0.45, 0.45, 0.9, (0.25, 0.25, 0.30)),
    ("box", "chair_4", 4.95, 3.3, 0.45, 0.45, 0.9, (0.25, 0.25, 0.30)),
    # Lobby / reception
    ("box", "reception_desk", -5.0, -2.0, 0.6, 1.6, 1.0, (0.88, 0.82, 0.70)),
    ("box", "sofa", -3.4, -3.5, 1.8, 0.7, 0.8, (0.72, 0.20, 0.20)),
    ("box", "coffee_table", -3.4, -2.6, 0.9, 0.5, 0.45, WOOD),
    ("cyl", "lobby_plant_1", -5.6, -3.6, 0.22, 1.0, (0.20, 0.60, 0.25)),
    ("cyl", "lobby_plant_2", -1.95, -1.15, 0.2, 1.0, (0.20, 0.60, 0.25)),
    # Kitchen
    ("box", "kitchen_counter", 0.9, -3.62, 2.4, 0.6, 0.9, (0.84, 0.84, 0.86)),
    ("box", "fridge", -1.05, -3.55, 0.7, 0.7, 1.8, (0.76, 0.79, 0.82)),
    ("box", "dining_table", 0.6, -2.1, 1.2, 0.8, 0.75, WOOD),
    # Storage
    ("box", "shelf_1", 3.7, -1.95, 2.0, 0.5, 1.6, (0.30, 0.32, 0.38)),
    ("box", "shelf_2", 3.7, -3.4, 2.0, 0.5, 1.6, (0.30, 0.32, 0.38)),
    ("box", "crates", 5.45, -3.5, 0.6, 0.6, 0.6, (0.76, 0.60, 0.40)),
    # Corridor
    ("cyl", "corridor_plant", -0.4, 0.42, 0.22, 1.0, (0.20, 0.60, 0.25)),
]

# Rooms: name -> (x_min, y_min, x_max, y_max, floor rgb).  Floors are visual only.
ROOMS = {
    "Lab": (-6.0, 0.7, -2.0, 4.0, (0.52, 0.70, 0.90)),
    "Office": (-2.0, 0.7, 2.0, 4.0, (0.92, 0.78, 0.42)),
    "Meeting Room": (2.0, 0.7, 6.0, 4.0, (0.55, 0.78, 0.52)),
    "Reception": (-6.0, -4.0, -1.5, -0.7, (0.86, 0.64, 0.48)),
    "Kitchen": (-1.5, -4.0, 2.5, -0.7, (0.93, 0.58, 0.38)),
    "Storage": (2.5, -4.0, 6.0, -0.7, (0.64, 0.58, 0.86)),
    "Corridor": (-6.0, -0.7, 6.0, 0.7, (0.70, 0.71, 0.73)),
}

# Named goal poses (x, y, yaw in degrees).  "reception" is also the spawn pose; its yaw
# is 0 so that maps built from wheel odometry (Gazebo Sim) only need a translation to
# line up with the world frame.
LOCATIONS = {
    "reception": (-3.8, -1.8, 0.0),
    "lab": (-3.0, 1.6, 90.0),
    "office": (0.6, 2.3, 90.0),
    "meeting_room": (2.8, 2.5, 0.0),
    "storage": (3.9, -2.68, 180.0),
    "kitchen": (-0.65, -2.7, 180.0),
    "corridor_east": (5.2, 0.0, 0.0),
    "corridor_west": (-5.3, 0.0, 180.0),
}
SPAWN = "reception"

# Ground-truth map settings
MAP_RES = 0.05
MAP_ORIGIN = (-6.5, -4.5)
MAP_SIZE = (13.0, 9.0)
BUILDING = (-6.0, -4.0, 6.0, 4.0)


# ---------------------------------------------------------------------------
def wall_boxes():
    """Convert wall segments into (cx, cy, length, thickness, yaw)."""
    boxes = []
    for x1, y1, x2, y2 in WALLS:
        length = math.hypot(x2 - x1, y2 - y1)
        yaw = math.atan2(y2 - y1, x2 - x1)
        boxes.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, length, WALL_THICKNESS, yaw))
    return boxes


def _material(rgb):
    r, g, b = rgb
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>"
            f"<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
            "<specular>0.05 0.05 0.05 1</specular></material>")


def _box_link(name, cx, cy, sx, sy, h, yaw, rgb, collide=True, z=None):
    z = h / 2.0 if z is None else z
    geom = f"<geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>"
    col = f"<collision name='collision'>{geom}</collision>" if collide else ""
    return (f"      <link name='{name}'>\n"
            f"        <pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>\n"
            f"        {col}\n"
            f"        <visual name='visual'>{geom}{_material(rgb)}</visual>\n"
            f"      </link>\n")


def _cyl_link(name, cx, cy, r, h, rgb):
    geom = f"<geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>"
    return (f"      <link name='{name}'>\n"
            f"        <pose>{cx:.4f} {cy:.4f} {h / 2.0:.4f} 0 0 0</pose>\n"
            f"        <collision name='collision'>{geom}</collision>\n"
            f"        <visual name='visual'>{geom}{_material(rgb)}</visual>\n"
            f"      </link>\n")


GROUND_AND_SUN = """    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.62 0.64 0.66 1</ambient><diffuse>0.62 0.64 0.66 1</diffuse></material>
        </visual>
      </link>
    </model>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <visualize>false</visualize>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.80 0.80 0.80 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
"""

GZ_GUI = """    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.55 0.55 0.55</ambient_light>
        <background_color>0.78 0.83 0.88</background_color>
        <camera_pose>0.0 -10.2 11.6 0 0.88 1.5708</camera_pose>
      </plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <gz-gui><property key="state" type="string">floating</property>
          <property key="width" type="double">5</property><property key="height" type="double">5</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui><property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property><property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui><property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property><property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui><property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property><property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="MarkerManager" name="Marker manager">
        <gz-gui><property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property><property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="SelectEntities" name="Select Entities">
        <gz-gui><property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property><property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
        <gz-gui><property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property><property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
        <use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
    </gui>
"""


def _links():
    links = []
    for i, (cx, cy, length, thick, yaw) in enumerate(wall_boxes()):
        links.append(_box_link(f"wall_{i:02d}", cx, cy, length, thick, WALL_HEIGHT, yaw,
                               (0.86, 0.85, 0.82)))
    for name, (x0, y0, x1, y1, rgb) in ROOMS.items():
        key = name.lower().replace(" ", "_")
        links.append(_box_link(f"floor_{key}", (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0,
                               0.002, 0.0, rgb, collide=False, z=0.0015))
    for item in FURNITURE:
        if item[0] == "box":
            _, name, cx, cy, sx, sy, h, rgb = item
            links.append(_box_link(name, cx, cy, sx, sy, h, 0.0, rgb))
        else:
            _, name, cx, cy, r, h, rgb = item
            links.append(_cyl_link(name, cx, cy, r, h, rgb))
    return links


def _building_model():
    return f"""    <model name="indoor_office">
      <static>true</static>
{''.join(_links())}    </model>
"""


def build_gz_world():
    """Gazebo Sim (Harmonic) version of the same world, for ROS 2 Jazzy."""
    return f"""<?xml version="1.0"?>
<!--
  Indoor office floor for the TurtleBot3 point-to-point navigation project (Gazebo Sim).
  GENERATED by tools/generate_indoor_office.py - edit the layout there, not here.
-->
<sdf version="1.8">
  <world name="default">
    <!-- 4 ms steps: Gazebo Sim publishes /clock on every step, and with the default 1 ms every ROS
         node would receive 1000 clock messages per second - too much for small PCs -->
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <shadows>false</shadows>
      <grid>false</grid>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.78 0.83 0.88 1</background>
    </scene>

{GZ_GUI}
{GROUND_AND_SUN}
{_building_model()}
  </world>
</sdf>
"""


def build_world():
    return f"""<?xml version="1.0"?>
<!--
  Indoor office floor for the TurtleBot3 point-to-point navigation project.
  GENERATED by tools/generate_indoor_office.py - edit the layout there, not here.
  Rooms: Reception (start), Lab, Office, Meeting Room, Kitchen, Storage, Corridor.
-->
<sdf version="1.7">
  <world name="default">

{GROUND_AND_SUN}
    <scene>
      <shadows>false</shadows>
      <grid>false</grid>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.78 0.83 0.88 1</background>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0.0 -10.2 11.6 0 0.88 1.5708</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- /gazebo/get_entity_state and /gazebo/set_entity_state (used by the evaluation runner) -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>

{_building_model()}
  </world>
</sdf>
"""


# ---------------------------------------------------------------------------
def _inside_box(px, py, cx, cy, sx, sy, yaw):
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = px - cx, py - cy
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return abs(lx) <= sx / 2.0 and abs(ly) <= sy / 2.0


def occupied(px, py):
    for cx, cy, length, thick, yaw in wall_boxes():
        if _inside_box(px, py, cx, cy, length, thick, yaw):
            return True
    for item in FURNITURE:
        if item[0] == "box":
            _, _, cx, cy, sx, sy, _, _ = item
            if _inside_box(px, py, cx, cy, sx, sy, 0.0):
                return True
        else:
            _, _, cx, cy, r, _, _ = item
            if (px - cx) ** 2 + (py - cy) ** 2 <= r * r:
                return True
    return False


def build_map():
    w = int(round(MAP_SIZE[0] / MAP_RES))
    h = int(round(MAP_SIZE[1] / MAP_RES))
    bx0, by0, bx1, by1 = BUILDING
    rows = []
    for r in range(h):  # row 0 is the top of the image (max y)
        y = MAP_ORIGIN[1] + (h - 1 - r + 0.5) * MAP_RES
        row = bytearray(w)
        for c in range(w):
            x = MAP_ORIGIN[0] + (c + 0.5) * MAP_RES
            if occupied(x, y):
                row[c] = 0
            elif bx0 <= x <= bx1 and by0 <= y <= by1:
                row[c] = 254
            else:
                row[c] = 205
        rows.append(bytes(row))
    return w, h, b"".join(rows)


def write_map(pgm_path, yaml_path):
    w, h, data = build_map()
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n# indoor_office ground truth\n{w} {h}\n255\n".encode())
        f.write(data)
    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\n"
                "mode: trinary\n"
                f"resolution: {MAP_RES:.3f}\n"
                f"origin: [{MAP_ORIGIN[0]:.3f}, {MAP_ORIGIN[1]:.3f}, 0.000]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n")
    return w, h


def write_locations(path):
    lines = [
        "# Named poses in the map/world frame of the indoor_office world.",
        "# GENERATED by tools/generate_indoor_office.py (x, y in metres, yaw in degrees).",
        f"spawn: {SPAWN}",
        "locations:",
    ]
    for name, (x, y, yaw) in LOCATIONS.items():
        lines.append(f"  {name}: {{x: {x:.2f}, y: {y:.2f}, yaw_deg: {yaw:.1f}}}")
    lines.append("rooms:")
    for name, (x0, y0, x1, y1, _) in ROOMS.items():
        lines.append(f"  \"{name}\": [{x0:.2f}, {y0:.2f}, {x1:.2f}, {y1:.2f}]")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def draw_png(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle, Polygon

    fig, ax = plt.subplots(figsize=(13, 9))
    for name, (x0, y0, x1, y1, rgb) in ROOMS.items():
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=rgb, ec="none", zorder=0))
        ax.text((x0 + x1) / 2, y1 - 0.25 if name != "Corridor" else 0.0, name,
                ha="center", va="center", fontsize=11, color="0.25", zorder=5)
    for cx, cy, length, thick, yaw in wall_boxes():
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(cx + c * dx - s * dy, cy + s * dx + c * dy)
               for dx, dy in [(-length / 2, -thick / 2), (length / 2, -thick / 2),
                              (length / 2, thick / 2), (-length / 2, thick / 2)]]
        ax.add_patch(Polygon(pts, closed=True, fc="0.35", ec="none", zorder=2))
    for item in FURNITURE:
        if item[0] == "box":
            _, name, cx, cy, sx, sy, _, rgb = item
            ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, fc=rgb, ec="0.2", lw=0.6, zorder=3))
        else:
            _, name, cx, cy, r, _, rgb = item
            ax.add_patch(Circle((cx, cy), r, fc=rgb, ec="0.2", lw=0.6, zorder=3))
    for name, (x, y, yaw) in LOCATIONS.items():
        ax.plot(x, y, "o", ms=9, color="crimson" if name == SPAWN else "navy", zorder=6)
        ax.annotate("", xy=(x + 0.45 * math.cos(math.radians(yaw)), y + 0.45 * math.sin(math.radians(yaw))),
                    xytext=(x, y), arrowprops=dict(arrowstyle="->", color="navy", lw=1.5), zorder=6)
        ax.text(x, y - 0.32, name, ha="center", va="top", fontsize=8.5, color="navy", zorder=6)
    ax.set_xlim(-6.6, 6.6)
    ax.set_ylim(-4.6, 4.6)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Indoor office floor - layout, furniture and named goal locations")
    ax.grid(True, alpha=0.25, zorder=1)
    fig.savefig(path, dpi=110, bbox_inches="tight")


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    default_pkg = os.path.join(here, "..", "ros2_ws", "src", "tb3_point_to_point")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pkg", default=default_pkg, help="package directory to write into")
    ap.add_argument("--png", default=None, help="optional floor-plan image to draw (needs matplotlib)")
    args = ap.parse_args(argv)

    pkg = os.path.abspath(args.pkg)
    for sub in ("worlds", "maps", "config"):
        os.makedirs(os.path.join(pkg, sub), exist_ok=True)
    world_path = os.path.join(pkg, "worlds", "indoor_office.world")
    with open(world_path, "w") as f:
        f.write(build_world())
    with open(os.path.join(pkg, "worlds", "indoor_office_gz.sdf"), "w") as f:
        f.write(build_gz_world())
    w, h = write_map(os.path.join(pkg, "maps", "indoor_office_ground_truth.pgm"),
                     os.path.join(pkg, "maps", "indoor_office_ground_truth.yaml"))
    write_locations(os.path.join(pkg, "config", "locations.yaml"))
    print(f"world     -> {world_path}")
    print(f"map       -> {w}x{h} px @ {MAP_RES} m")
    print(f"locations -> {len(LOCATIONS)} named poses")
    if args.png:
        draw_png(args.png)
        print(f"floor plan -> {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
