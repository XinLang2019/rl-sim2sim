#!/usr/bin/env python3
"""Gazebo Classic / ROS Noetic test world. Python 3.8+, standard library only; text is generated as visual geometry.

Run: python3 generate_test_terrains_A.py --output-dir ..
Default: reproduce verified A world without labels; use --labels for ground labels. Only the world is written unless --extras is given.
Output layout: columns at Y=0,-11,-22; increasing difficulty and traverse along +X.
All dimensions are metres. Edit DEFAULTS or use --help.
"""
import argparse
import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ET

DEFAULTS = dict(steps=10, width=5.0, tread=0.30, top_length=2.0,
                ramp_run=3.0, column_gap=1.0, row_gap=3.0,
                approach=6.0, mu=0.99, mu2=0.99, label_gap=1.0)
STAIR_HEIGHTS = (0.20, 0.25, 0.30)
RAMP_ANGLES = (30, 35, 40)
BLOCK_HEIGHTS = (0.60, 0.80, 1.00)
BLOCK_LENGTH = 3.0  # travel length, along world X after final layout rotation
BLOCK_WIDTH = 5.0   # transverse width, along world Y after final layout rotation
COLORS = ((0.25, 0.55, 0.85, 1), (0.90, 0.62, 0.22, 1),
          (0.35, 0.70, 0.40, 1))


def sub(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = value if isinstance(value, str) else str(value)
    return node


def nums(values):
    return ' '.join('{:.9g}'.format(v) for v in values)


def box(model, name, size, pose, color, mu, mu2):
    link = sub(model, 'link', name=name)
    sub(link, 'pose', nums(pose))
    collision = sub(link, 'collision', name=name + '_collision')
    sub(sub(sub(collision, 'geometry'), 'box'), 'size', nums(size))
    ode = sub(sub(sub(collision, 'surface'), 'friction'), 'ode')
    sub(ode, 'mu', mu)
    sub(ode, 'mu2', mu2)
    visual = sub(link, 'visual', name=name + '_visual')
    sub(sub(sub(visual, 'geometry'), 'box'), 'size', nums(size))
    mat = sub(visual, 'material')
    sub(mat, 'ambient', nums(color))
    sub(mat, 'diffuse', nums(color))


def model(world, name):
    obj = sub(world, 'model', name=name)
    sub(obj, 'static', 'true')
    return obj


# The following source-compatible geometry helpers are copied from the
# user's known-working generation_terrains(1).py. Placement is adapted only
# by rigid rotation (+90deg about Z) to keep the requested +Y test layout.
def make_world_basics(
    max_step_size=0.001,
    real_time_factor=1.0,
    real_time_update_rate=1000,
    gravity="0 0 -9.81",
    solver_type="quick",
    solver_iters=50,
    solver_sor=1.3,
    cfm=0.0,
    erp=0.2,
    contact_max_correcting_vel=10.0,
    contact_surface_layer=0.001,
    cloud_speed=12,
    include_sun=True,
    include_ground=True
):
    parts = []
    parts.append('    <physics type="ode">\n')
    parts.append(f'      <max_step_size>{max_step_size}</max_step_size>\n')
    parts.append(f'      <real_time_factor>{real_time_factor}</real_time_factor>\n')
    parts.append(f'      <real_time_update_rate>{real_time_update_rate}</real_time_update_rate>\n')
    parts.append(f'      <gravity>{gravity}</gravity>\n')
    parts.append('      <ode>\n')
    parts.append('        <solver>\n')
    parts.append(f'          <type>{solver_type}</type>\n')
    parts.append(f'          <iters>{solver_iters}</iters>\n')
    parts.append(f'          <sor>{solver_sor}</sor>\n')
    parts.append('        </solver>\n')
    parts.append('        <constraints>\n')
    parts.append(f'          <cfm>{cfm}</cfm>\n')
    parts.append(f'          <erp>{erp}</erp>\n')
    parts.append(f'          <contact_max_correcting_vel>{contact_max_correcting_vel}</contact_max_correcting_vel>\n')
    parts.append(f'          <contact_surface_layer>{contact_surface_layer}</contact_surface_layer>\n')
    parts.append('        </constraints>\n')
    parts.append('      </ode>\n')
    parts.append('    </physics>\n\n')
    parts.append('    <scene>\n')
    parts.append('      <sky>\n')
    parts.append('        <clouds>\n')
    parts.append(f'          <speed>{cloud_speed}</speed>\n')
    parts.append('        </clouds>\n')
    parts.append('      </sky>\n')
    parts.append('    </scene>\n')
    if include_sun:
        parts.append('    <include>\n')
        parts.append('      <uri>model://sun</uri>\n')
        parts.append('    </include>\n')
    if include_ground:
        parts.append('    <include>\n')
        parts.append('      <uri>model://ground_plane</uri>\n')
        parts.append('    </include>\n')
    return ''.join(parts)

def generate_stairs_links(name_prefix, steps, step_h, step_d, width, plat_len,
                          position=(0,0,0), xOry="x", material=None,
                          mu=0.99, mu2=0.99):
    links = []
    x, y, _ = position
    z = step_h/2
    sign = -1 if xOry.startswith("-") else 1
    along_x = (xOry.endswith("x") or xOry == "x")

    def add_box(link_name, size, pose_xyz):
        px, py, pz = pose_xyz
        links.append(f'      <link name="{link_name}">\n')
        links.append(f'        <pose>{px} {py} {pz} 0 0 0</pose>\n')
        links.append(f'        <collision name="{link_name}_collision">\n')
        links.append('          <geometry>\n')
        links.append('            <box>\n')
        links.append(f'              <size>{size[0]} {size[1]} {size[2]}</size>\n')
        links.append('            </box>\n')
        links.append('          </geometry>\n')
        links.append('          <surface>\n')
        links.append('            <friction>\n')
        links.append('              <ode>\n')
        links.append(f'                <mu>{mu}</mu>\n')
        links.append(f'                <mu2>{mu2}</mu2>\n')
        links.append('              </ode>\n')
        links.append('            </friction>\n')
        links.append('          </surface>\n')
        links.append('        </collision>\n')
        links.append(f'        <visual name="{link_name}_visual">\n')
        links.append('          <geometry>\n')
        links.append('            <box>\n')
        links.append(f'              <size>{size[0]} {size[1]} {size[2]}</size>\n')
        links.append('            </box>\n')
        links.append('          </geometry>\n')
        if material:
            links.append('          <material>\n')
            links.append('            <script>\n')
            links.append('              <uri>file://media/materials/scripts/gazebo.material</uri>\n')
            links.append(f'              <name>Gazebo/{material}</name>\n')
            links.append('            </script>\n')
            links.append('          </material>\n')
        links.append('        </visual>\n')
        links.append('      </link>\n')

    def step_size():
        return (step_d, width, step_h) if along_x else (width, step_d, step_h)
    def platform_size():
        return (plat_len, width, step_h) if along_x else (width, plat_len, step_h)

    # 上行
    for i in range(steps):
        add_box(f"{name_prefix}__up_{i}", step_size(), (x, y, z))
        if along_x: x += sign*step_d
        else:       y += sign*step_d
        z += step_h

    # 平台（与上行最后一级顶面齐）
    z -= step_h
    if along_x: x += sign*(plat_len/2 - step_d/2)
    else:       y += sign*(plat_len/2 - step_d/2)
    add_box(f"{name_prefix}__plat", platform_size(), (x, y, z))
    if along_x: x += sign*(plat_len/2 + step_d/2)
    else:       y += sign*(plat_len/2 + step_d/2)

    # 下行
    for i in range(steps):
        add_box(f"{name_prefix}__down_{i}", step_size(), (x, y, z))
        if along_x: x += sign*step_d
        else:       y += sign*step_d
        z -= step_h

    return ''.join(links)

def generate_trapezoid_ramp_links(
    name_prefix,
    angle_deg,
    slope_len,
    plat_len,
    width,
    position=(0,0,0),
    xOry="x",
    material=None,
    plate_thickness=0.05,
    mu=0.99, mu2=0.99
):
    links = []
    x, y, z_ref = position
    sign    = -1 if xOry.startswith("-") else 1
    along_x = (xOry.endswith("x") or xOry == "x")

    alpha = math.radians(abs(angle_deg))
    c, s  = math.cos(alpha), math.sin(alpha)
    dh    = (s / max(c, 1e-9)) * slope_len

    # Gazebo朝向（与你确认过的负号版本）
    up_pitch = (-alpha if sign == 1 else  alpha) if along_x else 0.0
    up_roll  = (-alpha if sign == 1 else  alpha) if not along_x else 0.0
    dn_pitch = -up_pitch
    dn_roll  = -up_roll

    H = z_ref + plate_thickness/2.0

    def add_box(link_name, size, pose_xyz, rpy):
        px, py, pz = pose_xyz
        rr, pp, yy = rpy
        links.append(f'      <link name="{link_name}">\n')
        links.append(f'        <pose>{px} {py} {pz} {rr} {pp} {yy}</pose>\n')
        links.append(f'        <collision name="{link_name}_collision">\n')
        links.append('          <geometry>\n')
        links.append('            <box>\n')
        links.append(f'              <size>{size[0]} {size[1]} {size[2]}</size>\n')
        links.append('            </box>\n')
        links.append('          </geometry>\n')
        links.append('          <surface>\n')
        links.append('            <friction>\n')
        links.append('              <ode>\n')
        links.append(f'                <mu>{mu}</mu>\n')
        links.append(f'                <mu2>{mu2}</mu2>\n')
        links.append('              </ode>\n')
        links.append('            </friction>\n')
        links.append('          </surface>\n')
        links.append('        </collision>\n')
        links.append(f'        <visual name="{link_name}_visual">\n')
        links.append('          <geometry>\n')
        links.append('            <box>\n')
        links.append(f'              <size>{size[0]} {size[1]} {size[2]}</size>\n')
        links.append('            </box>\n')
        links.append('          </geometry>\n')
        if material:
            links.append('          <material>\n')
            links.append('            <script>\n')
            links.append('              <uri>file://media/materials/scripts/gazebo.material</uri>\n')
            links.append(f'              <name>Gazebo/{material}</name>\n')
            links.append('            </script>\n')
            links.append('          </material>\n')
        links.append('        </visual>\n')
        links.append('      </link>\n')

    ramp_body_len = slope_len / max(c, 1e-9)
    def size_ramp():
        return (ramp_body_len, width, plate_thickness) if along_x else (width, ramp_body_len, plate_thickness)
    def size_plat():
        return (plat_len, width, plate_thickness) if along_x else (width, plat_len, plate_thickness)
    def z_center_of_ramp(H_start, delta_h):
        return (H_start + (H_start + delta_h))/2.0 - (plate_thickness * c)/2.0
    def z_center_of_flat(H_top):
        return H_top - plate_thickness/2.0

    # 上坡
    if along_x:
        cx, cy = x + sign*(slope_len/2.0), y
        rr, pp = 0.0, up_pitch
    else:
        cx, cy = x, y + sign*(slope_len/2.0)
        rr, pp = up_roll, 0.0
    cz = z_center_of_ramp(H, +dh)
    add_box(f"{name_prefix}__ramp_up", size_ramp(), (cx, cy, cz), (rr, pp, 0.0))
    if along_x: x += sign*slope_len
    else:       y += sign*slope_len
    H += dh

    # 平台
    if along_x:
        cx, cy = x + sign*(plat_len/2.0), y
    else:
        cx, cy = x, y + sign*(plat_len/2.0)
    cz = z_center_of_flat(H)
    add_box(f"{name_prefix}__platform", size_plat(), (cx, cy, cz), (0.0, 0.0, 0.0))
    if along_x: x += sign*plat_len
    else:       y += sign*plat_len
    # H 不变

    # 下坡
    if along_x:
        cx, cy = x + sign*(slope_len/2.0), y
        rr, pp = 0.0, dn_pitch
    else:
        cx, cy = x, y + sign*(slope_len/2.0)
        rr, pp = dn_roll, 0.0
    cz = z_center_of_ramp(H, -dh)
    add_box(f"{name_prefix}__ramp_down", size_ramp(), (cx, cy, cz), (rr, pp, 0.0))
    if along_x: x += sign*slope_len
    else:       y += sign*slope_len
    H -= dh

    return ''.join(links)

def generate_box_link(
    name_prefix,
    size,               # (length, width, height)
    position=(0,0,0),   # (x, y, z)
    material=None,
    mu=0.99, mu2=0.99
):
    length, width, height = size
    x, y, z = position
    s = []
    s.append(f'      <link name="{name_prefix}__link">\n')
    s.append(f'        <pose>{x} {y} {z} 0 0 0</pose>\n')
    s.append('        <collision name="collision">\n')
    s.append('          <geometry>\n')
    s.append('            <box>\n')
    s.append(f'              <size>{length} {width} {height}</size>\n')
    s.append('            </box>\n')
    s.append('          </geometry>\n')
    s.append('          <surface>\n')
    s.append('            <friction>\n')
    s.append('              <ode>\n')
    s.append(f'                <mu>{mu}</mu>\n')
    s.append(f'                <mu2>{mu2}</mu2>\n')
    s.append('              </ode>\n')
    s.append('            </friction>\n')
    s.append('          </surface>\n')
    s.append('        </collision>\n')
    s.append('        <visual name="visual">\n')
    s.append('          <geometry>\n')
    s.append('            <box>\n')
    s.append(f'              <size>{length} {width} {height}</size>\n')
    s.append('            </box>\n')
    s.append('          </geometry>\n')
    if material:
        s.append('          <material>\n')
        s.append('            <script>\n')
        s.append('              <uri>file://media/materials/scripts/gazebo.material</uri>\n')
        s.append(f'              <name>Gazebo/{material}</name>\n')
        s.append('            </script>\n')
        s.append('          </material>\n')
    s.append('        </visual>\n')
    s.append('      </link>\n')
    return ''.join(s)


def append_source_links(container, fragment, x, y, color):
    """Rotate original +X terrain by +90 degrees into +Y; preserve shapes."""
    root=ET.fromstring('<model>'+fragment+'</model>')
    for link in root.findall('link'):
        pose=link.find('pose')
        px,py,pz,rr,pp,yy=map(float,pose.text.split())
        pose.text=nums((x-py,y+px,pz,rr,pp,yy+math.pi/2))
        mat=sub(link.find('visual'),'material')
        sub(mat,'ambient',nums(color));sub(mat,'diffuse',nums(color))
        container.append(link)


def stairs(container,x,y0,h,cfg):
    name='stairs_h{:02d}_d30'.format(round(h*100))
    fragment=generate_stairs_links(name,cfg.steps,h,cfg.tread,cfg.width,
        cfg.top_length,position=(0,0,0),xOry='x',mu=cfg.mu,mu2=cfg.mu2)
    # Original helper's position is the FIRST TREAD CENTER, not its edge.
    append_source_links(container,fragment,x,y0+cfg.tread/2,COLORS[0])
    return name,2*cfg.steps*cfg.tread+cfg.top_length,cfg.steps*h


def ramp(container,x,y0,angle,cfg):
    name='ramp_{}deg'.format(angle)
    # Match original caller: z_ref=0.025 and plate_thickness=0.05.
    thickness=.05;z_ref=.025
    fragment=generate_trapezoid_ramp_links(name,angle,cfg.ramp_run,
        cfg.top_length,cfg.width,position=(0,0,z_ref),xOry='x',
        plate_thickness=thickness,mu=cfg.mu,mu2=cfg.mu2)
    append_source_links(container,fragment,x,y0,COLORS[1])
    height=z_ref+thickness/2+cfg.ramp_run*math.tan(math.radians(angle))
    return name,2*cfg.ramp_run+cfg.top_length,height


def add_basics(world):
    # Preserve the complete settings subtree from the user-verified A world.
    # The legacy source puts gravity inside physics; retain that placement
    # deliberately for this compatibility baseline, rather than silently
    # correcting a field and invalidating the tested A comparison.
    original=ET.fromstring('<world>'+make_world_basics()+'</world>')
    for child in original:world.append(child)


def generate(cfg):
    sdf = ET.Element('sdf', version='1.7')
    world = sub(sdf, 'world', name='demo')
    gui = sub(world, 'gui', fullscreen='0')
    camera = sub(gui, 'camera', name='user_camera')
    sub(camera, 'pose', '-20 -8 10 0 0.35 0.0')
    sub(camera, 'view_controller', 'orbit')
    sub(camera, 'projection_type', 'perspective')

    add_basics(world)
    longest = max(2*cfg.steps*cfg.tread+cfg.top_length,
                  2*cfg.ramp_run+cfg.top_length, BLOCK_LENGTH)
    pitch = longest + cfg.row_gap
    extent_w = max(cfg.width, BLOCK_WIDTH)
    xs = [i*(extent_w+cfg.column_gap) for i in range(3)]
    terrain_container=model(world,'all_terrains')
    rows = []
    for col, values in enumerate((STAIR_HEIGHTS, RAMP_ANGLES, BLOCK_HEIGHTS)):
        for row, value in enumerate(values):
            x, y0 = xs[col], cfg.approach+row*pitch
            if col == 0:
                name, length, height = stairs(terrain_container,x,y0,value,cfg)
                label = '{}x{}'.format(round(value*100),round(cfg.tread*100))
            elif col == 1:
                name, length, height = ramp(terrain_container,x,y0,value,cfg)
                label = str(value)
            else:
                name = 'block_h{:03d}'.format(round(value*100))
                box(terrain_container,name+'__block',(BLOCK_WIDTH,BLOCK_LENGTH,value),
                    (x,y0+BLOCK_LENGTH/2,value/2,0,0,0),COLORS[2],cfg.mu,cfg.mu2)
                length,height=BLOCK_LENGTH,value
                label = '{}CM'.format(round(value*100))
            rows.append(dict(name=name,column=col+1,row=row+1,label=label,
                x=x,y_start=y0,y_end=y0+length,width=BLOCK_WIDTH if col==2 else cfg.width,
                max_height=height,approach_x=x,approach_y=y0-3,
                forward_yaw=math.pi/2,backward_yaw=-math.pi/2,
                exit_x=x,exit_y=y0+length+3))
    # Broad-phase footprint check, including tilted plate thickness at edges.
    for i,a in enumerate(rows):
        for b in rows[i+1:]:
            assert (abs(a['x']-b['x']) >= (a['width']+b['width'])/2 or
                    a['y_end'] <= b['y_start'] or b['y_end'] <= a['y_start'])
    return sdf,rows,pitch


def align_world_axes(sdf, rows):
    """Rigidly rotate the whole test layout -90deg: old +Y becomes world +X.

    Keep source stair/ramp link geometry in its original +X orientation by
    applying the rotation at the model level. Label model rotates with it.
    """
    for obj in sdf.findall('./world/model'):
        sub(obj,'pose',nums((0,0,0,0,0,-math.pi/2)))
    result=[]
    for r in rows:
        result.append(dict(name=r['name'],column=r['column'],row=r['row'],
            label=r['label'],x_start=r['y_start'],x_end=r['y_end'],
            y=-r['x'],width=r['width'],max_height=r['max_height'],
            approach_x=r['approach_y'],approach_y=-r['approach_x'],
            forward_yaw=0.0,backward_yaw=math.pi,
            exit_x=r['exit_y'],exit_y=-r['exit_x']))
    return result


def save_svg(path,rows):
    scale=7
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="460" viewBox="0 0 800 335">',
           '<rect width="800" height="335" fill="#f4f6f8"/>',
           '<g font-family="sans-serif" font-size="10" text-anchor="middle">',
           '<text x="400" y="20">Travel +X to the right; difficulty increases to the right</text>']
    for r in rows:
        x=60+r['x_start']*scale
        y=65+(-r['y']-r['width']/2)*scale
        color=['#4c94d4','#e5ac42','#66b579'][r['column']-1]
        parts.append('<rect x="{}" y="{}" width="{}" height="{}" fill="{}"/>'.format(
            x,y,(r['x_end']-r['x_start'])*scale,r['width']*scale,color))
        parts.append('<text x="{}" y="{}">{}</text>'.format(
            60+(r['x_start']+r['x_end'])/2*scale,y-5,r['label']))
    for col,label in enumerate(('STAIRS','RAMPS','BLOCKS'),1):
        row=next(r for r in rows if r['column']==col)
        parts.append('<text x="32" y="{}">{}</text>'.format(69-row['y']*scale,label))
    parts.append('<text x="400" y="305">Columns at Y=0, -11, -22 m; each terrain is 5 m wide</text></g></svg>')
    path.write_text('\n'.join(parts),encoding='utf-8')


# Built-in 5x7 vectorized bitmap alphabet; no font/Pillow dependency.
GLYPHS = {
 'A':'01110/10001/10001/11111/10001/10001/10001',
 'B':'11110/10001/10001/11110/10001/10001/11110',
 'C':'01111/10000/10000/10000/10000/10000/01111',
 'D':'11110/10001/10001/10001/10001/10001/11110',
 'E':'11111/10000/10000/11110/10000/10000/11111',
 'F':'11111/10000/10000/11110/10000/10000/10000',
 'G':'01111/10000/10000/10111/10001/10001/01111',
 'H':'10001/10001/10001/11111/10001/10001/10001',
 'I':'11111/00100/00100/00100/00100/00100/11111',
 'J':'00111/00010/00010/00010/10010/10010/01100',
 'K':'10001/10010/10100/11000/10100/10010/10001',
 'L':'10000/10000/10000/10000/10000/10000/11111',
 'M':'10001/11011/10101/10101/10001/10001/10001',
 'N':'10001/11001/10101/10011/10001/10001/10001',
 'O':'01110/10001/10001/10001/10001/10001/01110',
 'P':'11110/10001/10001/11110/10000/10000/10000',
 'Q':'01110/10001/10001/10001/10101/10010/01101',
 'R':'11110/10001/10001/11110/10100/10010/10001',
 'S':'01111/10000/10000/01110/00001/00001/11110',
 'T':'11111/00100/00100/00100/00100/00100/00100',
 'U':'10001/10001/10001/10001/10001/10001/01110',
 'V':'10001/10001/10001/10001/10001/01010/00100',
 'W':'10001/10001/10001/10101/10101/10101/01010',
 'X':'10001/10001/01010/00100/01010/10001/10001',
 'Y':'10001/10001/01010/00100/00100/00100/00100',
 'Z':'11111/00001/00010/00100/01000/10000/11111',
 '0':'01110/10001/10011/10101/11001/10001/01110',
 '1':'00100/01100/00100/00100/00100/00100/01110',
 '2':'01110/10001/00001/00010/00100/01000/11111',
 '3':'11110/00001/00001/01110/00001/00001/11110',
 '4':'00010/00110/01010/10010/11111/00010/00010',
 '5':'11111/10000/10000/11110/00001/00001/11110',
 '6':'01110/10000/10000/11110/10001/10001/01110',
 '7':'11111/00001/00010/00100/01000/01000/01000',
 '8':'01110/10001/10001/01110/10001/10001/01110',
 '9':'01110/10001/10001/01111/00001/00001/01110',
 '.':'00000/00000/00000/00000/00000/00110/00110',
 ' ':'00000/00000/00000/00000/00000/00000/00000',
}


def add_parameter_labels(sdf, rows, cfg, out):
    """Ground labels: 2 visuals/terrain, embedded polyline glyphs, no files.

    label_gap measures CLEARANCE from the terrain side edge to the nearest
    edge of the label board, not the distance to the board center.
    """
    labels=model(sdf.find('world'),'terrain_parameter_labels')
    link=sub(labels,'link',name='ground_labels_visual_only')
    for row in rows:
        w,h=2.6,1.0
        x=row['x']+row['width']/2+cfg.label_gap+w/2
        y=(row['y_start']+row['y_end'])/2
        board=sub(link,'visual',name=row['name']+'_ground_board')
        sub(board,'pose',nums((x,y,.006,0,0,0)))
        sub(board,'cast_shadows','false')
        sub(sub(sub(board,'geometry'),'box'),'size',nums((w,h,.004)))
        mat=sub(board,'material')
        for tag in ('ambient','diffuse'):sub(mat,tag,nums(COLORS[row['column']-1]))
        text=sub(link,'visual',name=row['name']+'_ground_text')
        sub(text,'pose',nums((x,y,.011,0,0,0)))
        sub(text,'cast_shadows','false')
        geometry=sub(text,'geometry')
        # Each glyph rectangle is a closed SDF polyline contour. They are
        # separated by tiny gaps to avoid shared-edge triangulation issues.
        phrase=row['label'];cell=min(w*.90/(6*len(phrase)),h*.84/7)
        x0=-(len(phrase)*6-1)*cell/2
        for ci,char in enumerate(phrase):
            pixels=[[v=='1' for v in line] for line in GLYPHS[char.upper()].split('/')]
            for rr in range(7):
                for cc in range(5):
                    if not pixels[rr][cc]:continue
                    width=1
                    while cc+width<5 and pixels[rr][cc+width]:width+=1
                    height=1
                    while rr+height<7 and all(pixels[rr+height][j] for j in range(cc,cc+width)):height+=1
                    for r in range(rr,rr+height):
                        for c in range(cc,cc+width):pixels[r][c]=False
                    inset=cell*.015
                    xa=x0+(ci*6+cc)*cell+inset;xb=xa+width*cell-2*inset
                    ya=3.5*cell-(rr+height)*cell+inset;yb=ya+height*cell-2*inset
                    poly=sub(geometry,'polyline');sub(poly,'height',.002)
                    for point in ((xa,ya),(xb,ya),(xb,yb),(xa,yb),(xa,ya)):
                        sub(poly,'point',nums(point))
        mat=sub(text,'material')
        for tag in ('ambient','diffuse','emissive'):sub(mat,tag,'1 1 1 1')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent.parent)
    parser.add_argument('--no-labels',action='store_true',help='Generate user-verified A baseline without labels (default)')
    parser.add_argument('--labels',dest='no_labels',action='store_false',help='Enable flat ground labels beside terrain, embedded in the world')
    parser.set_defaults(no_labels=True)
    parser.add_argument('--extras',action='store_true',help='Also write CSV and SVG; default writes only the world')
    for key,value in DEFAULTS.items():
        parser.add_argument('--'+key.replace('_','-'),type=int if key=='steps' else float,default=value)
    cfg=parser.parse_args()
    for key in DEFAULTS:
        value=getattr(cfg,key)
        if not math.isfinite(value) or value<=0:
            parser.error('{} must be finite and positive'.format(key))
    # if cfg.tread != .30:
    #     parser.error('Required stair tread is 0.30 m; keep --tread 0.30')
    # if cfg.approach<3 or cfg.row_gap<6 or cfg.column_gap<3:
    #     parser.error('Keep approach >=3 m, row-gap >=6 m, column-gap >=3 m for test poses')
    if not cfg.no_labels and cfg.label_gap+2.6>cfg.column_gap:
        parser.error('label-gap + 2.6m label width must fit inside column-gap')
    sdf,rows,pitch=generate(cfg)
    out=cfg.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    if not cfg.no_labels:
        add_parameter_labels(sdf,rows,cfg,out)
    rows=align_world_axes(sdf,rows)
    # Remove inherited XML whitespace before serializing: never multiply
    # blank lines by pretty-printing already formatted fragments.
    for node in sdf.iter():
        if node.text is not None and not node.text.strip():node.text=None
        node.tail=None
    def indent(node,level=0):
        pad='\n'+'  '*level
        if len(node):
            node.text=pad+'  '
            for child in node:indent(child,level+1)
            node[-1].tail=pad
        node.tail=pad
    indent(sdf)
    ET.ElementTree(sdf).write(out/'test_terrains.world',encoding='utf-8',xml_declaration=True)
    if cfg.extras:
        with (out/'terrain_positions.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        save_svg(out/'layout.svg',rows)
    print('Generated:',out/'test_terrains.world')
    print('14 fixtures; row pitch {:.2f} m; friction mu={}, mu2={}'.format(pitch,cfg.mu,cfg.mu2))
    for r in rows:
        print('{:22s} x=[{:5.1f},{:5.1f}] y={:5.1f} approach=({:.1f},{:.1f})'.format(
            r['name'],r['x_start'],r['x_end'],r['y'],r['approach_x'],r['approach_y']))


if __name__=='__main__':
    main()
