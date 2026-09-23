"""Generate isolated physics-only RMUC cases and save true poses as CSV.

Source ROS Humble and meshnav_demo_ws/install/setup.bash before invocation.
Does not change production geometry or publish commands into another simulation.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import subprocess as sp
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT/'meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials_sim'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--y', type=float, default=5.95)
    parser.add_argument('--yaw', type=float, default=0, help='degrees')
    parser.add_argument('--x', type=float, default=-.5)
    parser.add_argument('--z', type=float, default=.015)
    parser.add_argument('--seconds', type=float, default=9)
    parser.add_argument('--speed', type=float, default=.2, help='world +X speed')
    args = parser.parse_args()
    reports = ROOT/'field/converted_rmuc2026/physics_audit'
    reports.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix='slope-', dir=reports))
    (out/'arguments.json').write_text(json.dumps(vars(args), indent=2))
    print(out, flush=True)
    urdf = sp.check_output(['xacro', str(PKG/'urdf/ceres.urdf.xacro'),
        'slope_aware_drive:=true', 'wheel_radius:=0.10', 'body_height:=0.10',
        'body_length:=0.32', 'body_width:=0.26', 'laser2d_mount_z:=0.060',
        'laser3d_mount_z:=0.05', 'laser3d_collision:=false'])
    (out/'robot.urdf').write_bytes(urdf)
    robot = ET.fromstring(sp.check_output(['ign','sdf','-p',str(out/'robot.urdf')])).find('model')
    for link in robot.findall('link'):
        for child in list(link):
            if child.tag in ('visual','sensor'):
                link.remove(child)
    for child in robot.findall('plugin'):
        if 'PosePublisher' not in child.get('name','') and 'SlopeAware' not in child.get('name',''):
            robot.remove(child)
    pose = robot.find('pose')
    if pose is None:
        pose = ET.SubElement(robot,'pose')
    yaw=math.radians(args.yaw)
    pose.text=f'{args.x} {args.y} {args.z} 0 0 {yaw}'
    field = ET.parse(PKG/'models/rmuc2026_field/model.sdf').getroot().find('model')
    for link in field.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    for uri in field.findall('.//uri'):
        uri.text=str(PKG/'models/rmuc2026_field'/uri.text)
    sdf=ET.Element('sdf',version='1.8')
    world=ET.SubElement(sdf,'world',name='slope_probe')
    physics=ET.SubElement(world,'physics',name='1ms',type='ignored')
    ET.SubElement(physics,'max_step_size').text='0.001'
    ET.SubElement(physics,'real_time_factor').text='1.0'
    ET.SubElement(world,'plugin',filename='libignition-gazebo-physics-system.so',name='ignition::gazebo::systems::Physics')
    world.extend([field,robot])
    ET.ElementTree(sdf).write(out/'probe.sdf')
    flags=shlex.split(sp.check_output(['pkg-config','--cflags','--libs','ignition-transport11',
        'ignition-msgs8','sdformat12','ignition-common4','ignition-plugin1'],text=True))
    sp.run(['g++',str(ROOT/'field/slope_probe_drive.cpp'),'-o',str(out/'drive'),
        '-I/usr/include/ignition/gazebo6',*flags,'-lignition-gazebo6'],check=True)
    env=dict(os.environ,IGN_PARTITION=out.name)
    with (out/'poses.csv').open('w') as stdout, (out/'stderr.log').open('w') as stderr:
        result=sp.run([str(out/'drive'),str(out/'probe.sdf'),str(args.speed*math.cos(yaw)),
            str(-args.speed*math.sin(yaw)),str(args.seconds)],env=env,stdout=stdout,stderr=stderr,timeout=60)
    print(f'returncode={result.returncode}; poses={out / "poses.csv"}',flush=True)
    raise SystemExit(result.returncode)

if __name__=='__main__':
    main()
