"""Check the desktop view follows the configurable real-robot interfaces."""

from pathlib import Path
from types import SimpleNamespace

import yaml

from pb_vehicle_adapter import rviz_support

PACKAGE = Path(__file__).resolve().parents[1]


def test_rviz_uses_real_clock_and_configured_topics_and_frame(monkeypatch):
    monkeypatch.setattr(rviz_support, 'get_package_share_directory', lambda _: str(PACKAGE))
    monkeypatch.setattr(rviz_support, 'Node', lambda **kw: SimpleNamespace(**kw))
    config = yaml.safe_load((PACKAGE / 'config/interfaces.yaml').read_text())
    config['move_base_flex']['ros__parameters']['global_frame'] = 'site_map'
    config['meshnav_navigator']['ros__parameters']['goal_topic'] = '/task/goal'
    config['meshnav_rviz']['ros__parameters'].update(
        cloud_topic='/lidar/points', obstacle_topic='/obstacles',
        robot_description_topic='/vehicle/description')
    node = rviz_support.make_rviz_node(config)
    assert node.arguments[-2:] == ['-f', 'site_map']
    assert node.parameters == [{'use_sim_time': False}]
    assert dict(node.remappings) == {
        '/rviz/goal_pose': '/task/goal', '/cloud_registered': '/lidar/points',
        '/dynamic_obstacles': '/obstacles', '/robot_description': '/vehicle/description'}
    view = yaml.safe_load(Path(node.arguments[1]).read_text())['Visualization Manager']
    assert any(t['Class'] == 'rviz_mesh_tools_plugins/MeshGoal' for t in view['Tools'])
    assert any(d['Class'] == 'rviz_mesh_tools_plugins/Mesh' for d in view['Displays'])
