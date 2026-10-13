"""Check launch composition and parameter precedence without starting processes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'real_launch', PACKAGE / 'launch' / 'meshnav_real.launch.py')
real_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(real_launch)


def run_launch(monkeypatch, interfaces, navigation=''):
    monkeypatch.setattr(real_launch, 'Node', lambda **kw: SimpleNamespace(**kw))
    context = LaunchContext()
    context.launch_configurations.update(
        interfaces_file=str(interfaces), navigation_params_file=str(navigation))
    return [a for a in real_launch._launch_nodes(context) if isinstance(a, SimpleNamespace)]


def test_interfaces_only_have_two_real_nodes_and_wall_time(monkeypatch):
    nodes = run_launch(monkeypatch, PACKAGE / 'config' / 'interfaces.yaml')
    assert [n.executable for n in nodes] == ['pb_cmd_vel_adapter', 'meshnav_navigator']
    assert all(n.parameters[-1] == {'use_sim_time': False} for n in nodes)


def test_mbf_uses_external_map_config_and_remaps_custom_velocity_topic(monkeypatch, tmp_path):
    config = yaml.safe_load((PACKAGE / 'config' / 'interfaces.yaml').read_text())
    config['pb_cmd_vel_adapter']['ros__parameters']['input_topic'] = '/custom/nav_velocity'
    interfaces = tmp_path / 'interfaces.yaml'
    interfaces.write_text(yaml.safe_dump(config))
    navigation = tmp_path / 'navigation.yaml'
    navigation.write_text(yaml.safe_dump({'move_base_flex': {'ros__parameters': {
        'use_sim_time': True, 'planners': ['mesh_planner'], 'controllers': ['mesh_controller'],
    }}}))
    nodes = run_launch(monkeypatch, interfaces, navigation)
    mbf = nodes[0]
    assert mbf.package == 'mbf_mesh_nav'
    assert mbf.parameters == [str(navigation), str(interfaces), {'use_sim_time': False}]
    assert mbf.remappings == [('/move_base_flex/cmd_vel', '/custom/nav_velocity')]


@pytest.mark.parametrize('mismatch', ['frame', 'controller'])
def test_bad_mbf_interface_is_rejected_before_creating_nodes(monkeypatch, tmp_path, mismatch):
    config = yaml.safe_load((PACKAGE / 'config' / 'interfaces.yaml').read_text())
    if mismatch == 'frame':
        config['move_base_flex']['ros__parameters']['robot_frame'] = 'different_base'
    else:
        config['meshnav_navigator']['ros__parameters']['controller'] = 'missing_plugin'
    interfaces = tmp_path / 'interfaces.yaml'
    interfaces.write_text(yaml.safe_dump(config))
    navigation = tmp_path / 'navigation.yaml'
    navigation.write_text(yaml.safe_dump({'move_base_flex': {'ros__parameters': {
        'planners': ['mesh_planner'], 'controllers': ['mesh_controller'],
    }}}))
    with pytest.raises(ValueError):
        run_launch(monkeypatch, interfaces, navigation)
