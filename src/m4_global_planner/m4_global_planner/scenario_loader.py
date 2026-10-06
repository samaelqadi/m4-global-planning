from copy import deepcopy
from pathlib import Path

import yaml


def default_scenario_path():
    """Find scenarios in a source checkout or the installed package share."""
    source_path = Path(__file__).parents[1] / 'test_data' / 'scenarios.yaml'
    if source_path.is_file():
        return source_path
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('m4_global_planner')) / 'test_data' / 'scenarios.yaml'


def load_scenario_data(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def load_scenarios(file_path):
    return load_scenario_data(file_path)['scenarios']


def _merge_settings(defaults, overrides):
    result = deepcopy(defaults)

    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge_settings(result[key], value)
        else:
            result[key] = deepcopy(value)

    return result


def get_scenario_parameters(data, scenario_name):
    defaults = data['test_parameters']
    overrides = data['scenarios'][scenario_name].get('overrides', {})
    return _merge_settings(defaults, overrides)
