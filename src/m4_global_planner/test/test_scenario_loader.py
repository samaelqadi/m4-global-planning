from m4_global_planner.scenario_loader import default_scenario_path, load_scenario_data


def test_default_scenario_file_is_available():
    path = default_scenario_path()
    assert path.is_file()
    assert 'scenarios' in load_scenario_data(path)
