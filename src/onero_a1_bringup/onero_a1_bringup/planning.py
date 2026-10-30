"""Online RRTConnect configuration without repeated expensive path simplification."""
from copy import deepcopy


def add_online_planner(config):
    ompl = config.planning_pipelines['ompl']
    planner = deepcopy(ompl['planner_configs']['RRTConnect'])
    # OMPL parses these planner-config strings with lexical_cast<bool>.
    # ROS boolean values become "false", which that parser rejects; use "0".
    planner.update(range=.2, simplify_solutions='0', interpolate='0')
    ompl['planner_configs']['RRTConnectOnline'] = planner
    for group in ompl.values():
        if isinstance(group, dict) and isinstance(group.get('planner_configs'), list):
            group['planner_configs'].append('RRTConnectOnline')
