"""Online RRTConnect configuration without repeated expensive path simplification."""
from copy import deepcopy
import math


def add_online_planner(config):
    ompl = config.planning_pipelines['ompl']
    extension = ompl.pop('rrtconnect_range', None)
    if extension is not None:
        extension = float(extension)
        if not math.isfinite(extension) or extension < 0.:
            raise ValueError('rrtconnect_range 必须为非负有限数值；0 使用 OMPL 自动步长')
        ompl['planner_configs']['RRTConnect']['range'] = extension
    planner = deepcopy(ompl['planner_configs']['RRTConnect'])
    # OMPL parses these planner-config strings with lexical_cast<bool>.
    # ROS boolean values become "false", which that parser rejects; use "0".
    planner.update(range=.2, simplify_solutions='0', interpolate='0')
    ompl['planner_configs']['RRTConnectOnline'] = planner
    for group in ompl.values():
        if isinstance(group, dict) and isinstance(group.get('planner_configs'), list):
            group['planner_configs'].append('RRTConnectOnline')
