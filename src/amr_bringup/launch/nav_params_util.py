"""Resolve nav2_params.yaml for a given planner / controller choice.

Writes a plain (alias-free) YAML to /tmp so rcl's YAML parser never sees anchors.
"""
import copy
import os
import tempfile

import yaml

PLANNERS = {'navfn': 'GridBased_navfn', 'smac': 'GridBased_smac', 'astar': 'GridBased_astar'}
CONTROLLERS = {'dwa': 'FollowPath_dwa', 'dwb': 'FollowPath_dwb', 'mppi': 'FollowPath_mppi'}


class _NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def resolve_params(base_file, planner='astar', controller='dwa', map_yaml=None):
    planner, controller = planner.lower(), controller.lower()
    if planner not in PLANNERS:
        raise ValueError(f'planner must be one of {list(PLANNERS)}, got {planner!r}')
    if controller not in CONTROLLERS:
        raise ValueError(f'controller must be one of {list(CONTROLLERS)}, got {controller!r}')

    with open(base_file) as f:
        params = yaml.safe_load(f)

    ps = params['planner_server']['ros__parameters']
    ps['GridBased'] = copy.deepcopy(ps[PLANNERS[planner]])
    for key in PLANNERS.values():
        ps.pop(key, None)

    cs = params['controller_server']['ros__parameters']
    cs['FollowPath'] = copy.deepcopy(cs[CONTROLLERS[controller]])
    for key in CONTROLLERS.values():
        cs.pop(key, None)

    if map_yaml:
        params['map_server']['ros__parameters']['yaml_filename'] = map_yaml

    fd, path = tempfile.mkstemp(prefix=f'nav2_{planner}_{controller}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.dump(params, f, Dumper=_NoAliasDumper, default_flow_style=False, sort_keys=False)
    return path
