"""Minimal rospkg stub resolving package paths to the local checkout."""

import os

# repo root = three levels up from this file (local_eval/harness/stubs/rospkg.py)
_REPO_ROOT = os.environ.get(
    'MRIM_REPO_ROOT',
    os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')))

_TASK_ROOT = os.path.join(_REPO_ROOT, 'mrim_task')

_PACKAGES = {
    'mrim_resources': os.path.join(_TASK_ROOT, 'mrim_resources'),
    'mrim_planner':   os.path.join(_TASK_ROOT, 'mrim_planner'),
    'mrim_manager':   os.path.join(_TASK_ROOT, 'mrim_manager'),
}


class RosPack:
    def get_path(self, package):
        return _PACKAGES[package]
