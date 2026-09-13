"""
Filesystem locations shared by the tools.

Everything defaults to folders at the repository root so a fresh clone works
without configuration. Each location can be overridden with an environment
variable, e.g. to keep mission state somewhere else on the Jetson.
"""

import os
from pathlib import Path


def repo_root():
    """The drone-2027 checkout: found by walking up from this file, or DRONE_REPO_ROOT."""
    env = os.environ.get("DRONE_REPO_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / "missions").is_dir() and (parent / "src").is_dir():
            return parent
    return Path.cwd()


def mission1_dir():
    """Mission 1 input and progress files (lap points, coverage area, coverage state)."""
    env = os.environ.get("DRONE_MISSION1_DIR")
    return Path(env).expanduser() if env else repo_root() / "missions" / "mission1"


def runtime_dir():
    """Files written while flying: screenshots, target registry, MediaMTX log."""
    env = os.environ.get("DRONE_RUNTIME_DIR")
    return Path(env).expanduser() if env else repo_root() / "runtime"


def config_dir():
    return repo_root() / "config"
