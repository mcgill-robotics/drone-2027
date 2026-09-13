#!/usr/bin/env python3
"""
Mission 1 orchestrator: laps, boustrophedon coverage, return home.

Moved from drone-2026 tests/mission1/test_mission1.py. It starts MicroXRCEAgent
once, records the current GPS position as the mission origin, then runs each phase
as its own process (`python -m drone.mission1.lap` / `.boustrophedon`) with
--no-agent so every phase reuses the same agent.

Examples:
    ros2 run drone mission1 --dry-run
    ros2 run drone mission1 --sitl --api-arm
    ros2 run drone mission1 --serial
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from drone.paths import mission1_dir
from drone.px4.agent import add_link_args, link_flags

KEEP_AGENT_RUNNING = False
CURRENT_PHASE = None
SHUTTING_DOWN = False


# load json file from path
def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    tmp_path.replace(path)


def terminate_process(proc, label, timeout=10.0):
    """
    asks a sigint, sigterm and sigkill respectively
    sigint for a safe ask stop, sigterm for a ask to kill,
    sigkill for a forced kill of a prcoess. label is for
    the process
    """
    if proc.poll() is not None:
        return

    print(f"[MISSION1] Stopping {label}...", flush=True)
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    except Exception:
        proc.terminate()

    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.1)

    print(f"[MISSION1] {label} did not stop after SIGINT; sending SIGTERM", flush=True)
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    except Exception:
        proc.terminate()

    deadline = time.time() + 3.0
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.1)

    print(f"[MISSION1] {label} did not stop after SIGTERM; sending SIGKILL", flush=True)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    except Exception:
        proc.kill()


def shutdown_current_phase():
    # kills the current phase process
    global CURRENT_PHASE
    if CURRENT_PHASE is not None and CURRENT_PHASE.poll() is None:
        terminate_process(CURRENT_PHASE, "active mission phase")
    CURRENT_PHASE = None


def handle_shutdown(signum, _frame):
    global SHUTTING_DOWN
    if SHUTTING_DOWN:
        raise KeyboardInterrupt
    SHUTTING_DOWN = True
    print(
        f"\n[MISSION1] Received signal {signum}; shutting down mission orchestration",
        flush=True,
    )
    shutdown_current_phase()
    raise KeyboardInterrupt


def run_phase(module, args):
    # runs a phase module as a subprocess (like laps for example)
    global CURRENT_PHASE
    cmd = [sys.executable, "-m", f"drone.mission1.{module}", *args]
    print(f"[MISSION1] Running: {' '.join(cmd)}", flush=True)
    proc = subprocess.Popen(cmd, start_new_session=True)
    CURRENT_PHASE = proc
    try:
        return proc.wait()
    except KeyboardInterrupt:
        terminate_process(proc, module)
        return 130
    finally:
        if CURRENT_PHASE is proc:
            CURRENT_PHASE = None


def import_movement():
    from drone.mission1 import movement

    return movement


def navigate_or_print(coord, dry_run, label):
    if coord is None:
        print(f"[MISSION1] No {label} coordinate configured")
        return True

    lat = float(coord["lat"])
    lon = float(coord["lon"])
    if dry_run:
        print(f"[MISSION1][DRY] Would navigate to {label}: lat={lat}, lon={lon}")
        return True

    print(f"[MISSION1] Navigating to {label}: lat={lat}, lon={lon}")
    movement = import_movement()
    return bool(movement.navigate_to_coordinate(lat, lon))


# formatting coordinates as command line arguments to be passed in subprocess scripts
def coord_args(prefix, coord):
    return [
        f"--{prefix}-lat",
        str(float(coord["lat"])),
        f"--{prefix}-lon",
        str(float(coord["lon"])),
    ]


def resolve_coord(path, *keys):
    if not path.exists():
        return None
    data = load_json(path)
    for key in keys:
        coord = data.get(key)
        if coord is not None:
            return coord
    return None


def populate_runtime_origin(lap_file, area_file, coord):
    lap_data = load_json(lap_file)
    lap_data["start"] = coord
    lap_data.setdefault("return", coord)
    if lap_data.get("return") is None:
        lap_data["return"] = coord
    save_json(lap_file, lap_data)

    area_data = load_json(area_file)
    boustro = area_data.get("boustrophedon")
    if isinstance(boustro, dict):
        boustro["origin"] = coord
    else:
        area_data["origin"] = coord
    save_json(area_file, area_data)


def parse_args():
    data_dir = mission1_dir()
    parser = argparse.ArgumentParser(
        description="Run Mission 1: laps, boustrophedon, return",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--lap-file", default=str(data_dir / "lap" / "lap_points.json"))
    parser.add_argument(
        "--boustrophedon-area-file",
        default=str(data_dir / "boustrophedon" / "area.json"),
    )
    parser.add_argument(
        "--boustrophedon-state-file",
        default=str(data_dir / "boustrophedon" / "coverage_state.json"),
    )
    parser.add_argument(
        "--boustrophedon-settings-file",
        default=str(data_dir / "boustrophedon" / "settings.yaml"),
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-laps", type=int, default=None)
    parser.add_argument("--regenerate-boustrophedon", action="store_true")
    parser.add_argument("--skip-laps", action="store_true")
    parser.add_argument("--skip-boustrophedon", action="store_true")
    parser.add_argument("--skip-return", action="store_true")
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--skip-setup", action="store_true")
    parser.add_argument(
        "--api-arm",
        action="store_true",
        help="Arm from code (only allowed with --sitl)",
    )
    add_link_args(parser)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.api_arm and not args.sitl:
        print(
            "[MISSION1] --api-arm is only allowed with --sitl. On the drone, the pilot arms with the RC switch."
        )
        return False

    lap_file = Path(args.lap_file).expanduser().resolve()
    boustro_area_file = Path(args.boustrophedon_area_file).expanduser().resolve()
    boustro_state_file = Path(args.boustrophedon_state_file).expanduser().resolve()
    boustro_settings_file = (
        Path(args.boustrophedon_settings_file).expanduser().resolve()
    )

    # Every flying phase uses the same link and reuses the agent started below.
    phase_flags = [] if args.dry_run else link_flags(args, no_agent=True)
    if args.api_arm:
        phase_flags.append("--api-arm")

    global KEEP_AGENT_RUNNING
    if not args.dry_run and not args.skip_setup:
        movement = import_movement()
        if not movement.setup_environment(args, boot=not args.no_agent):
            return False
        origin_coord = movement.current_gps_coordinate()
        if origin_coord is None:
            print("[MISSION1] Could not read current GPS coordinate for mission origin")
            return False

        print(
            f"[MISSION1] Runtime origin: lat={origin_coord['lat']:.7f}, lon={origin_coord['lon']:.7f}"
        )
        populate_runtime_origin(lap_file, boustro_area_file, origin_coord)

    return_coord = resolve_coord(lap_file, "return", "start")
    phase2_start = resolve_coord(lap_file, "after_lap", "transition")
    if phase2_start is None:
        print("[MISSION1] Lap file must define after_lap for phase 2 start")
        return False

    resume_boustrophedon = False
    if boustro_state_file.exists() and not args.regenerate_boustrophedon:
        state = load_json(boustro_state_file)
        path = state.get("path") or []
        resume_boustrophedon = bool(path) and state.get("status") in {
            "running",
            "paused",
            "stopped",
            "failed",
        }

    if resume_boustrophedon:
        print(
            "[MISSION1] Resuming phase 2 from saved boustrophedon state; skipping laps"
        )

    if not args.skip_laps and not resume_boustrophedon:
        lap_args = ["--lap-file", str(lap_file)]
        if args.dry_run:
            lap_args.append("--dry-run")
        if args.max_laps is not None:
            lap_args += ["--max-laps", str(args.max_laps)]
        lap_args += ["--takeoff-altitude", str(args.takeoff_altitude)]
        if run_phase("lap", lap_args + phase_flags) != 0:
            print("[MISSION1] Lap phase failed")
            return False

    if not args.skip_boustrophedon:
        boust_args = [
            "--area-file",
            str(boustro_area_file),
            "--state-file",
            str(boustro_state_file),
            "--settings-file",
            str(boustro_settings_file),
        ]
        boust_args += coord_args("start", phase2_start)
        if return_coord is not None:
            boust_args += coord_args("goal", return_coord)
        if args.dry_run:
            boust_args.append("--dry-run")
        if args.regenerate_boustrophedon:
            boust_args.append("--regenerate")
        boust_args += ["--takeoff-altitude", str(args.takeoff_altitude)]
        boust_code = run_phase("boustrophedon", boust_args + phase_flags)
        if boust_code == 3:
            print(
                "[MISSION1] Boustrophedon paused/stopped; leaving agent running for manual resume"
            )
            KEEP_AGENT_RUNNING = True
            return True
        if boust_code != 0:
            print("[MISSION1] Boustrophedon phase failed")
            return False

    if not args.skip_return and args.skip_boustrophedon:
        if args.dry_run:
            if not navigate_or_print(return_coord, True, "return"):
                return False
        else:
            movement = import_movement()
            if not movement.begin_phase(
                takeoff_altitude=args.takeoff_altitude, api_arm=args.api_arm
            ):
                return False
            if not navigate_or_print(return_coord, False, "return"):
                print("[MISSION1] Return navigation failed")
                return False
            if not movement.end_phase(land=True):
                return False

    print("[MISSION1] Mission 1 orchestration complete")
    return True


def cli():
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)
    try:
        success = main()
    except KeyboardInterrupt:
        print("[MISSION1] Interrupted; cleanup requested", flush=True)
        success = False
    finally:
        shutdown_current_phase()
        try:
            from drone.mission1 import movement

            movement.cleanup(stop_agent_process=not KEEP_AGENT_RUNNING)
        except Exception:
            pass
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(cli())
