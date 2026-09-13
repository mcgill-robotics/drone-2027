"""
Start and stop the Micro XRCE-DDS Agent, the bridge between PX4 and ROS 2.

This replaces boot_px4() / stop_px4() from drone-2026, which launched MAVROS.
Every tool takes the same link flags via add_link_args():

    --sitl              simulator: agent listens on UDP 8888
    --udp [PORT]        Ethernet to the flight controller: agent listens on UDP PORT
    --serial [DEVICE]   serial cable to the flight controller (default /dev/ttyTHS1)
    --baud BAUD         serial baud rate (default 921600)
    --no-agent          an agent is already running; just connect

Plain Python apart from subprocess: no ROS imports.
"""

import shutil
import subprocess
import time

AGENT_BIN = "MicroXRCEAgent"
DEFAULT_UDP_PORT = 8888
DEFAULT_SERIAL_DEVICE = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600

# How the Jetson reaches the flight controller when no link flag is given.
# Not confirmed yet (docs/px4_setup.md, step 0). Set to "serial" or "udp" once known.
HARDWARE_DEFAULT_LINK = None

_agent_process = None


def add_link_args(parser):
    """Add the PX4 link flags to an argparse parser."""
    group = parser.add_argument_group("PX4 link")
    link = group.add_mutually_exclusive_group()
    link.add_argument(
        "--sitl",
        action="store_true",
        help=f"Simulator: agent on UDP {DEFAULT_UDP_PORT}",
    )
    link.add_argument(
        "--udp",
        nargs="?",
        type=int,
        const=DEFAULT_UDP_PORT,
        default=None,
        metavar="PORT",
        help=f"Ethernet link: agent listens on UDP PORT (default {DEFAULT_UDP_PORT})",
    )
    link.add_argument(
        "--serial",
        nargs="?",
        const=DEFAULT_SERIAL_DEVICE,
        default=None,
        metavar="DEVICE",
        help=f"Serial link on DEVICE (default {DEFAULT_SERIAL_DEVICE})",
    )
    group.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default {DEFAULT_BAUD})",
    )
    group.add_argument(
        "--no-agent",
        action="store_true",
        help="Do not start MicroXRCEAgent; assume it is running",
    )
    return parser


def agent_command(args, agent_bin=AGENT_BIN):
    """Build the MicroXRCEAgent command line for parsed link args. Raises ValueError if no link was chosen."""
    if args.sitl:
        return [agent_bin, "udp4", "-p", str(DEFAULT_UDP_PORT)]
    if args.udp is not None:
        return [agent_bin, "udp4", "-p", str(args.udp)]
    if args.serial is not None:
        return [agent_bin, "serial", "--dev", args.serial, "-b", str(args.baud)]
    if HARDWARE_DEFAULT_LINK == "serial":
        return [
            agent_bin,
            "serial",
            "--dev",
            DEFAULT_SERIAL_DEVICE,
            "-b",
            str(args.baud),
        ]
    if HARDWARE_DEFAULT_LINK == "udp":
        return [agent_bin, "udp4", "-p", str(DEFAULT_UDP_PORT)]
    raise ValueError(
        "No PX4 link chosen. Pass --sitl, --udp [PORT] or --serial [DEVICE]. "
        "The drone's link type is not confirmed yet; see docs/px4_setup.md."
    )


def link_flags(args, no_agent=None):
    """The command-line flags that reproduce this link choice, for child processes."""
    flags = []
    if args.sitl:
        flags.append("--sitl")
    elif args.udp is not None:
        flags += ["--udp", str(args.udp)]
    elif args.serial is not None:
        flags += ["--serial", args.serial]
    flags += ["--baud", str(args.baud)]
    if args.no_agent if no_agent is None else no_agent:
        flags.append("--no-agent")
    return flags


def start_agent(cmd, suppress_output=True, settle_s=1.0):
    """Launch the agent as a separate process. Returns the Popen handle, or None if it failed."""
    global _agent_process

    if _agent_process is not None and _agent_process.poll() is None:
        print(f"[AGENT] Already running (PID: {_agent_process.pid})")
        return _agent_process

    if shutil.which(cmd[0]) is None:
        print(
            f"[AGENT] ERROR: {cmd[0]} not found on PATH. Install Micro-XRCE-DDS-Agent (see README)."
        )
        return None

    print(f"[AGENT] Starting: {' '.join(cmd)}")
    output = subprocess.DEVNULL if suppress_output else None
    try:
        _agent_process = subprocess.Popen(cmd, stdout=output, stderr=output)
    except Exception as e:
        print(f"[AGENT] Failed to start: {e}")
        _agent_process = None
        return None

    # The agent binds its port almost immediately; this only catches instant crashes
    # (port in use, missing serial device). Whether PX4 is reachable is checked by
    # PX4Interface.wait_for_connection().
    time.sleep(settle_s)
    if _agent_process.poll() is not None:
        print(f"[AGENT] ERROR: exited with code {_agent_process.poll()}")
        _agent_process = None
        return None

    print(f"[AGENT] Running (PID: {_agent_process.pid})")
    return _agent_process


def start_agent_from_args(args, suppress_output=True):
    """Start the agent for parsed link args. Returns True if an agent is (assumed) running."""
    if args.no_agent:
        return True
    try:
        cmd = agent_command(args)
    except ValueError as e:
        print(f"[AGENT] ERROR: {e}")
        return False
    return start_agent(cmd, suppress_output=suppress_output) is not None


def stop_agent(timeout=5.0):
    """Stop the agent this process started. Returns True if a process was stopped."""
    global _agent_process

    if _agent_process is None:
        return False

    if _agent_process.poll() is None:
        print(f"[AGENT] Stopping (PID: {_agent_process.pid})")
        _agent_process.terminate()
        try:
            _agent_process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            print("[AGENT] Force killing")
            _agent_process.kill()
            _agent_process.wait()

    _agent_process = None
    return True
