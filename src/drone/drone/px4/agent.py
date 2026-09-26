"""
Start and stop MicroXRCEAgent, the program that connects PX4 to ROS 2.

PX4 can't talk to ROS 2 directly. It sends its data over a serial cable or UDP to
MicroXRCEAgent, which republishes it as ROS 2 topics (and passes our orders back).
The agent is a separate program, so this file starts it as a background process.
Replaces boot_px4() / stop_px4() from drone-2026, which started MAVROS instead.

Every tool picks how to reach PX4 with the same flags (see add_link_args()):

    --sitl              simulator: agent listens on UDP 8888
    --udp [PORT]        Ethernet to the flight controller: agent listens on UDP PORT
    --serial [DEVICE]   serial cable to the flight controller (default /dev/ttyTHS1)
    --baud BAUD         serial baud rate (default 921600)
    --no-agent          an agent is already running; just connect

No ROS imports, so the unit tests can run it anywhere.
"""

import shutil
import subprocess
import time

AGENT_BIN = "MicroXRCEAgent"  # the agent program's name on PATH
DEFAULT_UDP_PORT = 8888  # port PX4 SITL sends to by default
# Jetson serial port wired to the flight controller
DEFAULT_SERIAL_DEVICE = "/dev/ttyTHS1"
# Serial speed; must match the port's baud setting in PX4 (docs/px4_setup.md)
DEFAULT_BAUD = 921600

# How the Jetson reaches the flight controller when no link flag is given.
# Not confirmed yet (docs/px4_setup.md, step 0). Set to "serial" or "udp" once known.
HARDWARE_DEFAULT_LINK = None

_agent_process = None  # the agent this process started, so stop_agent() can stop it


def add_link_args(parser):
    """
    Add the flags that choose how to reach PX4 to a command-line parser.

    Pick at most one of --sitl / --udp / --serial. --udp and --serial use their
    defaults when given without a value, and are None when not given at all.
    --baud only matters with --serial.

    This only defines the flags; agent_command() turns them into the command
    that starts the agent.
    """
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
    """
    Turn the parsed flags into the command that starts the agent.

    e.g. --sitl -> ["MicroXRCEAgent", "udp4", "-p", "8888"]
    With no link flag, falls back to HARDWARE_DEFAULT_LINK; if that is not set
    either, raises ValueError.
    """
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


def start_agent(cmd, suppress_output=True, settle_s=1.0):
    """
    Start the agent as a background process.

    Returns the process handle, or None if the agent is not installed or exited
    straight away. Does nothing if this process already started one.
    """
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

    # Wait a moment to catch an agent that crashes straight away (port already in use,
    # serial device missing). This does NOT check PX4 is reachable; init_px4() does that.
    time.sleep(settle_s)
    if _agent_process.poll() is not None:
        print(f"[AGENT] ERROR: exited with code {_agent_process.poll()}")
        _agent_process = None
        return None

    print(f"[AGENT] Running (PID: {_agent_process.pid})")
    return _agent_process


def start_agent_from_args(args, suppress_output=True):
    """
    Start the agent that the flags ask for. Returns True if it is running.

    With --no-agent nothing is started and it just returns True, trusting that an
    agent is already running (e.g. from agent.launch.py).
    """
    if args.no_agent:
        return True
    try:
        cmd = agent_command(args)
    except ValueError as e:
        print(f"[AGENT] ERROR: {e}")
        return False
    return start_agent(cmd, suppress_output=suppress_output) is not None


def stop_agent(timeout=5.0):
    """
    Stop the agent that start_agent() launched. Returns True if there was one.

    Asks it to quit, and force-kills it if it has not quit after `timeout` seconds.
    An agent started some other way (--no-agent) is left alone.
    """
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
