#!/usr/bin/env python3
"""Record-and-replay waypoint navigation for a 6-DOF Feetech STS3215 arm.

Usage:
  python -m cambot.tools.waypoint_nav record [-o FILE]
  python -m cambot.tools.waypoint_nav replay FILE [--speed FACTOR] [--interp linear|smooth] [--loop] [--seg-time SEC]

Record mode:
  Disables torque so you can manually position the arm.
  SPACE  Capture waypoint
  d      Delete last waypoint
  s      Save and exit
  q/ESC  Quit (prompts if unsaved)

Replay mode:
  Loads waypoints from JSON, interpolates between them at 50 Hz.
  q/ESC  Abort (disables torque)
"""

import argparse
import curses
import json
import sys
import time

import scservo_sdk as scs

from cambot.servo import (
    MOTOR_NAMES,
    ADDR_TORQUE_ENABLE,
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_TORQUE_LIMIT,
    ADDR_PRESENT_POSITION,
    POS_MIN,
    POS_MAX,
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    connect,
)

MOTOR_ID_LIST = sorted(MOTOR_NAMES)
COMMAND_RATE = 50  # Hz


# --- Low-level helpers ---

def read_position(pkt, ph, motor_id):
    pos, result, error = pkt.read2ByteTxRx(ph, motor_id, ADDR_PRESENT_POSITION)
    if result != scs.COMM_SUCCESS:
        return None
    return pos


def write_position(pkt, ph, motor_id, position):
    position = max(POS_MIN, min(POS_MAX, int(round(position))))
    result, error = pkt.write2ByteTxRx(ph, motor_id, ADDR_GOAL_POSITION, position)
    return result == scs.COMM_SUCCESS


def set_torque(pkt, ph, motor_id, enable):
    result, error = pkt.write1ByteTxRx(ph, motor_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)
    return result == scs.COMM_SUCCESS


def disable_all_torque(pkt, ph):
    for mid in MOTOR_ID_LIST:
        set_torque(pkt, ph, mid, False)


def init_sram(pkt, ph):
    for mid in MOTOR_ID_LIST:
        pkt.write1ByteTxRx(ph, mid, ADDR_ACCELERATION, 254)
        pkt.write2ByteTxRx(ph, mid, ADDR_TORQUE_LIMIT, 1000)


def read_all_positions(pkt, ph):
    """Read positions for all motors. Returns list of ints (None on error)."""
    return [read_position(pkt, ph, mid) for mid in MOTOR_ID_LIST]


# --- Interpolation ---

def lerp(p0, p1, t):
    """Linear interpolation between two position lists."""
    return [a + (b - a) * t for a, b in zip(p0, p1)]


def min_jerk(p0, p1, t):
    """Minimum-jerk (smooth) interpolation. Zero velocity/accel at boundaries."""
    s = 10 * t**3 - 15 * t**4 + 6 * t**5
    return [a + (b - a) * s for a, b in zip(p0, p1)]


INTERP_FUNCS = {
    "linear": lerp,
    "smooth": min_jerk,
}


# --- File I/O ---

def save_waypoints(waypoints, filename):
    data = {
        "motor_ids": MOTOR_ID_LIST,
        "motor_names": {str(k): v for k, v in MOTOR_NAMES.items()},
        "waypoints": [{"positions": wp} for wp in waypoints],
    }
    with open(filename, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_waypoints(filename):
    with open(filename) as f:
        data = json.load(f)
    return [wp["positions"] for wp in data["waypoints"]]


# --- Record mode (curses TUI) ---

def record_mode(stdscr, pkt, ph, output_file):
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.timeout(100)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)

    # Disable torque so user can move arm by hand
    disable_all_torque(pkt, ph)

    waypoints = []
    status_msg = ""
    saved = False

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        # Title
        title = "=== Waypoint Recorder ==="
        stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD | curses.color_pair(3))

        # Read current positions
        positions = read_all_positions(pkt, ph)

        # Motor table
        hdr = f"{'#':<4}{'Name':<16}{'Position':>8}"
        stdscr.addstr(2, 2, hdr, curses.A_BOLD)
        stdscr.addstr(3, 2, "-" * len(hdr))
        for i, mid in enumerate(MOTOR_ID_LIST):
            pos = positions[i]
            pos_str = str(pos) if pos is not None else "ERR"
            row = f"{mid:<4}{MOTOR_NAMES[mid]:<16}{pos_str:>8}"
            stdscr.addstr(4 + i, 2, row)

        # Waypoint list
        wp_y = 4 + len(MOTOR_ID_LIST) + 1
        wp_label = f"Waypoints: {len(waypoints)}"
        stdscr.addstr(wp_y, 2, wp_label, curses.A_BOLD | curses.color_pair(2))

        # Show last few waypoints that fit on screen
        max_display = max(0, h - wp_y - 8)
        start_idx = max(0, len(waypoints) - max_display)
        for j, idx in enumerate(range(start_idx, len(waypoints))):
            wp = waypoints[idx]
            wp_str = f"  [{idx:>3}] {wp}"
            if wp_y + 1 + j < h - 6:
                stdscr.addstr(wp_y + 1 + j, 2, wp_str[:w - 4])

        # Controls
        ctrl_y = h - 5
        stdscr.addstr(ctrl_y, 2, "Controls:", curses.A_BOLD | curses.color_pair(3))
        stdscr.addstr(ctrl_y + 1, 4, "SPACE: Capture waypoint   d: Delete last   s: Save & exit   q/ESC: Quit")

        # Status
        if status_msg:
            stdscr.addstr(ctrl_y + 3, 2, status_msg, curses.color_pair(4))

        stdscr.refresh()

        key = stdscr.getch()
        if key == -1:
            continue

        status_msg = ""

        if key == ord(" "):
            if any(p is None for p in positions):
                status_msg = "Cannot capture: some motors not responding"
            else:
                waypoints.append(list(positions))
                saved = False
                status_msg = f"Captured waypoint {len(waypoints) - 1}"

        elif key == ord("d"):
            if waypoints:
                waypoints.pop()
                saved = False
                status_msg = "Deleted last waypoint"
            else:
                status_msg = "No waypoints to delete"

        elif key == ord("s"):
            if not waypoints:
                status_msg = "No waypoints to save"
            else:
                save_waypoints(waypoints, output_file)
                saved = True
                status_msg = f"Saved {len(waypoints)} waypoints to {output_file}"
                # Brief pause so user sees the message, then exit
                stdscr.refresh()
                time.sleep(0.8)
                return

        elif key in (ord("q"), ord("Q"), 27):
            if waypoints and not saved:
                status_msg = "Unsaved waypoints! Press 's' to save or 'q' again to discard."
                stdscr.addstr(ctrl_y + 3, 2, status_msg, curses.A_BOLD | curses.color_pair(4))
                stdscr.refresh()
                # Wait for confirmation
                while True:
                    k2 = stdscr.getch()
                    if k2 == ord("s"):
                        save_waypoints(waypoints, output_file)
                        return
                    elif k2 in (ord("q"), ord("Q"), 27):
                        return
                    elif k2 != -1:
                        break  # go back to main loop
            else:
                return


# --- Replay mode ---

def replay_mode_curses(stdscr, pkt, ph, waypoints, seg_time, speed, interp, loop):
    """Curses wrapper for replay so we can catch q/ESC."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)

    interp_fn = INTERP_FUNCS[interp]
    effective_seg_time = seg_time / speed
    dt = 1.0 / COMMAND_RATE
    num_segments = len(waypoints) - 1

    # Enable torque and init SRAM
    init_sram(pkt, ph)
    for mid in MOTOR_ID_LIST:
        set_torque(pkt, ph, mid, True)

    # Move to first waypoint before starting timed playback
    for i, mid in enumerate(MOTOR_ID_LIST):
        write_position(pkt, ph, mid, waypoints[0][i])
    time.sleep(0.5)

    running = True
    pass_num = 0

    while running:
        pass_num += 1
        for seg in range(num_segments):
            p0 = waypoints[seg]
            p1 = waypoints[seg + 1]
            seg_start = time.monotonic()

            while True:
                elapsed = time.monotonic() - seg_start
                t = elapsed / effective_seg_time
                if t >= 1.0:
                    # Write final position of this segment
                    for i, mid in enumerate(MOTOR_ID_LIST):
                        write_position(pkt, ph, mid, p1[i])
                    break

                positions = interp_fn(p0, p1, t)
                for i, mid in enumerate(MOTOR_ID_LIST):
                    write_position(pkt, ph, mid, positions[i])

                # Display
                stdscr.erase()
                h, w = stdscr.getmaxyx()
                title = "=== Waypoint Replay ==="
                stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD | curses.color_pair(3))

                info = f"Segment {seg + 1}/{num_segments}  t={t:.2f}  Pass {pass_num}  Interp: {interp}"
                stdscr.addstr(2, 2, info, curses.color_pair(2))

                hdr = f"{'#':<4}{'Name':<16}{'Target':>8}{'Current':>8}"
                stdscr.addstr(4, 2, hdr, curses.A_BOLD)
                stdscr.addstr(5, 2, "-" * len(hdr))
                for i, mid in enumerate(MOTOR_ID_LIST):
                    target = int(round(positions[i]))
                    cur = read_position(pkt, ph, mid)
                    cur_str = str(cur) if cur is not None else "?"
                    row = f"{mid:<4}{MOTOR_NAMES[mid]:<16}{target:>8}{cur_str:>8}"
                    stdscr.addstr(6 + i, 2, row)

                ctrl_y = 6 + len(MOTOR_ID_LIST) + 1
                if loop:
                    stdscr.addstr(ctrl_y, 2, "Looping enabled", curses.color_pair(1))
                stdscr.addstr(ctrl_y + 1, 2, f"Seg time: {effective_seg_time:.2f}s  Speed: {speed:.1f}x",
                              curses.color_pair(2))
                stdscr.addstr(ctrl_y + 3, 2, "Press q/ESC to abort", curses.color_pair(4))
                stdscr.refresh()

                # Check for abort
                key = stdscr.getch()
                if key in (ord("q"), ord("Q"), 27):
                    return

                # Sleep remainder of tick
                tick_end = time.monotonic()
                sleep_time = dt - (tick_end - (seg_start + elapsed))
                if sleep_time > 0:
                    time.sleep(sleep_time)

        if not loop:
            # Hold final position, wait for q
            while True:
                stdscr.erase()
                h, w = stdscr.getmaxyx()
                title = "=== Replay Complete ==="
                stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD | curses.color_pair(1))
                stdscr.addstr(2, 2, f"Holding final position. Pass {pass_num}.", curses.color_pair(2))
                stdscr.addstr(4, 2, "Press q/ESC to exit", curses.color_pair(4))
                stdscr.refresh()
                key = stdscr.getch()
                if key in (ord("q"), ord("Q"), 27):
                    return
                time.sleep(0.1)


# --- CLI ---

def main():
    parser = argparse.ArgumentParser(description="Waypoint navigation for StereoBot arm")
    sub = parser.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="Record waypoints by manually positioning the arm")
    rec.add_argument("-o", "--output", default="waypoints.json", help="Output file (default: waypoints.json)")

    rep = sub.add_parser("replay", help="Replay waypoints from a JSON file")
    rep.add_argument("file", help="Waypoints JSON file")
    rep.add_argument("--speed", type=float, default=1.0, help="Speed multiplier (default: 1.0)")
    rep.add_argument("--interp", choices=["linear", "smooth"], default="linear",
                     help="Interpolation method (default: linear)")
    rep.add_argument("--loop", action="store_true", help="Loop playback")
    rep.add_argument("--seg-time", type=float, default=2.0, help="Seconds per segment (default: 2.0)")

    args = parser.parse_args()

    ph, pkt = connect(DEFAULT_PORT, DEFAULT_BAUDRATE)

    try:
        if args.command == "record":
            curses.wrapper(lambda stdscr: record_mode(stdscr, pkt, ph, args.output))
            print(f"Done. Output: {args.output}")

        elif args.command == "replay":
            waypoints = load_waypoints(args.file)
            if len(waypoints) < 2:
                print("Need at least 2 waypoints for replay.")
                sys.exit(1)
            print(f"Loaded {len(waypoints)} waypoints from {args.file}")
            curses.wrapper(lambda stdscr: replay_mode_curses(
                stdscr, pkt, ph, waypoints, args.seg_time, args.speed, args.interp, args.loop
            ))
    finally:
        disable_all_torque(pkt, ph)
        ph.closePort()


if __name__ == "__main__":
    main()
