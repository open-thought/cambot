#!/usr/bin/env python3
"""Interactive test program for a 6-DOF Feetech STS3215 servo arm.

Controls:
  1-6        Select motor
  Left/Right Move selected motor by step size
  +/-        Increase/decrease step size
  t          Toggle torque on selected motor
  T          Toggle torque on ALL motors
  h          Move selected motor to center (2048)
  r          Read and refresh all positions
  q / ESC    Quit
"""

import curses

import scservo_sdk as scs

from cambot.servo import (
    MOTOR_NAMES,
    ADDR_TORQUE_ENABLE,
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_TORQUE_LIMIT,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_LOAD,
    ADDR_PRESENT_VOLTAGE,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_CURRENT,
    POS_MIN,
    POS_MAX,
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    decode_sm,
    connect,
)

MOTOR_ID_LIST = sorted(MOTOR_NAMES)
DEFAULT_STEP = 20


def init_sram(packet_handler, port_handler):
    """Set SRAM registers that reset on power-up."""
    for mid in MOTOR_ID_LIST:
        # Acceleration (SRAM addr 41) — 254 for snappy response (resets to 0 on power cycle)
        packet_handler.write1ByteTxRx(port_handler, mid, ADDR_ACCELERATION, 254)
        # Torque limit (SRAM addr 48) — 1000 = 100% (resets on power cycle)
        packet_handler.write2ByteTxRx(port_handler, mid, ADDR_TORQUE_LIMIT, 1000)


def read_position(packet_handler, port_handler, motor_id):
    pos, result, error = packet_handler.read2ByteTxRx(port_handler, motor_id, ADDR_PRESENT_POSITION)
    if result != scs.COMM_SUCCESS:
        return None
    return pos


def read_load(packet_handler, port_handler, motor_id):
    raw, result, error = packet_handler.read2ByteTxRx(port_handler, motor_id, ADDR_PRESENT_LOAD)
    if result != scs.COMM_SUCCESS:
        return None
    return decode_sm(raw, 10)


def read_voltage(packet_handler, port_handler, motor_id):
    val, result, error = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_PRESENT_VOLTAGE)
    if result != scs.COMM_SUCCESS:
        return None
    return val * 0.1


def read_temperature(packet_handler, port_handler, motor_id):
    val, result, error = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_PRESENT_TEMPERATURE)
    if result != scs.COMM_SUCCESS:
        return None
    return val


def read_current(packet_handler, port_handler, motor_id):
    val, result, error = packet_handler.read2ByteTxRx(port_handler, motor_id, ADDR_PRESENT_CURRENT)
    if result != scs.COMM_SUCCESS:
        return None
    return val


def write_position(packet_handler, port_handler, motor_id, position):
    position = max(POS_MIN, min(POS_MAX, position))
    result, error = packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_GOAL_POSITION, position)
    return result == scs.COMM_SUCCESS


def set_torque(packet_handler, port_handler, motor_id, enable):
    result, error = packet_handler.write1ByteTxRx(
        port_handler, motor_id, ADDR_TORQUE_ENABLE, 1 if enable else 0
    )
    return result == scs.COMM_SUCCESS


def read_torque(packet_handler, port_handler, motor_id):
    val, result, error = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE)
    if result != scs.COMM_SUCCESS:
        return None
    return bool(val)


def _curses_main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.timeout(100)  # 100ms refresh
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)

    port_handler, packet_handler = connect(DEFAULT_PORT, DEFAULT_BAUDRATE)
    init_sram(packet_handler, port_handler)

    selected = 0  # index into MOTOR_ID_LIST
    step = DEFAULT_STEP
    positions = {mid: None for mid in MOTOR_ID_LIST}   # actual read-back position
    goals = {mid: None for mid in MOTOR_ID_LIST}        # commanded goal position
    torque_state = {mid: None for mid in MOTOR_ID_LIST}
    loads = {mid: None for mid in MOTOR_ID_LIST}
    voltages = {mid: None for mid in MOTOR_ID_LIST}
    temperatures = {mid: None for mid in MOTOR_ID_LIST}
    currents = {mid: None for mid in MOTOR_ID_LIST}
    status_msg = ""

    # Initial read
    for mid in MOTOR_ID_LIST:
        positions[mid] = read_position(packet_handler, port_handler, mid)
        goals[mid] = positions[mid]
        torque_state[mid] = read_torque(packet_handler, port_handler, mid)

    try:
        while True:
            stdscr.erase()
            h, w = stdscr.getmaxyx()

            # Title
            title = "=== 6-DOF Feetech Servo Test ==="
            stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD | curses.color_pair(3))

            # Motor table header
            hdr = f"{'#':<4}{'Name':<16}{'Pos':>6}{'Torq':>6}{'Load':>6}{'mA':>6}{'V':>6}{'C':>5}"
            stdscr.addstr(2, 2, hdr, curses.A_BOLD)
            stdscr.addstr(3, 2, "-" * len(hdr))

            # Motor rows
            for i, mid in enumerate(MOTOR_ID_LIST):
                pos = positions[mid]
                torque = torque_state[mid]
                load = loads[mid]
                voltage = voltages[mid]
                temp = temperatures[mid]
                current = currents[mid]

                pos_str = str(pos) if pos is not None else "ERR"
                torque_str = "ON" if torque else "OFF" if torque is not None else "?"
                load_str = f"{load:+d}" if load is not None else "?"
                current_str = str(current) if current is not None else "?"
                volt_str = f"{voltage:.1f}" if voltage is not None else "?"
                temp_str = str(temp) if temp is not None else "?"

                attr = curses.A_BOLD | curses.color_pair(1) if i == selected else 0
                marker = ">" if i == selected else " "

                row = f"{marker}{mid:<3}{MOTOR_NAMES[mid]:<16}{pos_str:>6}{torque_str:>6}{load_str:>6}{current_str:>6}{volt_str:>6}{temp_str:>5}"
                stdscr.addstr(4 + i, 2, row, attr)

            # Voltage warning
            any_voltage = next((v for v in voltages.values() if v is not None), None)
            info_y = 4 + len(MOTOR_ID_LIST) + 1
            if any_voltage is not None and any_voltage < 6.0:
                warn = f"WARNING: Voltage {any_voltage:.1f}V is below 6.0V! Servos need 6-7.4V for full torque."
                stdscr.addstr(info_y, 2, warn, curses.A_BOLD | curses.color_pair(4))
                info_y += 1

            # Info
            stdscr.addstr(info_y, 2, f"Selected: motor {MOTOR_ID_LIST[selected]} ({MOTOR_NAMES[MOTOR_ID_LIST[selected]]})", curses.color_pair(2))
            stdscr.addstr(info_y + 1, 2, f"Step size: {step}", curses.color_pair(2))

            # Controls
            ctrl_y = info_y + 3
            stdscr.addstr(ctrl_y, 2, "Controls:", curses.A_BOLD | curses.color_pair(3))
            controls = [
                "1-6: Select motor    Left/Right: Move    +/-: Step size",
                "t: Toggle torque     T: All torque       h: Go to center",
                "r: Refresh           q/ESC: Quit",
            ]
            for j, line in enumerate(controls):
                stdscr.addstr(ctrl_y + 1 + j, 4, line)

            # Status
            if status_msg:
                stdscr.addstr(ctrl_y + 5, 2, status_msg, curses.color_pair(4))

            stdscr.refresh()

            # Read all telemetry continuously
            for mid in MOTOR_ID_LIST:
                p = read_position(packet_handler, port_handler, mid)
                if p is not None:
                    positions[mid] = p
                loads[mid] = read_load(packet_handler, port_handler, mid)
                currents[mid] = read_current(packet_handler, port_handler, mid)
            # Voltage/temp less often (only selected motor to save bus time)
            sel_mid = MOTOR_ID_LIST[selected]
            voltages[sel_mid] = read_voltage(packet_handler, port_handler, sel_mid)
            temperatures[sel_mid] = read_temperature(packet_handler, port_handler, sel_mid)

            # Handle input
            key = stdscr.getch()
            if key == -1:
                continue

            status_msg = ""
            mid = MOTOR_ID_LIST[selected]

            if key in (ord('q'), ord('Q'), 27):  # q or ESC
                break
            elif ord('1') <= key <= ord('6'):
                selected = key - ord('1')
            elif key == curses.KEY_UP:
                selected = (selected - 1) % len(MOTOR_ID_LIST)
            elif key == curses.KEY_DOWN:
                selected = (selected + 1) % len(MOTOR_ID_LIST)
            elif key == curses.KEY_RIGHT:
                goal = goals[mid] if goals[mid] is not None else positions[mid]
                if goal is not None:
                    new_pos = min(POS_MAX, goal + step)
                    if write_position(packet_handler, port_handler, mid, new_pos):
                        goals[mid] = new_pos
                        status_msg = f"Motor {mid} -> {new_pos}"
                    else:
                        status_msg = f"Failed to write motor {mid}"
            elif key == curses.KEY_LEFT:
                goal = goals[mid] if goals[mid] is not None else positions[mid]
                if goal is not None:
                    new_pos = max(POS_MIN, goal - step)
                    if write_position(packet_handler, port_handler, mid, new_pos):
                        goals[mid] = new_pos
                        status_msg = f"Motor {mid} -> {new_pos}"
                    else:
                        status_msg = f"Failed to write motor {mid}"
            elif key in (ord('+'), ord('=')):
                step = min(200, step + 5)
            elif key in (ord('-'), ord('_')):
                step = max(1, step - 5)
            elif key == ord('t'):
                cur = torque_state[mid]
                new_val = not cur if cur is not None else True
                if set_torque(packet_handler, port_handler, mid, new_val):
                    torque_state[mid] = new_val
                    status_msg = f"Motor {mid} torque {'ON' if new_val else 'OFF'}"
                else:
                    status_msg = f"Failed to set torque on motor {mid}"
            elif key == ord('T'):
                # Toggle all - if any are on, turn all off; otherwise turn all on
                any_on = any(v for v in torque_state.values() if v)
                new_val = not any_on
                for m in MOTOR_ID_LIST:
                    set_torque(packet_handler, port_handler, m, new_val)
                    torque_state[m] = new_val
                status_msg = f"All torque {'ON' if new_val else 'OFF'}"
            elif key == ord('h'):
                center = 2048
                if write_position(packet_handler, port_handler, mid, center):
                    goals[mid] = center
                    status_msg = f"Motor {mid} -> center (2048)"
            elif key == ord('r'):
                for m in MOTOR_ID_LIST:
                    positions[m] = read_position(packet_handler, port_handler, m)
                    torque_state[m] = read_torque(packet_handler, port_handler, m)
                    voltages[m] = read_voltage(packet_handler, port_handler, m)
                    temperatures[m] = read_temperature(packet_handler, port_handler, m)
                status_msg = "Refreshed all"

    finally:
        # Disable torque on exit
        for mid in MOTOR_ID_LIST:
            set_torque(packet_handler, port_handler, mid, False)
        port_handler.closePort()


def main():
    curses.wrapper(_curses_main)


if __name__ == "__main__":
    main()
