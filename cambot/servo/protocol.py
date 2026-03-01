"""Low-level Feetech STS3215 protocol helpers.

Sign-magnitude encoding/decoding, serial connection, EPROM write helpers.
"""

from __future__ import annotations

import sys
import time

import scservo_sdk as scs

from .constants import (
    PROTOCOL_VERSION,
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    ADDR_TORQUE_ENABLE,
    ADDR_LOCK,
)


def decode_sm(raw: int, sign_bit: int) -> int:
    """Decode a sign-magnitude register value to a signed integer."""
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if (raw >> sign_bit) & 1 else magnitude


def encode_sm(value: int, sign_bit: int) -> int:
    """Encode a signed integer to sign-magnitude register value."""
    if value >= 0:
        return value & ((1 << sign_bit) - 1)
    return (abs(value) & ((1 << sign_bit) - 1)) | (1 << sign_bit)


def connect(
    port: str = DEFAULT_PORT,
    baudrate: int = DEFAULT_BAUDRATE,
    exit_on_fail: bool = True,
) -> tuple[scs.PortHandler, scs.PacketHandler]:
    """Open serial port and return (port_handler, packet_handler).

    When exit_on_fail is True (default for CLI scripts), calls sys.exit(1)
    on failure.  When False, raises RuntimeError instead.
    """
    ph = scs.PortHandler(port)
    pkt = scs.PacketHandler(PROTOCOL_VERSION)
    if not ph.openPort():
        msg = f"Cannot open {port}"
        if exit_on_fail:
            print(f"ERROR: {msg}")
            sys.exit(1)
        raise RuntimeError(msg)
    if not ph.setBaudRate(baudrate):
        msg = f"Cannot set baud rate {baudrate}"
        if exit_on_fail:
            print(f"ERROR: {msg}")
            sys.exit(1)
        raise RuntimeError(msg)
    return ph, pkt


def flush_serial(ph: scs.PortHandler) -> None:
    """Flush stale data from the serial buffer."""
    if hasattr(ph, "ser") and ph.ser is not None:
        ph.ser.reset_input_buffer()
    time.sleep(0.02)


def unlock_eprom(ph: scs.PortHandler, pkt: scs.PacketHandler, mid: int) -> None:
    """Disable torque and unlock EPROM for writing."""
    flush_serial(ph)
    pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.05)
    flush_serial(ph)
    pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 0)
    time.sleep(0.05)


def lock_eprom(ph: scs.PortHandler, pkt: scs.PacketHandler, mid: int) -> None:
    """Lock EPROM after writing."""
    time.sleep(0.01)
    pkt.write1ByteTxRx(ph, mid, ADDR_LOCK, 1)


def write_eprom_u16(
    ph: scs.PortHandler, pkt: scs.PacketHandler, mid: int, addr: int, value: int
) -> bool:
    """Write a 2-byte EPROM register with unlock/lock and read-back verify."""
    unlock_eprom(ph, pkt, mid)
    flush_serial(ph)
    pkt.write2ByteTxRx(ph, mid, addr, value)
    time.sleep(0.05)
    lock_eprom(ph, pkt, mid)
    time.sleep(0.05)
    # Verify by reading back
    flush_serial(ph)
    readback, res, _ = pkt.read2ByteTxRx(ph, mid, addr)
    if res != scs.COMM_SUCCESS:
        print(f"  DEBUG: read-back failed for addr={addr}")
        return False
    if readback != value:
        print(f"  DEBUG: addr={addr} expected {value} but read {readback}")
        return False
    return True
