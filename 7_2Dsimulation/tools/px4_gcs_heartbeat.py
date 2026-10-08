"""向 PX4 SITL 的 GCS MAVLink 端口发送 MAV_TYPE_GCS 心跳，满足 NAV_DLL_ACT>0 的解锁前置条件。

纯标准库实现（开发机未安装 pymavlink）。用法：python3 tools/px4_gcs_heartbeat.py [端口...]
默认向 18570/18571 持续发送 1 Hz 心跳，Ctrl-C 退出。
"""

from __future__ import annotations

import socket
import struct
import sys
import time

HEARTBEAT_MSGID = 0
CRC_EXTRA = 50


def x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def heartbeat_frame(seq: int, sysid: int = 255, compid: int = 190) -> bytes:
    # HEARTBEAT: custom_mode u32, type u8, autopilot u8, base_mode u8, system_status u8, mavlink_version u8
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)  # type=6 GCS, autopilot=8 INVALID
    header = struct.pack("<BBBBBB", len(payload), 0, 0, seq & 0xFF, sysid, compid)
    msgid = struct.pack("<I", HEARTBEAT_MSGID)[:3]
    crc = x25_crc(header + msgid + payload)
    crc = x25_crc(bytes([CRC_EXTRA]), crc)
    return b"\xfd" + header + msgid + payload + struct.pack("<H", crc)


def main() -> None:
    ports = [int(p) for p in sys.argv[1:]] or [18570, 18571]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    try:
        while True:
            for port in ports:
                sock.sendto(heartbeat_frame(seq), ("127.0.0.1", port))
            seq += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
