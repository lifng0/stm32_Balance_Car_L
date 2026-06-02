#!/usr/bin/env python3
import argparse
import struct
import sys
import time

import serial

SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01

CMD_PING = 0x01
CMD_SET_ENABLE = 0x02
CMD_SET_MODE = 0x03
CMD_SET_MOVE = 0x04
CMD_QUERY_STATUS = 0x05
CMD_HEARTBEAT = 0x06
CMD_EMERGENCY_STOP = 0x07

CMD_ACK = 0x81
CMD_NACK = 0x82
CMD_STATUS = 0x83
CMD_EVENT = 0x84
CMD_HEARTBEAT_ACK = 0x85

MODE_NAME = {
    0: "Normal",
    1: "Weight_M",
    2: "K210_QR",
    3: "K210_Line",
    4: "K210_Follow",
    5: "K210_SelfLearn",
    6: "K210_mnist",
}

EVENT_NAME = {
    1: "LOW_POWER",
    2: "POWER_RECOVER",
    4: "TIMEOUT_STOP",
}

ERROR_NAME = {
    1: "CHECKSUM",
    2: "LENGTH",
    3: "ILLEGAL_CMD",
    4: "ILLEGAL_PARAM",
    5: "BUSY_STATE",
}


def calc_checksum(cmd: int, seq: int, payload: bytes) -> int:
    checksum = VERSION ^ cmd ^ seq ^ len(payload)
    for value in payload:
        checksum ^= value
    return checksum & 0xFF


def build_frame(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    checksum = calc_checksum(cmd, seq, payload)
    return bytes([SOF1, SOF2, VERSION, cmd, seq, len(payload)]) + payload + bytes([checksum])


class FrameParser:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.state = "sof1"
        self.cmd = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray()

    def feed(self, value: int):
        if self.state == "sof1":
            if value == SOF1:
                self.state = "sof2"
            return None
        if self.state == "sof2":
            if value == SOF2:
                self.state = "ver"
            else:
                self.reset()
            return None
        if self.state == "ver":
            if value == VERSION:
                self.state = "cmd"
            else:
                self.reset()
            return None
        if self.state == "cmd":
            self.cmd = value
            self.state = "seq"
            return None
        if self.state == "seq":
            self.seq = value
            self.state = "len"
            return None
        if self.state == "len":
            self.length = value
            self.payload = bytearray()
            self.state = "payload" if self.length else "chk"
            return None
        if self.state == "payload":
            self.payload.append(value)
            if len(self.payload) >= self.length:
                self.state = "chk"
            return None
        if self.state == "chk":
            payload = bytes(self.payload)
            frame = {
                "cmd": self.cmd,
                "seq": self.seq,
                "payload": payload,
                "ok": calc_checksum(self.cmd, self.seq, payload) == value,
            }
            self.reset()
            return frame
        self.reset()
        return None


def open_port(device: str, baudrate: int) -> serial.Serial:
    return serial.Serial(device, baudrate=baudrate, timeout=0.1)


def read_frames(ser: serial.Serial, timeout: float):
    parser = FrameParser()
    frames = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = ser.read(64)
        if not data:
            continue
        for value in data:
            frame = parser.feed(value)
            if frame:
                frames.append(frame)
    return frames


def decode_frame(frame) -> str:
    if not frame["ok"]:
        return f"BAD_FRAME seq={frame['seq']} cmd=0x{frame['cmd']:02X}"

    cmd = frame["cmd"]
    payload = frame["payload"]

    if cmd == CMD_ACK and len(payload) >= 2:
        return f"ACK cmd=0x{payload[0]:02X} seq={payload[1]}"

    if cmd == CMD_NACK and len(payload) >= 3:
        return (
            f"NACK cmd=0x{payload[0]:02X} seq={payload[1]} "
            f"err={ERROR_NAME.get(payload[2], payload[2])}"
        )

    if cmd == CMD_STATUS and len(payload) >= 11:
        mode = payload[0]
        stop_flag = payload[1]
        low_power = payload[2]
        move_x = struct.unpack_from("<h", payload, 3)[0] / 10.0
        move_z = struct.unpack_from("<h", payload, 5)[0] / 10.0
        battery = struct.unpack_from("<H", payload, 7)[0] / 100.0
        angle = struct.unpack_from("<h", payload, 9)[0] / 10.0
        return (
            f"STATUS mode={MODE_NAME.get(mode, mode)} stop={stop_flag} "
            f"low_power={low_power} move_x={move_x:.1f} move_z={move_z:.1f} "
            f"battery={battery:.2f} angle={angle:.1f}"
        )

    if cmd == CMD_EVENT and len(payload) >= 1:
        return f"EVENT {EVENT_NAME.get(payload[0], payload[0])}"

    if cmd == CMD_HEARTBEAT_ACK:
        return f"HEARTBEAT_ACK seq={frame['seq']}"

    return f"FRAME cmd=0x{cmd:02X} seq={frame['seq']} payload={payload.hex(' ')}"


def send_and_print(ser: serial.Serial, frame: bytes, timeout: float) -> int:
    ser.write(frame)
    ser.flush()
    frames = read_frames(ser, timeout)
    if not frames:
        print("No response.")
        return 1
    for item in frames:
        print(decode_frame(item))
    return 0


def next_seq(args) -> int:
    return args.seq & 0xFF


def main():
    parser = argparse.ArgumentParser(description="STM32 <-> Raspberry Pi UART bridge tool")
    parser.add_argument("--device", required=True, help="serial device path, for example /dev/serial/by-id/xxx")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=1.0)

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("ping")
    subparsers.add_parser("status")

    enable_parser = subparsers.add_parser("enable")
    enable_parser.add_argument("value", type=int, choices=[0, 1])

    mode_parser = subparsers.add_parser("mode")
    mode_parser.add_argument("value", type=int, choices=range(0, 7))

    move_parser = subparsers.add_parser("move")
    move_parser.add_argument("move_x", type=float)
    move_parser.add_argument("move_z", type=float)

    heartbeat_parser = subparsers.add_parser("heartbeat")
    heartbeat_parser.add_argument("--count", type=int, default=10)
    heartbeat_parser.add_argument("--interval", type=float, default=0.1)

    subparsers.add_parser("stop")
    subparsers.add_parser("listen")

    args = parser.parse_args()

    with open_port(args.device, args.baudrate) as ser:
        seq = next_seq(args)

        if args.command == "ping":
            sys.exit(send_and_print(ser, build_frame(CMD_PING, seq), args.timeout))

        if args.command == "status":
            sys.exit(send_and_print(ser, build_frame(CMD_QUERY_STATUS, seq), args.timeout))

        if args.command == "enable":
            payload = bytes([args.value])
            sys.exit(send_and_print(ser, build_frame(CMD_SET_ENABLE, seq, payload), args.timeout))

        if args.command == "mode":
            payload = bytes([args.value])
            sys.exit(send_and_print(ser, build_frame(CMD_SET_MODE, seq, payload), args.timeout))

        if args.command == "move":
            payload = struct.pack("<hh", int(args.move_x * 10), int(args.move_z * 10))
            sys.exit(send_and_print(ser, build_frame(CMD_SET_MOVE, seq, payload), args.timeout))

        if args.command == "stop":
            sys.exit(send_and_print(ser, build_frame(CMD_EMERGENCY_STOP, seq), args.timeout))

        if args.command == "heartbeat":
            parser_obj = FrameParser()
            current_seq = seq
            for _ in range(args.count):
                ser.write(build_frame(CMD_HEARTBEAT, current_seq))
                ser.flush()
                end_time = time.time() + args.timeout
                while time.time() < end_time:
                    data = ser.read(64)
                    if not data:
                        continue
                    for value in data:
                        frame = parser_obj.feed(value)
                        if frame:
                            print(decode_frame(frame))
                current_seq = (current_seq + 1) & 0xFF
                time.sleep(args.interval)
            return

        if args.command == "listen":
            parser_obj = FrameParser()
            print("Listening... Ctrl+C to exit.")
            try:
                while True:
                    data = ser.read(64)
                    if not data:
                        continue
                    for value in data:
                        frame = parser_obj.feed(value)
                        if frame:
                            print(decode_frame(frame))
            except KeyboardInterrupt:
                return


if __name__ == "__main__":
    main()
