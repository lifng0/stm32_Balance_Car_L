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
CMD_SET_HOST_STATE = 0x08
CMD_QUERY_VISION = 0x09
CMD_K210_TEXT = 0x0A
CMD_K210_RAW = 0x0B

CMD_ACK = 0x81
CMD_NACK = 0x82
CMD_STATUS = 0x83
CMD_EVENT = 0x84
CMD_HEARTBEAT_ACK = 0x85
CMD_VISION_STATUS = 0x86

VISION_TYPE_NAME = {
    0: "NONE",
    1: "TEXT",
    2: "AI",
}

MODE_NAME = {
    0: "Normal",
    1: "Weight_M",
    3: "K210_Line",
    4: "K210_Follow",
    7: "Lidar_Avoid",
    8: "Lidar_Follow",
}

EVENT_NAME = {
    1: "LOW_POWER",
    2: "POWER_RECOVER",
    4: "TIMEOUT_STOP",
    0x10: "START_REQUEST",
    0x11: "MODE_SELECT",
    0x12: "STOP_ASSERT",
    0x13: "STOP_CLEAR",
    0x14: "SHUTDOWN_REQUEST",
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
    handle = serial.Serial()
    handle.port = device
    handle.baudrate = baudrate
    handle.timeout = 0.1
    handle.rtscts = False
    handle.dsrdtr = False
    # Keep USB-TTL control lines inactive; on some adapters DTR/RTS can reset
    # the attached STM32 or put the link into an inconsistent state.
    handle.dtr = False
    handle.rts = False
    handle.open()
    handle.dtr = False
    handle.rts = False
    return handle


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

    if cmd == CMD_VISION_STATUS and len(payload) >= 3:
        vision_type = payload[0]
        mode = payload[1]
        valid = payload[2]
        if vision_type == 1 and len(payload) >= 4:
            text_len = payload[3]
            text = payload[4 : 4 + text_len].decode("utf-8", errors="replace")
            return (
                f"VISION_STATUS type={VISION_TYPE_NAME.get(vision_type, vision_type)} "
                f"mode={MODE_NAME.get(mode, mode)} valid={valid} text={text!r}"
            )
        if vision_type == 2 and len(payload) >= 13:
            x, y, w, h, area = struct.unpack_from("<HHHHH", payload, 3)
            return (
                f"VISION_STATUS type={VISION_TYPE_NAME.get(vision_type, vision_type)} "
                f"mode={MODE_NAME.get(mode, mode)} valid={valid} "
                f"x={x} y={y} w={w} h={h} area={area}"
            )
        return (
            f"VISION_STATUS type={VISION_TYPE_NAME.get(vision_type, vision_type)} "
            f"mode={MODE_NAME.get(mode, mode)} valid={valid}"
        )

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
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=1.0)

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("ping")
    subparsers.add_parser("status")
    subparsers.add_parser("vision-status")
    k210_text_parser = subparsers.add_parser("k210-text")
    k210_text_parser.add_argument("text", help="ASCII/UTF-8 text forwarded to K210 UART2")

    enable_parser = subparsers.add_parser("enable")
    enable_parser.add_argument("value", type=int, choices=[0, 1])

    mode_parser = subparsers.add_parser("mode")
    mode_parser.add_argument("value", type=int, choices=[0, 1, 3, 4])

    host_state_parser = subparsers.add_parser("set-host-state")
    host_state_parser.add_argument("value", type=int, choices=range(0, 256))

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

        if args.command == "vision-status":
            sys.exit(send_and_print(ser, build_frame(CMD_QUERY_VISION, seq), args.timeout))

        if args.command == "k210-text":
            payload = args.text.encode("utf-8")
            if len(payload) > 32:
                raise SystemExit("k210-text payload must be 32 bytes or fewer")
            sys.exit(send_and_print(ser, build_frame(CMD_K210_TEXT, seq, payload), args.timeout))

        if args.command == "enable":
            payload = bytes([args.value])
            sys.exit(send_and_print(ser, build_frame(CMD_SET_ENABLE, seq, payload), args.timeout))

        if args.command == "mode":
            payload = bytes([args.value])
            sys.exit(send_and_print(ser, build_frame(CMD_SET_MODE, seq, payload), args.timeout))

        if args.command == "set-host-state":
            payload = bytes([args.value])
            sys.exit(send_and_print(ser, build_frame(CMD_SET_HOST_STATE, seq, payload), args.timeout))

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
