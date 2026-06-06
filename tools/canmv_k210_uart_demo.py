import image
import lcd
import sensor
import time
from fpioa_manager import fm
from machine import UART


UART_RX_PIN = 6
UART_TX_PIN = 8
UART_BAUDRATE = 115200

LCD_BG = lcd.BLUE
LCD_FG = lcd.WHITE
LCD_ACCENT = lcd.YELLOW
MAX_CHARS = 26

MODE_IDLE = "IDLE"
MODE_DISPLAY = "DISPLAY"
MODE_COLOR = "COLOR"

PIXELS_THRESHOLD = 100
AREA_THRESHOLD = 150
REPORT_INTERVAL_MS = 400
STATUS_INTERVAL_MS = 2000
IDLE_FLUSH_MS = 120
LEARN_BOX_SIZE = 50
LEARN_SAMPLE_COUNT = 50
PREVIEW_HOLD_FRAMES = 50


def safe_text(value):
    text = str(value)
    text = text.replace("\r", " ").replace("\n", " ")
    sanitized = []
    for ch in text:
        code = ord(ch)
        if 32 <= code <= 126:
            sanitized.append(ch)
        else:
            sanitized.append("?")
    return "".join(sanitized)


def trim_text(text):
    return safe_text(text)[:MAX_CHARS]


def decode_command_bytes(data):
    chars = []
    for value in data:
        if 32 <= value <= 126:
            chars.append(chr(value))
        elif value in (9,):
            chars.append(" ")
        else:
            chars.append("?")
    return "".join(chars)


def lcd_lines(title, line1="", line2="", line3="", fg=LCD_FG, bg=LCD_BG):
    lcd.clear(bg)
    lcd.rotation(0)
    lcd.draw_string(16, 16, trim_text(title), fg, bg)
    lcd.draw_string(16, 56, trim_text(line1), fg, bg)
    lcd.draw_string(16, 96, trim_text(line2), fg, bg)
    lcd.draw_string(16, 136, trim_text(line3), LCD_ACCENT, bg)


def write_frame(uart, body):
    uart.write("${}#\r\n".format(safe_text(body)))


def report_error(uart, where, exc):
    if uart is None:
        return
    try:
        write_frame(uart, "ERR:{}:{}:{}".format(where, type(exc).__name__, safe_text(exc)))
    except Exception:
        pass


def setup_uart():
    fm.register(UART_RX_PIN, fm.fpioa.UART2_RX)
    fm.register(UART_TX_PIN, fm.fpioa.UART2_TX)
    return UART(UART.UART2, UART_BAUDRATE, 8, 0, 0)


def setup_lcd():
    lcd.init()
    lcd.rotation(0)
    lcd_lines("K210 READY", "lcd init ok", "uart2 service")


def setup_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.run(1)
    time.sleep_ms(200)
    for _ in range(20):
        sensor.snapshot()


def now_ms():
    return time.ticks_ms()


def ms_since(last_tick):
    return time.ticks_diff(now_ms(), last_tick)


class K210Service:
    def __init__(self):
        self.uart = None
        self.mode = MODE_IDLE
        self.display_text = "wait command"
        self.sensor_ready = False
        self.cmd_buffer = bytearray()
        self.last_rx_tick = now_ms()
        self.last_status_tick = now_ms()
        self.last_report_tick = now_ms()
        self.last_status_text = ""
        self.last_color_report = ""
        self.last_detected_name = "NONE"
        self.learn_box = [
            (320 // 2) - (LEARN_BOX_SIZE // 2),
            (240 // 2) - (LEARN_BOX_SIZE // 2),
            LEARN_BOX_SIZE,
            LEARN_BOX_SIZE,
        ]
        self.learn_remaining = 0
        self.learn_threshold = None
        self.learning_active = False
        self.preview_remaining = 0

    def start(self):
        setup_lcd()
        lcd_lines("K210 START", "using IO6/IO8", "opening uart2...")
        self.uart = setup_uart()
        lcd_lines("K210 READY", "IO6=RX IO8=TX", "mode=IDLE", "loop alive")
        write_frame(self.uart, "BOOT:UART2_READY")
        self.send_status("boot")

    def send_status(self, reason):
        status = "MODE={},TARGET={},DETECTED={},REASON={}".format(
            self.mode,
            self.current_target_name(),
            self.last_detected_name,
            reason,
        )
        if status != self.last_status_text or ms_since(self.last_status_tick) >= STATUS_INTERVAL_MS:
            self.last_status_text = status
            self.last_status_tick = now_ms()
            write_frame(self.uart, "STATUS:" + status)

    def ensure_sensor(self):
        if self.sensor_ready:
            return
        setup_sensor()
        self.sensor_ready = True

    def current_target_name(self):
        return "LEARNED" if self.learn_threshold else "UNSET"

    def begin_color_learning(self):
        self.ensure_sensor()
        self.learn_threshold = [50, 50, 0, 0, 0, 0]
        self.preview_remaining = PREVIEW_HOLD_FRAMES
        self.learn_remaining = LEARN_SAMPLE_COUNT
        self.learning_active = False
        self.last_detected_name = "LEARNING"
        self.last_color_report = ""
        self.display_text = "learning color"
        lcd_lines("K210 COLOR", "place color", "inside white box", "wait green box")

    def set_mode(self, mode_name):
        mode_name = safe_text(mode_name).upper()
        if mode_name not in (MODE_IDLE, MODE_DISPLAY, MODE_COLOR):
            raise ValueError("unsupported mode")
        self.mode = mode_name
        if self.mode == MODE_COLOR:
            self.begin_color_learning()
        elif self.mode == MODE_DISPLAY:
            self.display_text = "display mode"
        else:
            self.display_text = "wait command"
            self.learning_active = False
            self.learn_remaining = 0
            self.preview_remaining = 0
        self.last_color_report = ""
        if self.mode != MODE_COLOR:
            self.last_detected_name = "NONE"
        self.draw_idle_screen()
        self.send_status("mode_change")

    def draw_idle_screen(self):
        lcd_lines(
            "K210 {}".format(self.mode),
            self.display_text,
            "target={}".format(self.current_target_name()),
            "uart alive",
        )

    def handle_command(self, raw_command):
        command = safe_text(raw_command).strip()
        if not command:
            return

        command_upper = command.upper()
        self.last_rx_tick = now_ms()

        if command_upper == "PING":
            lcd_lines("K210 CMD", "PING", "reply PONG")
            write_frame(self.uart, "PONG")
            self.send_status("ping")
            return

        if command_upper in ("STATUS", "GET:STATUS"):
            self.send_status("manual")
            return

        if command_upper.startswith("SHOW:"):
            text = command[5:].strip() or "<empty>"
            self.display_text = text
            self.mode = MODE_DISPLAY
            lcd_lines("K210 DISPLAY", "from pi/ros:", text, "show ok")
            write_frame(self.uart, "SHOW_OK:" + text)
            self.send_status("show")
            return

        if command_upper.startswith("MODE:"):
            mode_name = command_upper[5:].strip()
            try:
                self.set_mode(mode_name)
            except Exception as exc:
                lcd_lines("K210 MODE ERR", mode_name, safe_text(exc))
                write_frame(self.uart, "ERR:MODE")
                return
            write_frame(self.uart, "ACK:MODE=" + self.mode)
            return

        if command_upper in ("COLOR:ON", "COLOR:START"):
            self.set_mode(MODE_COLOR)
            write_frame(self.uart, "ACK:MODE=COLOR")
            return

        if command_upper in ("COLOR:OFF", "COLOR:STOP"):
            self.set_mode(MODE_IDLE)
            write_frame(self.uart, "ACK:MODE=IDLE")
            return

        if command_upper.startswith("COLOR:TARGET:"):
            # Keep the old command for compatibility, but the new logic always learns
            # the target color from the center ROI like the official example.
            if self.mode != MODE_COLOR:
                self.set_mode(MODE_COLOR)
            else:
                self.begin_color_learning()
            write_frame(self.uart, "ACK:TARGET=LEARNED")
            return

        if command_upper == "HELP":
            write_frame(self.uart, "ACK:CMDS=PING,SHOW,MODE,COLOR")
            return

        lcd_lines("K210 CMD", "unsupported:", command, "see HELP")
        write_frame(self.uart, "ERR:UNSUPPORTED")

    def poll_commands(self):
        if self.uart.any():
            data = self.uart.read()
            if data:
                for value in data:
                    if value in (10, 13):
                        if self.cmd_buffer:
                            body = decode_command_bytes(self.cmd_buffer)
                            self.cmd_buffer = bytearray()
                            self.handle_command(body)
                        continue
                    if len(self.cmd_buffer) < 96:
                        self.cmd_buffer.append(value)
                    else:
                        self.cmd_buffer = bytearray()
                self.last_rx_tick = now_ms()

        if self.cmd_buffer and ms_since(self.last_rx_tick) >= IDLE_FLUSH_MS:
            body = decode_command_bytes(self.cmd_buffer)
            self.cmd_buffer = bytearray()
            self.handle_command(body)

    def detect_color_blob(self, img):
        if not self.learn_threshold:
            return "NONE", None, 0
        best = None
        best_area = 0
        blobs = img.find_blobs(
            [self.learn_threshold],
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
            margin=10,
        )
        for blob in blobs:
            area = blob.w() * blob.h()
            if area > best_area:
                best = blob
                best_area = area
        return "LEARNED", best, best_area

    def run_learning_step(self):
        img = sensor.snapshot()
        if self.preview_remaining > 0:
            img.draw_rectangle(self.learn_box)
            img.draw_string(0, 0, "PUT COLOR IN BOX")
            img.draw_string(0, 24, "white box: {}".format(self.preview_remaining))
            lcd.display(img)
            self.preview_remaining -= 1
            if self.preview_remaining == 0:
                self.learning_active = True
                self.send_status("learning_start")
            return

        img.draw_rectangle(self.learn_box, color=(0, 255, 0))
        if self.learning_active and self.learn_remaining > 0:
            hist = img.get_histogram(roi=self.learn_box)
            lo = hist.get_percentile(0.01)
            hi = hist.get_percentile(0.99)
            self.learn_threshold[0] = (self.learn_threshold[0] + lo.l_value()) // 2
            self.learn_threshold[1] = (self.learn_threshold[1] + hi.l_value()) // 2
            self.learn_threshold[2] = (self.learn_threshold[2] + lo.a_value()) // 2
            self.learn_threshold[3] = (self.learn_threshold[3] + hi.a_value()) // 2
            self.learn_threshold[4] = (self.learn_threshold[4] + lo.b_value()) // 2
            self.learn_threshold[5] = (self.learn_threshold[5] + hi.b_value()) // 2
            self.learn_remaining -= 1
            for blob in img.find_blobs(
                [self.learn_threshold],
                pixels_threshold=100,
                area_threshold=100,
                merge=True,
                margin=10,
            ):
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_rectangle(self.learn_box, color=(0, 255, 0))

        img.draw_string(0, 0, "LEARNING LAB")
        img.draw_string(0, 24, "green box: {}".format(self.learn_remaining))
        lcd.display(img)

        if self.learn_remaining <= 0:
            self.learning_active = False
            self.display_text = "color detect"
            self.last_detected_name = "NONE"
            write_frame(
                self.uart,
                "ACK:LEARNED={},{},{},{},{},{}".format(
                    self.learn_threshold[0],
                    self.learn_threshold[1],
                    self.learn_threshold[2],
                    self.learn_threshold[3],
                    self.learn_threshold[4],
                    self.learn_threshold[5],
                ),
            )
            self.send_status("learned")

    def report_color(self, color_name, blob, area):
        if blob is None:
            payload = "COLOR:NONE"
            if payload != self.last_color_report or ms_since(self.last_report_tick) >= STATUS_INTERVAL_MS:
                write_frame(self.uart, payload)
                self.last_color_report = payload
                self.last_report_tick = now_ms()
            self.last_detected_name = "NONE"
            return

        payload = "COLOR:{},{},{},{},{},{}".format(
            color_name,
            blob.cx(),
            blob.cy(),
            blob.w(),
            blob.h(),
            area,
        )
        if payload != self.last_color_report or ms_since(self.last_report_tick) >= REPORT_INTERVAL_MS:
            write_frame(self.uart, payload)
            self.last_color_report = payload
            self.last_report_tick = now_ms()
        self.last_detected_name = color_name

    def run_color_mode(self):
        self.ensure_sensor()
        if self.preview_remaining > 0 or self.learning_active or self.learn_remaining > 0:
            self.run_learning_step()
            return
        img = sensor.snapshot()
        color_name, blob, area = self.detect_color_blob(img)
        if blob is not None:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_string(0, 0, "{} {}".format(color_name, area))
        else:
            img.draw_string(0, 0, "NO COLOR")
        img.draw_string(0, 24, "target=LEARNED")
        lcd.display(img)
        self.report_color(color_name, blob, area)

    def loop(self):
        heartbeat_tick = now_ms()
        while True:
            self.poll_commands()

            if self.mode == MODE_COLOR:
                self.run_color_mode()
            elif ms_since(heartbeat_tick) >= 1000:
                heartbeat_tick = now_ms()
                self.draw_idle_screen()

            if ms_since(self.last_status_tick) >= STATUS_INTERVAL_MS:
                self.send_status("periodic")

            time.sleep_ms(30)


def main():
    try:
        app = K210Service()
        app.start()
        app.loop()
    except Exception as exc:
        try:
            report_error(locals().get("app").uart if "app" in locals() else None, "FATAL", exc)
        except Exception:
            pass
        try:
            lcd_lines("K210 FATAL ERR", safe_text(type(exc).__name__), safe_text(exc), "restart script")
        except Exception:
            pass
        print("K210 fatal:", exc)
        raise


main()
