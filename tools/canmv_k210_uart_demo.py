import lcd
import time
from machine import UART
from board import board_info
from fpioa_manager import fm


# STM32 project confirms USART2 is wired as:
# - STM32 PA2 = USART2_TX
# - STM32 PA3 = USART2_RX
# So the UART wiring must be crossed:
# - K210 UART2_TX -> STM32 PA3 (USART2_RX)
# - K210 UART2_RX -> STM32 PA2 (USART2_TX)
#
# Yahboom's K210 module pin assignment page shows:
# - IO_6  = RXD on the expansion interface
# - IO_8  = TXD on the expansion interface
# - IO_16 = BOOT
# - IO_17 = K1 button
#
# So we must use the external UART pins, not IO16/IO17.
def resolve_board_pin(*names):
    for name in names:
        if hasattr(board_info, name):
            return getattr(board_info, name)
    raise AttributeError("Missing board_info pin definitions: {}".format(", ".join(names)))


K210_UART_TX_PIN = resolve_board_pin("IO8", "PIN8")
K210_UART_RX_PIN = resolve_board_pin("IO6", "PIN6")
UART_BAUDRATE = 115200
LCD_BG = lcd.BLUE
LCD_FG = lcd.WHITE
MAX_CHARS = 24


def setup_lcd():
    lcd.init()
    lcd.rotation(0)
    lcd.clear(LCD_BG)
    lcd.draw_string(20, 20, "UART2 LCD TEST", LCD_FG, LCD_BG)
    lcd.draw_string(20, 50, "Waiting for Pi...", LCD_FG, LCD_BG)


def setup_uart():
    fm.register(K210_UART_TX_PIN, fm.fpioa.UART2_TX, force=True)
    fm.register(K210_UART_RX_PIN, fm.fpioa.UART2_RX, force=True)
    return UART(UART.UART2, UART_BAUDRATE, 8, None, 1, timeout=100, read_buf_len=4096)


def draw_message(text):
    lcd.clear(LCD_BG)
    lcd.draw_string(20, 20, "UART2 LCD TEST", LCD_FG, LCD_BG)
    lcd.draw_string(20, 50, "From Pi via STM32:", LCD_FG, LCD_BG)
    lcd.draw_string(20, 90, text[:MAX_CHARS], lcd.YELLOW, LCD_BG)
    lcd.draw_string(20, 130, "Len: {}".format(len(text)), LCD_FG, LCD_BG)


def main():
    setup_lcd()
    uart = setup_uart()
    buffer = b""
    last_text = ""

    while True:
        data = uart.read()
        if data:
            buffer += data
            try:
                text = buffer.decode("utf-8", errors="replace").strip()
            except Exception:
                text = ""
            if text and text != last_text:
                last_text = text
                draw_message(last_text)
                uart.write("LCD:{}\r\n".format(last_text))
                buffer = b""
        time.sleep_ms(50)


main()
