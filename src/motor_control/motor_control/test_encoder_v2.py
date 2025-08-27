import lgpio
import time

# GPIO định nghĩa
ENC_A = 14
ENC_B = 15

# Mở chip GPIO
h = lgpio.gpiochip_open(0)

# Cấp quyền sử dụng 2 chân đầu vào
lgpio.gpio_claim_input(h, ENC_A)
lgpio.gpio_claim_input(h, ENC_B)

# Biến đếm encoder
encoder_count = 0

# Trạng thái trước đó của A và B
prev_state = (lgpio.gpio_read(h, ENC_A) << 1) | lgpio.gpio_read(h, ENC_B)

# Hàm callback khi có thay đổi ở A hoặc B
def encoder_callback(chip, gpio, level, tick):
    global encoder_count, prev_state

    # Đọc trạng thái hiện tại
    state_a = lgpio.gpio_read(h, ENC_A)
    state_b = lgpio.gpio_read(h, ENC_B)
    current_state = (state_a << 1) | state_b

    # Giải mã theo bảng quadrature
    transition = (prev_state << 2) | current_state

    if transition in [0b0001, 0b0111, 0b1110, 0b1000]:
        encoder_count += 1  # quay thuận
    elif transition in [0b0010, 0b0100, 0b1101, 0b1011]:
        encoder_count -= 1  # quay ngược

    prev_state = current_state

    print(f"Tick: {encoder_count}")

# Gán callback cho cả A và B
lgpio.gpio_set_alert_func(h, ENC_A, encoder_callback)
lgpio.gpio_set_alert_func(h, ENC_B, encoder_callback)

try:
    print("Đang đọc encoder... Nhấn Ctrl+C để thoát.")
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nDừng chương trình.")

finally:
    lgpio.gpiochip_close(h)
