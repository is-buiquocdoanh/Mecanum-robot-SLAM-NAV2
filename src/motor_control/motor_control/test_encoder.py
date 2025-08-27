import RPi.GPIO as GPIO
import time

class EncoderReader:
    def __init__(self):
        # Thiết lập chân encoder (BCM numbering)
        self.ENC_IN_LEFT_A = 14  # Chan A (GPIO 27)
        self.ENC_IN_LEFT_B = 15  # Chan B (GPIO 22)
        
        # Biến đếm xung và hướng quay
        self.counter = 0
        self.last_state_A = 0
        self.current_state_A = 0
        
        try:
            # Cấu hình GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ENC_IN_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ENC_IN_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Thiết lập ngắt ngoài (interrupt) cho kênh A
            GPIO.add_event_detect(
                self.ENC_IN_LEFT_A,
                GPIO.BOTH,
                callback=self.update_encoder,
                bouncetime=2  # Debounce 2ms
            )
    
        except Exception as e:
            print(f"Lỗi khi thiết lập ngắt: {e}")

    def update_encoder(self, channel):
        """Callback được gọi khi có sự thay đổi tín hiệu trên chân A"""
        self.current_state_A = GPIO.input(self.ENC_IN_LEFT_A)
        state_B = GPIO.input(self.ENC_IN_LEFT_B)
        
        # Xác định hướng quay dựa trên trạng thái A và B
        if self.current_state_A != self.last_state_A:
            if state_B != self.current_state_A:
                self.counter += 1  # Quay thuận (CW)
            else:
                self.counter -= 1  # Quay ngược (CCW)
        
        self.last_state_A = self.current_state_A
    
    def run(self):
        try:
            print("Đang đọc encoder... Nhấn Ctrl+C để dừng.")
            while True:
                print(f"Xung: {self.counter}", end="\r")
                time.sleep(0.1)
        except KeyboardInterrupt:
            GPIO.cleanup()
            print("\nĐã dừng chương trình.")

if __name__ == "__main__":
    encoder = EncoderReader()
    encoder.run()