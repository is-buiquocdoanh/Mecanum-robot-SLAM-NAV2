import os
import RPi.GPIO as GPIO

print(f"User: {os.getuid()}")  # ID user hiện tại
print(f"Groups: {os.getgroups()}")  # Các nhóm của user

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.IN)  # Thử chân GPIO 17
    print("GPIO setup thành công!")
except Exception as e:
    print(f"Lỗi GPIO: {e}")
