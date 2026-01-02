import RPi.GPIO as GPIO
import time
import os
import sys
import struct

# ==========================================
# [설정]
# ==========================================
DRIVER_PATH = "/dev/stm32_adc"
PIN_DATA_READY = 17 
PIN_CMD_TRIG   = 27 
BUFFER_SIZE = 4096          # 1회 전송 크기 (Bytes)
SAMPLE_SIZE = 2             # 16bit = 2 Bytes
TEST_DURATION = 5.0         # 측정할 시간 (초)

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_DATA_READY, GPIO.IN) 
    GPIO.setup(PIN_CMD_TRIG, GPIO.OUT, initial=GPIO.HIGH)

def send_command(fd, cmd_byte):
    """ STM32에 시작 명령(Dual Mode 등) 전송 """
    print(f">> Sending Start Command (Mode {cmd_byte})...")
    GPIO.output(PIN_CMD_TRIG, GPIO.LOW)
    time.sleep(0.1)
    try:
        fd.write(bytes([0x00, cmd_byte]))
        fd.flush()
    except Exception as e:
        print(f"Command Error: {e}")
    time.sleep(0.1)
    GPIO.output(PIN_CMD_TRIG, GPIO.HIGH)

def benchmark():
    setup_gpio()
    
    try:
        # 커널 드라이버 열기 (Unbuffered)
        fd = open(DRIVER_PATH, "rb+", buffering=0)
    except Exception as e:
        print(f"Failed to open driver: {e}")
        return

    # 1. STM32를 DUAL 모드(3)로 시작시킴
    send_command(fd, 3) 
    
    print(f"\n>> Starting Throughput Test for {TEST_DURATION} seconds...")
    print(">> Buffering data... (Please wait)")

    total_bytes = 0
    total_reads = 0
    start_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time
            
            # 지정된 시간이 지나면 종료
            if elapsed >= TEST_DURATION:
                break
            
            # [동기화] Data Ready 핀이 High가 될 때까지 대기 (Polling 방식이 가장 빠름)
            # GPIO 인터럽트 콜백보다 while 루프 폴링이 벤치마킹엔 더 정확할 수 있음
            while GPIO.input(PIN_DATA_READY) == 0:
                pass
                
            # [핵심] 데이터 읽기
            chunk = fd.read(BUFFER_SIZE)
            
            if chunk:
                total_bytes += len(chunk)
                total_reads += 1
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        end_time = time.time()
        
        # 종료 명령 전송
        send_command(fd, 0)
        fd.close()
        GPIO.cleanup()

    # ==========================================
    # [결과 리포트]
    # ==========================================
    real_duration = end_time - start_time
    mb_per_sec = (total_bytes / (1024 * 1024)) / real_duration
    kb_per_sec = (total_bytes / 1024) / real_duration
    
    # 2바이트가 1샘플이므로
    total_samples = total_bytes / SAMPLE_SIZE
    sps = total_samples / real_duration # Samples Per Second

    print("\n" + "="*40)
    print(f"  📊 THROUGHPUT BENCHMARK RESULT")
    print("="*40)
    print(f"  ⏱️  Duration      : {real_duration:.4f} sec")
    print(f"  📦 Total Data    : {total_bytes:,} Bytes")
    print(f"  🔄 Total Reads   : {total_reads:,} times")
    print("-" * 40)
    print(f"  🚀 Speed (Raw)   : {kb_per_sec:.2f} KB/s")
    print(f"  🚀 Speed (MB)    : {mb_per_sec:.2f} MB/s")
    print("-" * 40)
    print(f"  📈 Sample Rate   : {sps/1000:.2f} kSPS")
    print("="*40 + "\n")

if __name__ == "__main__":
    if os.geteuid() != 0:
        print("Error: Run as root (sudo)")
    else:
        benchmark()
