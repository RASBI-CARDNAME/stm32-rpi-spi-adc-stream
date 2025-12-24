import RPi.GPIO as GPIO
import struct
import time
import sys
import os

# ==========================================
# [1] 설정값
# ==========================================
PIN_DATA_READY = 17 
PIN_CMD_TRIG   = 27  

# 우리가 만든 드라이버 경로
DRIVER_PATH = "/dev/stm32_adc"

BUFFER_SIZE_BYTES = 4096 
SAMPLE_COUNT = BUFFER_SIZE_BYTES // 2 

# ==========================================
# [2] 초기화
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(PIN_DATA_READY, GPIO.IN) 
GPIO.setup(PIN_CMD_TRIG, GPIO.OUT, initial=GPIO.HIGH)

# [핵심 변경] spidev 대신 파일 열기
try:
    # unbuffered 모드(0)로 열어서 즉시성 확보
    driver_fd = open(DRIVER_PATH, "rb+", buffering=0)
    print(f">> Kernel Driver Opened: {DRIVER_PATH}")
except Exception as e:
    print(f"Error opening driver: {e}")
    print("Hint: Did you run 'insmod stm32_adc_driver.ko'?")
    sys.exit(1)

is_running = False 

print("=== STM32 ADC High-Speed Receiver (CLI Mode) ===")
print("=== Press Enter (or 't'+Enter) to Input Command ===")

# ==========================================
# [3] 데이터 수신 콜백
# ==========================================
def data_ready_callback(channel):
    if not is_running:
        return

    # GPIO 상태 읽기 (디버깅용)
    # state = GPIO.input(PIN_DATA_READY) 
    # (고속 통신 중에는 GPIO 읽는 것도 오버헤드가 될 수 있어 생략 가능)
    
    try:
        # [핵심 변경] spidev.readbytes -> file.read
        raw_bytes = driver_fd.read(BUFFER_SIZE_BYTES)
        
        if not raw_bytes: return

        # 데이터 해석 (Big Endian u16)
        adc_values = struct.unpack('>' + 'H' * SAMPLE_COUNT, raw_bytes)
        
        # 화면에 너무 많이 출력되면 렉 걸리므로 앞부분 10개만 출력
        # \r을 사용하여 한 줄에서 계속 갱신되도록 함 (보기 깔끔함)
        sys.stdout.write(f"\r[DATA] Count: {len(adc_values)} | Values: {adc_values[:10]}   ")
        sys.stdout.flush()

    except Exception as e:
        # print(f"Data Error: {e}")
        pass

# 이벤트 감지 등록 (RISING Edge 추천 - STM32가 다 채우고 올릴 때)
GPIO.add_event_detect(PIN_DATA_READY, GPIO.RISING, callback=data_ready_callback)

# ==========================================
# [4] 명령 전송 함수
# ==========================================
def send_command(cmd_byte):
    print(f"\n>> Sending Command: {cmd_byte} ...")
    
    # 핸드쉐이킹 시퀀스
    GPIO.output(PIN_CMD_TRIG, GPIO.LOW)
    time.sleep(0.5) # STM32 인터럽트 대기
    
    # [핵심 변경] spidev.xfer -> file.write
    try:
        # [Dummy, Command]
        cmd_packet = bytes([0x00, cmd_byte])
        driver_fd.write(cmd_packet)
        driver_fd.flush()
    except Exception as e:
        print(f"Write Error: {e}")
    
    time.sleep(0.1)
    print(">> Done")
    GPIO.output(PIN_CMD_TRIG, GPIO.HIGH)

# ==========================================
# [5] 메인 루프 (터미널 입력 처리)
# ==========================================
try:
    while True:
        # 사용자 입력 대기 (Blocking)
        user_input = input() 

        # 입력이 들어오면 일단 멈춤
        print("\n[PAUSED] Input detected...")
        is_running = False
        time.sleep(0.1) 
        
        cmd = input("Enter Command (q=STOP, w=CH1, e=CH2, r=DUAL): ").strip()
        
        if cmd == 'q':
            send_command(0) # STOP
            print(">> ADC Stopped.")
            
        elif cmd == 'w':
            send_command(1) # CH1
            print(">> Mode Changed to CH1. Resuming...")
            is_running = True
            
        elif cmd == 'e':
            send_command(2) # CH2
            print(">> Mode Changed to CH2. Resuming...")
            is_running = True
            
        elif cmd == 'r':
            send_command(3) # DUAL
            print(">> Mode Changed to DUAL. Resuming...")
            is_running = True
        
        else:
            print(">> Invalid Command. Resuming...")
            is_running = True # 잘못 눌렀으면 다시 시작
            
except KeyboardInterrupt:
    print("\nShutting down...")
    if driver_fd: driver_fd.close()
    GPIO.cleanup()