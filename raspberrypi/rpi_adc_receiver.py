import spidev
import RPi.GPIO as GPIO
import struct
import time
import sys

# ==========================================
# [1] 설정 (Configuration)
# ==========================================
PIN_DATA_READY = 17  # STM32(PB6) -> RPi (데이터 준비 신호, Data Ready)
PIN_CMD_TRIG   = 27  # RPi -> STM32(PB7) (명령 트리거, Command Trigger)

SPI_BUS = 0
SPI_DEVICE = 0       
SPI_SPEED = 18000000 # 18 MHz (STM32 HSI 64MHz 클럭 기준 안정적인 속도)

BUFFER_SIZE_BYTES = 4096 
SAMPLE_COUNT = BUFFER_SIZE_BYTES // 2 # 16비트(2바이트) 샘플 개수

# ==========================================
# [2] 초기화 (GPIO & SPI)
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 입력 핀 설정 (STM32가 Push-Pull 출력을 사용하므로 RPi 내부 풀업/풀다운 저항 불필요)
GPIO.setup(PIN_DATA_READY, GPIO.IN) 
# 출력 핀 설정 (기본 유휴 상태는 HIGH)
GPIO.setup(PIN_CMD_TRIG, GPIO.OUT, initial=GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = 0b00 

# [중요] 파이썬 코드 내에서 현재 데이터 수신 중인지 상태를 추적해야 함!
is_running = False 

print("=== STM32 ADC High-Speed Receiver Started ===")
print("=== Press 't' + Enter to Input Command ===")

# ==========================================
# [3] 데이터 수신 콜백 함수
# ==========================================
def data_ready_callback(channel):
    # 't'를 눌러 명령 모드로 진입했다면 is_running이 False가 됨.
    # 이때는 SPI 버스 충돌 방지를 위해 SPI 동작을 수행하지 않고 리턴.
    if not is_running:
        return

    # 1. 현재 핀 상태 확인 (더블 버퍼링의 전반부인지 후반부인지 확인용 등)
    state = GPIO.input(PIN_DATA_READY)
    
    # 2. SPI를 통해 데이터 읽기
    # STM32가 멈춰있는데 readbytes를 호출하면, 라즈베리 파이가 클럭을 생성하지만
    # MISO 라인이 플로팅 상태라 쓰레기 값이나 노이즈를 읽게 됨.
    try:
        raw_bytes = spi.readbytes(BUFFER_SIZE_BYTES)
        # 빅 엔디안('>') unsigned short('H') 형식으로 언팩
        adc_values = struct.unpack('>' + 'H' * SAMPLE_COUNT, bytes(raw_bytes))
        
        tag = "[FRONT]" if state == 1 else "[BACK ]"
        # 데이터 출력 (속도가 너무 빠르면 주석 처리하여 성능 확보 필요)
        print(f"{tag} Count: {len(adc_values)}, Data: {adc_values[:5]}")

    except Exception as e:
        print(f"Data Error: {e}")

# 이벤트 감지 등록 (상승 에지와 하강 에지 모두 감지 - BOTH)
GPIO.add_event_detect(PIN_DATA_READY, GPIO.BOTH, callback=data_ready_callback)

# ==========================================
# [4] 명령 전송 함수
# ==========================================
def send_command(cmd_byte):
    # 1. 명령 전송 시에는 안전을 위해 SPI 속도를 낮춤
    spi.max_speed_hz = 500000 

    print(f">> Sending Command: {cmd_byte} ...")
    
    # 트리거 핀을 LOW로 떨어뜨려 명령 모드 시작 알림
    GPIO.output(PIN_CMD_TRIG, GPIO.LOW)
    
    # STM32가 현재 작업을 중단(Abort)하고 플러시(Flush)할 시간을 줌 (0.5초)
    time.sleep(0.5) 
    
    # 명령 전송 (16비트 정렬을 맞추기 위해 0x00 패딩 + 명령 바이트)
    spi.xfer([0x00, cmd_byte]) 
    
    print(">> Done")
    # 트리거 핀을 다시 HIGH로 올려 스트리밍 모드 복귀 준비
    GPIO.output(PIN_CMD_TRIG, GPIO.HIGH)

    # [중요] 고속 데이터 수신을 위해 SPI 속도를 다시 18MHz로 복구!
    spi.max_speed_hz = SPI_SPEED 

# ==========================================
# [5] 메인 루프 (터미널 입력 처리)
# ==========================================
try:
    while True:
        # 1. 기본적으로 데이터는 콜백 함수에 의해 계속 출력됨.
        # 2. 여기서 사용자의 키보드 입력(Enter 등)을 기다림.
        user_input = input() 

        # 사용자가 't' 또는 공백 후 Enter를 누르면 (안전 우선)
        if user_input.strip() == 't':
            
            # [1] 수신 일시 정지 (SPI 버스 점유 해제)
            print("\n[PAUSED] Stopping Data Stream...")
            is_running = False
            time.sleep(0.1) # 실행 중이던 콜백이 끝날 때까지 잠시 대기
            
            # [2] 명령 입력 대기
            cmd = input("Enter Command (q=STOP, w=CH1, e=CH2, r=DUAL): ").strip()
            
            if cmd == 'q':
                send_command(0) # STOP 명령
                print(">> ADC Stopped. (Type 't' to enter command again)")
                # is_running을 False로 유지 (정지 상태 지속)
                
            elif cmd == 'w':
                send_command(1) # CH1 모드
                print(">> Mode Changed to CH1. Resuming...")
                is_running = True
                
            elif cmd == 'e':
                send_command(2) # CH2 모드
                print(">> Mode Changed to CH2. Resuming...")
                is_running = True
                
            elif cmd == 'r':
                send_command(3) # DUAL 모드
                print(">> Mode Changed to DUAL. Resuming...")
                is_running = True
            
            else:
                print(">> Invalid Command. Press 't' again.")
                # 잘못된 명령인 경우: 안전을 위해 정지 상태 유지 (원하면 True로 변경 가능)
                
except KeyboardInterrupt:
    print("\nShutting down...")
    spi.close()
    GPIO.cleanup()
