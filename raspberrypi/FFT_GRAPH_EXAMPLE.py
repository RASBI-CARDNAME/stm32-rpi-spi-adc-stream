import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from scipy.fft import fft, fftfreq
import spidev
import RPi.GPIO as GPIO
import struct
import time
import threading
import sys

# ==========================================
# [1] 설정 (Configuration)
# ==========================================
# 데이터 준비 신호 핀 (STM32로부터의 입력)
PIN_DATA_READY = 17 
# 명령 트리거 신호 핀 (명령 시퀀스를 시작하기 위해 STM32로 보내는 출력)
PIN_CMD_TRIG   = 27  

# SPI 버스/장치 설정
SPI_BUS = 0
SPI_DEVICE = 0       
# 고속 SPI 클럭 (18 MHz)
SPI_SPEED = 18000000 

# 한 번의 SPI 읽기에 대한 데이터 버퍼 크기
BUFFER_SIZE_BYTES = 4096 
# 12비트 샘플 개수 (각 샘플은 2바이트(H 포맷))
SAMPLE_COUNT = BUFFER_SIZE_BYTES // 2 

# ADC 샘플링 레이트 (전체 약 0.82 MSps)
FS = 820000 

class ADC_GUI_App:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 ADC High-Speed Streamer")
        self.root.geometry("1000x800")
        
        # SPI 및 GPIO 설정
        self.init_hardware()
        
        # 상태 변수
        self.is_running = False
        self.latest_data = None
        # latest_data에 대한 스레드 안전 접근을 위한 잠금(Lock)
        self.lock = threading.Lock()
        # 읽기 속도 조절(스로틀링)을 위한 타임스탬프
        self.last_read_time = 0 
        
        # GUI 설정
        self.create_widgets()
        self.setup_plots()
        
        # 애니메이션 루프 시작 (플롯 업데이트 루프)
        self.root.after(50, self.update_plot) 

    def init_hardware(self):
        # 브로드컴 SOC 채널 번호 지정 방식 사용 (BCM)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # 데이터 준비 신호용 입력 핀
        GPIO.setup(PIN_DATA_READY, GPIO.IN) 
        # 명령 트리거용 출력 핀 (초기값 HIGH)
        GPIO.setup(PIN_CMD_TRIG, GPIO.OUT, initial=GPIO.HIGH)

        # SPI 초기화
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED
        # SPI 모드 0 (CPOL=0, CPHA=0)
        self.spi.mode = 0b00 
        
        # 데이터 준비 핀에 인터럽트 감지 설정 (상승 또는 하강 에지)
        GPIO.add_event_detect(PIN_DATA_READY, GPIO.BOTH, callback=self.data_ready_callback)

    def create_widgets(self):
        control_frame = ttk.LabelFrame(self.root, text="Control Panel")
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        # 명령 버튼 (STM32로 0, 1, 2, 3 전송)
        ttk.Button(control_frame, text="STOP (q)", command=lambda: self.send_cmd_thread(0)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(control_frame, text="CH1 Only (w)", command=lambda: self.send_cmd_thread(1)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(control_frame, text="CH2 Only (e)", command=lambda: self.send_cmd_thread(2)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(control_frame, text="DUAL Mode (r)", command=lambda: self.send_cmd_thread(3)).pack(side=tk.LEFT, padx=5, pady=5)
        
        # 상태 표시
        self.status_var = tk.StringVar(value="Status: STOPPED")
        ttk.Label(control_frame, textvariable=self.status_var, font=("Arial", 12, "bold")).pack(side=tk.RIGHT, padx=10)

    def setup_plots(self):
        # 두 개의 서브플롯 생성 (시간 영역 및 주파수 영역)
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.subplots_adjust(hspace=0.4)
        
        # 파형 플롯 (시간 영역)
        self.ax1.set_title("Time Domain Waveform")
        self.ax1.set_xlabel("Sample Index")
        self.ax1.set_ylabel("ADC Value")
        # ADC는 12비트 (0 ~ 4095)
        self.ax1.set_ylim(0, 4096)
        
        # [수정] 플롯을 위해 두 개의 라인 생성: CH1 및 CH2
        # line1은 단일 모드(CH1 또는 CH2) 또는 듀얼 모드의 CH1에 사용됨.
        self.line1, = self.ax1.plot([], [], 'g-', label='CH1 / Mixed') 
        # line2는 듀얼 모드의 CH2에만 사용됨.
        self.line2, = self.ax1.plot([], [], 'orange', label='CH2', alpha=0.7)
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)
        
        # FFT 플롯 (주파수 영역) - 0 ~ 20kHz
        self.ax2.set_title("Frequency Domain (FFT) - 0 to 20kHz")
        self.ax2.set_xlabel("Frequency (Hz)")
        self.ax2.set_ylabel("Magnitude")
        self.ax2.set_xlim(0, 20000) 
        self.line3, = self.ax2.plot([], [], 'b-') # line3로 변수명 변경됨
        self.ax2.grid(True)
        
        # Tkinter 창에 플롯 삽입
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # ==========================================
    # 로직: SPI & 콜백
    # ==========================================
    # 데이터 준비 GPIO 핀 상태가 변경될 때 호출됨 (인터럽트)
    def data_ready_callback(self, channel):
        if not self.is_running:
            return

        # CPU 부하 조절: SPI 읽기 속도를 약 20 FPS로 제한 (1/0.05초)
        current_time = time.time()
        if (current_time - self.last_read_time) < 0.05:
            return
        self.last_read_time = current_time

        try:
            # STM32로부터 전체 버퍼(4096 바이트) 읽기
            raw_bytes = self.spi.readbytes(BUFFER_SIZE_BYTES)
            # 2바이트(16비트 부호 없는 short, 'H') 빅 엔디안('>') 값으로 언팩
            adc_values = struct.unpack('>' + 'H' * SAMPLE_COUNT, bytes(raw_bytes))
            
            # 스레드 안전하게 데이터 저장
            with self.lock:
                self.latest_data = np.array(adc_values)
                
        except Exception as e:
            print(f"SPI Read Error: {e}")

    # GUI 멈춤 방지를 위해 별도 스레드에서 명령 전송 프로세스 시작
    def send_cmd_thread(self, cmd_byte):
        t = threading.Thread(target=self.send_command_safe, args=(cmd_byte,))
        t.start()

    # STM32로 명령 바이트를 안전하게 보내기 위한 시퀀스
    def send_command_safe(self, cmd_byte):
        print(f">> Mode Switching to {cmd_byte}...")
        self.status_var.set(f"Status: Switching to Mode {cmd_byte}...")
        
        # 데이터 처리를 일시적으로 중단
        self.is_running = False
        time.sleep(0.1) 
        
        # 안정적인 명령 전송을 위해 SPI 속도를 일시적으로 낮춤
        self.spi.max_speed_hz = 500000 
        
        # 명령 시퀀스 프로토콜:
        GPIO.output(PIN_CMD_TRIG, GPIO.LOW) # 트리거를 LOW로 당김
        time.sleep(0.5) # STM32가 명령 모드로 진입할 때까지 대기
        self.spi.xfer([0x00, cmd_byte]) # 더미 바이트(0x00)와 명령 바이트 전송
        GPIO.output(PIN_CMD_TRIG, GPIO.HIGH) # 스트리밍 모드 재개를 위해 트리거를 HIGH로 당김
        
        # 고속 SPI 클럭 복구
        self.spi.max_speed_hz = SPI_SPEED
        
        # 명령에 따라 실행 상태 업데이트
        if cmd_byte == 0:
            self.status_var.set("Status: STOPPED")
            self.is_running = False 
        else:
            self.status_var.set(f"Status: RUNNING (Mode {cmd_byte})")
            self.is_running = True 
            
        print(">> Command Done.")

    # ==========================================
    # 로직: 플롯 업데이트
    # ==========================================
    # 플롯을 새로 고치기 위해 tk.after에 의해 주기적으로 호출됨
    def update_plot(self):
        data_to_plot = None
        # 안전하게 최신 데이터를 가져오고 버퍼 비우기
        with self.lock:
            if self.latest_data is not None:
                data_to_plot = self.latest_data.copy()
                self.latest_data = None 
        
        if data_to_plot is not None:
            # 상태 문자열에서 현재 작동 모드 확인
            status_text = self.status_var.get()
            
            # [핵심 로직] 듀얼 모드(모드 3)인가?
            if "Mode 3" in status_text: 
                # 데이터를 (N/4, 4) 배열로 재구조화
                # 데이터 구조: [ [CH1_a, CH1_b, CH2_a, CH2_b], ... ] (인터리빙됨)
                matrix = data_to_plot.reshape(-1, 4)
                
                # CH1 데이터 복원: 0번째와 1번째 열 결합
                # flatten()은 순서대로 1D 배열 생성: [CH1_a, CH1_b, CH1_a, CH1_b ...]
                ch1_full = matrix[:, 0:2].flatten()
                
                # CH2 데이터 복원: 2번째와 3번째 열 결합
                ch2_full = matrix[:, 2:4].flatten()
                
                # X축 생성 (샘플 수는 원래 SAMPLE_COUNT의 두 배가 됨)
                x_axis = np.arange(len(ch1_full))
                
                # 두 채널 플롯
                self.line1.set_data(x_axis, ch1_full)
                self.line2.set_data(x_axis, ch2_full)
                
                self.ax1.set_xlim(0, len(ch1_full))
                
                # FFT 대상 설정 (FFT에는 CH1 데이터 사용)
                target_fft_data = ch1_full
                
                # [중요] FFT를 위한 올바른 샘플링 레이트 계산
                # FS(820k)는 전체 합산 속도이므로, 채널당 속도는 절반임
                current_fs = FS / 2 

            else:
                # 단일 모드 (CH1 또는 CH2 전용)
                self.line1.set_data(np.arange(len(data_to_plot)), data_to_plot)
                # CH2 라인 지우기
                self.line2.set_data([], []) 
                self.ax1.set_xlim(0, len(data_to_plot))
                
                target_fft_data = data_to_plot
                # 단일 모드에서는 전체 속도 = 채널 속도
                current_fs = FS 

            # 2. FFT 업데이트 (target_fft_data 사용)
            N = len(target_fft_data)
            # 견고한 FFT 계산을 위해 모드에 따른 샘플링 레이트 재계산
            current_fs = FS / 2 if "Mode 3" in status_text else FS
            
            # 고속 푸리에 변환 수행
            yf = fft(target_fft_data)
            xf = fftfreq(N, 1 / current_fs)
            
            # 양의 주파수 측만 선택 (나이퀴스트 주파수까지)
            idx = slice(0, N//2)
            xf_plot = xf[idx]
            yf_plot = np.abs(yf[idx]) # 크기 스펙트럼

            self.line3.set_data(xf_plot, yf_plot) # FFT 플롯 업데이트
            
            # FFT Y축 자동 스케일링 (20kHz 뷰로 제한)
            visible_mask = (xf_plot <= 20000)
            if np.any(visible_mask):
                max_val = np.max(yf_plot[visible_mask])
                if max_val > 0:
                    self.ax2.set_ylim(0, max_val * 1.2)
            
            # 캔버스 다시 그리기
            self.canvas.draw()
        
        # 다음 업데이트 예약
        self.root.after(50, self.update_plot)

    def on_closing(self):
        print("Closing App...")
        self.is_running = False
        # 종료 전 STOP 명령(모드 0) 전송
        self.send_command_safe(0) 
        # SPI 닫기 및 GPIO 리소스 정리
        self.spi.close()
        GPIO.cleanup()
        self.root.destroy()
        sys.exit()

if __name__ == "__main__":
    root = tk.Tk()
    app = ADC_GUI_App(root)
    # 창이 닫힐 때 정리 함수가 호출되도록 보장
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
