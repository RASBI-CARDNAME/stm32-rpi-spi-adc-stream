# 📘 STM32 - Raspberry Pi 고속 ADC 데이터 스트리밍 (SPI/DMA)

## 1. 프로젝트 개요 (Overview)
STM32F103(Slave)에서 수집한 고속 ADC 데이터를 SPI 인터페이스를 통해 Raspberry Pi 4(Master)로 실시간 스트리밍하는 시스템입니다.
기존 Python `spidev`의 성능 한계를 극복하기 위해 **리눅스 커널 모듈(Character Device Driver)을 직접 구현**하여 안정적인 고속 데이터 통신을 달성했습니다.

*   **STM32:** Dual-mode ADC로 고속 샘플링 후 DMA를 통해 CPU 개입 없이 SPI 송신
*   **Raspberry Pi:** 커널 모듈을 통해 데이터를 수신하고 Python으로 FFT 및 시각화 처리
*   **핵심 성과:** SPI Clock 18MHz 환경에서 약 13Mbps의 데이터 전송 대역폭 확보

---

## 2. 기술 스택 (Tech Stack)
*   **Hardware:** STM32F103C8T6 (Blue Pill), Raspberry Pi 4
*   **Firmware (STM32):** C, HAL Library (Dual ADC Interleaved, DMA Circular Buffer)
*   **Driver (RPi):** Linux Kernel Module (C), Device Tree Overlay
*   **Application (RPi):** Python 3, NumPy (Signal Processing)

---

## 3. 하드웨어 연결 (Hardware Setup)
안정적인 18MHz SPI 통신을 위해 점퍼 와이어의 길이를 최소화하였습니다.

| Signal | STM32 Pin (SPI1) | RPi Pin | Note |
|:---:|:---:|:---:|:---|
| **SCK** | PA5 | Pin 23 (SCLK) | Clock Line |
| **MISO** | PA6 | Pin 21 (MISO) | Slave Out, Master In |
| **MOSI** | PA7 | Pin 19 (MOSI) | Control Commands |
| **NSS** | PA4 | Pin 24 (CE0) | Hardware CS |
| **Ready** | PB6 | GPIO 17 | Handshake Signal (Data Ready) |
| **Trig** | PB3 | GPIO 27 | EXTI Interrupt (Start/Stop) |

---

## 4. 핵심 기능 및 성능 (Performance)

### 🚀 고속 데이터 처리 (High-Speed Processing)
*   **Dual ADC Interleaved Mode:** ADC1과 ADC2를 교차 사용하여 샘플링 속도 2배 향상
*   **DMA Circular Buffer:** 4096 샘플 버퍼를 사용하여 CPU 부하 없이 연속 데이터 전송
*   **Performance Metrics:**
    *   **SPI Clock:** 18 MHz
    *   **ADC Clock:** 10.66 MHz (DIV6 Prescaler)
    *   **Effective Sampling Rate:** ~0.82 MSps (Mega Samples Per Second)
    *   **Required Bandwidth:** ~13.1 Mbps (16-bit per sample)

### 🛡️ 안정성 확보 (Robustness)
*   **SPI Overrun (OVR) 방지:** 고속 스트리밍 중 SPI 멈춤 현상을 방지하기 위해 `__HAL_SPI_CLEAR_OVRFLAG` 및 RX 버퍼 Flush 루틴 구현
*   **Handshaking:** GPIO 펄스를 이용한 데이터 동기화로 패킷 밀림 현상 방지

---

## 5. 트러블 슈팅 (Troubleshooting)
개발 과정에서 발생한 주요 문제와 해결 과정입니다.

### Q1. 고속 SPI 통신 시 데이터가 밀리거나 멈추는 현상 (OVR Error)
*   **원인:** STM32의 SPI 데이터 레지스터(DR)를 제때 읽지 않으면 Overrun 플래그가 발생하여 통신이 중단됨.
*   **해결:** 모드 전환 시 `Abort` 기능을 사용하여 SPI를 리셋하고, `__HAL_SPI_CLEAR_OVRFLAG` 매크로를 통해 명시적으로 플래그를 클리어하도록 펌웨어 로직 개선.

### Q2. spidev 버퍼 크기 제한 문제
*   **원인:** 리눅스 기본 `spidev` 드라이버의 버퍼 제한으로 인해 대용량 연속 데이터를 한 번에 가져오기 어려움.
*   **해결:** 직접 커널 모듈(Character Device)을 작성하여 커널 공간에서 데이터를 효율적으로 버퍼링하도록 개선.

---

## 6. 사용 방법 (Usage)

### 1) Kernel Module 빌드 및 로드
```bash
# 커널 모듈 디렉토리로 이동
cd kernel_module

# 모듈 컴파일
make

# 모듈 로드
sudo insmod stm32_adc_driver.ko

# Python 스크립트 실행
python3 adc_receiver.py
```

## 7. 폴더 구조 (Directory Structure)
```text
stm32/
├── Inc/
│    └── adc_set_mode.h
└── Src/
├── main.c
└── adc_set_mode.c
raspberrypi/
├── adc_receiver.py
kernel_module/
├──Makefile
├──cli_monitor.py
├──stm32_adc_driver.c
README.md
```

## 8. 커널 모듈 사용법

1. **디바이스 트리 적용**
   - 디바이스 트리를 `dtbo` 파일로 컴파일하고 시스템에 적용합니다.

2. **커널 모듈 컴파일**
   - 모듈 디렉터리로 이동하여 `make`를 실행해 커널 모듈을 컴파일합니다.

3. **모듈 로드**
   - 생성된 `.ko` 모듈을 `insmod` 명령어로 로드합니다.

4. **핀 매핑 및 예제**
   - 핀 매핑은 위에 언급한대로이며, Python 예제 코드를 실행할 수 있습니다.

## 라이선스
MIT 라이선스 
