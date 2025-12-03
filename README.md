# 📘 STM32 → Raspberry Pi High-Speed ADC Streaming over SPI

## 🧩 Project Overview
This project streams high-speed ADC samples from an **STM32F103C8T6** to a **Raspberry Pi 4** using the SPI interface.
- **STM32 (Slave):** Reads dual-mode ADC samples at high speed and sends them continuously via SPI DMA.
- **Raspberry Pi (Master):** Receives the data using Python (`spidev`) and performs further processing (FFT, logging, visualization, etc.).

---

## 🛠 Hardware Setup

- **MCU:** STM32F103C8T6 (Blue Pill)
- **Master:** Raspberry Pi 4
- **Interface:** SPI Mode 0 (CPOL=0, CPHA=0)

### 🔌 Connections

| Signal | STM32 Pin (SPI1) | Raspberry Pi Pin | Note |
| :--- | :--- | :--- | :--- |
| **SCK** | PA5 | SPI0 SCLK (Pin 23) | Short wires recommended |
| **MISO** | PA6 | SPI0 MISO (Pin 21) | Slave Out, Master In |
| **MOSI** | PA7 | SPI0 MOSI (Pin 19) | Optional (Control Cmd) |
| **NSS/CS**| PA4 | SPI0 CE0 (Pin 24) | Hardware NSS Input |
| **Data Ready** | PB6 | GPIO 17 | Handshake Signal |
| **Cmd Trig** | PB3 | GPIO 27 | EXTI Interrupt |

> **⚠️ Note:** Jumper wires must be kept **extremely short** to ensure stability at **18 MHz** SPI Clock.

---

## ⚙ STM32 Features

- **Dual ADC Interleaved Mode:** Increases sampling rate by using ADC1 and ADC2 alternately.
- **DMA-driven Circular Buffer:** Uses a `4096` sample buffer with Half/Full Transfer interrupts for continuous streaming.
- **SPI Slave Mode:** High-speed transmission using DMA.
- **Control System:** Receives commands from RPi via SPI to switch modes (Stop, CH1, CH2, Dual).
- **Robust Error Handling:**
    - **OVR (Overrun) Protection:** Prevents SPI freeze during high-speed streaming.
    - **Flush DR:** Clears `RXNE` garbage data before receiving commands.

---

## 🐍 Raspberry Pi Software

Uses **Python 3** with the `spidev` and `RPi.GPIO` libraries. This repo contains sample code.

## 📈 Performance

| Component | Value | Notes |
| :--- | :--- | :--- |
| **ADC Clock** | 10.66 MHz | DIV6 Prescaler |
| **Sampling Mode** | Dual ADC Interleaved | |
| **Sampling Time** | 13.5 Cycles | Optimized for stability |
| **Effective SPS** | ~0.82 MSps | Mega Samples Per Second (Dual) |
| **Required Bandwidth**| ~13.1 Mbps | 16-bit per sample |
| **SPI Clock** | **18 MHz** | Sufficient headroom (1.37x) |

---

## 🔧 SPI Overrun (OVR) Protection

To prevent SPI lockup during high-speed transitions, the firmware implements:

1. **Clear OVR flag:** Using `__HAL_SPI_CLEAR_OVRFLAG(&hspi1)`
2. **Flush RX Buffer:** Reading `DR` register while `RXNE` flag is set.
3. **Re-initialization:** Aborting and resetting SPI peripheral after mode-change triggers.

---

## Buffer Size Configuration
```text
cat /sys/module/spidev/parameters/bufsiz
```
Use the command above to check the buffer size. If it shows 4096, you need to increase the SPI buffer size.

## 📂 Directory Structure

```text
stm32/
 ├── Inc/
 │    └── adc_set_mode.h
 └── Src/
      ├── main.c
      └── adc_set_mode.c
raspberrypi/
 ├── adc_receiver.py
README.md
```

## 📜 License
MIT License 
