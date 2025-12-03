import spidev
import RPi.GPIO as GPIO
import struct
import time
import sys

# ==========================================
# [1] Configuration
# ==========================================
PIN_DATA_READY = 17  # STM32(PB6) -> RPi (Data Ready Signal)
PIN_CMD_TRIG   = 27  # RPi -> STM32(PB7) (Command Trigger)

SPI_BUS = 0
SPI_DEVICE = 0       
SPI_SPEED = 18000000 # 18 MHz (Stable based on STM32 HSI 64MHz)

BUFFER_SIZE_BYTES = 4096 
SAMPLE_COUNT = BUFFER_SIZE_BYTES // 2 

# ==========================================
# [2] Initialization (GPIO & SPI)
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Input Pin (STM32 uses Push-Pull, no internal pull-up/down needed)
GPIO.setup(PIN_DATA_READY, GPIO.IN) 
# Output Pin (Idle state is HIGH)
GPIO.setup(PIN_CMD_TRIG, GPIO.OUT, initial=GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = 0b00 

# [Critical] Python needs to track the current state!
is_running = False 

print("=== STM32 ADC High-Speed Receiver Started ===")
print("=== Press 't' + Enter to Input Command ===")

# ==========================================
# [3] Data Receiver Callback
# ==========================================
def data_ready_callback(channel):
    # If 't' is pressed to enter command mode, is_running becomes False.
    # Do not touch SPI at this time (Prevent bus collision).
    if not is_running:
        return

    # 1. Check current pin state
    state = GPIO.input(PIN_DATA_READY)
    
    # 2. Read data via SPI
    # If readbytes is called while STM32 is stopped, RPi generates clock
    # and reads garbage values/noise because MISO is floating.
    try:
        raw_bytes = spi.readbytes(BUFFER_SIZE_BYTES)
        adc_values = struct.unpack('>' + 'H' * SAMPLE_COUNT, bytes(raw_bytes))
        
        tag = "[FRONT]" if state == 1 else "[BACK ]"
        # Print data (Comment out if it's too fast)
        print(f"{tag} Count: {len(adc_values)}, Data: {adc_values[:5]}")

    except Exception as e:
        print(f"Data Error: {e}")

# Register Event Detect (Detect both Rising & Falling edges)
GPIO.add_event_detect(PIN_DATA_READY, GPIO.BOTH, callback=data_ready_callback)

# ==========================================
# [4] Command Transmission Function
# ==========================================
def send_command(cmd_byte):
    # 1. Lower the speed when sending commands (Safety measure)
    spi.max_speed_hz = 500000 

    print(f">> Sending Command: {cmd_byte} ...")
    
    GPIO.output(PIN_CMD_TRIG, GPIO.LOW)
    # Time for STM32 to Abort/Flush and prepare (0.5s)
    time.sleep(0.5) 
    
    # Send command (Padding 0x00 + Command for 16-bit alignment)
    spi.xfer([0x00, cmd_byte]) 
    
    print(">> Done")
    GPIO.output(PIN_CMD_TRIG, GPIO.HIGH)

    # [Important] Restore SPI speed for high-speed data reception!
    spi.max_speed_hz = SPI_SPEED 

# ==========================================
# [5] Main Loop (Terminal Input Handling)
# ==========================================
try:
    while True:
        # 1. Data is printed continuously by default.
        # 2. User input (Enter, etc.) is caught here.
        user_input = input() 

        # Pause if 't' or just Enter is pressed (Safety First)
        if user_input.strip() == 't':
            
            # [1] Pause reception (Release SPI Bus)
            print("\n[PAUSED] Stopping Data Stream...")
            is_running = False
            time.sleep(0.1) # Wait briefly for any active callback to finish
            
            # [2] Wait for command input
            cmd = input("Enter Command (q=STOP, w=CH1, e=CH2, r=DUAL): ").strip()
            
            if cmd == 'q':
                send_command(0) # STOP
                print(">> ADC Stopped. (Type 't' to enter command again)")
                # Keep is_running as False (Remain stopped)
                
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
                print(">> Invalid Command. Press 't' again.")
                # Invalid command: keep stopped (Safety) or you could choose to resume.
                
except KeyboardInterrupt:
    print("\nShutting down...")
    spi.close()
    GPIO.cleanup()
