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
# [1] Configuration
# ==========================================
# Data Ready signal pin (Input from STM32)
PIN_DATA_READY = 17 
# Command Trigger signal pin (Output to STM32 to initiate command sequence)
PIN_CMD_TRIG   = 27  

# SPI Bus/Device settings
SPI_BUS = 0
SPI_DEVICE = 0       
# High speed SPI clock (18 MHz)
SPI_SPEED = 18000000 

# Data buffer size for one SPI read
BUFFER_SIZE_BYTES = 4096 
# Number of 12-bit samples (Each sample is 2 bytes (H format))
SAMPLE_COUNT = BUFFER_SIZE_BYTES // 2 

# ADC Sampling Rate (Approx. 0.82 MSps total)
FS = 820000 

class ADC_GUI_App:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 ADC High-Speed Streamer")
        self.root.geometry("1000x800")
        
        # SPI & GPIO Setup
        self.init_hardware()
        
        # State Variables
        self.is_running = False
        self.latest_data = None
        # Lock for thread-safe access to latest_data
        self.lock = threading.Lock()
        # Time stamp for throttling read rate
        self.last_read_time = 0 
        
        # GUI Setup
        self.create_widgets()
        self.setup_plots()
        
        # Start Animation Loop (Plot update loop)
        self.root.after(50, self.update_plot) 

    def init_hardware(self):
        # Use Broadcom SOC channel numbering
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # Input pin for Data Ready signal
        GPIO.setup(PIN_DATA_READY, GPIO.IN) 
        # Output pin for Command Trigger (set HIGH initially)
        GPIO.setup(PIN_CMD_TRIG, GPIO.OUT, initial=GPIO.HIGH)

        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED
        # SPI Mode 0 (CPOL=0, CPHA=0)
        self.spi.mode = 0b00 
        
        # Set up interrupt detection on the Data Ready pin (Rising or Falling edge)
        GPIO.add_event_detect(PIN_DATA_READY, GPIO.BOTH, callback=self.data_ready_callback)

    def create_widgets(self):
        control_frame = ttk.LabelFrame(self.root, text="Control Panel")
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        # Command buttons (send 0, 1, 2, or 3 to STM32)
        ttk.Button(control_frame, text="STOP (q)", command=lambda: self.send_cmd_thread(0)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(control_frame, text="CH1 Only (w)", command=lambda: self.send_cmd_thread(1)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(control_frame, text="CH2 Only (e)", command=lambda: self.send_cmd_thread(2)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(control_frame, text="DUAL Mode (r)", command=lambda: self.send_cmd_thread(3)).pack(side=tk.LEFT, padx=5, pady=5)
        
        # Status display
        self.status_var = tk.StringVar(value="Status: STOPPED")
        ttk.Label(control_frame, textvariable=self.status_var, font=("Arial", 12, "bold")).pack(side=tk.RIGHT, padx=10)

    def setup_plots(self):
        # Create two subplots (Time Domain and Frequency Domain)
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.subplots_adjust(hspace=0.4)
        
        # Waveform Plot (Time Domain)
        self.ax1.set_title("Time Domain Waveform")
        self.ax1.set_xlabel("Sample Index")
        self.ax1.set_ylabel("ADC Value")
        # ADC is 12-bit (0 to 4095)
        self.ax1.set_ylim(0, 4096)
        
        # [Modification] Create two lines for plotting: CH1 and CH2
        # line1 will be used for single mode (CH1 or CH2) or CH1 in DUAL mode.
        self.line1, = self.ax1.plot([], [], 'g-', label='CH1 / Mixed') 
        # line2 is only used for CH2 in DUAL mode.
        self.line2, = self.ax1.plot([], [], 'orange', label='CH2', alpha=0.7)
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)
        
        # FFT Plot (Frequency Domain)
        self.ax2.set_title("Frequency Domain (FFT) - 0 to 20kHz")
        self.ax2.set_xlabel("Frequency (Hz)")
        self.ax2.set_ylabel("Magnitude")
        self.ax2.set_xlim(0, 20000) 
        self.line3, = self.ax2.plot([], [], 'b-') # Renamed variable to line3
        self.ax2.grid(True)
        
        # Embed the plot into the Tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # ==========================================
    # Logic: SPI & Callbacks
    # ==========================================
    # Called when the Data Ready GPIO pin state changes (interrupt)
    def data_ready_callback(self, channel):
        if not self.is_running:
            return

        # CPU Throttling: Limit the SPI read rate to approx 20 FPS (1/0.05s)
        current_time = time.time()
        if (current_time - self.last_read_time) < 0.05:
            return
        self.last_read_time = current_time

        try:
            # Read the entire buffer (4096 bytes) from the STM32
            raw_bytes = self.spi.readbytes(BUFFER_SIZE_BYTES)
            # Unpack 2-byte (16-bit unsigned short, 'H') big-endian ('>') values
            adc_values = struct.unpack('>' + 'H' * SAMPLE_COUNT, bytes(raw_bytes))
            
            # Store data thread-safely
            with self.lock:
                self.latest_data = np.array(adc_values)
                
        except Exception as e:
            print(f"SPI Read Error: {e}")

    # Start the command sending process in a separate thread to prevent GUI freeze
    def send_cmd_thread(self, cmd_byte):
        t = threading.Thread(target=self.send_command_safe, args=(cmd_byte,))
        t.start()

    # Sequence to safely send a command byte to the STM32
    def send_command_safe(self, cmd_byte):
        print(f">> Mode Switching to {cmd_byte}...")
        self.status_var.set(f"Status: Switching to Mode {cmd_byte}...")
        
        # Stop data processing temporarily
        self.is_running = False
        time.sleep(0.1) 
        
        # Temporarily lower SPI speed for reliable command transfer
        self.spi.max_speed_hz = 500000 
        
        # Command sequence protocol:
        GPIO.output(PIN_CMD_TRIG, GPIO.LOW) # Pull trigger LOW
        time.sleep(0.5) # Wait for STM32 to enter command mode
        self.spi.xfer([0x00, cmd_byte]) # Send a dummy byte (0x00) and the command byte
        GPIO.output(PIN_CMD_TRIG, GPIO.HIGH) # Pull trigger HIGH to resume streaming mode
        
        # Restore high speed SPI clock
        self.spi.max_speed_hz = SPI_SPEED
        
        # Update running state based on command
        if cmd_byte == 0:
            self.status_var.set("Status: STOPPED")
            self.is_running = False 
        else:
            self.status_var.set(f"Status: RUNNING (Mode {cmd_byte})")
            self.is_running = True 
            
        print(">> Command Done.")

    # ==========================================
    # Logic: Plot Update
    # ==========================================
    # Called periodically by tk.after to refresh the plots
    def update_plot(self):
        data_to_plot = None
        # Safely retrieve the latest data and clear the buffer
        with self.lock:
            if self.latest_data is not None:
                data_to_plot = self.latest_data.copy()
                self.latest_data = None 
        
        if data_to_plot is not None:
            # Check the current operational mode from the status string
            status_text = self.status_var.get()
            
            # [CORE LOGIC] Is it DUAL Mode (Mode 3)?
            if "Mode 3" in status_text: 
                # Reshape the data into an array of (N/4, 4)
                # Data Structure: [ [CH1_a, CH1_b, CH2_a, CH2_b], ... ] (Interleaved)
                matrix = data_to_plot.reshape(-1, 4)
                
                # Restore CH1 data: Combine 0th and 1st columns
                # flatten() creates a 1D array in order: [CH1_a, CH1_b, CH1_a, CH1_b ...]
                ch1_full = matrix[:, 0:2].flatten()
                
                # Restore CH2 data: Combine 2nd and 3rd columns
                ch2_full = matrix[:, 2:4].flatten()
                
                # Create X-axis (The number of samples is now doubled from original SAMPLE_COUNT)
                x_axis = np.arange(len(ch1_full))
                
                # Plot the two channels
                self.line1.set_data(x_axis, ch1_full)
                self.line2.set_data(x_axis, ch2_full)
                
                self.ax1.set_xlim(0, len(ch1_full))
                
                # Set FFT target (Use CH1 data for FFT)
                target_fft_data = ch1_full
                
                # [IMPORTANT] Calculate the correct sampling rate for FFT
                # FS (820k) is the total combined speed, so the per-channel speed is half.
                current_fs = FS / 2 

            else:
                # Single Mode (CH1 or CH2 Only)
                self.line1.set_data(np.arange(len(data_to_plot)), data_to_plot)
                # Clear CH2 line
                self.line2.set_data([], []) 
                self.ax1.set_xlim(0, len(data_to_plot))
                
                target_fft_data = data_to_plot
                # In single mode, total speed = channel speed
                current_fs = FS 

            # 2. Update FFT (using target_fft_data)
            N = len(target_fft_data)
            # Re-calculate the sampling rate based on the mode for robust FFT calculation
            current_fs = FS / 2 if "Mode 3" in status_text else FS
            
            # Perform Fast Fourier Transform
            yf = fft(target_fft_data)
            xf = fftfreq(N, 1 / current_fs)
            
            # Select only the positive frequency side (up to Nyquist frequency)
            idx = slice(0, N//2)
            xf_plot = xf[idx]
            yf_plot = np.abs(yf[idx]) # Magnitude spectrum

            self.line3.set_data(xf_plot, yf_plot) # Update FFT plot
            
            # Auto-scale Y-axis for FFT (limited to 20kHz view)
            visible_mask = (xf_plot <= 20000)
            if np.any(visible_mask):
                max_val = np.max(yf_plot[visible_mask])
                if max_val > 0:
                    self.ax2.set_ylim(0, max_val * 1.2)
            
            # Redraw the canvas
            self.canvas.draw()
        
        # Schedule the next update
        self.root.after(50, self.update_plot)

    def on_closing(self):
        print("Closing App...")
        self.is_running = False
        # Send STOP command (Mode 0) before closing
        self.send_command_safe(0) 
        # Close SPI and clean up GPIO resources
        self.spi.close()
        GPIO.cleanup()
        self.root.destroy()
        sys.exit()

if __name__ == "__main__":
    root = tk.Tk()
    app = ADC_GUI_App(root)
    # Ensure cleanup function is called when window is closed
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
