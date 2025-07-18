"""
Smart Street Lighting System with Solar Power
MicroPython implementation for ESP32
Green IoT Project

Features:
- Motion detection with PIR sensor
- Ambient light sensing with LDR
- PWM-based LED brightness control
- Solar panel monitoring
- LoRaWAN communication
- Energy-efficient operation
"""

import machine
import time
import json
import gc
from machine import Pin, ADC, PWM, I2C, Timer
import network
import ubinascii

# Hardware Configuration
LED_PIN = 2
PIR_PIN = 13
LDR_PIN = 34
SCL_PIN = 22
SDA_PIN = 21

# LoRa pins (if using SX1276 module)
LORA_SCK = 18
LORA_MOSI = 23
LORA_MISO = 19
LORA_CS = 5
LORA_RST = 14
LORA_IRQ = 26

# Thresholds
DARK_THRESHOLD = 1000
DAWN_DUSK_THRESHOLD = 2000
MOTION_TIMEOUT = 30000  # 30 seconds
LOW_BATTERY_THRESHOLD = 3.3

# Light levels (0-1023 for PWM)
LIGHT_OFF = 0
LIGHT_DIM = 256      # 25%
LIGHT_MEDIUM = 512   # 50%
LIGHT_BRIGHT = 1023  # 100%


class SmartStreetLight:
    def __init__(self):
        # Initialize hardware
        self.led = PWM(Pin(LED_PIN))
        self.led.freq(5000)
        self.pir = Pin(PIR_PIN, Pin.IN, Pin.PULL_DOWN)
        self.ldr = ADC(Pin(LDR_PIN))
        self.ldr.atten(ADC.ATTN_11DB)  # Full range: 0-3.3V
        
        # I2C for power monitoring (INA219)
        self.i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        
        # State variables
        self.motion_detected = False
        self.last_motion_time = 0
        self.current_brightness = 0
        self.energy_saving_mode = False
        self.battery_voltage = 0
        self.solar_current = 0
        self.power_consumption = 0
        
        # Timers
        self.sensor_timer = Timer(0)
        self.transmit_timer = Timer(1)
        self.diagnostic_timer = Timer(2)
        
        # Statistics
        self.total_energy_saved = 0
        self.motion_events = 0
        self.operating_hours = 0
        
        # Device ID
        self.device_id = ubinascii.hexlify(machine.unique_id()).decode()
        
        # Setup interrupts and timers
        self.setup_interrupts()
        self.setup_timers()
        
    def setup_interrupts(self):
        """Configure PIR interrupt for motion detection"""
        self.pir.irq(trigger=Pin.IRQ_RISING, handler=self.motion_handler)
        
    def motion_handler(self, pin):
        """Interrupt handler for motion detection"""
        self.motion_detected = True
        self.last_motion_time = time.ticks_ms()
        self.motion_events += 1
        
    def setup_timers(self):
        """Setup periodic timers for various tasks"""
        # Sensor reading every 1 second
        self.sensor_timer.init(period=1000, mode=Timer.PERIODIC, 
                              callback=lambda t: self.update_sensors())
        
        # Data transmission every 5 minutes
        self.transmit_timer.init(period=300000, mode=Timer.PERIODIC,
                                callback=lambda t: self.transmit_data())
        
        # Diagnostics every minute
        self.diagnostic_timer.init(period=60000, mode=Timer.PERIODIC,
                                  callback=lambda t: self.run_diagnostics())
    
    def read_ldr(self):
        """Read and average LDR values"""
        readings = []
        for _ in range(10):
            readings.append(self.ldr.read())
            time.sleep_ms(10)
        return sum(readings) // len(readings)
    
    def read_power_metrics(self):
        """Read power metrics from INA219"""
        try:
            # INA219 I2C address is 0x40
            ina219_addr = 0x40
            
            # Read voltage (simplified - full implementation would need calibration)
            voltage_reg = self.i2c.readfrom_mem(ina219_addr, 0x02, 2)
            self.battery_voltage = ((voltage_reg[0] << 8) | voltage_reg[1]) * 0.001
            
            # Read current
            current_reg = self.i2c.readfrom_mem(ina219_addr, 0x04, 2)
            self.solar_current = ((current_reg[0] << 8) | current_reg[1]) * 0.1
            
            # Calculate power
            self.power_consumption = self.battery_voltage * abs(self.solar_current)
            
            # Update energy saving mode
            if self.battery_voltage < LOW_BATTERY_THRESHOLD:
                self.energy_saving_mode = True
            elif self.battery_voltage > 3.7:
                self.energy_saving_mode = False
                
        except Exception as e:
            print(f"Power reading error: {e}")
    
    def calculate_brightness(self, ldr_value):
        """Calculate target brightness based on conditions"""
        # Check motion timeout
        if self.motion_detected:
            if time.ticks_diff(time.ticks_ms(), self.last_motion_time) > MOTION_TIMEOUT:
                self.motion_detected = False
        
        # Daylight - turn off
        if ldr_value > DAWN_DUSK_THRESHOLD:
            return LIGHT_OFF
            
        # Dawn/Dusk
        elif ldr_value > DARK_THRESHOLD:
            if self.motion_detected and not self.energy_saving_mode:
                return LIGHT_MEDIUM
            return LIGHT_DIM
            
        # Night time
        else:
            if self.motion_detected:
                return LIGHT_MEDIUM if self.energy_saving_mode else LIGHT_BRIGHT
            else:
                return LIGHT_OFF if self.energy_saving_mode else LIGHT_DIM
    
    def smooth_transition(self, target):
        """Smoothly transition LED brightness"""
        step = 50 if target > self.current_brightness else -50
        
        while self.current_brightness != target:
            self.current_brightness += step
            
            # Clamp to target
            if step > 0 and self.current_brightness > target:
                self.current_brightness = target
            elif step < 0 and self.current_brightness < target:
                self.current_brightness = target
            
            # Clamp to valid range
            self.current_brightness = max(0, min(1023, self.current_brightness))
            
            self.led.duty(self.current_brightness)
            time.sleep_ms(20)
    
    def update_sensors(self):
        """Update sensor readings and adjust lighting"""
        ldr_value = self.read_ldr()
        target_brightness = self.calculate_brightness(ldr_value)
        
        if target_brightness != self.current_brightness:
            self.smooth_transition(target_brightness)
        
        # Update operating hours when light is on
        if self.current_brightness > 0:
            self.operating_hours += 1/3600  # Convert seconds to hours
        
        # Calculate energy saved compared to always-on light
        max_power = 100  # 100W traditional streetlight
        current_power = (self.current_brightness / 1023) * max_power
        power_saved = max_power - current_power
        self.total_energy_saved += power_saved / 3600  # Wh
    
    def prepare_telemetry(self):
        """Prepare telemetry data for transmission"""
        data = {
            "id": self.device_id,
            "ldr": self.read_ldr(),
            "motion": self.motion_detected,
            "brightness": int((self.current_brightness / 1023) * 100),
            "battery_v": round(self.battery_voltage, 2),
            "solar_ma": round(self.solar_current, 1),
            "power_mw": round(self.power_consumption, 1),
            "energy_save": self.energy_saving_mode,
            "uptime": time.ticks_ms() // 1000,
            "motion_events": self.motion_events,
            "energy_saved_wh": round(self.total_energy_saved, 2),
            "operating_hours": round(self.operating_hours, 2),
            "free_mem": gc.mem_free()
        }
        return data
    
    def transmit_data(self):
        """Transmit data via LoRaWAN"""
        try:
            data = self.prepare_telemetry()
            json_data = json.dumps(data)
            
            # In a real implementation, this would use a LoRa library
            print(f"Transmitting: {json_data}")
            
            # Simulate LoRa transmission
            # lora.send(json_data.encode())
            
        except Exception as e:
            print(f"Transmission error: {e}")
    
    def run_diagnostics(self):
        """Run system diagnostics"""
        print("\n=== System Diagnostics ===")
        print(f"Device ID: {self.device_id}")
        print(f"Battery Voltage: {self.battery_voltage:.2f}V")
        print(f"Solar Current: {self.solar_current:.1f}mA")
        print(f"Power Consumption: {self.power_consumption:.1f}mW")
        print(f"Energy Saving: {'ON' if self.energy_saving_mode else 'OFF'}")
        print(f"Current Brightness: {int((self.current_brightness/1023)*100)}%")
        print(f"Motion Events: {self.motion_events}")
        print(f"Energy Saved: {self.total_energy_saved:.2f}Wh")
        print(f"Operating Hours: {self.operating_hours:.2f}h")
        print(f"Free Memory: {gc.mem_free()} bytes")
        print("========================\n")
        
        # Force garbage collection
        gc.collect()
    
    def adaptive_scheduling(self):
        """Implement adaptive scheduling based on patterns"""
        current_hour = (time.ticks_ms() // 3600000) % 24
        
        # Late night hours (2 AM - 5 AM) - maximum energy saving
        if 2 <= current_hour < 5:
            self.energy_saving_mode = True
        
        # Rush hours (6-9 AM, 5-8 PM) - normal operation
        elif (6 <= current_hour < 9) or (17 <= current_hour < 20):
            self.energy_saving_mode = False
    
    def run(self):
        """Main run loop"""
        print("Smart Street Light System Started")
        print(f"Device ID: {self.device_id}")
        
        try:
            while True:
                # Read power metrics every 10 seconds
                if time.ticks_ms() % 10000 < 100:
                    self.read_power_metrics()
                
                # Adaptive scheduling
                self.adaptive_scheduling()
                
                # Small delay to prevent busy waiting
                time.sleep_ms(100)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.sensor_timer.deinit()
        self.transmit_timer.deinit()
        self.diagnostic_timer.deinit()
        self.led.duty(0)
        print("Cleanup complete")


# Configuration class for easy customization
class Config:
    # Network settings
    WIFI_SSID = "YOUR_WIFI_SSID"
    WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
    
    # LoRaWAN settings
    LORA_APP_EUI = "0000000000000000"
    LORA_APP_KEY = "00000000000000000000000000000000"
    
    # Thresholds (can be adjusted based on location)
    DARK_THRESHOLD = 1000
    DAWN_DUSK_THRESHOLD = 2000
    MOTION_TIMEOUT = 30000
    
    # Power settings
    LOW_BATTERY_THRESHOLD = 3.3
    CRITICAL_BATTERY_THRESHOLD = 3.0
    
    # Transmission intervals (ms)
    TELEMETRY_INTERVAL = 300000  # 5 minutes
    EMERGENCY_INTERVAL = 60000   # 1 minute when battery critical


def connect_wifi(ssid, password, timeout=10):
    """Connect to WiFi network"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print(f'Connecting to WiFi: {ssid}')
        wlan.connect(ssid, password)
        
        start_time = time.time()
        while not wlan.isconnected() and time.time() - start_time < timeout:
            time.sleep(1)
            print('.', end='')
        
        if wlan.isconnected():
            print(f'\nConnected! IP: {wlan.ifconfig()[0]}')
            return True
        else:
            print('\nConnection failed!')
            return False
    
    return True


# Main execution
if __name__ == "__main__":
    # Initialize the street light system
    streetlight = SmartStreetLight()
    
    # Optional: Connect to WiFi for additional features
    # connect_wifi(Config.WIFI_SSID, Config.WIFI_PASSWORD)
    
    # Run the system
    streetlight.run()