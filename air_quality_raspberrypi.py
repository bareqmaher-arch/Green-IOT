#!/usr/bin/env python3
"""
Air Quality Monitoring System with Edge Computing
Raspberry Pi Implementation
Green IoT Project

Features:
- Multiple air quality sensors support
- Edge computing for data processing
- Machine learning for anomaly detection
- Real-time data visualization
- Cloud connectivity
- Alert system
"""

import time
import json
import logging
import threading
import queue
import numpy as np
import pandas as pd
from datetime import datetime
import serial
import smbus2
import RPi.GPIO as GPIO
from collections import deque
import paho.mqtt.client as mqtt
import requests
from sklearn.ensemble import IsolationForest
import pickle
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/air_quality.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Hardware Configuration
I2C_BUS = 1
BME280_ADDR = 0x76
PMS5003_SERIAL = '/dev/ttyAMA0'
MQ_SENSORS_PINS = {
    'MQ2': 17,   # Smoke, LPG, CO
    'MQ7': 27,   # Carbon Monoxide
    'MQ135': 22  # Air Quality (NH3, NOx, alcohol, benzene)
}

# Alert thresholds (WHO guidelines)
THRESHOLDS = {
    'PM2.5': {'good': 12, 'moderate': 35.4, 'unhealthy_sensitive': 55.4, 
              'unhealthy': 150.4, 'very_unhealthy': 250.4, 'hazardous': 350.4},
    'PM10': {'good': 54, 'moderate': 154, 'unhealthy_sensitive': 254,
             'unhealthy': 354, 'very_unhealthy': 424, 'hazardous': 504},
    'CO': {'good': 4.4, 'moderate': 9.4, 'unhealthy_sensitive': 12.4,
           'unhealthy': 15.4, 'very_unhealthy': 30.4, 'hazardous': 40.4},
    'NO2': {'good': 53, 'moderate': 100, 'unhealthy_sensitive': 360,
            'unhealthy': 649, 'very_unhealthy': 1249, 'hazardous': 1649},
    'O3': {'good': 54, 'moderate': 70, 'unhealthy_sensitive': 85,
           'unhealthy': 105, 'very_unhealthy': 200, 'hazardous': 300}
}


class SensorInterface:
    """Base class for sensor interfaces"""
    
    def read_data(self):
        raise NotImplementedError


class PMS5003Sensor(SensorInterface):
    """Interface for PMS5003 PM2.5/PM10 sensor"""
    
    def __init__(self, port=PMS5003_SERIAL, baudrate=9600):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
        self.ser.flushInput()
        
    def read_data(self):
        """Read PM2.5 and PM10 data from PMS5003"""
        try:
            # Wait for start bytes
            while True:
                if self.ser.read() == b'\x42' and self.ser.read() == b'\x4d':
                    break
            
            # Read data
            data = self.ser.read(30)
            if len(data) == 30:
                # Parse data (simplified)
                pm25 = (data[10] << 8) | data[11]
                pm10 = (data[12] << 8) | data[13]
                
                return {
                    'PM2.5': pm25,
                    'PM10': pm10,
                    'timestamp': datetime.now().isoformat()
                }
        except Exception as e:
            logger.error(f"PMS5003 read error: {e}")
            return None


class BME280Sensor(SensorInterface):
    """Interface for BME280 temperature, humidity, pressure sensor"""
    
    def __init__(self, bus=I2C_BUS, addr=BME280_ADDR):
        self.bus = smbus2.SMBus(bus)
        self.addr = addr
        self._load_calibration()
        
    def _load_calibration(self):
        """Load calibration data from sensor"""
        # Simplified calibration loading
        # In production, implement full calibration routine
        pass
    
    def read_data(self):
        """Read temperature, humidity, and pressure"""
        try:
            # Read raw data
            data = self.bus.read_i2c_block_data(self.addr, 0xF7, 8)
            
            # Calculate values (simplified)
            temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            humidity_raw = (data[6] << 8) | data[7]
            
            # Apply calibration (simplified formulas)
            temperature = (temp_raw / 16384.0 - 0.5) * 100
            humidity = humidity_raw / 65536.0 * 100
            
            return {
                'temperature': round(temperature, 2),
                'humidity': round(humidity, 2),
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"BME280 read error: {e}")
            return None


class MQSensor(SensorInterface):
    """Interface for MQ series gas sensors"""
    
    def __init__(self, pin, sensor_type):
        self.pin = pin
        self.sensor_type = sensor_type
        GPIO.setup(pin, GPIO.IN)
        
        # Calibration values for different gases
        self.calibration = {
            'MQ2': {'R0': 10.0, 'RL': 5.0},
            'MQ7': {'R0': 10.0, 'RL': 10.0},
            'MQ135': {'R0': 10.0, 'RL': 20.0}
        }
        
    def read_data(self):
        """Read gas concentration"""
        try:
            # Read analog value (would need ADC in real implementation)
            # For now, simulate with GPIO digital read
            digital_value = GPIO.input(self.pin)
            
            # Calculate PPM based on sensor type
            if self.sensor_type == 'MQ7':
                co_ppm = self._calculate_co_ppm(digital_value)
                return {'CO': co_ppm, 'timestamp': datetime.now().isoformat()}
            elif self.sensor_type == 'MQ135':
                air_quality = self._calculate_air_quality(digital_value)
                return {'AQI': air_quality, 'timestamp': datetime.now().isoformat()}
            
        except Exception as e:
            logger.error(f"MQ sensor read error: {e}")
            return None
    
    def _calculate_co_ppm(self, value):
        """Calculate CO concentration in PPM"""
        # Simplified calculation
        return round(value * 100, 2)
    
    def _calculate_air_quality(self, value):
        """Calculate air quality index"""
        # Simplified calculation
        return round(value * 500, 2)


class EdgeProcessor:
    """Edge computing processor for air quality data"""
    
    def __init__(self, window_size=60):
        self.window_size = window_size
        self.data_buffer = {
            'PM2.5': deque(maxlen=window_size),
            'PM10': deque(maxlen=window_size),
            'CO': deque(maxlen=window_size),
            'temperature': deque(maxlen=window_size),
            'humidity': deque(maxlen=window_size)
        }
        self.anomaly_detector = None
        self.load_ml_model()
        
    def load_ml_model(self):
        """Load pre-trained anomaly detection model"""
        model_path = '/opt/airquality/models/anomaly_detector.pkl'
        if os.path.exists(model_path):
            with open(model_path, 'rb') as f:
                self.anomaly_detector = pickle.load(f)
        else:
            # Create new model if not exists
            self.anomaly_detector = IsolationForest(
                contamination=0.1,
                random_state=42
            )
    
    def process_data(self, sensor_data):
        """Process sensor data at the edge"""
        processed = {
            'raw_data': sensor_data,
            'timestamp': datetime.now().isoformat()
        }
        
        # Update buffers
        for key, value in sensor_data.items():
            if key in self.data_buffer and value is not None:
                self.data_buffer[key].append(value)
        
        # Calculate statistics
        processed['statistics'] = self._calculate_statistics()
        
        # Detect anomalies
        processed['anomaly'] = self._detect_anomaly(sensor_data)
        
        # Calculate AQI
        processed['aqi'] = self._calculate_aqi(sensor_data)
        
        # Determine alerts
        processed['alerts'] = self._check_alerts(sensor_data)
        
        return processed
    
    def _calculate_statistics(self):
        """Calculate statistical measures"""
        stats = {}
        for param, values in self.data_buffer.items():
            if len(values) > 0:
                stats[param] = {
                    'mean': np.mean(values),
                    'std': np.std(values),
                    'min': np.min(values),
                    'max': np.max(values),
                    'trend': self._calculate_trend(values)
                }
        return stats
    
    def _calculate_trend(self, values):
        """Calculate trend direction"""
        if len(values) < 2:
            return 'stable'
        
        recent = np.mean(list(values)[-10:])
        older = np.mean(list(values)[-20:-10])
        
        if recent > older * 1.1:
            return 'increasing'
        elif recent < older * 0.9:
            return 'decreasing'
        else:
            return 'stable'
    
    def _detect_anomaly(self, data):
        """Detect anomalies using ML model"""
        if self.anomaly_detector is None:
            return False
        
        # Prepare features
        features = []
        for key in ['PM2.5', 'PM10', 'CO']:
            if key in data and data[key] is not None:
                features.append(data[key])
            else:
                features.append(0)
        
        if len(features) == 3:
            try:
                prediction = self.anomaly_detector.predict([features])
                return prediction[0] == -1  # -1 indicates anomaly
            except:
                return False
        
        return False
    
    def _calculate_aqi(self, data):
        """Calculate Air Quality Index"""
        aqi_values = []
        
        # PM2.5 AQI calculation
        if 'PM2.5' in data and data['PM2.5'] is not None:
            pm25_aqi = self._calculate_pm25_aqi(data['PM2.5'])
            aqi_values.append(pm25_aqi)
        
        # PM10 AQI calculation
        if 'PM10' in data and data['PM10'] is not None:
            pm10_aqi = self._calculate_pm10_aqi(data['PM10'])
            aqi_values.append(pm10_aqi)
        
        # Return highest AQI
        return max(aqi_values) if aqi_values else 0
    
    def _calculate_pm25_aqi(self, pm25):
        """Calculate AQI from PM2.5 concentration"""
        # EPA formula for PM2.5 AQI calculation
        if pm25 <= 12.0:
            return self._linear_scale(pm25, 0, 12.0, 0, 50)
        elif pm25 <= 35.4:
            return self._linear_scale(pm25, 12.1, 35.4, 51, 100)
        elif pm25 <= 55.4:
            return self._linear_scale(pm25, 35.5, 55.4, 101, 150)
        elif pm25 <= 150.4:
            return self._linear_scale(pm25, 55.5, 150.4, 151, 200)
        elif pm25 <= 250.4:
            return self._linear_scale(pm25, 150.5, 250.4, 201, 300)
        elif pm25 <= 350.4:
            return self._linear_scale(pm25, 250.5, 350.4, 301, 400)
        else:
            return self._linear_scale(pm25, 350.5, 500.4, 401, 500)
    
    def _calculate_pm10_aqi(self, pm10):
        """Calculate AQI from PM10 concentration"""
        if pm10 <= 54:
            return self._linear_scale(pm10, 0, 54, 0, 50)
        elif pm10 <= 154:
            return self._linear_scale(pm10, 55, 154, 51, 100)
        elif pm10 <= 254:
            return self._linear_scale(pm10, 155, 254, 101, 150)
        elif pm10 <= 354:
            return self._linear_scale(pm10, 255, 354, 151, 200)
        elif pm10 <= 424:
            return self._linear_scale(pm10, 355, 424, 201, 300)
        elif pm10 <= 504:
            return self._linear_scale(pm10, 425, 504, 301, 400)
        else:
            return self._linear_scale(pm10, 505, 604, 401, 500)
    
    def _linear_scale(self, value, low_conc, high_conc, low_aqi, high_aqi):
        """Linear interpolation for AQI calculation"""
        return ((high_aqi - low_aqi) / (high_conc - low_conc)) * (value - low_conc) + low_aqi
    
    def _check_alerts(self, data):
        """Check for alert conditions"""
        alerts = []
        
        for param, value in data.items():
            if param in THRESHOLDS and value is not None:
                for level, threshold in THRESHOLDS[param].items():
                    if value > threshold:
                        alerts.append({
                            'parameter': param,
                            'value': value,
                            'level': level,
                            'threshold': threshold,
                            'message': f"{param} level is {level}: {value} (threshold: {threshold})"
                        })
                        break
        
        return alerts


class CloudConnector:
    """Handles cloud connectivity and data transmission"""
    
    def __init__(self, mqtt_broker, mqtt_port=1883):
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.connected = False
        
    def connect(self):
        """Connect to MQTT broker"""
        try:
            self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.client.loop_start()
            self.connected = True
            logger.info(f"Connected to MQTT broker: {self.mqtt_broker}")
        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
            self.connected = False
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("MQTT connected successfully")
            self.connected = True
        else:
            logger.error(f"MQTT connection failed with code: {rc}")
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        self.connected = False
        logger.warning("MQTT disconnected")
    
    def publish_data(self, topic, data):
        """Publish data to cloud"""
        if self.connected:
            try:
                payload = json.dumps(data)
                self.client.publish(topic, payload, qos=1)
                logger.debug(f"Published to {topic}: {payload}")
                return True
            except Exception as e:
                logger.error(f"Publish failed: {e}")
                return False
        else:
            logger.warning("Not connected to MQTT broker")
            return False
    
    def send_alert(self, alert_data):
        """Send alert to cloud"""
        alert_topic = "airquality/alerts"
        return self.publish_data(alert_topic, alert_data)


class AirQualityMonitor:
    """Main air quality monitoring system"""
    
    def __init__(self, config):
        self.config = config
        self.sensors = {}
        self.processor = EdgeProcessor()
        self.cloud = CloudConnector(config['mqtt_broker'])
        self.data_queue = queue.Queue()
        self.running = False
        
        # Initialize sensors
        self._initialize_sensors()
        
        # Connect to cloud
        self.cloud.connect()
        
    def _initialize_sensors(self):
        """Initialize all sensors"""
        try:
            # PM sensor
            self.sensors['pms5003'] = PMS5003Sensor()
            logger.info("PMS5003 sensor initialized")
        except Exception as e:
            logger.error(f"PMS5003 initialization failed: {e}")
        
        try:
            # Environmental sensor
            self.sensors['bme280'] = BME280Sensor()
            logger.info("BME280 sensor initialized")
        except Exception as e:
            logger.error(f"BME280 initialization failed: {e}")
        
        # Gas sensors
        GPIO.setmode(GPIO.BCM)
        for sensor_type, pin in MQ_SENSORS_PINS.items():
            try:
                self.sensors[sensor_type] = MQSensor(pin, sensor_type)
                logger.info(f"{sensor_type} sensor initialized on pin {pin}")
            except Exception as e:
                logger.error(f"{sensor_type} initialization failed: {e}")
    
    def read_all_sensors(self):
        """Read data from all sensors"""
        combined_data = {}
        
        for name, sensor in self.sensors.items():
            try:
                data = sensor.read_data()
                if data:
                    combined_data.update(data)
            except Exception as e:
                logger.error(f"Error reading {name}: {e}")
        
        return combined_data
    
    def process_and_transmit(self):
        """Process data and transmit to cloud"""
        while self.running:
            try:
                # Get data from queue
                raw_data = self.data_queue.get(timeout=1)
                
                # Process at edge
                processed_data = self.processor.process_data(raw_data)
                
                # Send to cloud
                self.cloud.publish_data("airquality/data", processed_data)
                
                # Check for alerts
                if processed_data['alerts']:
                    for alert in processed_data['alerts']:
                        self.cloud.send_alert(alert)
                        logger.warning(f"Alert: {alert['message']}")
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Processing error: {e}")
    
    def run(self):
        """Main monitoring loop"""
        self.running = True
        
        # Start processing thread
        process_thread = threading.Thread(target=self.process_and_transmit)
        process_thread.start()
        
        logger.info("Air Quality Monitoring System Started")
        
        try:
            while self.running:
                # Read sensors
                sensor_data = self.read_all_sensors()
                
                if sensor_data:
                    # Add to processing queue
                    self.data_queue.put(sensor_data)
                    
                    # Local display/logging
                    logger.info(f"Sensor data: {sensor_data}")
                
                # Read interval
                time.sleep(self.config['read_interval'])
                
        except KeyboardInterrupt:
            logger.info("Shutting down...")
            self.running = False
            process_thread.join()
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        GPIO.cleanup()
        self.cloud.client.loop_stop()
        self.cloud.client.disconnect()
        logger.info("Cleanup complete")


# Configuration
config = {
    'mqtt_broker': 'localhost',
    'mqtt_port': 1883,
    'read_interval': 5,  # seconds
    'device_id': 'rpi_edge_001',
    'location': {
        'lat': 33.3152,
        'lon': 44.3661,
        'name': 'Baghdad, Iraq'
    }
}


if __name__ == "__main__":
    monitor = AirQualityMonitor(config)
    monitor.run()
            