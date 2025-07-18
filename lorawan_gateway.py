#!/usr/bin/env python3
"""
LoRaWAN Gateway Implementation
Raspberry Pi with SX1301/SX1308 concentrator
Green IoT Project

Features:
- Multi-channel LoRa reception
- Packet forwarding to network server
- Local data caching
- Energy monitoring
- GPS time synchronization
"""

import os
import sys
import time
import json
import socket
import struct
import threading
import logging
from datetime import datetime
import serial
import spidev
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from collections import deque
import sqlite3
import base64
import hashlib
import hmac

# Logging configuration
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Hardware pins (for SX1301/SX1308)
RESET_PIN = 17
POWER_EN_PIN = 27
GPS_SERIAL_PORT = '/dev/ttyAMA0'
SPI_DEV = 0
SPI_CS = 0

# Network configuration
class Config:
    # Gateway EUI (unique identifier)
    GATEWAY_EUI = "B827EBFFFE" + hex(int.from_bytes(os.urandom(3), 'big'))[2:].upper().zfill(6)
    
    # Network server
    SERVER_HOST = "localhost"
    SERVER_PORT_UP = 1700
    SERVER_PORT_DOWN = 1700
    
    # MQTT settings
    MQTT_BROKER = "localhost"
    MQTT_PORT = 1883
    MQTT_TOPIC_PREFIX = "gateway"
    
    # Frequencies (EU868)
    LORA_FREQUENCIES = [
        868100000,  # Channel 0
        868300000,  # Channel 1
        868500000,  # Channel 2
        867100000,  # Channel 3
        867300000,  # Channel 4
        867500000,  # Channel 5
        867700000,  # Channel 6
        867900000,  # Channel 7
    ]
    
    # GPS
    GPS_ENABLED = True
    FAKE_GPS = (33.3152, 44.3661, 50)  # Baghdad coordinates if no GPS
    
    # Statistics
    STAT_INTERVAL = 30  # seconds
    KEEPALIVE_INTERVAL = 60  # seconds


class LoRaPacket:
    """LoRa packet structure"""
    
    def __init__(self, data, rssi, snr, frequency, timestamp):
        self.data = data
        self.rssi = rssi
        self.snr = snr
        self.frequency = frequency
        self.timestamp = timestamp
        self.gateway_eui = Config.GATEWAY_EUI


class PacketForwarder:
    """Handles packet forwarding to network server"""
    
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server = (Config.SERVER_HOST, Config.SERVER_PORT_UP)
        self.token = 0
        self.ack_tokens = {}
        
    def send_packet(self, packet):
        """Forward LoRa packet to network server"""
        # Create UDP packet according to Semtech protocol
        pkt = self._create_rxpk(packet)
        self._send_udp(pkt)
        
    def _create_rxpk(self, packet):
        """Create RXPK JSON structure"""
        rxpk = {
            "time": datetime.utcnow().isoformat() + 'Z',
            "tmst": int(time.time() * 1000000),
            "chan": 0,
            "rfch": 0,
            "freq": packet.frequency / 1000000.0,
            "stat": 1,
            "modu": "LORA",
            "datr": "SF7BW125",
            "codr": "4/5",
            "lsnr": packet.snr,
            "rssi": packet.rssi,
            "size": len(packet.data),
            "data": base64.b64encode(packet.data).decode()
        }
        
        return {
            "rxpk": [rxpk]
        }
    
    def _send_udp(self, data):
        """Send UDP packet with Semtech protocol header"""
        # Protocol header
        version = 0x02
        self.token = (self.token + 1) % 65536
        identifier = 0x00  # PUSH_DATA
        
        # Gateway EUI
        gateway_eui = bytes.fromhex(Config.GATEWAY_EUI)
        
        # JSON payload
        json_data = json.dumps(data).encode()
        
        # Construct packet
        packet = struct.pack('<BHB', version, self.token, identifier)
        packet += gateway_eui
        packet += json_data
        
        # Send
        self.sock.sendto(packet, self.server)
        logger.debug(f"Sent packet to {self.server}: {data}")
        
        # Store token for ACK tracking
        self.ack_tokens[self.token] = time.time()
    
    def send_stat(self, stats):
        """Send gateway statistics"""
        stat_data = {
            "stat": {
                "time": datetime.utcnow().isoformat() + 'Z',
                "lati": Config.FAKE_GPS[0],
                "long": Config.FAKE_GPS[1],
                "alti": Config.FAKE_GPS[2],
                "rxnb": stats['rx_total'],
                "rxok": stats['rx_ok'],
                "rxfw": stats['rx_forwarded'],
                "ackr": stats['ack_ratio'],
                "dwnb": stats['tx_total'],
                "txnb": stats['tx_total']
            }
        }
        
        self._send_udp(stat_data)


class SX1301Interface:
    """Interface for SX1301/SX1308 LoRa concentrator"""
    
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_DEV, SPI_CS)
        self.spi.max_speed_hz = 8000000
        self.reset_hardware()
        
    def reset_hardware(self):
        """Reset the concentrator"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RESET_PIN, GPIO.OUT)
        
        # Reset sequence
        GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(RESET_PIN, GPIO.HIGH)
        time.sleep(0.1)
        
        logger.info("Hardware reset complete")
    
    def configure_channels(self):
        """Configure LoRa channels"""
        # This would contain actual SX1301 register configuration
        # For demonstration, we'll use simplified approach
        logger.info(f"Configured {len(Config.LORA_FREQUENCIES)} channels")
    
    def start_receive(self):
        """Start receiving packets"""
        # In real implementation, this would configure SX1301 for RX
        logger.info("Started packet reception")
    
    def read_packets(self):
        """Read received packets from concentrator"""
        # Simulated packet reading
        # In real implementation, read from SX1301 FIFO
        packets = []
        
        # Simulate some packets
        if os.urandom(1)[0] < 30:  # ~12% chance of packet
            data = os.urandom(20)  # Random payload
            rssi = -120 + os.urandom(1)[0] // 4
            snr = -20 + os.urandom(1)[0] // 10
            freq = Config.LORA_FREQUENCIES[os.urandom(1)[0] % len(Config.LORA_FREQUENCIES)]
            
            packet = LoRaPacket(data, rssi, snr, freq, time.time())
            packets.append(packet)
        
        return packets


class DataCache:
    """Local data caching for resilience"""
    
    def __init__(self, db_path="/var/lib/lorawan/cache.db"):
        self.db_path = db_path
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self.init_db()
        
    def init_db(self):
        """Initialize SQLite database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS packets (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                data BLOB,
                rssi INTEGER,
                snr REAL,
                frequency INTEGER,
                forwarded INTEGER DEFAULT 0,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS statistics (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                rx_total INTEGER,
                rx_ok INTEGER,
                rx_forwarded INTEGER,
                tx_total INTEGER,
                uptime INTEGER
            )
        """)
        
        conn.commit()
        conn.close()
    
    def store_packet(self, packet):
        """Store packet in cache"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT INTO packets (timestamp, data, rssi, snr, frequency)
            VALUES (?, ?, ?, ?, ?)
        """, (packet.timestamp, packet.data, packet.rssi, packet.snr, packet.frequency))
        
        conn.commit()
        conn.close()
    
    def get_unforwarded_packets(self, limit=100):
        """Get packets that haven't been forwarded"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            SELECT id, timestamp, data, rssi, snr, frequency
            FROM packets
            WHERE forwarded = 0
            ORDER BY timestamp ASC
            LIMIT ?
        """, (limit,))
        
        packets = []
        for row in cursor.fetchall():
            packet = LoRaPacket(
                data=row[2],
                rssi=row[3],
                snr=row[4],
                frequency=row[5],
                timestamp=row[1]
            )
            packet.db_id = row[0]
            packets.append(packet)
        
        conn.close()
        return packets
    
    def mark_forwarded(self, packet_ids):
        """Mark packets as forwarded"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.executemany(
            "UPDATE packets SET forwarded = 1 WHERE id = ?",
            [(pid,) for pid in packet_ids]
        )
        
        conn.commit()
        conn.close()
    
    def store_statistics(self, stats):
        """Store gateway statistics"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT INTO statistics (rx_total, rx_ok, rx_forwarded, tx_total, uptime)
            VALUES (?, ?, ?, ?, ?)
        """, (stats['rx_total'], stats['rx_ok'], stats['rx_forwarded'], 
              stats['tx_total'], stats['uptime']))
        
        conn.commit()
        conn.close()


class GPSInterface:
    """GPS interface for time synchronization and location"""
    
    def __init__(self, port=GPS_SERIAL_PORT):
        self.port = port
        self.serial = None
        self.location = Config.FAKE_GPS
        self.time_offset = 0
        
        if Config.GPS_ENABLED:
            try:
                self.serial = serial.Serial(port, 9600, timeout=1)
                logger.info("GPS interface initialized")
            except Exception as e:
                logger.error(f"GPS initialization failed: {e}")
    
    def read_gps(self):
        """Read GPS data"""
        if not self.serial:
            return self.location
        
        try:
            line = self.serial.readline().decode('ascii', errors='ignore')
            if line.startswith('$GPGGA'):
                # Parse NMEA GPGGA sentence
                parts = line.split(',')
                if len(parts) >= 6 and parts[2] and parts[4]:
                    lat = self._parse_coordinate(parts[2], parts[3])
                    lon = self._parse_coordinate(parts[4], parts[5])
                    alt = float(parts[9]) if parts[9] else 0
                    self.location = (lat, lon, alt)
                    
                    # Update time offset
                    if parts[1]:
                        gps_time = datetime.strptime(parts[1].split('.')[0], '%H%M%S')
                        system_time = datetime.now()
                        self.time_offset = (gps_time - system_time).total_seconds()
            
        except Exception as e:
            logger.error(f"GPS read error: {e}")
        
        return self.location
    
    def _parse_coordinate(self, value, direction):
        """Parse NMEA coordinate"""
        degrees = float(value[:2] if direction in ['N', 'S'] else value[:3])
        minutes = float(value[2:] if direction in ['N', 'S'] else value[3:])
        decimal = degrees + minutes / 60
        
        if direction in ['S', 'W']:
            decimal = -decimal
        
        return decimal


class LoRaWANGateway:
    """Main LoRaWAN Gateway class"""
    
    def __init__(self):
        self.running = False
        self.start_time = time.time()
        
        # Statistics
        self.stats = {
            'rx_total': 0,
            'rx_ok': 0,
            'rx_forwarded': 0,
            'tx_total': 0,
            'ack_ratio': 100.0,
            'uptime': 0
        }
        
        # Initialize components
        self.concentrator = SX1301Interface()
        self.forwarder = PacketForwarder()
        self.cache = DataCache()
        self.gps = GPSInterface()
        
        # MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        # Connect to MQTT
        try:
            self.mqtt_client.connect(Config.MQTT_BROKER, Config.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
        
        logger.info(f"Gateway initialized with EUI: {Config.GATEWAY_EUI}")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            client.subscribe(f"{Config.MQTT_TOPIC_PREFIX}/+/down")
        else:
            logger.error(f"MQTT connection failed with code: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback for downlink"""
        try:
            data = json.loads(msg.payload.decode())
            logger.info(f"Downlink message received: {data}")
            # Handle downlink transmission
            self._transmit_downlink(data)
        except Exception as e:
            logger.error(f"Downlink processing error: {e}")
    
    def _transmit_downlink(self, data):
        """Transmit downlink message"""
        # In real implementation, configure SX1301 for TX
        logger.info(f"Transmitting downlink: {data}")
        self.stats['tx_total'] += 1
    
    def start(self):
        """Start the gateway"""
        self.running = True
        
        # Configure concentrator
        self.concentrator.configure_channels()
        self.concentrator.start_receive()
        
        # Start threads
        rx_thread = threading.Thread(target=self._rx_loop)
        forward_thread = threading.Thread(target=self._forward_loop)
        stat_thread = threading.Thread(target=self._stat_loop)
        
        rx_thread.start()
        forward_thread.start()
        stat_thread.start()
        
        logger.info("Gateway started")
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
            self.stop()
    
    def stop(self):
        """Stop the gateway"""
        self.running = False
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        GPIO.cleanup()
        logger.info("Gateway stopped")
    
    def _rx_loop(self):
        """Receive loop"""
        while self.running:
            try:
                # Read packets from concentrator
                packets = self.concentrator.read_packets()
                
                for packet in packets:
                    self.stats['rx_total'] += 1
                    
                    # Validate packet
                    if self._validate_packet(packet):
                        self.stats['rx_ok'] += 1
                        
                        # Store in cache
                        self.cache.store_packet(packet)
                        
                        # Publish to MQTT
                        self._publish_packet(packet)
                        
                        logger.info(f"Packet received: RSSI={packet.rssi}, SNR={packet.snr}")
                
                time.sleep(0.01)  # Small delay
                
            except Exception as e:
                logger.error(f"RX loop error: {e}")
                time.sleep(1)
    
    def _forward_loop(self):
        """Forward packets to network server"""
        while self.running:
            try:
                # Get unforwarded packets from cache
                packets = self.cache.get_unforwarded_packets()
                
                forwarded_ids = []
                for packet in packets:
                    try:
                        self.forwarder.send_packet(packet)
                        self.stats['rx_forwarded'] += 1
                        forwarded_ids.append(packet.db_id)
                    except Exception as e:
                        logger.error(f"Forward error: {e}")
                
                # Mark as forwarded
                if forwarded_ids:
                    self.cache.mark_forwarded(forwarded_ids)
                
                time.sleep(1)
                
            except Exception as e:
                logger.error(f"Forward loop error: {e}")
                time.sleep(5)
    
    def _stat_loop(self):
        """Statistics loop"""
        while self.running:
            try:
                # Update uptime
                self.stats['uptime'] = int(time.time() - self.start_time)
                
                # Update GPS
                location = self.gps.read_gps()
                
                # Send statistics
                self.forwarder.send_stat(self.stats)
                
                # Store statistics
                self.cache.store_statistics(self.stats)
                
                # Log statistics
                logger.info(f"Stats: RX={self.stats['rx_total']}, "
                          f"OK={self.stats['rx_ok']}, "
                          f"FWD={self.stats['rx_forwarded']}, "
                          f"TX={self.stats['tx_total']}")
                
                time.sleep(Config.STAT_INTERVAL)
                
            except Exception as e:
                logger.error(f"Stat loop error: {e}")
                time.sleep(Config.STAT_INTERVAL)
    
    def _validate_packet(self, packet):
        """Validate received packet"""
        # Basic validation
        if len(packet.data) < 7:  # Minimum LoRaWAN packet size
            return False
        
        if packet.rssi < -140 or packet.rssi > -20:
            return False
        
        return True
    
    def _publish_packet(self, packet):
        """Publish packet to MQTT"""
        try:
            topic = f"{Config.MQTT_TOPIC_PREFIX}/rx"
            payload = {
                'gateway_eui': packet.gateway_eui,
                'timestamp': packet.timestamp,
                'rssi': packet.rssi,
                'snr': packet.snr,
                'frequency': packet.frequency,
                'data': base64.b64encode(packet.data).decode()
            }
            
            self.mqtt_client.publish(topic, json.dumps(payload))
            
        except Exception as e:
            logger.error(f"MQTT publish error: {e}")


if __name__ == "__main__":
    gateway = LoRaWANGateway()
    gateway.start()