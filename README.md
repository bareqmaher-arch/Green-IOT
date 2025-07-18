# Green IoT Smart City System


A comprehensive Green IoT implementation for sustainable smart cities, featuring edge computing, solar-powered infrastructure, and real-time environmental monitoring.

## 🌍 Overview

This project implements a complete Green IoT ecosystem for smart cities, focusing on:
- **Energy Efficiency**: Smart street lighting with solar power integration
- **Environmental Monitoring**: Real-time air quality sensing with edge computing
- **Sustainable Infrastructure**: LoRaWAN communication for low-power, long-range connectivity
- **Data Intelligence**: Machine learning for anomaly detection and predictive maintenance

## 📋 Features

### Smart Street Lighting
- Adaptive brightness control based on ambient light and motion detection
- Solar panel integration with battery management
- Energy consumption monitoring and optimization
- LoRaWAN connectivity for remote management
- Achieved 73% energy savings compared to traditional lighting

### Air Quality Monitoring
- Multi-parameter sensing (PM2.5, PM10, CO, NO2, temperature, humidity)
- Edge computing for real-time data processing
- Machine learning-based anomaly detection
- Alert system for hazardous conditions
- Web-based dashboard with real-time visualization

### System Architecture
- Distributed edge computing nodes
- MQTT-based communication
- InfluxDB time-series data storage
- Grafana dashboards for visualization
- Docker containerization for easy deployment

## 🚀 Quick Start

### Prerequisites
- Raspberry Pi 4 (for edge nodes)
- Arduino/ESP32 (for sensor nodes)
- Docker and Docker Compose
- Python 3.8+
- Node.js 14+ (for dashboard)

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/bareqmaher-arch/Green-IOT
cd green-iot
```



2. **Deploy with Docker**
```bash
make build
make deploy
```

3. **Flash sensor nodes**
```bash
# For Arduino IDE
# Open smart_streetlight_esp32.ino or air_quality_node.ino
# Select your board and port
# Upload the sketch

# For MicroPython (ESP32)
esptool.py --chip esp32 erase_flash
esptool.py --chip esp32 write_flash -z 0x1000 esp32-20210902-v1.17.bin
ampy --port /dev/ttyUSB0 put main.py
```

## 🔧 Hardware Setup

### Street Light Node
- **Microcontroller**: ESP32/ESP8266
- **Sensors**: PIR (HC-SR501), LDR, INA219 (power monitoring)
- **Actuators**: LED array with MOSFETs
- **Power**: Solar panel (10W), Li-ion battery (18650), TP4056 charger
- **Communication**: LoRa module (SX1276/SX1278)

### Air Quality Node
- **Microcontroller**: ESP32 or Arduino Mega
- **PM Sensor**: PMS5003/PMS7003
- **Gas Sensors**: MQ-7 (CO), MQ-135 (Air Quality)
- **Environmental**: BME280 (temperature, humidity, pressure)
- **Communication**: LoRa module or WiFi

### Gateway
- **Hardware**: Raspberry Pi 4
- **LoRa Concentrator**: SX1301/SX1308
- **Storage**: 32GB SD card minimum
- **Connectivity**: Ethernet/WiFi for backhaul

## 📊 Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Sensor Nodes   │     │  Sensor Nodes   │     │  Sensor Nodes   │
│   (ESP32)       │     │   (Arduino)     │     │   (ESP8266)     │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │ LoRaWAN               │ LoRaWAN               │ LoRaWAN
         └───────────────────────┴───────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │   LoRaWAN Gateway       │
                    │   (Raspberry Pi)        │
                    └────────────┬────────────┘
                                 │ MQTT
                    ┌────────────┴────────────┐
                    │    Edge Processing      │
                    │  - Data Aggregation     │
                    │  - ML Inference         │
                    │  - Alert Generation     │
                    └────────────┬────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │     Cloud Platform      │
                    │  - InfluxDB             │
                    │  - Grafana              │
                    │  - Web Dashboard        │
                    └─────────────────────────┘
```

## 💻 API Reference

### MQTT Topics
- `airquality/data` - Air quality sensor data
- `streetlight/status` - Street light status updates
- `gateway/stats` - Gateway statistics
- `alerts/+` - System alerts

### REST API Endpoints
```
GET  /api/v1/sensors          - List all sensors
GET  /api/v1/sensors/:id      - Get sensor details
GET  /api/v1/data/latest      - Get latest readings
GET  /api/v1/data/historical  - Get historical data
POST /api/v1/alerts           - Create alert
GET  /api/v1/system/status    - System status
```

## 📈 Performance

### Energy Savings
- Street lighting: 73% reduction in energy consumption
- Sensor nodes: 6-month battery life with solar charging
- Gateway: <10W power consumption

### Data Processing
- Edge processing reduces cloud traffic by 85%
- Real-time anomaly detection with <100ms latency
- Support for 1000+ sensor nodes per gateway

## 🔒 Security

- TLS encryption for MQTT communication
- API authentication with JWT tokens
- Secure boot on edge devices
- Regular security updates

## 🧪 Testing

Run the test suite:
```bash
make test
```

Run specific tests:
```bash
python -m pytest tests/test_air_quality.py
python -m pytest tests/test_streetlight.py
```

## 📝 Configuration

Edit `config.yaml` for system-wide settings:

```yaml
system:
  name: "Green IoT Smart City"
  location:
    city: "Baghdad"
    coordinates:
      latitude: 33.3152
      longitude: 44.3661

sensors:
  air_quality:
    pm_sensor:
      type: "PMS5003"
      serial_port: "/dev/ttyUSB0"
```

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- **Karrar M. Khudhair** - *Initial work* - [karrarmaher@iku.edu.iq](mailto:karrarmaher@iku.edu.iq)
- **Bareq M. Khudhair** - *Initial work* - [bareq.maher@iku.edu.iq](mailto:bareq.maher@iku.edu.iq)

## 🙏 Acknowledgments

- Imam Alkadhim University, Department of Computer Engineering
- Open source communities for libraries and tools
- Contributors and testers

## 📚 Documentation

- [Installation Guide](docs/installation.md)
- [Hardware Assembly](docs/hardware.md)
- [API Documentation](docs/api.md)
- [Troubleshooting](docs/troubleshooting.md)

## 📊 Project Status

- [x] Core system implementation
- [x] Street lighting module
- [x] Air quality monitoring
- [x] Edge computing integration
- [x] Web dashboard
- [ ] Mobile application
- [ ] Advanced ML models
- [ ] Multi-city deployment

## 🌟 Star History

[![Star History Chart](https://api.star-history.com/svg?repos=bareqMaher/greeniot-smartcity&type=Date)](https://star-history.com/#KarrarMaher/greeniot-smartcity&Date)

---
