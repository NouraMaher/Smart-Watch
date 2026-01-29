# ⌚ Smart Watch Health Monitoring System  
### ESP32-Based Smart Health Tracker
#### An ESP32-based smart watch prototype for essential health tracking with a focus on affordability and accuracy.

<p align="center">
  <img src="https://img.shields.io/badge/ESP32-Embedded-blue?style=for-the-badge">
  <img src="https://img.shields.io/badge/IoT-Health%20Tech-green?style=for-the-badge">
  <img src="https://img.shields.io/badge/Status-Completed-success?style=for-the-badge">
</p>

---

## 📌 Overview
This project presents a **Smart Watch Health Monitoring System** based on the **ESP32 microcontroller**.  
It focuses on essential health tracking such as **Heart Rate, SpO₂, and Activity Monitoring** using low-cost and efficient sensors.

The system is designed as an affordable alternative to premium smart watches.

---

## 🎯 Problem Statement
Most commercial smart watches are expensive and include many unnecessary features.  
This project aims to provide a **low-cost, focused, and reliable health tracker** for essential daily monitoring.

---

## ⚙️ System Functionality
The system operates in real time as follows:

- Continuously reads biometric and motion data
- Processes and filters raw sensor signals
- Displays health data on a TFT screen
- Allows smooth navigation between multiple UI modes

---

## 🧠 How It Works
1. Sensors send real-time data to the ESP32.
2. The firmware filters noise using Moving Average algorithms.
3. Heart rate and SpO₂ values are calculated from raw signals.
4. Step detection is performed using motion analysis.
5. Data is displayed through a multi-screen TFT user interface.

---

## 🧩 Hardware Components
| Component | Description |
|---------|------------|
| ESP32 | Main controller |
| MAX30102 | Heart Rate & SpO₂ Sensor |
| MPU6050 | Motion Sensor (Accelerometer & Gyro) |
| TFT LCD 240×240 | Display |
| Li-Po Battery 3.7V | Power supply |

---

## 🔌 Communication & Interfaces
- I²C: MAX30102, MPU6050  
- SPI: TFT Display  

---

## 💻 Software & Tools
- Arduino IDE  
- C/C++ (Embedded Programming)  
- TFT Display Libraries (ILI9xxx)  
- I²C & SPI Protocols  

---

## 🚀 Future Enhancements
- Waterproof design (IP68)  
- Mobile app integration via BLE  
- Long-term health data storage  
- Smart notifications and alerts  

---
## 📜 Copyright

This project is available for personal and educational use. If you intend to use it for commercial purposes, please contact me for permission.

---

## 📞 Contact

For questions or suggestions, please open an issue or contact:
Noura Maher Elamin
[LinkedIn](https://www.linkedin.com/in/nouramaher/)
[GitHub](https://github.com/NouraMaher)

---
