# ThermoScout: Thermoelectric-Powered Surveillance Rover
ThermoScout is an autonomous, thermoelectric-powered surveillance rover designed to operate without external electricity. It uses Thermoelectric Generators (TEGs) to harvest waste heat and power all subsystems, including mobility, sensing and onboard AI threat detection. The rover performs real-time fire and knife detection using lightweight machine learning models on a Raspberry Pi, while autonomously navigating using ultrasonic sensing. This repository contains the complete hardware, software, documentation and methodology for the system.

## 📁 Repository Structure
The entire repository is organized as follows:
```txt
ThermoScout-Thermoelectric-Surveillance-Robot/
│
├── README.md
│
├── CODE/
│   └── main.py
│
├── HARDWARE/
│   ├── Circuit_Diagram.jpg
│   ├── TEG_Setup.jpg
│   └── Chassis_Design
│      ├── Level1.png
│      ├── Level2.png
│      └── Level3.png
│
├── DOCUMENTATION/
│   ├── IEEE_Paper.pdf
│   └── EL_Report.pdf
│
└── MEDIA/
    ├── Images/
    │   ├── Model_View.jpg
    │   ├── Model_Front_View.jpg
    │   ├── Fire_Detection.jpg
    │   ├── Knife_Detection.jpg
    │   └── TEGs.jpg
    └── Videos/
        ├── TEG_Testing.mp4
        ├── Fire_Detection.mp4
        └── Knife_Detection.mp4

```

## 🚀 Project Overview
Traditional surveillance systems rely on fixed cameras and external power, which makes them unsuitable for remote, hazardous or infrastructure-limited environments. ThermoScout eliminates these limitations by using TEG-powered energy harvesting, embedded AI, mobile robotics and autonomous navigation. It performs continuous monitoring, detects fire and knife threats in real-time and navigates autonomously using ultrasonic distance measurements—while being powered only by harvested heat energy.

## ⚡ Key Features
• Energy-autonomous operation using thermoelectric generators  
• Real-time fire and knife detection using Roboflow pre-defined models  
• Ultrasonic sensor + servo-based obstacle detection and avoidance  
• 4-wheel locomotion controlled via L298N motor driver  
• Raspberry Pi 4B for all processing, running fully headless  
• Boost + buck converter–based power conditioning  
• Completely off-grid surveillance capability  

## 🧩 System Architecture
ThermoScout consists of three primary subsystems:  
1. Power Generation & Conditioning: Four TEC1-12706 TEGs generate voltage depending on temperature differentials (~0.19V at ΔT=30°C, ~0.38V at ΔT=50°C). The MT3608 boost converter amplifies this voltage and charges Li-ion batteries, which then power motors and sensors through buck converters.  
2. Mobility & Sensing: A 4-wheel platform powered by DC gear motors is controlled using an L298N driver. A Raspberry Pi Camera captures real-time video, while an HC-SR04 ultrasonic sensor mounted on a servo performs directional scanning for obstacle detection.  
3. AI Computation & Threat Detection: Fire and knife detection models trained on Roboflow run on-device using TensorFlow Lite. OpenCV processes live frames from the camera. Alerts are triggered via buzzer and LEDs when threats are detected.  

System workflow:  
Thermal Input → TEGs → Boost → Battery → Buck → Motors + Pi  
Camera Input → AI Model → Threat Detected? → Alerts  
Ultrasonic Data → Navigation Logic → Movement Decisions  

## 🛠️ Hardware Used
• Raspberry Pi 4B  
• Raspberry Pi Camera Module  
• TEC1-12706 TEG modules × 4  
• MT3608 Boost Converter  
• 18650 Li-ion Batteries × 2  
• Buck Converters (5V and 6V)  
• L298N Motor Driver  
• DC Gear Motors × 4  
• HC-SR04 Ultrasonic Sensor  
• SG90 / TowerPro Micro Servo  
• Aluminum Heat Sinks × 4  
• Wheels + 4-wheel chassis  
• Thermal paste, wiring, jumpers  

## 💻 Software Stack
• Python 3  
• OpenCV 
• TensorFlow Lite
• RPi.GPIO  
• Roboflow (training datasets)  
• VNC Viewer  
• Raspberry Pi OS  

## 🔥 AI Threat Detection
### Fire Detection
The Pi Camera captures frames, which are passed through a lightweight fire detection Roboflow model. The system draws bounding boxes around fire regions and activates LED/buzzer alerts when fire is detected.

### Knife Detection
A Roboflow-trained knife model identifies sharp objects in various orientations and lighting. Real-time detections are visualized using bounding boxes, and alerts are triggered for confirmed threats.

Screenshots of these detections are stored in /MEDIA/Images.

## 🧪 Testing & Validation
### Energy Harvesting Results
• ΔT = 30°C → ~0.19V per TEG  
• ΔT = 50°C → ~0.38V per TEG  
The harvested voltage was successfully boosted and regulated to run the entire rover.

### Navigation & Sensor Testing
The ultrasonic sensor provided stable distance readings. The robot reliably stopped, turned and avoided obstacles during movement.

### AI Detection Testing
Fire and knife detection were validated with controlled tests. Bounding boxes were displayed correctly and alerts were triggered instantly.

### Full System Integration
Energy harvesting, locomotion, sensor feedback and AI inference were tested together. All subsystems operated successfully in unified execution.

## 📸 Media
The /MEDIA folder includes:  
• Model photos  
• TEG setup images  
• Fire detection outputs  
• Knife detection outputs  
• Video demonstrations  

## 👥 Contributors
Surabhi M  
Dishita Reddy Garudadri  
Kshama Bhat K  
Reshmi M
