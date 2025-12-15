# Autonomous Sensor Node Design

## Course Project – Group 6

### Team Members
- Chengyu Yang  
- Arno Liu  
- Shijia Liu  
- Zheyang Zheng  

---

## Project Overview

This project focuses on the design and implementation of **autonomous sensor nodes** capable of long-term operation using onboard power supplies. The sensor nodes are designed to independently acquire, process, and transmit sensor data, targeting applications in wearable, industrial, and medical electronics.

Two types of sensor nodes were developed in this project:
- **Photoplethysmography (PPG) Sensor Node**
- **Accelerometer (ACC) Sensor Node**

Each sensor node integrates sensing elements, power management, and a microcontroller unit (MCU) with its peripheral circuitry.

---

## Project Objectives

- Design complete sensor node hardware, including schematic and PCB layout  
- Implement low-power power management and dual-path power supply  
- Integrate MCU, sensors, and peripheral circuits  
- Verify functionality through prototype assembly and testing  
- Characterise sensor performance and validate signal acquisition  

---

## Repository Structure

```text
.
├── Datasheet_tech notes/
│   └── Datasheets and technical notes for key components
│
├── PCB_design/
│   └── Early PCB design files created using KiCad.
│       This folder contains the initial PCB design iterations
│       before the project was migrated to EasyEDA.
│
├── Final PCB Design using Easy EDA/
│   └── Finalised PCB schematics and layout designed in EasyEDA.
│       All later revisions and manufacturing-ready files are
│       maintained in this directory.
│
├── Labbook/
│   └── Laboratory notes, testing logs, and debugging records
│
├── autonomous_sensor_code/
│   └── PPGtest/
│       └── Firmware and test code for the PPG sensor node
│
├── Idea on components selection
│   └── Early design notes and rationale for component selection
│
├── ppg_processed_data.csv
│   └── Processed PPG measurement data used for analysis
│
├── README.md
│   └── Project overview and objectives
│
├── .gitattributes
│   └── Git configuration file
│
└── .DS_Store
    └── System file (can be ignored)



