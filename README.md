# Multi-Threaded Buck Converter Control System

## Overview
This project implements a real-time, RTOS-based control system for a buck converter-driven LED current regulator. The system combines embedded software, power electronics, and fault-tolerant design to maintain stable current output under real-time constraints.

Built using RTXv5 RTOS, the system demonstrates advanced concepts including multi-threading, interrupt-driven control loops, synchronization, and fault detection with recovery mechanisms.

---

## Key Features
- RTOS-based multi-threaded architecture (RTXv5)
- Closed-loop current control using PWM
- Interrupt-driven control loop (ADC + Timer ISR)
- Real-time waveform visualization (oscilloscope-style)
- Synchronization using mutexes and event flags
- Shared ADC resource management across threads
- Fault detection and recovery mechanisms
- Watchdog-based system recovery

---

## Hardware

### Buck Converter Architecture

![Buck Converter Architecture](hardware/Buck%20Converter%20Architecture.png)

The power stage consists of a buck converter circuit (MOSFET, diode, inductor, capacitor) driving an LED load with regulated current. Current sensing is achieved via a resistor feedback network.

### Pin Connections

![Pin Connections](hardware/Pin%20Connections.png)

### Working Hardware

![Buck Converter Working Hardware](hardware/Buck%20Converter%20Working%20Hardware.jpeg)

---

## System Architecture

The system is composed of tightly integrated hardware and software components:

### Power Stage
- Buck converter circuit (MOSFET, diode, inductor, capacitor)
- LED load driven with regulated current
- Current sensing via resistor feedback

---

### Control System (RTOS-Based)

The control system runs on multiple concurrent threads:

- **Control Loop (ISR-driven)**
  - Executes in ADC interrupt handler
  - Updates PWM duty cycle using feedback
  - Maintains desired current setpoint  

- **Thread_Update_Setpoint**
  - Adjusts current setpoint dynamically  

- **Thread_Draw_Waveforms**
  - Displays real-time current waveforms  
  - Uses buffered data for stable visualization  

- **Thread_Draw_UI_Controls**
  - Updates system parameters on LCD  

- **Thread_Read_Touchscreen**
  - Handles user input via ADC sharing  

- **Thread_Fault_Injector**
  - Injects faults for testing robustness  

- **ADC Server (ISR + Queue)**
  - Manages shared ADC access across threads  

This architecture ensures time-critical control is isolated from lower-priority tasks. 

---

## Control Strategy

- PWM generated using timer (TPM module)
- Control loop executed at lower frequency than switching frequency
- ADC sampling synchronized with control loop
- Feedback-based duty cycle adjustment

The system balances:
- High switching frequency → reduced ripple  
- Lower control frequency → reduced CPU load 

---

## Synchronization & Real-Time Design

To maintain deterministic behavior:

- **Mutexes**
  - Protect LCD access across threads  
  - Prevent race conditions  

- **Event Flags (RTOS)**
  - Synchronize ADC ISR and waveform rendering  
  - Enable oscilloscope-style triggered display  

- **State Machine (Alternative Approach)**
  - Deterministic synchronization without RTOS overhead  

This ensures safe interaction between ISR and threads without data corruption.

---

## Fault Detection & Reliability

The system is designed to detect and recover from faults in real time:

### Detection Techniques
- Range and sanity checks on data  
- Timing analysis and thread monitoring  
- PRIMASK register monitoring (interrupt faults)  
- RTOS error callbacks  

### Recovery Mechanisms
- Immediate interrupt re-enablement  
- Data correction and fallback strategies  
- Thread recreation  
- System reset via watchdog timer  

The watchdog ensures recovery if the system becomes unresponsive. 

---

## Example Fault Handling

A critical fault where interrupts are disabled is detected by a monitoring thread:

- Detection latency: ~5–10 ms  
- Recovery time: ~50–100 µs  
- System resumes normal operation without reset  

Without protection → complete system freeze  
With protection → seamless recovery  

This demonstrates robust fault tolerance in real-time systems. 

---

## Results

Complete analysis, timing data, synchronization behavior, and fault testing results are documented here:

📄 [`results/Buck_Converter_Report.pdf`](results/Buck_Converter_Report.pdf)

---

## Key Learnings
- Designing real-time embedded systems using RTOS  
- Synchronizing ISR and threads safely  
- Implementing fault-tolerant embedded architectures  
- Managing shared hardware resources (ADC)  
- Debugging with logic analyzers and waveform tools  

---

## Future Improvements
- Advanced PID tuning and adaptive control  
- FreeRTOS port for cross-platform compatibility  
- Efficiency and thermal optimization  
- Hardware PCB implementation  

---

## Author
**Archit Kulkarni**
