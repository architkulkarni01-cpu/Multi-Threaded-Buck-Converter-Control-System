# Results & Analysis

## Overview
This folder contains the complete results, analysis, and validation of the RTOS-based multi-threaded buck converter control system.

All detailed observations, timing analysis, synchronization behavior, and fault recovery performance are documented in the full report below.

---

## 📄 Full Report

👉 [Download Project Report](./Buck_Converter_Report.pdf)

---

## Key Results Summary

### Stable Closed-Loop Control
- Successfully maintained regulated current through LED load  
- PWM duty cycle dynamically adjusted using feedback  
- Stable operation observed under varying conditions  

---

### Real-Time Performance
- Control loop executed deterministically via ADC interrupt  
- System maintained timing consistency despite multiple concurrent threads  
- Separation of ISR (critical) and threads (non-critical) ensured real-time behavior  

---

### Multi-Threading & Synchronization
- Concurrent threads operated without race conditions  
- Mutexes ensured safe access to shared peripherals (LCD)  
- Event flags enabled synchronized waveform rendering  
- ADC resource successfully shared across threads using ISR-based handling  

---

### Fault Detection & Recovery

The system demonstrated strong fault tolerance through:

- Detection of invalid system states and timing anomalies  
- Monitoring of interrupt behavior (e.g., PRIMASK faults)  
- RTOS error handling mechanisms  

#### Recovery Performance
- Fault detection latency: ~5–10 ms  
- Recovery time: ~50–100 µs  
- System resumed normal operation without manual intervention  

---

### Watchdog Reliability
- Watchdog timer implemented as a fail-safe mechanism  
- System automatically reset in case of critical failure  
- Ensured robustness against system hangs or deadlocks  

---

### Fault Injection Testing
- Artificial faults introduced to evaluate system robustness  
- System successfully:
  - Detected faults  
  - Recovered gracefully  
  - Maintained operational stability  

---

## What This Validates

These results demonstrate:

- Reliable real-time control using RTOS  
- Safe interaction between ISR and threads  
- Robust handling of shared hardware resources  
- Fault-tolerant embedded system design  
- Practical implementation of watchdog-based recovery  

---

## Notes
For complete technical details, waveform observations, timing diagrams, and implementation specifics, refer to the full report.
