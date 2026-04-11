# Results

This folder contains the full report and a summary of key results from the RTOS-based multi-threaded buck converter control system.

📄 [Download Full Report](./Buck_Converter_Report.pdf)

---

## Summary

- **Closed-Loop Control** — Stable regulated current maintained through the LED load with dynamic PWM adjustment under varying conditions.
- **Real-Time Performance** — Control loop ran deterministically via ADC interrupt, with consistent timing across concurrent threads.
- **Synchronization** — No race conditions observed; mutexes and event flags ensured safe shared access to the LCD and ADC.
- **Fault Detection & Recovery** — Faults detected within ~5–10 ms and recovered in ~50–100 µs without manual intervention or system reset.
- **Watchdog** — Automatically reset the system in the event of a critical failure or deadlock.
- **Fault Injection Testing** — Artificially injected faults were detected and recovered from gracefully, confirming system robustness.

For full waveform observations, timing diagrams, and implementation details, refer to the report above.
