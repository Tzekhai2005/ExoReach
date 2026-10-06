# Firmware Development Log

The final firmware (`final/main_demoday.cpp`) wasn't written in one pass. It
was built up subsystem by subsystem, with each new piece of hardware brought
up and verified in isolation before being wired into the wider system. This
log walks through that sequence and points at the file for each stage.

## Stage 1 — Bring-up (`development/01_bringup/`)

Each subsystem was tested standalone, off the critical path of the others,
so a wiring or driver bug in one couldn't be confused for a bug in another.

| File | What it verified |
|---|---|
| `01_test_dual_adc_dma.c` | Dual-channel continuous ADC acquisition for the two EMG channels, plus the notch/high-pass/envelope filter chain, run in isolation before anything else touched the ADC. |
| `02_test_adc_diagnostic.cpp` | A diagnostic harness for that same acquisition path — raw counts, per-channel min/mean/max, and channel-tag sync checks — used to separate genuine hardware/wiring issues from software bugs. |
| `03_serial_echo_test.cpp` | Isolated the console input pattern (dedicated blocking-read task + non-blocking heartbeat) that the rest of the firmware relies on for runtime commands, before adding it anywhere real-time-sensitive. |
| `04_foc_motor_test.cpp` / `05_foc_motor_test_v2.cpp` | Motor and encoder bring-up: SimpleFOC calibration (zero electrical angle, sensor direction), then closed-loop voltage control from serial input, run alone on the bench with the shaft free. |
| `06_imu_module_dohun_kim.cpp` | BNO055 IMU bring-up — I²C driver, high-g interrupt, and the spasm/collision detection heuristics. Written by Dohun Kim as a standalone module (see attribution note in the file). |

## Stage 2 — Subsystem integration (`development/02_integration/`)

Once each piece worked alone, they were combined incrementally — one mock
replaced by the real thing at a time — so a regression could always be
traced to the one subsystem that had just changed.

1. **`01_emg_control_test.cpp`** — the real EMG pipeline (acquisition →
   filtering → MVC calibration → feature extraction → torque law → safety
   limits) wired up, but FOC, IMU, and AI mode selection were all mocked
   (static angle/velocity, flags set manually over serial).
2. **`02_emg_plus_real_foc.cpp`** — the FOC mock replaced with the real
   motor control loop from Stage 1, so EMG effort now drove an actual
   shaft. IMU and AI were still mocked.
3. **`03_integration_real_imu.cpp`** — the IMU mock replaced with the real
   BNO055 driver, bringing in live gravity-angle compensation and
   hardware-verified spasm/collision detection. AI mode selection was
   still driven from serial.
4. **`04_integration_v2.cpp`** — a refinement pass on the above (adds a
   `USE_STORED_CALIB` path so the motor could skip re-alignment on every
   boot once calibration constants were known).

## Stage 3 — Data collection (`development/03_data_collection/`)

**`data_collection.cpp`** adds a labelled CSV export mode on top of the
Stage 2 firmware: five debug modes stream time-stamped feature vectors
(RMS, MAV, WL, ZC, SSC, VAR per channel, plus IMU and encoder state) tagged
with the operator-selected label (Rest / Reach / Lift / Hold / Spasm), with
and without the motor energised, so recordings could be checked for
motor-EMI contamination before being used to train the classifier. This is
the dataset the on-device neural network (Appendix E9 of the final report)
was trained on.

## Final system (`firmware/final/main_demoday.cpp`)

All of the above, plus the trained TensorFlow Lite Micro classifier
replacing the serial-driven mock mode selector, and the full safety
hierarchy (driver fault → collision → spasm → effort deadband → ROM
soft-brake → torque/slew-rate limiting) in its final form. This is the
firmware that ran on demo day.
