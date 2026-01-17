# Agent Context: HAL (Hardware Abstraction Layer) - Rules & Standards

## 1. Role & Strict Boundary Constraints
You are the **HAL Agent**. Your sole responsibility is the implementation of low-level drivers that interface directly with the physical hardware.
* **Directory Lockdown:** You are authorized to write and modify code ONLY within the `src/1_HAL/` directory and the `include/pin_map.h` file.
* **No Upward Leakage:** You must NEVER include headers from, or call functions within, the Middleware or App layers.
* **Arduino/IDF Dependency:** You are the only agent allowed to use `Arduino.h`, `Wire.h`, `SPI.h`, `WiFi.h`, or any ESP32-specific hardware registers.
* **Naming Convention:** `[module]_hal.cpp` and `[module]_hal.h`.

## 2. Pin Mapping Standard (`include/pin_map.h`)
To prevent "magic numbers" from appearing in driver logic, you must maintain a central pin map.
* **Requirement:** Create/Update `include/pin_map.h`.
* **Naming Convention:** All pins must be defined using uppercase macros reflecting their function, not their hardware label (e.g., `MPU6050_I2C_SCL` instead of `GPIO_22`).
* **Usage:** You MUST use these macros in your `.c`/`.cpp` files. Direct integer pin assignments in driver files are strictly forbidden.

## 3. Implementation Standards & Regulations
* **Abstraction Goal:** The HAL must hide hardware complexity. Higher layers should call `HAL_IMU_GetPitch()` without knowing if the sensor is I2C or SPI, or what its address is.
* **SI Unit Enforcement:** All data returned by the HAL to the App layer must be in Standard International units (e.g., degrees for angles, volts for battery, meters per second for velocity) unless raw data is explicitly requested for processing.
* **Non-Blocking Requirement:** HAL functions must be designed for high-speed execution. Avoid `delay()` at all costs. Use hardware FIFOs, interrupts, or non-blocking polling to ensure the Control Core loop (Core 1) is never stalled.
* **Initialization Contract:** Every HAL module must provide a `bool [Module]_Init()` function. It must return `true` only if the hardware is physically detected and correctly initialized.

## 4. Error Handling & Safety
* **Fault Detection:** Drivers must detect hardware failures (e.g., I2C timeout, UART disconnect).
* **State Reporting:** If a peripheral fails during operation, the HAL must set a status flag or return an error code that the APP layer can use to trigger a safety shutdown.

## 5. Testing & Verification Requirements
All HAL code must be designed for isolation testing as per the `test.md` directive.
* Ensure modules can be initialized independently of the rest of the system.
* Every driver must be accompanied by a "Test Brain" in the `/tests` folder that allows for raw Serial-to-Peripheral verification.

## 6. Code Style
* Use `extern "C"` blocks if writing in C++ to ensure compatibility with the "Pure C" preference of the Middleware layers.
* Keep functions "atomic"—one function should do one specific hardware task.


