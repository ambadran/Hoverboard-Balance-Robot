# Project Architecture & Component Wiring

This document tracks the dependency graph of the "Self-Balancing Hover-Bot" project.
It serves as the "source of truth" for how components interact across the strict layered architecture.

## 1. Architectural Rules (The "Gravity" of Code)
*   **Dependency Direction:** Strictly **DOWNWARDS**.
    *   `3_APP` -> depends on -> `2_MID` & `1_HAL`
    *   `2_MID` -> depends on -> `1_HAL`
    *   `1_HAL` -> depends on -> **Hardware Only** (No other internal layers).
*   **Initialization:** Orchestrated explicitly by `3_APP/main.c`. Lower layers do not initialize their dependencies; they assume they are ready.

---

## 2. Middleware Layer (`src/2_MID/`)

The Logic & Systems Layer.

### A. Component: `CONTROL/pid_controller`
*   **Responsibility:** Abstract PID logic, Zero-Calibration, and Runtime Tuning.
*   **Type:** Stateless Logic / Control Loop.
*   **External Libraries:** `QuickPID` (PlatformIO Lib).
*   **Downstream Dependencies (HAL):**
    1.  **`src/1_HAL/eeprom_hal`**:
        *   **Usage:** `HAL_EEPROM_LoadPID()`
        *   **Context:** Called during `MID_CONTROL_PID_Init` to restore saved tuning parameters.
    2.  **`src/1_HAL/imu_hal`**:
        *   **Usage:** `HAL_IMU_GetPitch()` (Mapped via `PID_INPUT_FUNC` in `config.h`)
        *   **Context:**
            *   **Init:** Called repeatedly to calculate `zero_offset`.
            *   **Step:** Called every loop cycle to get the process variable (Input).
*   **Configuration (`include/config.h`):**
    *   `PID_INPUT_FUNC`: Maps the abstract PID input to the concrete `HAL_IMU_GetPitch` function.
    *   `PID_P_MODE`, `PID_D_MODE`: Configures QuickPID behavior.
    *   `MPU_OFF_*`: Calibration offsets for the underlying IMU.

---

## 3. HAL Layer (`src/1_HAL/`)

The Hardware Abstraction Layer. These are "Leaf Nodes" in our dependency graph.

### A. Component: `eeprom_hal`
*   **Dependencies:** None (Internal Flash API).
*   **Role:** Persists Structs (PID Params) to Non-Volatile Memory.

### B. Component: `imu_hal`
*   **Dependencies:** `Wire` (Arduino I2C).
*   **Role:** Manages MPU6050 and DMP (Digital Motion Processor).

### C. Component: `motor_hal`
*   **Dependencies:** `Serial` (UART).
*   **Role:** Sends FOC frames to the Hoverboard mainboard.

---

## 4. Application Layer (`src/3_APP/`)

*To be documented as implemented (Workers, Main).*
