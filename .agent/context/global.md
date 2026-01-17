# Global Project Context: Self-Balancing Hover-Bot (Core-Specific Architecture)

## 1. Project Vision & Objective
The objective is the development of a high-torque, self-balancing two-wheeled robot using industrial-grade brushless DC (BLDC) motors. The system utilizes a distributed control architecture:
* **Master (Logic):** ESP32 Microcontroller (Dual-Core).
* **Slave (Power/Commutation):** STM32-based Hoverboard Mainboard running the "Emanuel Feru" FOC/Sinusoidal firmware variant.


## 2. Technical Stack & Toolchain
* **Hardware Platform:** ESP32-WROOM-32 (DevKit V1).
* **Sensors:** MPU6050 6-Axis IMU (Interfaced via I2C).
* **Actuators:** Dual 250W-350W Hoverboard Hub Motors.
* **Environment:** PlatformIO CLI within a Linux/Tmux/Vim workflow.
* **Framework:** Arduino-ESP32 (leveraging the underlying FreeRTOS).
* **Communication:** UART (Motor Control), I2C (IMU), WiFi/Bluetooth (Telemetry & Tuning).
* **(IMP) Language:** (V. IMPORTANT) Supposed to be C++ However, we will *ALWAYS* implement with *"Pure C"* whenever possible.


## 3. Strict Layered Architecture (Dependency Inversion)
To ensure code maintainability and testability, the project is strictly partitioned. No layer may "skip" a level to talk to hardware or high-level logic.

1.  **HAL (Hardware Abstraction Layer):** * **Scope:** `src/1_HAL/`
    * **Responsibility:** Encapsulates peripheral calls (Wire, Serial, WiFi-Hardware-Init, BT-Stack-Init).
    * **Rule:** Must only return raw-data structs or SI-unit floats. No control logic allowed.

2.  **MIDDLEWARE - CONTROL:**
    * **Scope:** `src/2_MID/CONTROL/`
    * **Responsibility:** High-level math: Sensor Fusion (Filters) and PID Control Law.
    * **Rule:** Zero hardware dependencies (`No Arduino.h`), unless utilizing internal hardware multipliers/dividers for FPU optimization. Must be syntax-testable on host machines; performance-tested on the MCU.

3.  **MIDDLEWARE - SIGNAL:**
    * **Scope:** `src/2_MID/SIGNAL/`
    * **Responsibility:** Communication session management (Bluetooth/WiFi protocol handling). 
    * **Logic:** Encapsulates JSON parsing, command buffering, and protocol-specific data framing.

4.  **MIDDLEWARE - OS:**
    * **Scope:** `src/2_MID/OS/`
    * **Responsibility:** **The System Mechanism.** Provides the `os_handler` which encapsulates FreeRTOS task creation, core affinity, and thread-safe "Shared Memory" accessors (Mutex/Atomic wrappers).

5.  **APP - WORKERS:**
    * **Scope:** `src/3_APP/WORKERS/`
    * **Responsibility:** **The Business Logic.** 
      * `balance_worker.c`: The Core 1 logic loop (Fetch shared data -> Read IMU -> PID -> Motor Drive).
      * `remote_worker.c`: The Core 0 logic loop (Process Session -> Update shared data).

6.  **APP - MAIN:**
    * **Scope:** `src/3_APP/main.c`
    * **Responsibility:** **The Orchestrator.** Initializes HAL modules in sequence, initializes the OS Shared Memory, and assigns the Workers to their respective cores via the `os_handler`.


## 4. Multi-Core & RTOS Strategy
 The ESP32 dual-core capability is utilized to guarantee deterministic control timing regardless of network load. In other words, to separate high-priority control logic from low-priority telemetry.
* **Core 1 (The Control Core):** 
    * **Task:** Executes `balance_worker`.
    * **Frequency:** 200Hz (5ms period).
    * **Priority:** Highest.
    * **Workflow:** HAL_Read -> SIG_Process -> HAL_Write.
* **Core 0 (The System Core):** 
    * **Task:**  Executes `remote_worker`.
    * **Priority:** Lower.
    * **Workflow:** WiFi handling, WebSocket/HTTP Server for real-time PID tuning, and Serial logging. Updates Shared Memory.
* **Data Flow:** All inter-core data exchange MUST go through the `OS` layer accessors to ensure memory atomicity and prevent race conditions.


## 5. Coding Standards & Constraints
* **No Blocking:** No `delay()` calls in any layer. Use `vTaskDelay()` or non-blocking timers.
* **Memory Management:** Strictly NO heap allocation (`malloc`/`new`) inside the `Task_Balance` loop. Use static allocation at startup to prevent fragmentation and non-deterministic behavior.


## 6. Safety Considerations & Interlocks
* **Tilt-Limit Kill Switch:** The system must monitor the pitch angle. If pitch exceeds ±45 degrees, the APP layer must force a `Stop` command to the motors immediately.
* **Heartbeat Failsafe:** The APP layer must detect a loss of communication heartbeat from the SIGNAL middleware. If the remote connection is lost for more than a defined timeout, the robot must transition to a 'Safe-Idle' state.
* **Startup Lock:** Motors must remain disabled until the IMU reports a successful calibration and the robot is held near the vertical "Zero" point.


## 7. Testing & Verification Methodology
The project uses a Hardware-in-the-Loop (HIL) Smoke Testing approach.
* Every source file must have a corresponding environment in `platformio.ini`.
* Tests reside in `/tests` and replace the `main.cpp` logic to verify specific module functionality in isolation.
* **Test Brains:** Residing in `/tests`, these specialized main files initialize the stack UP TO the module being tested, allowing for isolated Serial-based verification.

