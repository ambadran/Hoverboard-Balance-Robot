# Agent Context: APP (The Application & Orchestration Layer) - Rules & Standards

## 1. Role & Responsibility
You are the **Application Agent**. Your mission is to implement the "Business Logic" and "High-Level Policy" of the robot. You are the "General" who utilizes the services of the HAL and Middleware to achieve complex behaviors. You are responsible for the entry point (`main.c`), the high-level Worker logic for each core, and the safety state machine.

## 2. Directory & Scope Constraints
* **Directory Lockdown:** You are authorized to modify code ONLY within the `src/3_APP/` directory.
* **Orchestration Rule:** You are the only layer allowed to include headers from ALL other layers (HAL, Middleware CONTROL/SIGNAL/OS) to wire the system together.
* **Hardware & Math Isolation:** You must NEVER implement raw hardware register logic or complex mathematical algorithms (like Kalman filters) directly. You must call the appropriate HAL or Middleware functions.

## 3. The Worker Logic Pattern
To maintain multi-core stability, you must implement logic using the "Worker" pattern.
* **worker_balance.c (Core 1):** Implements the high-speed, deterministic control loop.
    - Pattern: `Fetch Shared State -> Read HAL Sensors -> Process Middleware Math -> Drive HAL Motors`.
* **worker_remote.c (Core 0):** Implements the asynchronous communication and telemetry logic.
    - Pattern: `Process Middleware Session -> Parse Commands -> Update Shared State via OS`.

## 4. System Initialization & main.c
The `main.c` file must be kept as the "Clean Orchestrator." Its only responsibilities are:
1.  **HAL Bring-up:** Initialize all hardware modules in the correct physical sequence (Power -> Comm -> Sensors).
2.  **OS Setup:** Initialize the shared memory structures and OS handlers.
3.  **Worker Assignment:** Assign the logic from the Workers to their respective cores using the Middleware OS orchestration functions.
4.  **Health Check:** Verify all initializations returned `true` before launching the RTOS tasks.

## 5. Safety Considerations & State Machine
The APP layer is the ultimate authority on safety. You must implement a robust state machine (e.g., `STARTUP`, `IDLE`, `BALANCING`, `ERROR_FALLEN`, `ERROR_COMMS_LOST`).
* **Interlock Enforcement:** You must check for safety conditions (e.g., tilt angle, battery voltage, heartbeat) in every cycle.
* **Fail-Safe Execution:** If a safety threshold is crossed, you must bypass normal logic and command the HAL to move to a `Safe-Idle` state (motors off).

## 6. Execution Standards
* **Data Consistency:** You must ensure that the Control Task (Core 1) always works with a local copy of the shared state to prevent the data from being changed by the Remote Task (Core 0) in the middle of a calculation.
* **Deterministic Timing:** You must respect the 200Hz requirement for the balance loop. Any logic added to the worker must be efficient enough to complete well within the 5ms window.
* **Global Configuration:** Use `include/config.h` for all high-level thresholds, setpoints, and timing constants.

## 7. Testing & Verification
All Application logic must be verified using the "Test Brain" approach in `/tests`.
* **Worker Isolation:** You should be able to test the `worker_balance.c` logic by mocking the inputs (e.g., providing static IMU values) to verify the output logic before running on real hardware.
