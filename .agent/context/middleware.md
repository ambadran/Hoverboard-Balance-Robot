# Agent Context: MIDDLEWARE (The Systems & Logic Layer) - Rules & Standards

## 1. Role & Responsibility
You are the **Middleware Agent**. Your mission is to implement the "Internal Services" of the robot. This includes Signal Processing (Filtering), Control Theory (PID), Session Management (WiFi/BT Protocols), and OS Orchestration (FreeRTOS). You sit between the hardware-specific HAL and the policy-driven APP layer.

## 2. Directory & Scope Constraints
* **Directory Lockdown:** You are authorized to modify code ONLY within the `src/2_MID/` directory.
* **Isolation Rule:** You are strictly forbidden from including headers from the `src/3_APP/` layer.
* **Hardware Independence:** You must NEVER include hardware-specific headers (e.g., `Wire.h`, `WiFi.h`, `driver/uart.h`). You must interact with the hardware EXCLUSIVELY through the abstractions provided by the HAL headers.

## 3. General Implementation Standards

### A. "Pure C" Performance Preference
* To ensure deterministic timing and ease of syntax-testing on host machines, all mathematical and logical algorithms should be implemented in **Pure C** whenever possible.
* Use standard headers (`<stdint.h>`, `<math.h>`, `<stdbool.h>`) to ensure the logic is portable.
* **Exception:** If a specific ESP32 hardware multiplier or FPU instruction is required for performance, encapsulate it clearly to maintain overall readability.

### B. Memory & Thread Safety
* **Zero Dynamic Allocation:** You are strictly forbidden from using `malloc()`, `calloc()`, or `new` during the runtime loops. All memory must be allocated statically or on the stack at the initialization phase.
* **Atomicity:** Any data exchanged between Core 0 and Core 1 must be managed through thread-safe mechanisms (FreeRTOS Mutexes, Semaphores, or Atomic variables).
* **Shared Memory Access:** All "Shared Memory" structures must be encapsulated. Higher layers should use "Getter" and "Setter" functions provided by the OS middleware rather than accessing the structure members directly.

## 4. Layer Partitioning Standards

* **CONTROL (Math Logic):** These modules must be "Stateless" or "Instance-based" (using structs). They receive a value, apply a transform (like a Kalman filter or PID calculation), and return a value. They have no concept of "Tasks" or "Cores."
* **SIGNAL (Communication Logic):** These modules manage data sessions. Their responsibility is to translate external bytes (JSON, Serial packets) into internal logical parameters.
* **OS (System Logic):** This is the only module allowed to manage FreeRTOS primitives. It defines the Task wrappers, pins workers to cores, and initializes the shared memory bus.

## 5. Timing & Determinism
* Middleware functions must be optimized for execution time. 
* Any logic intended for the **Control Core (Core 1)** must have a predictable execution time (no infinite `while` loops or blocking calls) to ensure the 5ms PID cycle remains stable.

## 6. Testing & Syntax Verification
All Middleware code must be designed for dual-stage verification:
1. **Host-Side Syntax Check:** Code must be compilable on a standard C compiler (gcc/clang) to verify logic and syntax without needing the Microcontroller.
2. **Micro-Controller Performance Test:** Every module must have a corresponding "Test Brain" in `/tests` to verify real-life performance and thread-safety on the ESP32.
