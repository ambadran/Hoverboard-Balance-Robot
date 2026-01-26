#include "pid_controller.h"
#include "../../../include/config.h"
#include "../../../include/platform_abstraction.h"
#include "../../1_HAL/eeprom_hal.h"
#include <QuickPID.h>
#include <math.h>

// --- Internal State Variables ---
// QuickPID requires variables to be bound by reference (or pointers)
static float pid_input = 0.0f;
static float pid_output = 0.0f;
static float pid_setpoint = 0.0f;

// Calibration Offset
static float zero_offset = 0.0f;

// The QuickPID Object
// p, i, d, POn, DOn
// POn::POnM (Measure) or POnE (Error). Balancing usually uses POnE.
// DOn::DOnM (Measure) or DOnE (Error). Balancing usually prefers DOnM (Derivative on Measurement) to avoid spikes on setpoint change, 
// BUT for a self-balancing robot where setpoint is usually 0, DOnE vs DOnM is less critical, but DOnM is safer.
// However, standard PID often uses DOnE. Let's stick to defaults or configure clearly.
// QuickPID defaults: POnE, DOnM.
static QuickPID my_pid(&pid_input, &pid_output, &pid_setpoint);

// Global Status Structure
static PID_Status_t pid_status = {0};

// Shadow variable for Sample Time (since QuickPID lacks a getter)
static uint32_t current_sample_time_us = PID_SAMPLE_TIME_US;

/**
 * @brief Helper function to calibrate zero offset.
 *        Polls the IMU Update function directly since we are outside the main loop.
 */
static void MID_CONTROL_PID_Calibrate(void) {
    float sum = 0.0f;
    int samples_collected = 0;

    // We assume the IMU hardware is already Init'd by Main.
    
    while (samples_collected < PID_CALIBRATION_SAMPLES) {
        // Poll the hardware directly
        // HAL_IMU_Update returns true if new FIFO data was processed
        if (PID_INPUT_UPDATE_FUNC()) {
            sum += PID_INPUT_FUNC();
            samples_collected++;
        } else {
            // No new data yet, yield briefly to avoid hard lockup if this was RTOS
            // In pure init (single thread), this is just a busy wait.
            // Using platform delay to give time for DMP to fill FIFO (approx 5-10ms usually)
            PLATFORM_DELAY_MS(1);
        }
    }
    
    zero_offset = sum / (float)PID_CALIBRATION_SAMPLES;
}

void MID_CONTROL_PID_Init(void) {
    // 3. Run Zero-Calibration Routine
    MID_CONTROL_PID_Calibrate();

    // Automatic means "On". Manual means "Off".
    // Set Mode to Timer (it checks internal micros() against SampleTime)
    my_pid.SetMode(QuickPID::Control::timer);

    my_pid.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    // 2. Load PID Parameters
    HAL_PID_Params_t params;
    HAL_EEPROM_LoadPID(&params);
    // Sanity Check / Fallback if EEPROM is empty (nan or zero)
    if (isnan(params.kp) || (params.kp == 0.0f && params.ki == 0.0f && params.kd == 0.0f)) {
        params.kp = PID_DEFAULT_KP;
        params.ki = PID_DEFAULT_KI;
        params.kd = PID_DEFAULT_KD;
    }
    // set the tunings
    my_pid.SetTunings(params.kp, params.ki, params.kd);

    // Set the direction
    my_pid.SetControllerDirection(QuickPID::Action::reverse);
    
    // We expect this Step function to be called at ~200Hz (5ms).
    my_pid.SetSampleTimeUs(PID_SAMPLE_TIME_US); 
    
    // Proportional Mode
    my_pid.SetProportionalMode(PID_P_MODE == 0 ? QuickPID::pMode::pOnError : QuickPID::pMode::pOnMeas);

    // Derivative Mode
    my_pid.SetDerivativeMode(PID_D_MODE == 0 ? QuickPID::dMode::dOnError : QuickPID::dMode::dOnMeas);

    // Anti-Windup Configuration
    QuickPID::iAwMode aw_mode = QuickPID::iAwMode::iAwCondition; // Default 0
    if (PID_AW_MODE == 1) aw_mode = QuickPID::iAwMode::iAwClamp;
    else if (PID_AW_MODE == 2) aw_mode = QuickPID::iAwMode::iAwOff;
    my_pid.SetAntiWindupMode(aw_mode);
}

bool MID_CONTROL_PID_Step(float target_setpoint, float *pid_output_ptr) {
    // Update Bindings
    pid_setpoint = target_setpoint;
 
    // Read Input and apply offset
    // The PID works on the error: Setpoint - Input
    pid_input = PID_INPUT_MULTIPLE*(PID_INPUT_FUNC() - zero_offset);

    // --- SAFETY CHECK: Tilt Limit ---
    if (fabs(pid_input) > PID_TILT_LIMIT_DEG) {
        // Force Stop
        pid_output = 0.0f;
        if (pid_output_ptr) *pid_output_ptr = 0.0f;
        
        // Reset PID internals so it doesn't "wind up" or kick when we recover
        MID_CONTROL_PID_Reset();
        
        // Update Status for Monitoring
        pid_status.setpoint = pid_setpoint;
        pid_status.input = pid_input;
        pid_status.output = 0.0f;
        pid_status.error = pid_setpoint - pid_input;
        pid_status.p_term = 0.0f;
        pid_status.i_term = 0.0f;
        pid_status.d_term = 0.0f;
        
        return true; // We processed the step (Safety Triggered)
    }
    // --------------------------------

    // Compute PID
    // Returns true if new output was computed (time elapsed)
    bool computed = my_pid.Compute();

    if (computed) {
        // Output is updated in pid_output
        
        // Update Status for Monitoring
        pid_status.setpoint = pid_setpoint;
        pid_status.input = pid_input;
        pid_status.output = pid_output;
        pid_status.error = pid_setpoint - pid_input;
        
        // Access internal terms
        pid_status.p_term = my_pid.GetPterm();
        pid_status.i_term = my_pid.GetIterm();
        pid_status.d_term = my_pid.GetDterm();
    }

    if (pid_output_ptr) {
        *pid_output_ptr = pid_output;
    }
    
    return computed;
}

void MID_CONTROL_PID_SetGains(float kp, float ki, float kd) {
    my_pid.SetTunings(kp, ki, kd);
    // Optionally save to EEPROM here if "SetGains" implies persistence, 
    // but typically runtime tuning is volatile until explicitly saved.
    // The directive didn't explicitly say "save on set", only "load on init".
}

float MID_CONTROL_PID_GetKp(void) {
    return my_pid.GetKp();
}

float MID_CONTROL_PID_GetKi(void) {
    return my_pid.GetKi();
}

float MID_CONTROL_PID_GetKd(void) {
    return my_pid.GetKd();
}

PID_Status_t MID_CONTROL_PID_GetStatus(void) {
    return pid_status;
}

void MID_CONTROL_PID_SetMode(MID_PID_Mode_t mode) {
    switch (mode) {
        case MID_PID_MODE_MANUAL:
            my_pid.SetMode(QuickPID::Control::manual);
            break;
        case MID_PID_MODE_AUTOMATIC:
            my_pid.SetMode(QuickPID::Control::automatic);
            break;
        case MID_PID_MODE_TIMER:
            my_pid.SetMode(QuickPID::Control::timer);
            break;
    }
}

MID_PID_Mode_t MID_CONTROL_PID_GetMode(void) {
    uint8_t mode = my_pid.GetMode();
    switch (mode) {
        case (uint8_t)QuickPID::Control::manual:
            return MID_PID_MODE_MANUAL;
        case (uint8_t)QuickPID::Control::automatic:
            return MID_PID_MODE_AUTOMATIC;
        case (uint8_t)QuickPID::Control::timer:
            return MID_PID_MODE_TIMER;
        default:
            return MID_PID_MODE_MANUAL;
    }
}

void MID_CONTROL_PID_SetSampleTime(uint32_t us) {
    current_sample_time_us = us;
    my_pid.SetSampleTimeUs(us);
}

uint32_t MID_CONTROL_PID_GetSampleTime(void) {
    return current_sample_time_us;
}

void MID_CONTROL_PID_Reset(void) {
    // Force clear the static bindings to ensure a true "Cold Start".
    // This prevents QuickPID's Bumpless Transfer from reloading the 
    // last output into the Integral term.
    pid_output = 0.0f;
    pid_input = 0.0f;
    pid_setpoint = 0.0f;

    my_pid.Reset();
}
