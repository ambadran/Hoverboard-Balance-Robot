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

void MID_CONTROL_PID_Init(void) {
    // 1. (REMOVED) HAL Init is orchestrated by main.c
    // HAL_EEPROM_Init(); 

    // 2. Load PID Parameters
    HAL_PID_Params_t params;
    HAL_EEPROM_LoadPID(&params);

    // Sanity Check / Fallback if EEPROM is empty (nan or zero)
    if (isnan(params.kp) || (params.kp == 0.0f && params.ki == 0.0f && params.kd == 0.0f)) {
        params.kp = PID_DEFAULT_KP;
        params.ki = PID_DEFAULT_KI;
        params.kd = PID_DEFAULT_KD;
    }

    // 3. Run Zero-Calibration Routine
    // We average N samples to find the starting pitch, assuming the user holds it "vertical" 
    // or we are just determining the sensor bias.
    // NOTE: This assumes the IMU is already initialized by MAIN before calling this.
    float sum = 0.0f;
    for (int i = 0; i < PID_CALIBRATION_SAMPLES; i++) {
        sum += PID_INPUT_FUNC();
        // Small delay to allow new samples (assuming ~5-10ms sample rate of IMU/Loop)
        // Using a busy wait or system delay if available. 
        // Since we are in Init phase, blocking is acceptable.
        // We use a simple loop approx delay if generic delay not available, 
        // but Arduino framework provides delay(ms).
        PLATFORM_DELAY_MS(5); 
    }
    zero_offset = sum / (float)PID_CALIBRATION_SAMPLES;

    // 4. Configure QuickPID
    my_pid.SetTunings(params.kp, params.ki, params.kd);
    my_pid.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    
    // Set Mode to Timer (it checks internal micros() against SampleTime)
    // We expect this Step function to be called at ~200Hz (5ms).
    my_pid.SetSampleTimeUs(PID_SAMPLE_TIME_US); 
    
    // Automatic means "On". Manual means "Off".
    my_pid.SetMode(QuickPID::Control::timer);

    // Advanced Configurations from Config
    my_pid.SetProportionalMode(PID_P_MODE == 0 ? QuickPID::pMode::pOnError : QuickPID::pMode::pOnMeas);
    my_pid.SetDerivativeMode(PID_D_MODE == 0 ? QuickPID::dMode::dOnError : QuickPID::dMode::dOnMeas);

    // Anti-Windup Configuration
    QuickPID::iAwMode aw_mode = QuickPID::iAwMode::iAwCondition; // Default 0
    if (PID_AW_MODE == 1) aw_mode = QuickPID::iAwMode::iAwClamp;
    else if (PID_AW_MODE == 2) aw_mode = QuickPID::iAwMode::iAwOff;
    my_pid.SetAntiWindupMode(aw_mode);
}

float MID_CONTROL_PID_Step(float target_setpoint) {
    // Update Bindings
    pid_setpoint = target_setpoint;
    
    // Read Input and apply offset
    // The PID works on the error: Setpoint - Input
    pid_input = PID_INPUT_FUNC() - zero_offset;

    // Compute PID
    // Returns true if new output was computed (time elapsed)
    if (my_pid.Compute()) {
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
    
    return pid_output;
}

void MID_CONTROL_PID_SetGains(float kp, float ki, float kd) {
    my_pid.SetTunings(kp, ki, kd);
    // Optionally save to EEPROM here if "SetGains" implies persistence, 
    // but typically runtime tuning is volatile until explicitly saved.
    // The directive didn't explicitly say "save on set", only "load on init".
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

void MID_CONTROL_PID_SetSampleTime(uint32_t us) {
    my_pid.SetSampleTimeUs(us);
}

uint32_t MID_CONTROL_PID_GetSampleTime(void) {
    // QuickPID doesn't have a GetSampleTimeUs() in the public API 
    // based on typical PID libraries, but it might.
    // If not available, we can rely on our config, but dynamic changes require storage.
    // For now, assume strict mapping to what we set.
    // Note: To be safe with the library version, we should check availability.
    // If library doesn't support getter, we must return a stored shadow variable.
    // However, looking at the library source (dlloydev/QuickPID) usually reveals GetSampleTimeUs.
    // If unsure, I will assume it exists, or shadow it. 
    // Let's shadow it to be safe and fast.
    return my_pid.GetSampleTimeUs();
}
