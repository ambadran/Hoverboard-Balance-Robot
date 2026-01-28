#include <EEPROM.h>
#include "eeprom_hal.h"

// Define the size of the EEPROM buffer (in bytes)
// We need at least 12 bytes for 3 floats, but 512 is standard/safe.
#define EEPROM_SIZE 512

extern "C" {

// --- Private Helper Prototypes ---
static void write_float_no_commit(uint32_t addr, float val);
static float read_float(uint32_t addr);
static bool commit_changes(void);

bool HAL_EEPROM_Init(void) {
    // EEPROM.begin() returns false if it fails (e.g., out of memory)
    return EEPROM.begin(EEPROM_SIZE);
}

void HAL_EEPROM_SavePID(const HAL_PID_Params_t* params) {
    if (params == NULL) {
        return;
    }
    // Write all fields
    write_float_no_commit(EEPROM_ADDR_KP, params->kp);
    write_float_no_commit(EEPROM_ADDR_KI, params->ki);
    write_float_no_commit(EEPROM_ADDR_KD, params->kd);
    write_float_no_commit(EEPROM_ADDR_MIN_POWER, params->min_power);
    write_float_no_commit(EEPROM_ADDR_DEADBAND, params->deadband);
    write_float_no_commit(EEPROM_ADDR_LPF_ALPHA, params->lpf_alpha);
    
    // Commit once at the end
    commit_changes();
}

void HAL_EEPROM_LoadPID(HAL_PID_Params_t* params) {
    if (params == NULL) {
        return;
    }
    params->kp = read_float(EEPROM_ADDR_KP);
    params->ki = read_float(EEPROM_ADDR_KI);
    params->kd = read_float(EEPROM_ADDR_KD);
    params->min_power = read_float(EEPROM_ADDR_MIN_POWER);
    params->deadband = read_float(EEPROM_ADDR_DEADBAND);
    params->lpf_alpha = read_float(EEPROM_ADDR_LPF_ALPHA);
}

// --- Granular Accessors ---

void HAL_EEPROM_SetKp(float val) {
    // Optimization: Read first, only write/commit if changed
    if (read_float(EEPROM_ADDR_KP) != val) {
        write_float_no_commit(EEPROM_ADDR_KP, val);
        commit_changes();
    }
}

float HAL_EEPROM_GetKp(void) {
    return read_float(EEPROM_ADDR_KP);
}

void HAL_EEPROM_SetKi(float val) {
    if (read_float(EEPROM_ADDR_KI) != val) {
        write_float_no_commit(EEPROM_ADDR_KI, val);
        commit_changes();
    }
}

float HAL_EEPROM_GetKi(void) {
    return read_float(EEPROM_ADDR_KI);
}

void HAL_EEPROM_SetKd(float val) {
    if (read_float(EEPROM_ADDR_KD) != val) {
        write_float_no_commit(EEPROM_ADDR_KD, val);
        commit_changes();
    }
}

float HAL_EEPROM_GetKd(void) {
    return read_float(EEPROM_ADDR_KD);
}

void HAL_EEPROM_SetMinPower(float val) {
    if (read_float(EEPROM_ADDR_MIN_POWER) != val) {
        write_float_no_commit(EEPROM_ADDR_MIN_POWER, val);
        commit_changes();
    }
}

float HAL_EEPROM_GetMinPower(void) {
    return read_float(EEPROM_ADDR_MIN_POWER);
}

void HAL_EEPROM_SetDeadband(float val) {
    if (read_float(EEPROM_ADDR_DEADBAND) != val) {
        write_float_no_commit(EEPROM_ADDR_DEADBAND, val);
        commit_changes();
    }
}

float HAL_EEPROM_GetDeadband(void) {
    return read_float(EEPROM_ADDR_DEADBAND);
}

void HAL_EEPROM_SetLpfAlpha(float val) {
    if (read_float(EEPROM_ADDR_LPF_ALPHA) != val) {
        write_float_no_commit(EEPROM_ADDR_LPF_ALPHA, val);
        commit_changes();
    }
}

float HAL_EEPROM_GetLpfAlpha(void) {
    return read_float(EEPROM_ADDR_LPF_ALPHA);
}

// --- Private Helper Implementations ---

static void write_float_no_commit(uint32_t addr, float val) {
    if (addr + sizeof(float) > EEPROM_SIZE) {
        return;
    }
    EEPROM.put(addr, val);
}

static float read_float(uint32_t addr) {
    if (addr + sizeof(float) > EEPROM_SIZE) {
        return 0.0f;
    }
    float val = 0.0f;
    EEPROM.get(addr, val);
    return val;
}

static bool commit_changes(void) {
    return EEPROM.commit();
}

}