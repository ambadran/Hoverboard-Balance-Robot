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