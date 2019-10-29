#include "bno055.h"
#include <string.h>

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
static uint8_t accelerometerRange = 4;

void bno055_setPage(uint8_t page) { bno055_writeData(BNO055_PAGE_ID, page); }

bno055_opmode_t bno055_getOperationMode() {
  bno055_opmode_t mode;
  bno055_readData(BNO055_OPR_MODE, &mode, 1);
  return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode) {
  bno055_writeData(BNO055_OPR_MODE, mode);
  bno055_delay(30);
}

void bno055_setOperationModeConfig() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void bno055_setExternalCrystalUse(bool state) {
  bno055_setPage(0);
  uint8_t tmp = 0;
  bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x0;
  bno055_writeData(BNO055_SYS_TRIGGER, tmp);
  bno055_delay(700);
}

void bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
void bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }

void bno055_reset() {
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(700);
}


uint8_t bno055_getInterruptEnable() {
  uint8_t byte;
  bno055_setPage(1);
  bno055_readData(BNO055_INT_EN, &byte, 1);
  return byte;
}

/* Enable/disable interrupts.
 * Chip default value after reset: all interrupts are disabled
 */
void bno055_setInterruptEnable(uint8_t byte) {
  bno055_setPage(1);
  bno055_writeData(BNO055_INT_EN, byte);
}

uint8_t bno055_getInterruptMask() {
  uint8_t mask;
  bno055_setPage(1);
  bno055_readData(BNO055_INT_MSK, &mask, 1);
  return mask;
}

/* Activate/deactivate pin state change from interrupt.
 * Chip default value after reset: no interrupt triggers a pin change
 */
void bno055_setInterruptMask(uint8_t mask) {
  bno055_setPage(1);
  bno055_writeData(BNO055_INT_MSK, mask);
}

int8_t bno055_getTemp() {
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return t;
}

void bno055_setup() {
  bno055_reset();

  uint8_t id = 0;
  bno055_readData(BNO055_CHIP_ID, &id, 1);
  if (id != BNO055_ID) {
    printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
  }
  bno055_setPage(0);
  bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

  // Select BNO055 config mode
  bno055_setOperationModeConfig();
  bno055_delay(10);
}

int16_t bno055_getSWRevision() {
  bno055_setPage(0);
  uint8_t buffer[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t bno055_getBootloaderRevision() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t bno055_getSystemStatus() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

bno055_self_test_result_t bno055_getSelfTestResult() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {
      .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

uint8_t bno055_getSystemError() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

bno055_calibration_state_t bno055_getCalibrationState() {
  bno055_setPage(0);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}


bno055_calibration_data_t bno055_getCalibrationData() {
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  // Assumes litle endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i=0; i < 22; i++) {
    // TODO(oliv4945): create multibytes write
    bno055_writeData(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
  }

  bno055_setOperationMode(operationMode);
}

bno055_vector_t bno055_getVector(uint8_t vec) {
  uint8_t buffer[6];
  bno055_vector_t xyz = {.x = 0, .y = 0, .z = 0};
  double scale = 1;

  bno055_setPage(0);
  bno055_readData(vec, buffer, 6);

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale  = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  }
  switch (vec) {
    case BNO055_VECTOR_MAGNETOMETER:
      scale = magScale;
      break;
    case BNO055_VECTOR_ACCELEROMETER:
    case BNO055_VECTOR_LINEARACCEL:
    case BNO055_VECTOR_GRAVITY:
      scale = accelScale;
      break;
    case BNO055_VECTOR_GYROSCOPE:
      scale = angularRateScale;
      break;
    case BNO055_VECTOR_EULER:
      scale = eulerScale;
      break;
  }

  xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
  xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
  xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;

  return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer() {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagnetometer() {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope() {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler() {
  return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel() {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity() {
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}


/* Set duration in ms to trigger high G interrupt.
 * Chip default value after reset: 32 ms
 * Return: 0 if success, 1 if duration to short, 2 if duration too long
 */
uint8_t bno055_setInterruptAccelHighGDuration(uint16_t duration_ms) {
  if (duration_ms < 2) { return 1; }
  if (duration_ms > 512) { return 2; }

  bno055_setPage(1);
  bno055_writeData(BNO055_ACC_HG_DURATION, (uint8_t) (duration_ms/2 - 1));
  return 0;
}


/* Set interrupt thresholds.
 * Chip default values after reset:
 *   anyMotion:    0x20 => 2 g: 0.078 g  -  4 g: 0.156 g  -  8 g: 0.313 g  -  16 g: 0.625 g
 *   highG:        0xC0 => 2 g: 1.500 g  -  4 g: 3.000 g  -  8 g: 6.000 g  -  16 g: 12.00 g
 *   noSlowMotion: 0x01 => 2 g: 0.039 g  -  4 g: 0.078 g  -  8 g: 0.156 g  -  16 g: 0.313 g
 * Return:
 *   0 if success
 *   1 if accelerometer range is not correct
 */
uint8_t bno055_setInterruptAccelThresholds(
      double anyMotion_g,
      double highG_g,
      double noSlowMotion_g
    ) {
  double scale = 1000;

  switch (accelerometerRange) {
    case 2:
      scale /= 3.91;
      break;
    case 4:
      scale /= 7.81;
      break;
    case 8:
      scale /=  15.63;
      break;
    case 16:
      scale /=  31.25;
      break;
    default:
      return 1;
  }

  bno055_setPage(1);
  bno055_writeData(BNO055_ACC_AM_THRES, (uint8_t) (anyMotion_g * scale));
  bno055_writeData(BNO055_ACC_HG_THRESH, (uint8_t) (highG_g * scale / 2));
  bno055_writeData(BNO055_ACC_NM_THRESH, (uint8_t) (noSlowMotion_g * scale));
  return 0;
}


/* Set slow motion interrupt parameters, duration in seconds.
 * noSlowMotion = 0 => Slow motion
 * noSlowMotion = 1 => No motion
 * Chip default values after reset:
 *   No motion enabled (slow motion disabled)
 *   Triggers after 6 seconds
 * Return:
 *   0 if success
 *   1 if parameter is too low in no motion mode (<1 second)
 *   2 if parameter is too high in no motion mode (>=337)
 *   3 if parameter is too high in slow motion (> 63 values)
 *   4 if noSlowMotion > 1
 * /!\ [17-19] seconds are changed to 16 due to coding limitations
 * /!\ [81-87] seconds are changed to 80 due to coding limiations
 */
uint8_t bno055_setInterruptNoSlowMotion(
      uint8_t noSlowMotion,
      uint16_t parameter
    ) {
  uint8_t byte = 0;

  if (noSlowMotion == 0) {
    if (parameter < 1) {
      return 1;
    } else if (parameter < 20) {
      byte = ((parameter - 1) & 0x0F) << 1;
    } else if (parameter < 88) {
      byte = (((parameter - 20)/4) & 0x0F) << 1;
      byte |= 0x20;
    } else if (parameter < 337) {
      byte = ((parameter - 88)/8) << 1;
      byte |= 0x40;
    } else {
      return 2;
    }
    byte |= 0x01;
  } else if (noSlowMotion == 1) {
    if (parameter > 63) { return 3; }
    byte = parameter << 1;
  } else {
    return 4;
  }

  bno055_setPage(1);
  bno055_writeData(BNO055_ACC_NM_SET, byte);

  return 0;
}
