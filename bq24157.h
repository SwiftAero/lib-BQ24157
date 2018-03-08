#ifndef __BQ24157_H
#define __BQ24157_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define BQ24157_I2C_ADDRESS                 0xD4

#define HAL_I2C_INSTANCE                    hi2c2
#define HAL_BQ24157_TIMEOUT                 1

#define BQ24157_STATUS_CONTROL_REG          0x00
#define BQ24157_CONTROL_REG                 0x01
#define BQ24157_CONTROL_VOLTAGE_REG         0x02
#define BQ24157_VENDOR_PART_REV_REG         0x03
#define BQ24157_BATT_TERM_CURRENT_REG       0x04
#define BQ24157_SPEC_CHRG_EN_PIN_REG        0x05
#define BQ24157_SAFTEY_LIMIT_REG            0x06

// In value v, MODify BIT at position p to be at state s
#define MOD_BIT(v,p,s) ((v&(~(1U<<p)))|(-s&(1U<<p)))

typedef enum
{
    BQ24157_STAT_READY = 0,
    BQ24157_STAT_CHARGING,
    BQ24157_STAT_CHARGE_DONE,
    BQ24157_STAT_FAULT,
    BQ24157_ERROR = -1
}
BQ24157_STAT_TYPE;

typedef enum
{
    BQ24157_CHARGE_FAULT_NORMAL = 0,
    BQ24157_CHARGE_FAULT_VBUSOVP,
    BQ24157_CHARGE_FAULT_SLEEP,
    BQ24157_CHARGE_FAULT_VBUSLOW,
    BQ24157_CHARGE_FAULT_OUTPUTOVP,
    BQ24157_CHARGE_FAULT_THERMALSHUTDOWN,
    BQ24157_CHARGE_FAULT_TIMERFAULT,
    BQ24157_CHARGE_FAULT_NOBATTERY,
    BQ24157_BOOST_FAULT_NORMAL,
    BQ24157_BOOST_FAULT_VBUSOVP,
    BQ24157_BOOST_FAULT_OVERLOAD,
    BQ24157_BOOST_FAULT_BATTLOW,
    BQ24157_BOOST_FAULT_BATTOVP,
    BQ24157_BOOST_FAULT_THERMALSHUTDOWN,
    BQ24157_BOOST_FAULT_TIMERFAULT,
    BQ24157_BOOST_FAULT_NA,
    BQ24157_FAULT_ERROR = -1
}
BQ24157_FAULT_TYPE;

typedef enum
{
    BQ24157_INPUT_CURRENT_100MA = 0,
    BQ24157_INPUT_CURRENT_500MA,
    BQ24157_INPUT_CURRENT_800MA,
    BQ24157_INPUT_CURRENT_NO_LIMIT,
    BQ24157_INPUT_CURRENT_ERROR = -1
}
BQ24157_INPUT_CURRENT_LIMIT_TYPE;

int16_t _rSense_mOhm;

typedef struct
{
    uint16_t rSense_mOhm;
    uint16_t chargeCurrent_mA;
    uint16_t terminationCurrent_mA;
    uint16_t lowBatteryThreshold_mV;
    uint16_t inputCurrentLimit_mA;
    uint16_t batteryRegulationVoltage_mV;
    uint16_t specialChargeVoltage_mV;
    uint16_t maxBatteryRegulationVoltage_mV;
    uint16_t maxChargeCurrent_mA;
    BQ24157_STAT_TYPE status;
    BQ24157_FAULT_TYPE fault;
    bool isCharging;
    bool isBoosting;
    bool isEnabled;
    bool otgPinStatus;
}
bq24157_info;

bool                                bq24157_register_write(uint8_t reg, uint8_t data);
bool                                bq24157_register_read(uint8_t reg, uint8_t *data);

bool                                bq24157_init(uint16_t rsense_mohm, uint16_t batVoltage_mV, uint16_t chargeCurrent_mA);
bool                                bq24157_update( bq24157_info *charger );

bool                                bq24157_setRSense(uint16_t rsense_mohm);
int16_t                             bq24157_getRSense(void);

bool                                bq24157_resetSafteyTimer(void);
int8_t                              bq24157_getOtgPinStatus(void);
int8_t                              bq24157_getStatPinFunction(void);
bool                                bq24157_setStatPinFunction(bool status);
BQ24157_STAT_TYPE                   bq24157_getStatus(void);
int8_t                              bq24157_getBoostMode(void);
BQ24157_FAULT_TYPE                  bq24157_getFault(void);
BQ24157_INPUT_CURRENT_LIMIT_TYPE    bq24157_getInputCurrentLimit(void);
bool                                bq24157_setInputCurrentLimit(BQ24157_INPUT_CURRENT_LIMIT_TYPE inputCurrent);
int16_t                             bq24157_getWeakBatteryThreshold(void);
bool                                bq24157_setWeakBatteryThreshold(uint16_t voltage);
int8_t                              bq24157_getChargeCurrentTerm(void);
bool                                bq24157_setChargeCurrentTerm(bool status);
int8_t                              bq24157_getChargerEnable(void);
bool                                bq24157_setChargerEnable(bool status);
int8_t                              bq24157_getHighImpedanceMode(void);
bool                                bq24157_setHighImpedanceMode(bool status);
int8_t                              bq24157_getOperationMode(void);
bool                                bq24157_setOperationMode(bool mode);
int16_t                             bq24157_getBatteryRegulationVoltage(void);
bool                                bq24157_setBatteryRegulationVoltage(uint16_t voltage);
int8_t                              bq24157_getOtgPinPolarity(void);
bool                                bq24157_setOtgPinPolarity(bool polarity);
int8_t                              bq24157_getOtgPinEnable(void);
bool                                bq24157_setOtgPinEnable(bool enable);
int8_t                              bq24157_getVendorCode(void);
int8_t                              bq24157_getPartCode(void);
int8_t                              bq24157_getPartRevision(void);
bool                                bq24157_setReset(void);
int16_t                             bq24157_getChargeCurrentSenseVoltage(void);
bool                                bq24157_setChargeCurrentSenseVoltage(uint16_t voltage);
int16_t                             bq24157_getTermCurrentSenseVoltage(void);
bool                                bq24157_setTermCurrentSenseVoltage(uint16_t voltage);
int8_t                              bq24157_getLowChargeCurrentSenseVoltage(void);
bool                                bq24157_setLowChargeCurrentSenseVoltage(bool low_chg);
int8_t                              bq24157_getDpmStatus(void);
int8_t                              bq24157_getCdStatus(void);
int16_t                             bq24157_getSpecialChargerVoltage(void);
bool                                bq24157_setSpecialChargerVoltage(uint16_t voltage);
int16_t                             bq24157_getMaxChargeCurrentSenseVoltage(void);
bool                                bq24157_setMaxChargeCurrentSenseVoltage(uint16_t voltage);
int16_t                             bq24157_getMaxBatteryRegulationVoltage(void);
bool                                bq24157_setMaxBatteryRegulationVoltage(uint16_t voltage);

#ifdef __cplusplus
}
#endif
#endif /* __BQ24157_H */