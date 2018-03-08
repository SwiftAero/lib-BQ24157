#include "bq24157.h"
#include "i2c.h"

extern int16_t _rSense_mOhm;

bool bq24157_register_write(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef ret;
    uint8_t i2c_data[2];

    i2c_data[0] = reg;
    i2c_data[1] = data;

    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ24157_I2C_ADDRESS, i2c_data, 2, HAL_BQ24157_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    return true;
}

bool bq24157_register_read(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef ret;
    
    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ24157_I2C_ADDRESS, &reg, 1, HAL_BQ24157_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    ret = HAL_I2C_Master_Receive( &HAL_I2C_INSTANCE, (uint16_t)BQ24157_I2C_ADDRESS, data, 1, HAL_BQ24157_TIMEOUT );
    if(ret != HAL_OK )
    {
        return false;
    }
    
    return true;
}

bool bq24157_init( uint16_t rsense_mohm, uint16_t batVoltage_mV, uint16_t chargeCurrent_mA )
{
    if( !bq24157_setReset() )
    {
        return false;
    }

    if( !bq24157_setRSense( rsense_mohm ) )
    {
        return false;
    }

    if( !bq24157_setBatteryRegulationVoltage( batVoltage_mV ) )
    {
        return false;
    }

    if( !bq24157_setChargeCurrentSenseVoltage( ( chargeCurrent_mA * rsense_mohm ) / 100 ) )
    {
        return false;
    }

    if( !bq24157_setInputCurrentLimit( BQ24157_INPUT_CURRENT_500MA ) )
    {
        return false;
    }

    if( !bq24157_setTermCurrentSenseVoltage( 34 ) )
    {
        return false;
    }

    if( !bq24157_setChargeCurrentTerm( true ) )
    {
        return false;
    }

    if( !bq24157_setWeakBatteryThreshold( 3500 ) )
    {
        return false;
    }

    return true;
}

bool bq24157_update( bq24157_info *charger )
{
    int16_t value;
    BQ24157_STAT_TYPE status;
    BQ24157_FAULT_TYPE fault;
    
    if( _rSense_mOhm <= 0 )
    {
        return false;
    }
    charger->rSense_mOhm = _rSense_mOhm;

    value = bq24157_getChargeCurrentSenseVoltage();
    if( value < 0 )
    {
        return false;
    }
    charger->chargeCurrent_mA = ( ( (uint16_t)(value * 10 ) / charger->rSense_mOhm ) * 10 );

    value = bq24157_getTermCurrentSenseVoltage();
    if( value < 0 )
    {
        return false;
    }
    charger->terminationCurrent_mA = ( ( (uint16_t)( value * 10 ) / charger->rSense_mOhm ) * 10 );

    value = bq24157_getWeakBatteryThreshold();
    if( value < 0 )
    {
        return false;
    }
    charger->lowBatteryThreshold_mV = (uint16_t)(value);

    switch( bq24157_getInputCurrentLimit() )
    {
        case BQ24157_INPUT_CURRENT_100MA:
        {
            charger->inputCurrentLimit_mA = 100;
            break;
        }
        case BQ24157_INPUT_CURRENT_500MA:
        {
            charger->inputCurrentLimit_mA = 500;
            break;
        }
        case BQ24157_INPUT_CURRENT_800MA:
        {
            charger->inputCurrentLimit_mA = 800;
            break;
        }
        case BQ24157_INPUT_CURRENT_NO_LIMIT:
        {
            charger->inputCurrentLimit_mA = 0;
            break;
        }
        default:
        {
            return false;
        }
    }

    value = bq24157_getBatteryRegulationVoltage();
    if( value < 0 )
    {
        return false;
    }
    charger->batteryRegulationVoltage_mV = (uint16_t)(value);

    value = bq24157_getSpecialChargerVoltage();
    if( value < 0 )
    {
        return false;
    }
    charger->specialChargeVoltage_mV = (uint16_t)(value);

    value = bq24157_getMaxBatteryRegulationVoltage();
    if( value < 0 )
    {
        return false;
    }
    charger->maxBatteryRegulationVoltage_mV = (uint16_t)(value);

    value = bq24157_getMaxChargeCurrentSenseVoltage();
    if( value < 0 )
    {
        return false;
    }
    charger->maxChargeCurrent_mA = ( ( (uint16_t)(value) * 10 / charger->rSense_mOhm ) * 10 );

    value = bq24157_getChargerEnable();
    if( value < 0 )
    {
        return false;
    }
    charger->isEnabled = !value;

    value = bq24157_getBoostMode();
    if( value < 0 )
    {
        return false;
    }
    charger->isBoosting = value;

    value = bq24157_getOtgPinStatus();
    if( value < 0 )
    {
        return false;
    }
    charger->otgPinStatus = value;

    status = bq24157_getStatus();
    if( status == BQ24157_ERROR )
    {
        return false;
    }
    charger->status = status;

    if( charger->status == BQ24157_STAT_CHARGING )
    {
        charger->isCharging = true;
    }
    else
    {
        charger->isCharging = false;
    }

    fault = bq24157_getFault();
    if( fault == BQ24157_FAULT_ERROR )
    {
        return false;
    }
    charger->fault = fault;
    
    return true;
}

bool bq24157_setRSense(uint16_t rsense_mohm)
{
    // Range check Rsense value.  Minimum value is 55 mOhm (1.55A max charge current), there
    // is no theoretical minimum for charge current, therefor no maximum value so the uint16_t
    // max of 65535 is arbitary chosen

    if( rsense_mohm <= 55 )
    {
        return false;
    }

    _rSense_mOhm = rsense_mohm;

    return true;
}

int16_t bq24157_getRSense(void)
{
    return _rSense_mOhm;
}

bool bq24157_resetSafteyTimer(void)
{
    uint8_t temp;

    // Read register memory location 0x00
    if( !bq24157_register_read( BQ24157_STATUS_CONTROL_REG, &temp ) )
    {
        return false;
    }

    // Set B7
    temp = temp | 0x80;

    // Write new register value to memory location 0x00
    if( !bq24157_register_write( BQ24157_STATUS_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getOtgPinStatus(void)
{
    uint8_t temp;

    // Read register memory location 0x00
    if( !bq24157_register_read( BQ24157_STATUS_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    // Check B7
    if( ( temp & 0x80 ) != 0 )
    {   // B7 is set
        return true;
    }
    else
    {   // B7 is clear
        return false;
    }
}

int8_t bq24157_getStatPinFunction(void)
{
    uint8_t temp;

    // Read register memory location 0x00
    if( !bq24157_register_read( BQ24157_STATUS_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    // Check B6
    if( ( temp & 0x40 ) != 0 )
    {   // B6 is set
        return true;
    }
    else
    {   // B6 is clear
        return false;
    }
}

bool bq24157_setStatPinFunction(bool status)
{
    uint8_t temp;

    // Read register memory location 0x00
    if( !bq24157_register_read( BQ24157_STATUS_CONTROL_REG, &temp ) )
    {
        return false;
    }

    // Set B6
    temp = MOD_BIT( temp, 6, status );

    // Write new register value to memory location 0x00
    if( !bq24157_register_write( BQ24157_STATUS_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

BQ24157_STAT_TYPE bq24157_getStatus(void)
{
    uint8_t temp;

    // Read register memory location 0x00
    if( !bq24157_register_read( BQ24157_STATUS_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    return( (BQ24157_STAT_TYPE)( ( temp >> 4 ) & 0x03 ) );
}

int8_t bq24157_getBoostMode(void)
{
    uint8_t temp;

    // Read register memory location 0x00
    if( !bq24157_register_read( BQ24157_STATUS_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 3 ) & 0x01  );
}

BQ24157_FAULT_TYPE bq24157_getFault(void)
{
    uint8_t temp;

    // Read register memory location 0x00
    if( !bq24157_register_read( BQ24157_STATUS_CONTROL_REG, &temp ) )
    {
        return BQ24157_FAULT_ERROR;
    }

    temp &= 0x07;

    if( bq24157_getBoostMode() == true )
    {
        temp += 8;
    }

    return( (BQ24157_FAULT_TYPE)temp );
}

BQ24157_INPUT_CURRENT_LIMIT_TYPE bq24157_getInputCurrentLimit(void)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return BQ24157_INPUT_CURRENT_ERROR;
    }

    return( (BQ24157_INPUT_CURRENT_LIMIT_TYPE)( ( temp >> 6 ) & 0x03 ) );
}

bool bq24157_setInputCurrentLimit(BQ24157_INPUT_CURRENT_LIMIT_TYPE inputCurrent)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return false;
    }

    temp &= 0x3F;
    temp |= ( inputCurrent << 6 );

    // Write new register value to memory location 0x01
    if( !bq24157_register_write( BQ24157_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

int16_t bq24157_getWeakBatteryThreshold(void)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    temp = ( temp >> 4 ) & 0x03;

    switch( temp )
    {
        case 0x00:
        {
            return( 3400 );
        }
        case 0x01:
        {
            return( 3500 );
        }
        case 0x02:
        {
            return( 3600 );
        }
        case 0x03:
        {
            return( 3700 );
        }
        default:
        {
            return -1;
        }
    }

    return -1;
}

bool bq24157_setWeakBatteryThreshold(uint16_t voltage)
{
    uint8_t temp;

    // Range check voltage - allowed values are 3400, 3500, 3600 and 3700
    if( !(voltage == 3400 || voltage == 3500 || voltage == 3600 || voltage == 3700 ) )
    {
        return false;
    }

    // Voltage is offset by 3.4V and has resolution of 0.1V
    voltage = ( voltage - 3400 ) / 100;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    temp &= 0xCF;
    temp |= ( voltage << 4 );

    // Write new register value to memory location 0x01
    if( !bq24157_register_write( BQ24157_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getChargeCurrentTerm(void)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 3 ) & 0x01 );
}

bool bq24157_setChargeCurrentTerm(bool status)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return false;
    }

    temp = MOD_BIT( temp, 3, status );

    // Write new register value to memory location 0x01
    if( !bq24157_register_write( BQ24157_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getChargerEnable(void)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 2 ) & 0x01 );
}

bool bq24157_setChargerEnable(bool status)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return false;
    }

    temp = MOD_BIT( temp, 2, status );

    // Write new register value to memory location 0x01
    if( !bq24157_register_write( BQ24157_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getHighImpedanceMode(void)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 1 ) & 0x01 );
}

bool bq24157_setHighImpedanceMode(bool status)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return false;
    }

    temp = MOD_BIT( temp, 1, status );

    // Write new register value to memory location 0x01
    if( !bq24157_register_write( BQ24157_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getOperationMode(void)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return -1;
    }

    return( temp & 0x01 );
}

bool bq24157_setOperationMode(bool mode)
{
    uint8_t temp;

    // Read register memory location 0x01
    if( !bq24157_register_read( BQ24157_CONTROL_REG, &temp ) )
    {
        return false;
    }

    temp = MOD_BIT( temp, 0, mode );

    // Write new register value to memory location 0x01
    if( !bq24157_register_write( BQ24157_CONTROL_REG, temp ) )
    {
        return false;
    }

    return true;
}

int16_t bq24157_getBatteryRegulationVoltage(void)
{
    uint8_t temp;

    // Read register memory location 0x02
    if( !bq24157_register_read( BQ24157_CONTROL_VOLTAGE_REG, &temp ) )
    {
        return -1;
    }

    // Charge voltage range is 3.5V to 4.44V with an offset of 3.5V and steps of 0.02V
    return( 3500 + ( ( ( temp >> 2 ) & 0x3F ) * 20 ) );
}

bool bq24157_setBatteryRegulationVoltage(uint16_t voltage)
{
    uint8_t temp;

    // Range check voltage 3500 - 4440
    if( !( voltage >= 3500 && voltage <= 4440 ) )
    {
        return false;
    }

    // Read register memory location 0x02
    if( !bq24157_register_read( BQ24157_CONTROL_VOLTAGE_REG, &temp ) )
    {
        return false;
    }

    // Charge voltage range is 3.5V to 4.44V with an offset of 3.5V and steps of 0.02V
    temp &= 0x03;
    temp |= ( ( ( voltage - 3500 ) / 20 ) << 2 );

    // Write new register value to memory location 0x02
    if( !bq24157_register_write( BQ24157_CONTROL_VOLTAGE_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getOtgPinPolarity(void)
{
    uint8_t temp;

    // Read register memory location 0x02
    if( !bq24157_register_read( BQ24157_CONTROL_VOLTAGE_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 1 ) & 0x01 );
}

bool bq24157_setOtgPinPolarity(bool polarity)
{
    uint8_t temp;

    // Read register memory location 0x02
    if( !bq24157_register_read( BQ24157_CONTROL_VOLTAGE_REG, &temp ) )
    {
        return false;
    }

    temp = MOD_BIT( temp, 1, polarity );

    // Write new register value to memory location 0x02
    if( !bq24157_register_write( BQ24157_CONTROL_VOLTAGE_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getOtgPinEnable(void)
{
    uint8_t temp;

    // Read register memory location 0x02
    if( !bq24157_register_read( BQ24157_CONTROL_VOLTAGE_REG, &temp ) )
    {
        return -1;
    }

    return( temp & 0x01 );
}

bool bq24157_setOtgPinEnable(bool enable)
{
    uint8_t temp;

    // Read register memory location 0x02
    if( !bq24157_register_read( BQ24157_CONTROL_VOLTAGE_REG, &temp ) )
    {
        return false;
    }

    temp = MOD_BIT( temp, 0, enable);

    // Write new register value to memory location 0x02
    if( !bq24157_register_write( BQ24157_CONTROL_VOLTAGE_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getVendorCode(void)
{
    uint8_t temp;

    // Read register memory location 0x03
    if( !bq24157_register_read( BQ24157_VENDOR_PART_REV_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 5 ) & 0x07 );
}

int8_t bq24157_getPartCode(void)
{
    uint8_t temp;

    // Read register memory location 0x03
    if( !bq24157_register_read( BQ24157_VENDOR_PART_REV_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 3 ) & 0x03 );
}

int8_t bq24157_getPartRevision(void)
{
    uint8_t temp;

    // Read register memory location 0x03
    if( !bq24157_register_read( BQ24157_VENDOR_PART_REV_REG, &temp ) )
    {
        return -1;
    }

    return( temp & 0x07 );
}

bool bq24157_setReset(void)
{
    uint8_t temp;

    // Read register memory location 0x04
    if( !bq24157_register_read( BQ24157_BATT_TERM_CURRENT_REG, &temp ) )
    {
        return false;
    }

    // Set B7
    temp = temp | 0x80;

    // Write new register value to memory location 0x04
    if( !bq24157_register_write( BQ24157_BATT_TERM_CURRENT_REG, temp ) )
    {
        return false;
    }

    return true;
}

int16_t bq24157_getChargeCurrentSenseVoltage(void)
{
    uint8_t temp;

    // Read register memory location 0x04
    if( !bq24157_register_read( BQ24157_BATT_TERM_CURRENT_REG, &temp ) )
    {
        return -1;
    }

    // Return the charge current sense voltage in unit of 0.1mV includes offset of 37.4mV
    return( ( ( temp >> 4 ) * 68 ) + 374 );
}

bool bq24157_setChargeCurrentSenseVoltage(uint16_t voltage)
{
    uint8_t temp;

    // Range check voltage.  Units are 0.1mV and range is 37.4mV to 90.4mV
    if( !( voltage >= 374 && voltage <= 904 ) )
    {
        return false;
    }

    // Read register memory location 0x04
    if( !bq24157_register_read( BQ24157_BATT_TERM_CURRENT_REG, &temp ) )
    {
        return false;
    }

    temp &= 0x8F;
    temp |= ( ( voltage - 374 ) / 68 ) << 4;

    // Write new register value to memory location 0x04
    if( !bq24157_register_write( BQ24157_BATT_TERM_CURRENT_REG, temp ) )
    {
        return false;
    }

    return true;
}

int16_t bq24157_getTermCurrentSenseVoltage(void)
{
    uint8_t temp;

    // Read register memory location 0x04
    if( !bq24157_register_read( BQ24157_BATT_TERM_CURRENT_REG, &temp ) )
    {
        return -1;
    }

    temp &= 0x07;

    // Return the charge current sense voltage in unit of 0.1mV offset 3.4mV
    return( ( temp * 34 ) + 34 );
}

bool bq24157_setTermCurrentSenseVoltage(uint16_t voltage)
{
    uint8_t temp;
    
    // Range check voltage.  Units are 0.1mV and range is 3.4mV to 27.2mV
    if( !( voltage >= 34 && voltage <= 272 ) )
    {
        return false;
    }

    // Read register memory location 0x04
    if( !bq24157_register_read( BQ24157_BATT_TERM_CURRENT_REG, &temp ) )
    {
        return false;
    }

    temp &= 0xF8;
    temp |= ( ( voltage - 34 ) / 34 );

    // Write new register value to memory location 0x04
    if( !bq24157_register_write( BQ24157_BATT_TERM_CURRENT_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getLowChargeCurrentSenseVoltage(void)
{
    uint8_t temp;

    // Read register memory location 0x05
    if( !bq24157_register_read( BQ24157_SPEC_CHRG_EN_PIN_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 5 ) & 0x01 );
}

bool bq24157_setLowChargeCurrentSenseVoltage(bool low_chg)
{
    uint8_t temp;

    // Read register memory location 0x05
    if( !bq24157_register_read( BQ24157_SPEC_CHRG_EN_PIN_REG, &temp ) )
    {
        return false;
    }

    temp = MOD_BIT( temp, 5, low_chg );

    // Write new register value to memory location 0x05
    if( !bq24157_register_write( BQ24157_SPEC_CHRG_EN_PIN_REG, temp ) )
    {
        return false;
    }

    return true;
}

int8_t bq24157_getDpmStatus(void)
{
    uint8_t temp;

    // Read register memory location 0x05
    if( !bq24157_register_read( BQ24157_SPEC_CHRG_EN_PIN_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 4 ) & 0x01 );
}

int8_t bq24157_getCdStatus(void)
{
    uint8_t temp;

    // Read register memory location 0x05
    if( !bq24157_register_read( BQ24157_SPEC_CHRG_EN_PIN_REG, &temp ) )
    {
        return -1;
    }

    return( ( temp >> 3 ) & 0x01 );
}

int16_t bq24157_getSpecialChargerVoltage(void)
{
    uint8_t temp;

    // Read register memory location 0x05
    if( !bq24157_register_read( BQ24157_SPEC_CHRG_EN_PIN_REG, &temp ) )
    {
        return -1;
    }

    // Special charger voltage in mV offset of 4200mV
    return( ( ( (int16_t)(temp) & 0x07 ) * 80 ) + 4200 );
}

bool bq24157_setSpecialChargerVoltage(uint16_t voltage)
{
    uint8_t temp;

    // Range check voltage.  Units are mV and allowed range is 4.2V-4.76V
    if( !( voltage >= 4200 && voltage <= 4760 ) )
    {
        return false;
    }

    // Read register memory location 0x05
    if( !bq24157_register_read( BQ24157_SPEC_CHRG_EN_PIN_REG, &temp ) )
    {
        return false;
    }

    temp = temp | ( ( ( voltage - 4200 ) / 80 ) & 0xFF );

    // Write new register value to memory location 0x05
    if( !bq24157_register_write( BQ24157_SPEC_CHRG_EN_PIN_REG, temp ) )
    {
        return false;
    }

    return true;
}

int16_t bq24157_getMaxChargeCurrentSenseVoltage(void)
{
    uint8_t temp;

    // Read register memory location 0x06
    if( !bq24157_register_read( BQ24157_SAFTEY_LIMIT_REG, &temp ) )
    {
        return -1;
    }

    // Return the charge current sense voltage in unit of 0.1mV includes offset of 37.4mV
    return( ( ( temp >> 4 ) * 68 ) + 374 );
}

bool bq24157_setMaxChargeCurrentSenseVoltage(uint16_t voltage)
{
    uint8_t temp;

    // Range check voltage.  Units are 0.1mV and range is 37.4mV to 105.4mV
    if( !( voltage >= 374 && voltage <= 1054 ) )
    {
        return false;
    }

    // Read register memory location 0x04
    if( !bq24157_register_read( BQ24157_BATT_TERM_CURRENT_REG, &temp ) )
    {
        return false;
    }

    temp = temp | ( ( ( voltage - 374 ) / 68 ) << 4 );

    // Write new register value to memory location 0x04
    if( !bq24157_register_write( BQ24157_BATT_TERM_CURRENT_REG, temp ) )
    {
        return false;
    }

    return true;
}

int16_t bq24157_getMaxBatteryRegulationVoltage(void)
{
    uint8_t temp;

    // Read register memory location 0x06
    if( !bq24157_register_read( BQ24157_SAFTEY_LIMIT_REG, &temp ) )
    {
        return -1;
    }

    // Return max battery regulation voltage in mV with offset of 4200mV
    return( ( ( temp & 0x0F ) * 20 ) + 4200 );
}

bool bq24157_setMaxBatteryRegulationVoltage(uint16_t voltage)
{
    uint8_t temp;

    // Range check voltage.  Units are mV and range is 4200mV to 4440mV
    if( !( voltage >= 4200 && voltage <= 4440 ) )
    {
        return false;
    }

    // Read register memory location 0x06
    if( !bq24157_register_read( BQ24157_SAFTEY_LIMIT_REG, &temp ) )
    {
        return false;
    }

    temp = temp | ( ( voltage - 4200 ) / 20 );

    // Write new register value to memory location 0x06
    if( !bq24157_register_write( BQ24157_SAFTEY_LIMIT_REG, temp ) )
    {
        return false;
    }

    return true;
}