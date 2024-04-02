/*
MIT License

Copyright (c) 2024 Giuseppe Giglio <g.giglio001@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * \file pca9865lib.c
 * \brief This file contains the definitions of the functions that are used to
 *       manage the PCA9865 PWM controller.
 */

/* Standard library includes */
#include <stdlib.h>

/* Local includes */
#include "pca9865lib.h"
#include "us-i2c.h"



/* Unexported functions definitions */

/**
 * \brief This function is used to write a byte to a register of the PCA9865.
 * \param [in] controllerConf -- The configuration of the PCA9865 controller.
 * \param [in] reg -- The register to write to.
 * \param [in] data -- The data to write to the register.
 * \returns PCA9865LIB_SUCCESS if the write operation is successful, otherwise
 *          PCA9865LIB_ERROR.
 */
int16_t PCA9865WriteReg(PCA9865I2CConf_t *controllerConf, uint8_t reg, uint8_t data)
{

    /* Verifying input */

    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Verifying requested register is not forbidden. */
    if ((reg > PCA9865_LED15_OFF_H_REG_ADDR && reg < PCA9865_ALL_LED_ON_L_REG_ADDR)
            || reg == PCA9865_TEST_MODE_REG_ADDR)
    {
        return PCA9865LIB_ERROR;
    }

    /* Writing data to the register */
    if (i2cWrite(&controllerConf->i2cConf, controllerConf->i2cAddr, 
                    1, reg, 1, &data) != US_I2C_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

/**
 * \brief This function is used to read a byte from a register of the PCA9865.
 * \param [in] controllerConf -- The configuration of the PCA9865 controller.
 * \param [in] reg -- The register to read from.
 * \param [out] data -- The data read from the register.
 * \returns PCA9865LIB_SUCCESS if the read operation is successful, otherwise
 *          PCA9865LIB_ERROR.
 */
int16_t PCA9865ReadReg(PCA9865I2CConf_t *controllerConf, uint8_t reg, uint8_t *data)
{

    /* Verifying input */

    if (controllerConf == NULL || data == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Verifying requested register is not forbidden. */
    if ((reg > PCA9865_LED15_OFF_H_REG_ADDR && reg < PCA9865_ALL_LED_ON_L_REG_ADDR)
            || reg == PCA9865_TEST_MODE_REG_ADDR)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading data from the register */
    if (i2cRead(&controllerConf->i2cConf, controllerConf->i2cAddr, 
                    1, reg, 1, data) != US_I2C_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

/* Exported Functions Definitions */

int16_t PCA9865_Init(PCA9865I2CConf_t *controllerConf, uint8_t i2cAddress, 
                        uint16_t i2cDevNumber)
{
    
    /* Verifying input */
    
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Opening I2C channel towards PCA9865 */
    if (i2cInit(i2cDevNumber, &controllerConf->i2cConf) != US_I2C_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /* Setting PCA9865 I2C address */
    controllerConf->i2cAddr = i2cAddress;

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_GetMode1Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode1Reg_u *modeReg)
{
    
    /* Verifying input */
    if (controllerConf == NULL || modeReg == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading mode 1 reg */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE1_REG_ADDR, (uint8_t *) &modeReg->regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_SetMode1Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode1Reg_u modeReg)
{

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Writing mode 1 reg */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE1_REG_ADDR, modeReg.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_GetMode2Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode2Reg_u *modeReg)
{
    
    /* Verifying input */
    if (controllerConf == NULL || modeReg == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading mode 2 reg */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE2_REG_ADDR, (uint8_t *) &modeReg->regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_SetMode2Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode2Reg_u modeReg)
{

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Writing mode 2 reg */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE2_REG_ADDR, (uint8_t) modeReg.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_GetPrescaler(PCA9865I2CConf_t *controllerConf, uint8_t *prescale)
{
    
    /* Verifying input */
    if (controllerConf == NULL || prescale == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading prescale reg */
    if (PCA9865ReadReg(controllerConf, PCA9865_PRE_SCALE_REG_ADDR, prescale) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_SetPrescaler(PCA9865I2CConf_t *controllerConf, uint8_t prescale)
{

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    if (prescale < PCA9865_MIN_PRESCALER)
    {
        prescale = PCA9865_MIN_PRESCALER;
    }

    /* Writing prescale reg */
    if (PCA9865WriteReg(controllerConf, PCA9865_PRE_SCALE_REG_ADDR, prescale) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_GetPWM(PCA9865I2CConf_t *controllerConf, uint8_t channel, 
                        uint16_t *onValue, uint16_t *offValue)
{

    uint8_t tmpValue_l = 0U; /** Temporary value to store lowbyte */
    uint8_t tmpValue_h = 0U; /** Temporary value to store highbyte */

    /* Verifying input */
    if (controllerConf == NULL || onValue == NULL || offValue == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    if (channel > PCA9865_MAX_PWM_CHANNELS)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading on value */
    if (PCA9865ReadReg(controllerConf, PCA9865_LED0_ON_L_REG_ADDR + (channel * 4U), 
                        (uint8_t *) &tmpValue_l) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    if (PCA9865ReadReg(controllerConf, PCA9865_LED0_ON_H_REG_ADDR + (channel * 4U), 
                        (uint8_t *) &tmpValue_h) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    *onValue = (uint16_t) ((tmpValue_h << 8U) | tmpValue_l);

    /* Reading off value */
    if (PCA9865ReadReg(controllerConf, PCA9865_LED0_OFF_L_REG_ADDR + (channel * 4U), 
                        (uint8_t *) &tmpValue_l) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    if (PCA9865ReadReg(controllerConf, PCA9865_LED0_OFF_H_REG_ADDR + (channel * 4U), 
                        (uint8_t *) &tmpValue_h) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    *offValue = (uint16_t) ((tmpValue_h << 8U) | tmpValue_l);

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_SetPWM(PCA9865I2CConf_t *controllerConf, uint8_t channel, 
                        uint16_t onValue, uint16_t offValue)
{
    
        /* Verifying input */
        if (controllerConf == NULL)
        {
            return PCA9865LIB_ERROR;
        }

        if (channel > PCA9865_MAX_PWM_CHANNELS)
        {
            return PCA9865LIB_ERROR;
        }

        if (onValue > PCA9865_MAX_PWM_VALUE || offValue > PCA9865_MAX_PWM_VALUE)
        {
            return PCA9865LIB_ERROR;
        }
    
        /* Writing on value */
        if (PCA9865WriteReg(controllerConf, PCA9865_LED0_ON_L_REG_ADDR + (channel * 4U), 
                            (uint8_t) onValue) != PCA9865LIB_SUCCESS)
        {
            return PCA9865LIB_ERROR;
        }
    
        if (PCA9865WriteReg(controllerConf, PCA9865_LED0_ON_H_REG_ADDR + (channel * 4U), 
                            (uint8_t) (onValue >> 8U)) != PCA9865LIB_SUCCESS)
        {
            return PCA9865LIB_ERROR;
        }
    
        /* Writing off value */
        if (PCA9865WriteReg(controllerConf, PCA9865_LED0_OFF_L_REG_ADDR + (channel * 4U), 
                            (uint8_t) offValue) != PCA9865LIB_SUCCESS)
        {
            return PCA9865LIB_ERROR;
        }
    
        if (PCA9865WriteReg(controllerConf, PCA9865_LED0_OFF_H_REG_ADDR + (channel * 4U), 
                            (uint8_t) (offValue >> 8U)) != PCA9865LIB_SUCCESS)
        {
            return PCA9865LIB_ERROR;
        }
    
        return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_GetAllPWM(PCA9865I2CConf_t *controllerConf, uint16_t *onValue, 
                            uint16_t *offValue)
{
    uint8_t tmpValue_l = 0U; /** Temporary value to store lowbyte */
    uint8_t tmpValue_h = 0U; /** Temporary value to store highbyte */

    /* Verifying input */
    if (controllerConf == NULL || onValue == NULL || offValue == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading on value */
    if (PCA9865ReadReg(controllerConf, PCA9865_ALL_LED_ON_L_REG_ADDR, 
                        (uint8_t *) &tmpValue_l) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    if (PCA9865ReadReg(controllerConf, PCA9865_ALL_LED_ON_H_REG_ADDR, 
                        (uint8_t *) &tmpValue_h) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    *onValue = (uint16_t) ((tmpValue_h << 8U) | tmpValue_l);

    /* Reading off value */
    if (PCA9865ReadReg(controllerConf, PCA9865_ALL_LED_OFF_L_REG_ADDR, 
                        (uint8_t *) &tmpValue_l) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    if (PCA9865ReadReg(controllerConf, PCA9865_ALL_LED_OFF_H_REG_ADDR, 
                        (uint8_t *) &tmpValue_h) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    *offValue = (uint16_t) ((tmpValue_h << 8U) | tmpValue_l);

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_SetAllPWM(PCA9865I2CConf_t *controllerConf, uint16_t onValue, 
                            uint16_t offValue)
{
        
    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    if (onValue > PCA9865_MAX_PWM_VALUE || offValue > PCA9865_MAX_PWM_VALUE)
    {
        return PCA9865LIB_ERROR;
    }

    /* Writing on value */
    if (PCA9865WriteReg(controllerConf, PCA9865_ALL_LED_ON_L_REG_ADDR, 
                        (uint8_t) onValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    if (PCA9865WriteReg(controllerConf, PCA9865_ALL_LED_ON_H_REG_ADDR, 
                        (uint8_t) (onValue >> 8U)) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /* Writing off value */
    if (PCA9865WriteReg(controllerConf, PCA9865_ALL_LED_OFF_L_REG_ADDR, 
                        (uint8_t) offValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    if (PCA9865WriteReg(controllerConf, PCA9865_ALL_LED_OFF_H_REG_ADDR, 
                        (uint8_t) (offValue >> 8U)) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_Reset(PCA9865I2CConf_t *controllerConf)
{
    PCA9865Mode1Reg_u mode1RegRead = {0U}; /** Mode 1 Reg current value */

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading MODE1 Reg first */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE1_REG_ADDR, (uint8_t *) &mode1RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /** Setting reset bit */
    mode1RegRead.bitfield.restart = 1;

    /* Writing reset command */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE1_REG_ADDR, mode1RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_Sleep(PCA9865I2CConf_t *controllerConf)
{
    PCA9865Mode1Reg_u mode1RegRead = {0U}; /** Mode 1 Reg current value */

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading MODE1 Reg first */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE1_REG_ADDR, (uint8_t *) &mode1RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /** Setting sleep bit */
    mode1RegRead.bitfield.sleep = 1;

    /* Writing sleep command */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE1_REG_ADDR, mode1RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_WakeUp(PCA9865I2CConf_t *controllerConf)
{
    PCA9865Mode1Reg_u mode1RegRead = {0U}; /** Mode 1 Reg current value */

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading MODE1 Reg first */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE1_REG_ADDR, (uint8_t *) &mode1RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /** Clearing sleep bit */
    mode1RegRead.bitfield.sleep = 0;

    /* Writing wake up command */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE1_REG_ADDR, mode1RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_EnableOutput(PCA9865I2CConf_t *controllerConf)
{
    PCA9865Mode2Reg_u mode2RegRead = {0U}; /** Mode 2 Reg current value */

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading MODE2 Reg first */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE2_REG_ADDR, (uint8_t *) &mode2RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /** Clearing OE bits */
    mode2RegRead.bitfield.outne = 0;

    /* Writing enable output command */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE1_REG_ADDR, mode2RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_DisableOutput(PCA9865I2CConf_t *controllerConf)
{
    PCA9865Mode2Reg_u mode2RegRead = {0U}; /** Mode 2 Reg current value */

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading MODE2 Reg first */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE2_REG_ADDR, (uint8_t *) &mode2RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /** Setting OE bits */
    mode2RegRead.bitfield.outne = 3;

    /* Writing disable output command */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE1_REG_ADDR, mode2RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_SetOutputInversion(PCA9865I2CConf_t *controllerConf, uint8_t invrt)
{
    PCA9865Mode2Reg_u mode2RegRead = {0U}; /** Mode 2 Reg current value */

    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Reading MODE2 Reg first */
    if (PCA9865ReadReg(controllerConf, PCA9865_MODE2_REG_ADDR, (uint8_t *) &mode2RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    /** Setting INVRT bits */
    mode2RegRead.bitfield.invrt = invrt;

    /* Writing set output inversion command */
    if (PCA9865WriteReg(controllerConf, PCA9865_MODE1_REG_ADDR, mode2RegRead.regValue) != PCA9865LIB_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}

int16_t PCA9865_Close(PCA9865I2CConf_t *controllerConf)
{
    /* Verifying input */
    if (controllerConf == NULL)
    {
        return PCA9865LIB_ERROR;
    }

    /* Closing I2C channel */
    if (i2cClose(&controllerConf->i2cConf) != US_I2C_SUCCESS)
    {
        return PCA9865LIB_ERROR;
    }

    return PCA9865LIB_SUCCESS;
}
