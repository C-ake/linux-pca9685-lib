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
 * \file pca9865lib.h
 * \brief This file contains the declarations of the functions that are used to
 *       manage the PCA9865 PWM controller.
 */

#ifndef PCA9865LIB_H
#define PCA9865LIB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Standard library includes */
#include <stdint.h>

/* Local includes */
#include "us-i2c.h"

/* Macros */

/* PCA9865 Registers Addresses */
#define PCA9865_MODE1_REG_ADDR ((uint8_t) 0x00)
#define PCA9865_MODE2_REG_ADDR ((uint8_t) 0x01)
#define PCA9865_I2C_SUBADDR1_REG_ADDR ((uint8_t) 0x02)
#define PCA9865_I2C_SUBADDR2_REG_ADDR ((uint8_t) 0x03)
#define PCA9865_I2C_SUBADDR3_REG_ADDR ((uint8_t) 0x04)
#define PCA9865_ALLCALL_ADDR_REG_ADDR ((uint8_t) 0x05)
#define PCA9865_LED0_ON_L_REG_ADDR ((uint8_t) 0x06)
#define PCA9865_LED0_ON_H_REG_ADDR ((uint8_t) 0x07)
#define PCA9865_LED0_OFF_L_REG_ADDR ((uint8_t) 0x08)
#define PCA9865_LED0_OFF_H_REG_ADDR ((uint8_t) 0x09)
#define PCA9865_LED1_ON_L_REG_ADDR ((uint8_t) 0x0A)
#define PCA9865_LED1_ON_H_REG_ADDR ((uint8_t) 0x0B)
#define PCA9865_LED1_OFF_L_REG_ADDR ((uint8_t) 0x0C)
#define PCA9865_LED1_OFF_H_REG_ADDR ((uint8_t) 0x0D)
#define PCA9865_LED2_ON_L_REG_ADDR ((uint8_t) 0x0E)
#define PCA9865_LED2_ON_H_REG_ADDR ((uint8_t) 0x0F)
#define PCA9865_LED2_OFF_L_REG_ADDR ((uint8_t) 0x10)
#define PCA9865_LED2_OFF_H_REG_ADDR ((uint8_t) 0x11)
#define PCA9865_LED3_ON_L_REG_ADDR ((uint8_t) 0x12)
#define PCA9865_LED3_ON_H_REG_ADDR ((uint8_t) 0x13)
#define PCA9865_LED3_OFF_L_REG_ADDR ((uint8_t) 0x14)
#define PCA9865_LED3_OFF_H_REG_ADDR ((uint8_t) 0x15)
#define PCA9865_LED4_ON_L_REG_ADDR ((uint8_t) 0x16)
#define PCA9865_LED4_ON_H_REG_ADDR ((uint8_t) 0x17)
#define PCA9865_LED4_OFF_L_REG_ADDR ((uint8_t) 0x18)
#define PCA9865_LED4_OFF_H_REG_ADDR ((uint8_t) 0x19)
#define PCA9865_LED5_ON_L_REG_ADDR ((uint8_t) 0x1A)
#define PCA9865_LED5_ON_H_REG_ADDR ((uint8_t) 0x1B)
#define PCA9865_LED5_OFF_L_REG_ADDR ((uint8_t) 0x1C)
#define PCA9865_LED5_OFF_H_REG_ADDR ((uint8_t) 0x1D)
#define PCA9865_LED6_ON_L_REG_ADDR ((uint8_t) 0x1E)
#define PCA9865_LED6_ON_H_REG_ADDR ((uint8_t) 0x1F)
#define PCA9865_LED6_OFF_L_REG_ADDR ((uint8_t) 0x20)
#define PCA9865_LED6_OFF_H_REG_ADDR ((uint8_t) 0x21)
#define PCA9865_LED7_ON_L_REG_ADDR ((uint8_t) 0x22)
#define PCA9865_LED7_ON_H_REG_ADDR ((uint8_t) 0x23)
#define PCA9865_LED7_OFF_L_REG_ADDR ((uint8_t) 0x24)
#define PCA9865_LED7_OFF_H_REG_ADDR ((uint8_t) 0x25)
#define PCA9865_LED8_ON_L_REG_ADDR ((uint8_t) 0x26)
#define PCA9865_LED8_ON_H_REG_ADDR ((uint8_t) 0x27)
#define PCA9865_LED8_OFF_L_REG_ADDR ((uint8_t) 0x28)
#define PCA9865_LED8_OFF_H_REG_ADDR ((uint8_t) 0x29)
#define PCA9865_LED9_ON_L_REG_ADDR ((uint8_t) 0x2A)
#define PCA9865_LED9_ON_H_REG_ADDR ((uint8_t) 0x2B)
#define PCA9865_LED9_OFF_L_REG_ADDR ((uint8_t) 0x2C)
#define PCA9865_LED9_OFF_H_REG_ADDR ((uint8_t) 0x2D)
#define PCA9865_LED10_ON_L_REG_ADDR ((uint8_t) 0x2E)
#define PCA9865_LED10_ON_H_REG_ADDR ((uint8_t) 0x2F)
#define PCA9865_LED10_OFF_L_REG_ADDR ((uint8_t) 0x30)
#define PCA9865_LED10_OFF_H_REG_ADDR ((uint8_t) 0x31)
#define PCA9865_LED11_ON_L_REG_ADDR ((uint8_t) 0x32)
#define PCA9865_LED11_ON_H_REG_ADDR ((uint8_t) 0x33)
#define PCA9865_LED11_OFF_L_REG_ADDR ((uint8_t) 0x34)
#define PCA9865_LED11_OFF_H_REG_ADDR ((uint8_t) 0x35)
#define PCA9865_LED12_ON_L_REG_ADDR ((uint8_t) 0x36)
#define PCA9865_LED12_ON_H_REG_ADDR ((uint8_t) 0x37)
#define PCA9865_LED12_OFF_L_REG_ADDR ((uint8_t) 0x38)
#define PCA9865_LED12_OFF_H_REG_ADDR ((uint8_t) 0x39)
#define PCA9865_LED13_ON_L_REG_ADDR ((uint8_t) 0x3A)
#define PCA9865_LED13_ON_H_REG_ADDR ((uint8_t) 0x3B)
#define PCA9865_LED13_OFF_L_REG_ADDR ((uint8_t) 0x3C)
#define PCA9865_LED13_OFF_H_REG_ADDR ((uint8_t) 0x3D)
#define PCA9865_LED14_ON_L_REG_ADDR ((uint8_t) 0x3E)
#define PCA9865_LED14_ON_H_REG_ADDR ((uint8_t) 0x3F)
#define PCA9865_LED14_OFF_L_REG_ADDR ((uint8_t) 0x40)
#define PCA9865_LED14_OFF_H_REG_ADDR ((uint8_t) 0x41)
#define PCA9865_LED15_ON_L_REG_ADDR ((uint8_t) 0x42)
#define PCA9865_LED15_ON_H_REG_ADDR ((uint8_t) 0x43)
#define PCA9865_LED15_OFF_L_REG_ADDR ((uint8_t) 0x44)
#define PCA9865_LED15_OFF_H_REG_ADDR ((uint8_t) 0x45)
#define PCA9865_ALL_LED_ON_L_REG_ADDR ((uint8_t) 0xFA)
#define PCA9865_ALL_LED_ON_H_REG_ADDR ((uint8_t) 0xFB)
#define PCA9865_ALL_LED_OFF_L_REG_ADDR ((uint8_t) 0xFC)
#define PCA9865_ALL_LED_OFF_H_REG_ADDR ((uint8_t) 0xFD)
#define PCA9865_PRE_SCALE_REG_ADDR ((uint8_t) 0xFE)
#define PCA9865_TEST_MODE_REG_ADDR ((uint8_t) 0xFF)


/* Return codes */
#define PCA9865LIB_SUCCESS ((int16_t) 0)
#define PCA9865LIB_ERROR ((int16_t) -1)

/* Misc Defines */
#define PCA9865_MAX_PWM_VALUE ((uint16_t) 4096U)
#define PCA9865_MAX_PWM_CHANNELS ((uint8_t) 16U)
#define PCA9865_MAX_PRESCALER ((uint8_t) 0xFFU)
#define PCA9865_MIN_PRESCALER ((uint8_t) 0x03U)
#define PCA9865_INT_CLOCK_FREQ ((uint32_t) 25000000U)


#define COMPUTE_PRESCALER_VALUE(frequency) \
    (uint8_t) ((PCA9865_INT_CLOCK_FREQ / (PCA9865_MAX_PWM_VALUE * frequency)) - 1U)


/* Typedefs */

/**
 * \struct PCA9865I2CConf_t "pca9865lib.h" pca9865lib.h
 * \brief This structure contains the configuration parameters 
 *        of i2c communication to the PCA9865 controller.
*/
typedef struct PCA9865I2CConf_s
{
    i2cConfiguration_t i2cConf; /** I2C configuration parameters */
    uint8_t i2cAddr; /** I2C address of the PCA9865 controller */
} PCA9865I2CConf_t;

/**
 * \struct PCA9865ModeReg_t "pca9865lib.h" pca9865lib.h
 * \brief This structure holds MODE1 register bitfield.
*/
typedef struct PCA9865Mode1Reg_s
{
    uint8_t restart : 1; /** Restart bit */
    uint8_t extclk : 1; /** External clock bit */
    uint8_t ai : 1; /** Auto increment bit */
    uint8_t sleep : 1; /** Sleep bit */
    uint8_t sub1 : 1; /** Subaddress 1 bit */
    uint8_t sub2 : 1; /** Subaddress 2 bit */
    uint8_t sub3 : 1; /** Subaddress 3 bit */
    uint8_t allcall : 1; /** All call bit */
} PCA9865Mode1Reg_t;

typedef union
{
    uint8_t regValue;
    PCA9865Mode1Reg_t bitfield;
} PCA9865Mode1Reg_u;

/**
 * \struct PCA9865Mode2Reg_t "pca9865lib.h" pca9865lib.h
 * \brief This structure holds MODE2 register bitfield.
*/
typedef struct PCA9865Mode2Reg_s
{
    uint8_t reserved : 3; /** Reserved bits */
    uint8_t invrt : 1; /** Output logic state inversion bit */
    uint8_t ocha : 1; /** Outputs change on ACK bit */
    uint8_t outdrv : 1; /** Output driver bit */
    uint8_t outne : 2; /** Output not enabled bits */
} PCA9865Mode2Reg_t;

typedef union
{
    uint8_t regValue;
    PCA9865Mode2Reg_t bitfield;
} PCA9865Mode2Reg_u;


/* Functions declarations */

/**
 * \brief This function initializes and verifies communication to
 *        the PCA9865 controller.
 * \param [in] controllerConf -- PCA9865 I2C configuration parameters.
 * \param [in] i2cAddress -- I2C slave address of the PCA9865 controller.
 * \param [in] i2cDevNumber -- Linux I2C dev number. (e.g: /dev/i2c-1, i2cDevNumber = 1)
 * \returns PCA9865LIB_SUCCESS if communication is successfully established,
 * and mode and prescaler registers are successfully configured, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_Init(PCA9865I2CConf_t *controllerConf, uint8_t i2cAddress, 
                        uint16_t i2cDevNumber);

/**
 * \brief This function gets the MODE1 register value.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [out] modeReg -- Pointer to the structure that holds the MODE1 register bitfield.
 * \returns PCA9865LIB_SUCCESS if the MODE1 register value is successfully read, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_GetMode1Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode1Reg_u *modeReg);

/**
 * \brief This function sets the MODE1 register value.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [in] modeReg -- MODE1 register value.
 * \returns PCA9865LIB_SUCCESS if the MODE1 register value is successfully set, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_SetMode1Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode1Reg_u modeReg);

/**
 * \brief This function gets the MODE2 register value.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [out] mode2Reg -- Pointer to the structure that holds the MODE2 register bitfield.
 * \returns PCA9865LIB_SUCCESS if the MODE2 register value is successfully read, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_GetMode2Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode2Reg_u *mode2Reg);

/**
 * \brief This function sets the MODE2 register value.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [in] mode2Reg -- MODE2 register value.
 * \returns PCA9865LIB_SUCCESS if the MODE2 register value is successfully set, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_SetMode2Reg(PCA9865I2CConf_t *controllerConf, PCA9865Mode2Reg_u mode2Reg);

/**
 * \brief This function gets the prescaler value.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [out] prescaler -- Pointer to the prescaler value.
 * \returns PCA9865LIB_SUCCESS if the prescaler value is successfully read, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_GetPrescaler(PCA9865I2CConf_t *controllerConf, uint8_t *prescaler);

/**
 * \brief This function sets the prescaler value.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [in] prescaler -- Prescaler value.
 * \returns PCA9865LIB_SUCCESS if the prescaler value is successfully set, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_SetPrescaler(PCA9865I2CConf_t *controllerConf, uint8_t prescaler);

/**
 * \brief This function gets the ON and OFF values of a PWM channel.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [in] channel -- PWM channel number.
 * \param [out] onValue -- Pointer to the ON value.
 * \param [out] offValue -- Pointer to the OFF value.
 * \returns PCA9865LIB_SUCCESS if the ON and OFF values are successfully read, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_GetPWM(PCA9865I2CConf_t *controllerConf, uint8_t channel, 
                        uint16_t *onValue, uint16_t *offValue);

/**
 * \brief This function sets the ON and OFF values of a PWM channel.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [in] channel -- PWM channel number.
 * \param [in] onValue -- ON value.
 * \param [in] offValue -- OFF value.
 * \returns PCA9865LIB_SUCCESS if the ON and OFF values are successfully set, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_SetPWM(PCA9865I2CConf_t *controllerConf, uint8_t channel, 
                        uint16_t onValue, uint16_t offValue);


/**
 * \brief This function gets the ON and OFF values of all PWM channels.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [out] onValue -- Pointer to the ON value.
 * \param [out] offValue -- Pointer to the OFF value.
 * \returns PCA9865LIB_SUCCESS if the ON and OFF values are successfully read, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_GetAllPWM(PCA9865I2CConf_t *controllerConf, uint16_t *onValue, 
                            uint16_t *offValue);

/**
 * \brief This function sets the ON and OFF values of all PWM channels.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [in] onValue -- ON value.
 * \param [in] offValue -- OFF value.
 * \returns PCA9865LIB_SUCCESS if the ON and OFF values are successfully set, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_SetAllPWM(PCA9865I2CConf_t *controllerConf, uint16_t onValue, 
                            uint16_t offValue);


/**
 * \brief This function resets the PCA9865 controller.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \returns PCA9865LIB_SUCCESS if the controller is successfully reset, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_Reset(PCA9865I2CConf_t *controllerConf);

/**
 * \brief This function puts the PCA9865 controller in sleep mode.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \returns PCA9865LIB_SUCCESS if the controller is successfully put in sleep mode, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_Sleep(PCA9865I2CConf_t *controllerConf);

/**
 * \brief This function wakes up the PCA9865 controller.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \returns PCA9865LIB_SUCCESS if the controller is successfully woken up, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_WakeUp(PCA9865I2CConf_t *controllerConf);

/**
 * \brief This function enables the output of the PCA9865 controller.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \returns PCA9865LIB_SUCCESS if the controller output is successfully enabled, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_EnableOutput(PCA9865I2CConf_t *controllerConf);

/**
 * \brief This function disables the output of the PCA9865 controller.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \returns PCA9865LIB_SUCCESS if the controller output is successfully disabled, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_DisableOutput(PCA9865I2CConf_t *controllerConf);

/**
 * \brief This function sets the output logic state inversion of the PCA9865 controller.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \param [in] invrt -- Output logic state inversion bit.
 * \returns PCA9865LIB_SUCCESS if the output logic state inversion is successfully set, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_SetOutputInversion(PCA9865I2CConf_t *controllerConf, uint8_t invrt);

/**
 * \brief This function closes communication with the controller.
 * \param [in] controllerConf -- Pointer to the controller configuration data structure.
 * \returns PCA9865LIB_SUCCESS if the communication is successfully closed, otherwise PCA9865LIB_ERROR.
*/
int16_t PCA9865_Close(PCA9865I2CConf_t *controllerConf);


#ifdef __cplusplus
}
#endif

#endif // PCA9865LIB_H