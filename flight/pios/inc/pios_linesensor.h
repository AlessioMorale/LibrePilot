/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_linesensor
 * @brief PIOS interface for for the QTR-8RC Reflectance Sensor Array
 * @{
 *
 * @file       pios_linesensor.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 * @brief      Implements a driver for the QTR-8RC Reflectance Sensor Array
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*******************************************************************
* QTR-8RC Reflectance Sensor Array
* Work with the following principle:
* uses 8 EXTI configurations for each sensor output
* A single timer is used for time measurement and sampling time base
*
* - The driver waits for the timer CC interrupt
* - AS soon as interrupt is received, previous acquired times are
* copied to the final variable and made available for read via API.
* output are set high and charging timeout configured in Timer CC register
* - A second interrupt is received, lines are configured back as
* ADC inputs and sampling timeout is configured in Timer CC register
* - Exti are triggered by inputs going low, within the ISRs timer
* counter is read and result stored for each pin.
*
*******************************************************************/

#include "pios.h"

#ifndef FLIGHT_PIOS_INC_PIOS_LINESENSOR_H_
#define FLIGHT_PIOS_INC_PIOS_LINESENSOR_H_

extern bool PIOS_Linesensor_EXT_Int(void);

#define DefineLinesensorEXTI_Config(_num, _exti_line, _gpio, _pin, _exti_irq) \
    static const struct pios_exti_cfg pios_exti_linesensor##_num##_cfg __exti_config = { \
        .vector = PIOS_Linesensor_EXT_Int, \
        .line   = _exti_line, \
        .pin    = { \
            .gpio = _gpio, \
            .init = { \
                .GPIO_Pin   = _pin, \
                .GPIO_Speed = GPIO_Speed_100MHz, \
                .GPIO_Mode  = GPIO_Mode_IN, \
                .GPIO_OType = GPIO_OType_OD, \
                .GPIO_PuPd  = GPIO_PuPd_NOPULL, \
            }, \
        }, \
        .irq                                       = { \
            .init                                  = { \
                .NVIC_IRQChannel    = _exti_irq, \
                .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH, \
                .NVIC_IRQChannelSubPriority        = 0, \
                .NVIC_IRQChannelCmd = ENABLE, \
            }, \
        }, \
        .exti                                      = { \
            .init                                  = { \
                .EXTI_Line    = _exti_line,  \
                .EXTI_Mode    = EXTI_Mode_Interrupt, \
                .EXTI_Trigger = EXTI_Trigger_Falling, \
                .EXTI_LineCmd = ENABLE, \
            }, \
        }, \
    }


#define NUM_SENSOR 6
/* Global Types */
struct pios_linesensor_cfg {
    const struct pios_exti_cfg *exti_cfg[NUM_SENSOR]; /* Pointer to the EXTI configuration */
    TIM_TimeBaseInitTypeDef    timerInit;
    TIM_TypeDef *timer;
    uint16_t    sampling_interval;
    uint16_t    blanking_interval;
};

enum pios_linesensor_dev_magic {
    PIOS_LINESENSOR_DEV_MAGIC = 0x114e5e45,
};

bool PIOS_Linesensor_EXT_Int(void);
extern void PIOS_Linesensor_setup();
extern void PIOS_Linesensor_start();
extern void PIOS_Linesensor_read(uint16_t *readout);
extern int32_t PIOS_Linesensor_Init(const struct pios_linesensor_cfg *cfg);
#endif /* FLIGHT_PIOS_INC_PIOS_LINESENSOR_H_ */
