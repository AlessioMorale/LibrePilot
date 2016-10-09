/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_linesensor
 * @brief PIOS interface for for the QTR-8RC Reflectance Sensor Array
 * @{
 *
 * @file       pios_linesensor.c
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
* - The driver waits for the CC interrupt from the timer
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

#ifdef PIOS_INCLUDE_LINESENSOR
#include "pios_linesensor.h"


enum pios_linesensor_fsm {
    PIOS_LINESENSOR_START = 0,
    PIOS_LINESENSOR_BLANK = 1,
    PIOS_LINESENSOR_ACQ_TIMING = 2,
    PIOS_LINESENSOR_END   = 3
};

typedef struct {
    enum pios_linesensor_dev_magic   magic;
    const struct pios_linesensor_cfg *cfg;
    volatile uint16_t timings[NUM_SENSOR];
    volatile uint16_t reads[NUM_SENSOR];
    volatile bool     status[NUM_SENSOR];
    uint8_t fsm_status;
    uint32_t tim_id;
} pios_linesensor_dev_t;

#if 0
static void linesensor_tim_ed_cb(uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count);
static void linesensor_tim_of_cb(uint32_t tim_id, uint32_t context, uint8_t chan_idx, uint16_t count);

const struct pios_tim_callbacks linefollower_callback = {
    .overflow = &linesensor_tim_of_cb,
    .edge     = &linesensor_tim_ed_cb,
};
#endif

static pios_linesensor_dev_t *dev;

static void setOutputs(bool blank);


static bool linesensor_validate(pios_linesensor_dev_t *linesensor_dev)
{
    return linesensor_dev != NULL
           && linesensor_dev->magic == PIOS_LINESENSOR_DEV_MAGIC;
}

int32_t PIOS_Linesensor_Init(const struct pios_linesensor_cfg *cfg)
{
    PIOS_Assert(cfg);

    dev = pios_malloc(sizeof(pios_linesensor_dev_t));
    memset(dev, 0, sizeof(pios_linesensor_dev_t));
    dev->cfg   = cfg;
    dev->fsm_status = PIOS_LINESENSOR_START;
    dev->magic = PIOS_LINESENSOR_DEV_MAGIC;
    setOutputs(false);
#if 0
    struct pios_tim_channel tim = {
        .timer      = dev->cfg->timer,
        .timer_chan = 1,
    };

    PIOS_TIM_InitChannels(&dev->tim_id, &tim, 1, &linefollower_callback, (uint32_t)dev);
#endif
    for (uint32_t i = 0; i < NUM_SENSOR; i++) {
        PIOS_EXTI_Init(dev->cfg->exti_cfg[i]);
    }
    TIM_Cmd(dev->cfg->timer, DISABLE);
    // Configure timebase and internal clock
    TIM_TimeBaseInit(dev->cfg->timer, &dev->cfg->timerInit);
    TIM_OC1PreloadConfig(dev->cfg->timer, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(dev->cfg->timer, ENABLE);
#if 0
    TIM_OCInitTypeDef oc = {
        .TIM_OCMode       = TIM_OCMode_PWM1,
        .TIM_OutputState  = TIM_OutputState_Enable,
        .TIM_OutputNState = TIM_OutputNState_Disable,
        .TIM_Pulse        = 0,
        .TIM_OCPolarity   = TIM_OCPolarity_High,
        .TIM_OCNPolarity  = TIM_OCNPolarity_High,
        .TIM_OCIdleState  = TIM_OCIdleState_Reset,
        .TIM_OCNIdleState = TIM_OCNIdleState_Reset,
    };


    TIM_OC1Init(dev->cfg->timer, &oc);
    TIM_CtrlPWMOutputs(dev->cfg->timer, ENABLE);
    TIM_CCxCmd(dev->cfg->timer, TIM_Channel_1, ENABLE);
    TIM_ITConfig(dev->cfg->timer, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(dev->cfg->timer, TIM_IT_Update, ENABLE);
#endif
    TIM_Cmd(dev->cfg->timer, ENABLE);
    return 0;
}

void PIOS_Linesensor_read(uint16_t *readout)
{
    for (uint8_t i = 0; i < NUM_SENSOR; i++) {
        readout[i] = (uint16_t)(dev->status[i] ? dev->timings[i] : 0xFFFF);
    }
    PIOS_Linesensor_setup();
}

bool PIOS_Linesensor_EXT_Int(void)
{
    if (!linesensor_validate(dev)) {
        return false;
    }

    if (dev->fsm_status == PIOS_LINESENSOR_ACQ_TIMING) {
        for (uint8_t i = 0; i < NUM_SENSOR; i++) {
            if (!dev->status[i]) {
                if (GPIO_ReadInputDataBit(dev->cfg->exti_cfg[i]->pin.gpio, dev->cfg->exti_cfg[i]->pin.init.GPIO_Pin) == Bit_RESET) {
                    dev->timings[i] = TIM_GetCounter(dev->cfg->timer) - dev->timings[i];
                    dev->status[i]  = true;
                }
            }
        }
    }

    return false;
}

void PIOS_Linesensor_setup()
{
    dev->fsm_status = PIOS_LINESENSOR_BLANK;
    setOutputs(true);
}

void PIOS_Linesensor_start()
{
    dev->fsm_status = PIOS_LINESENSOR_ACQ_TIMING;
    setOutputs(false);
}

static void setOutputs(bool blank)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    if (blank) {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    } else {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    }

    for (uint8_t i = 0; i < NUM_SENSOR; i++) {
        GPIO_InitStructure.GPIO_Pin = dev->cfg->exti_cfg[i]->pin.init.GPIO_Pin;
        GPIO_Init(dev->cfg->exti_cfg[i]->pin.gpio, &GPIO_InitStructure);
        if (!blank) {
            dev->status[i]  = false;
            dev->timings[i] = TIM_GetCounter(dev->cfg->timer);
            GPIO_WriteBit(dev->cfg->exti_cfg[i]->pin.gpio, dev->cfg->exti_cfg[i]->pin.init.GPIO_Pin, Bit_SET);
        }
    }
}
#if 0
static void linesensor_tim_ed_cb(__attribute__((unused)) uint32_t tim_id, __attribute__((unused)) uint32_t context, __attribute__((unused)) uint8_t chan_idx, __attribute__((unused)) uint16_t count)
{
    // PIOS_Linesensor_start();
    dev->reads[0]++;
}

static void linesensor_tim_of_cb(__attribute__((unused)) uint32_t tim_id, __attribute__((unused)) uint32_t context, __attribute__((unused)) uint8_t chan_idx, __attribute__((unused)) uint16_t count)
{
    /*for(uint8_t i = 0; i < NUM_SENSOR; i++){
           //dev->reads[i] = (uint16_t)(dev->status[i] ? dev->timings[i] : 0xFFFF);
       }
       PIOS_Linesensor_setup();*/
    dev->reads[1]++;
}
#endif
#endif /* PIOS_INCLUDE_LINESENSOR */
