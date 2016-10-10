/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup LineFollowerModule Module
 * @brief      Module Manage the line follower operations.
 * Updates the FlightBatteryState object
 * @{
 *
 * @file       linefollowermodule.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2015-2016.
 * @brief      Module to read a line sensor.
 *
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

#include "openpilot.h"
#include <pid.h>
#include <mathmisc.h>
// uavo
#include <linesensor.h>
#include <linefollowercontrol.h>
#include <linefollowerstatus.h>
#include <linefollowersettings.h>
#include <stabilizationdesired.h>
#include <accessorydesired.h>
#include <ratedesired.h>
#include <taskinfo.h>

//
// Configuration
//
#define TASK_PERIOD_TICK ((TickType_t)2)
#define STACK_SIZE_BYTES 1500
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define MAX_QUEUE_SIZE   2

#define UPDATE_EXPECTED  (1.0f / 500)
#define UPDATE_MIN       1.0e-6f
#define UPDATE_MAX       1.0f
#define UPDATE_ALPHA     1.0e-2f
// Private types

// Private variables
// Private variables
static xTaskHandle taskHandle;
static xQueueHandle queue;

// Private functions
static void lfcontrolcb(__attribute__((unused)) UAVObjEvent *ev);
static void lfsettingscb(__attribute__((unused)) UAVObjEvent *ev);

static void lineFollowerTask(void *parameters);
static volatile bool controlupdated  = false;
static volatile bool settingsupdated = false;

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t LineFollowerModuleInitialize(void)
{
    LineSensorInitialize();
    LineFollowerControlInitialize();
    LineFollowerStatusInitialize();
    LineFollowerSettingsInitialize();
    StabilizationDesiredInitialize();
    RateDesiredInitialize();
    AccessoryDesiredInitialize();

    static UAVObjEvent ev;
    memset(&ev, 0, sizeof(UAVObjEvent));
    LineFollowerControlConnectCallback(&lfcontrolcb);
    LineFollowerSettingsConnectCallback(&lfsettingscb);
    // Create object queue
    queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));

    // Listen for FlightPlanControl updates
    LineSensorConnectQueue(queue);
    lfcontrolcb(NULL);
    lfsettingscb(NULL);

    return 0;
}

int32_t LineFollowerModuleStart(void)
{
    taskHandle = NULL;

    xTaskCreate(lineFollowerTask, (const char *)"LF", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_LINEFOLLOWER, taskHandle);

    return 0;
}

MODULE_INITCALL(LineFollowerModuleInitialize, LineFollowerModuleStart);

static void lineFollowerTask(__attribute__((unused)) void *parameters)
{
    LineFollowerControlData control;
    LineFollowerStatusData status;
    LineFollowerSettingsData settings;
    LineSensorData sensor;
    uint32_t armingTimer = 0;
    bool arming = false;
    UAVObjEvent ev;
    LineFollowerStatusStatusOptions currentstatus = LINEFOLLOWERSTATUS_STATUS_IDLE;
    struct pid linePid;

    pid_zero(&linePid);
    float yawrate  = 0.0f;
    float throttle = 0.0f;
    PiOSDeltatimeConfig timeval;
    float dT;

    PIOS_DELTATIME_Init(&timeval, UPDATE_EXPECTED, UPDATE_MIN, UPDATE_MAX, UPDATE_ALPHA);


    while (1) {
        if (settingsupdated) {
            settingsupdated = false;
            LineFollowerSettingsGet(&settings);
            pid_configure(&linePid, settings.LineSensorPID.Kp, settings.LineSensorPID.Ki, settings.LineSensorPID.Kd, settings.LineSensorPID.ILimit);
        }
        if (xQueueReceive(queue, &ev, TASK_PERIOD_TICK) == pdTRUE) {
            dT = PIOS_DELTATIME_GetAverageSeconds(&timeval);
            LineSensorGet(&sensor);
            yawrate   = pid_apply(&linePid, -sensor.value, dT);
            status.yawrate = yawrate;
            status.dT = dT;
        }

        // TODO! Throttle processing


        status.throttle = throttle;

        if (status.Status == LINEFOLLOWERSTATUS_STATUS_RUN) {
            RateDesiredYawSet(&yawrate);
            // RateDesiredThrustSet(&throttle);
        }


        if (controlupdated) {
            controlupdated = false;
            LineFollowerControlGet(&control);
            switch (control.Command) {
            case LINEFOLLOWERCONTROL_COMMAND_IDLE:
                status.Status = LINEFOLLOWERSTATUS_STATUS_IDLE;
                break;
            case LINEFOLLOWERCONTROL_COMMAND_CALIBRATE:
                break;
            case LINEFOLLOWERCONTROL_COMMAND_RUN:
                if (currentstatus != LINEFOLLOWERSTATUS_STATUS_RUN) {
                    arming = true;
                    armingTimer = PIOS_DELAY_GetRaw();
                    status.Status = LINEFOLLOWERSTATUS_STATUS_ARMING;
                }
                break;
            case LINEFOLLOWERCONTROL_COMMAND_STOP:
                status.Status = LINEFOLLOWERSTATUS_STATUS_IDLE;
            }
        }

        if (arming) {
            if (PIOS_DELAY_GetuSSince(armingTimer) > (uint32_t)settings.StartDelay * 1000000) {
                arming = false;
                status.Status = LINEFOLLOWERSTATUS_STATUS_RUN;
            }
        }
        LineFollowerStatusSet(&status);
    }
}


static void lfcontrolcb(__attribute__((unused)) UAVObjEvent *ev)
{
    controlupdated = true;
}

static void lfsettingscb(__attribute__((unused)) UAVObjEvent *ev)
{
    settingsupdated = true;
}

/**
 * @}
 */

/**
 * @}
 */
