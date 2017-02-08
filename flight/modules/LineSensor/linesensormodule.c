/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup LineSensorModule Module
 * @brief      Module to read a line sensor.
 * Updates the FlightBatteryState object
 * @{
 *
 * @file       linesensormodule.c
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

/**
 * Output object: FlightBatteryState
 *
 * This module will periodically generate information on the battery state.
 *
 * UAVObjects are automatically generated by the UAVObjectGenerator from
 * the object definition XML file.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "openpilot.h"
#include <linesensor.h>
#include <linesensorsettings.h>
#include <pios_linesensor.h>
#include <mathmisc.h>
//
// Configuration
//
#define SAMPLE_PERIOD_MS 1
// Private types

// Private variables

// Private functions
static void onTimer(UAVObjEvent *ev);
static void settingscb(__attribute__((unused)) UAVObjEvent *ev);
static LineSensorSettingsData settings;
/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t LineSensorModuleInitialize(void)
{
    LineSensorInitialize();
    LineSensorSettingsInitialize();
    static UAVObjEvent ev;

    memset(&ev, 0, sizeof(UAVObjEvent));
    LineSensorSettingsConnectCallback(&settingscb);
    settingscb(NULL);
    EventPeriodicCallbackCreate(&ev, onTimer, SAMPLE_PERIOD_MS / portTICK_RATE_MS);
    return 0;
}


MODULE_INITCALL(LineSensorModuleInitialize, 0);

static void settingscb(__attribute__((unused)) UAVObjEvent *ev)
{
    LineSensorSettingsGet(&settings);
}

static void onTimer(__attribute__((unused)) UAVObjEvent *ev)
{
    static bool calibrationSaved = false;
    static LineSensorData sensorData;
    static uint8_t status;

    switch (status) {
    case 0:
        PIOS_Linesensor_start();
        break;
    default:
        PIOS_Linesensor_read(sensorData.rawsensors);
        static float max = 1;
        static float min = 0xffff;
        switch (settings.CalibrationMode) {
        case LINESENSORSETTINGS_CALIBRATIONMODE_ENABLED:
        {
            calibrationSaved = false;
            for (uint32_t i = 0; i < NUM_SENSOR; i++) {
                uint16_t value = sensorData.rawsensors[i];
                if (value != 0xFFFF) {
                    max = value > max ? value : max;
                    min = value < min ? value : min;
                } else {
                    value = max;
                }
            }
            max = (0.999f) * max;
            min = (1.001f) * min;
        }
        break;

        case LINESENSORSETTINGS_CALIBRATIONMODE_MANUAL:
        {
            max = settings.max;
            min = settings.min;
            break;
        }
        case LINESENSORSETTINGS_CALIBRATIONMODE_DONE:
            if (!calibrationSaved) {
                calibrationSaved = true;
                LineSensorSettingsmaxSet(&max);
                LineSensorSettingsminSet(&min);
            }
            break;
        }

        sensorData.max = max;
        sensorData.min = min;
        float invrange = 1.0f / (sensorData.max - sensorData.min);
        float n1 = 0;
        float n2 = 0;
        for (uint32_t i = 0; i < NUM_SENSOR; i++) {
            float val = ((float)sensorData.rawsensors[i] - sensorData.min) * invrange;

            sensorData.sensors[i] = val;
            n1 += val * (float)i;
            n2 += val;
        }

        if (n2 > settings.TrackTreshold.Warning) {
            sensorData.TrackStatus = LINESENSOR_TRACKSTATUS_OK;
        } else if (n2 > settings.TrackTreshold.Lost) {
            sensorData.TrackStatus = LINESENSOR_TRACKSTATUS_WARNING;
        } else {
            sensorData.TrackStatus = LINESENSOR_TRACKSTATUS_NOTRACK;
        }

        sensorData.value = (n1 / n2 - 2.5f) * settings.range + settings.offset;

        LineSensorSet(&sensorData);
        status = 0;
        return;
    }
    status++;
}

/**
 * @}
 */

/**
 * @}
 */
