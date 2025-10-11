/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_gps_lap_timer.h"
#include "cms/cms_menu_ledstrip.h"

#include "common/utils.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "cms_menu_misc.h"

//
// Misc
//

static const void *cmsx_menuRcOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    inhibitSaveMenu();

    return NULL;
}

static const void *cmsx_menuRcConfirmBack(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);

    if (self && ((self->flags & OSD_MENU_ELEMENT_MASK) == OME_Back)) {
        return NULL;
    } else {
        return MENU_CHAIN_BACK;
    }
}

static int16_t rcDataInt[AUX4 + 1];

static const void *cmsx_menuRcOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

    for (int i = 0; i <= AUX4; i++) {
        rcDataInt[i] = lrintf(rcData[i]);
    }

    return NULL;
}

//
// RC preview
//
static const OSD_Entry cmsx_menuRcEntries[] =
{
    { "-- RC PREV --", OME_Label, NULL, NULL},

    { "ROLL",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[ROLL],     1, 2500, 0 } },
    { "PITCH", OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[PITCH],    1, 2500, 0 } },
    { "THR",   OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[THROTTLE], 1, 2500, 0 } },
    { "YAW",   OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[YAW],      1, 2500, 0 } },

    { "AUX1",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[AUX1],     1, 2500, 0 } },
    { "AUX2",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[AUX2],     1, 2500, 0 } },
    { "AUX3",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[AUX3],     1, 2500, 0 } },
    { "AUX4",  OME_INT16 | DYNAMIC, NULL, &(OSD_INT16_t){ &rcDataInt[AUX4],     1, 2500, 0 } },

    { "BACK",  OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuRcPreview = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XRCPREV",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuRcOnEnter,
    .onExit = cmsx_menuRcConfirmBack,
    .onDisplayUpdate = cmsx_menuRcOnDisplayUpdate,
    .entries = cmsx_menuRcEntries
};

static uint8_t motorConfig_motorIdle;
static uint8_t rxConfig_fpvCamAngleDegrees;
static uint8_t mixerConfig_crashflip_rate;

#ifdef USE_RC_SMOOTHING_FILTER
static uint8_t cmsx_rc_smoothing_enabled;
static uint8_t cmsx_rc_smoothing_filter_type;
static uint8_t cmsx_feedforward_filter_type_menu;
static uint8_t cmsx_rc_smoothing_setpoint_cutoff;
static uint8_t cmsx_rc_smoothing_throttle_cutoff;
static uint8_t cmsx_rc_smoothing_auto_factor_rpy;
static uint8_t cmsx_rc_smoothing_auto_factor_throttle;
static uint16_t cmsx_rc_smoothing_active_setpoint;
static uint16_t cmsx_rc_smoothing_active_throttle;
static uint16_t cmsx_rc_smoothing_detected_rate;
static uint8_t cmsx_ff_smooth_value;

static const char * const cmsx_rc_smoothing_filter_names[] = { "PT2", "PT3" };
#endif

static const void *cmsx_menuMiscOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    motorConfig_motorIdle = motorConfig()->motorIdle / 10;
    rxConfig_fpvCamAngleDegrees = rxConfig()->fpvCamAngleDegrees;
    mixerConfig_crashflip_rate = mixerConfig()->crashflip_rate;

    return NULL;
}

static const void *cmsx_menuMiscOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    motorConfigMutable()->motorIdle = 10 * motorConfig_motorIdle;
    rxConfigMutable()->fpvCamAngleDegrees = rxConfig_fpvCamAngleDegrees;
    mixerConfigMutable()->crashflip_rate = mixerConfig_crashflip_rate;

    return NULL;
}

#ifdef USE_RC_SMOOTHING_FILTER
static const void *cmsx_menuRcSmoothingOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_rc_smoothing_enabled = rxConfig()->rc_smoothing_mode;
    cmsx_rc_smoothing_filter_type = MIN(rxConfig()->rc_smoothing_filter_type, (uint8_t)RC_SMOOTHING_FILTER_PT3);
    cmsx_feedforward_filter_type_menu = MIN(rxConfig()->feedforward_smoothing_filter_type, (uint8_t)RC_SMOOTHING_FILTER_PT3);
    cmsx_rc_smoothing_setpoint_cutoff = rxConfig()->rc_smoothing_setpoint_cutoff;
    cmsx_rc_smoothing_throttle_cutoff = rxConfig()->rc_smoothing_throttle_cutoff;
    cmsx_rc_smoothing_auto_factor_rpy = rxConfig()->rc_smoothing_auto_factor_rpy;
    cmsx_rc_smoothing_auto_factor_throttle = rxConfig()->rc_smoothing_auto_factor_throttle;
    cmsx_ff_smooth_value = currentPidProfile->feedforward_smooth_factor;
    return NULL;
}

static const void *cmsx_menuRcSmoothingOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    rxConfigMutable()->rc_smoothing_mode = cmsx_rc_smoothing_enabled;
    rxConfigMutable()->rc_smoothing_filter_type = MIN(cmsx_rc_smoothing_filter_type, (uint8_t)RC_SMOOTHING_FILTER_PT3);
    rxConfigMutable()->feedforward_smoothing_filter_type = MIN(cmsx_feedforward_filter_type_menu, (uint8_t)RC_SMOOTHING_FILTER_PT3);
    rxConfigMutable()->rc_smoothing_setpoint_cutoff = cmsx_rc_smoothing_setpoint_cutoff;
    rxConfigMutable()->rc_smoothing_throttle_cutoff = cmsx_rc_smoothing_throttle_cutoff;
    rxConfigMutable()->rc_smoothing_auto_factor_rpy = cmsx_rc_smoothing_auto_factor_rpy;
    rxConfigMutable()->rc_smoothing_auto_factor_throttle = cmsx_rc_smoothing_auto_factor_throttle;

    initRcProcessing();

    return NULL;
}

static const void *cmsx_menuRcSmoothingOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

    const rcSmoothingFilter_t *data = getRcSmoothingData();
    if (data) {
        cmsx_rc_smoothing_active_setpoint = data->setpointCutoffFrequency;
        cmsx_rc_smoothing_active_throttle = data->throttleCutoffFrequency;
    } else {
        cmsx_rc_smoothing_active_setpoint = 0;
        cmsx_rc_smoothing_active_throttle = 0;
    }

    cmsx_rc_smoothing_detected_rate = getRxRateValid() ? lrintf(getCurrentRxRateHz()) : 0;
    cmsx_ff_smooth_value = currentPidProfile->feedforward_smooth_factor;

    return NULL;
}

static const OSD_Entry cmsx_menuRcSmoothingEntries[] = {
    { "-- RC SMOOTH --", OME_Label, NULL, NULL },
    { "RC SMOOTH",    OME_Bool,   NULL, &cmsx_rc_smoothing_enabled },
    { "RC FILTER",    OME_TAB,    NULL, &(OSD_TAB_t){ &cmsx_rc_smoothing_filter_type, ARRAYLEN(cmsx_rc_smoothing_filter_names) - 1, cmsx_rc_smoothing_filter_names } },
    { "FF FILTER",    OME_TAB,    NULL, &(OSD_TAB_t){ &cmsx_feedforward_filter_type_menu, ARRAYLEN(cmsx_rc_smoothing_filter_names) - 1, cmsx_rc_smoothing_filter_names } },
    { "SP CUT",       OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_rc_smoothing_setpoint_cutoff, 0, UINT8_MAX, 1 } },
    { "THR CUT",      OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_rc_smoothing_throttle_cutoff, 0, UINT8_MAX, 1 } },
    { "AUTO FACTOR",  OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_rc_smoothing_auto_factor_rpy, RC_SMOOTHING_AUTO_FACTOR_MIN, RC_SMOOTHING_AUTO_FACTOR_MAX, 1 } },
    { "THR FACTOR",   OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_rc_smoothing_auto_factor_throttle, RC_SMOOTHING_AUTO_FACTOR_MIN, RC_SMOOTHING_AUTO_FACTOR_MAX, 1 } },
    { "RC AUTO SP",   OME_UINT16 | DYNAMIC, NULL, &(OSD_UINT16_t){ &cmsx_rc_smoothing_active_setpoint, 0, 2000, 0 } },
    { "RC AUTO TH",   OME_UINT16 | DYNAMIC, NULL, &(OSD_UINT16_t){ &cmsx_rc_smoothing_active_throttle, 0, 2000, 0 } },
    { "RC RATE",      OME_UINT16 | DYNAMIC, NULL, &(OSD_UINT16_t){ &cmsx_rc_smoothing_detected_rate, 0, 4000, 0 } },
    { "FF SMOOTH",    OME_UINT8  | DYNAMIC, NULL, &(OSD_UINT8_t){ &cmsx_ff_smooth_value, 0, 95, 0 } },
    { "BACK",         OME_Back, NULL, NULL },
    { NULL,            OME_END, NULL, NULL }
};

static CMS_Menu cmsx_menuRcSmoothing = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XRC_SMTH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuRcSmoothingOnEnter,
    .onExit = cmsx_menuRcSmoothingOnExit,
    .onDisplayUpdate = cmsx_menuRcSmoothingOnDisplayUpdate,
    .entries = cmsx_menuRcSmoothingEntries
};
#endif

static const OSD_Entry menuMiscEntries[]=
{
    { "-- MISC --", OME_Label, NULL, NULL },

    { "IDLE OFFSET",   OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t) { &motorConfig_motorIdle,      0,  200, 1 } },
    { "FPV CAM ANGLE", OME_UINT8,                   NULL, &(OSD_UINT8_t) { &rxConfig_fpvCamAngleDegrees, 0,   90, 1 } },
    { "CRASHFLIP RATE", OME_UINT8 | REBOOT_REQUIRED,   NULL,          &(OSD_UINT8_t) { &mixerConfig_crashflip_rate,           0,  100, 1 } },
#ifdef USE_RC_SMOOTHING_FILTER
    { "RC SMOOTH",    OME_Submenu, cmsMenuChange, &cmsx_menuRcSmoothing },
#endif
    { "RC PREV",       OME_Submenu, cmsMenuChange, &cmsx_menuRcPreview},
#ifdef USE_GPS_LAP_TIMER
    { "GPS LAP TIMER",  OME_Submenu, cmsMenuChange, &cms_menuGpsLapTimer },
#endif // USE_GPS_LAP_TIMER

    { "BACK", OME_Back, NULL, NULL},
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuMisc = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XMISC",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuMiscOnEnter,
    .onExit = cmsx_menuMiscOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuMiscEntries
};

#endif // CMS
