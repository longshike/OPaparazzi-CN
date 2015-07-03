/* This file has been generated by gen_settings from /home/dino/paparazzi/var/aircrafts/MY_apogee/settings_modules.xml /home/dino/paparazzi/conf/settings/fixedwing_basic.xml /home/dino/paparazzi/var/aircrafts/MY_apogee/settings_telemetry.xml */
/* Please DO NOT EDIT */

#ifndef SETTINGS_H
#define SETTINGS_H

#define RCSettings(mode_changed) { \
}

#include "autopilot.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/periodic_telemetry.h"
#include "subsystems/gps.h"
#include "generated/modules.h"

#define SETTINGS_NAMES { \
 { "telemetry_mode_Ap" }, \
 { "telemetry_mode_Fbw" }, \
 { "flight_altitude" }, \
 { "nav_course" }, \
 { "nav_shift" }, \
 { "autopilot_flight_time" }, \
 { "nav_radius" }, \
 { "pprz_mode" }, \
 { "launch" }, \
 { "kill_throttle" }, \
 { "gps.reset" }, \
 { "gps_ubx_gps_ubx_ucenter_periodic_status" }, \
};
#define SETTINGS_NAMES_SHORT { \
 "tel_mod_Ap" , \
 "tel_mod_Fbw" , \
 "fli_alt" , \
 "nav_cou" , \
 "nav_shi" , \
 "aut_fli_tim" , \
 "nav_rad" , \
 "ppr_mod" , \
 "lau" , \
 "kil_thr" , \
 "gps_res" , \
 "gps_ubx_gps_ubx_" , \
};
#define NB_SETTING 12
#define DlSetting(_idx, _value) { \
  switch (_idx) { \
    case 0: telemetry_mode_Ap = _value; break;\
    case 1: telemetry_mode_Fbw = _value; break;\
    case 2: flight_altitude = _value; break;\
    case 3: nav_course = _value; break;\
    case 4: nav_IncreaseShift( _value ); _value = nav_shift; break;\
    case 5: autopilot_ResetFlightTimeAndLaunch( _value ); _value = autopilot_flight_time; break;\
    case 6: nav_SetNavRadius( _value ); _value = nav_radius; break;\
    case 7: pprz_mode = _value; break;\
    case 8: launch = _value; break;\
    case 9: kill_throttle = _value; break;\
    case 10: gps_Reset( _value ); _value = gps.reset; break;\
    case 11: gps_ubx_gps_ubx_ucenter_periodic_status = _value; break;\
    default: break;\
  }\
}
#define PeriodicSendDlValue(_trans, _dev) { \
  static uint8_t i;\
  float var;\
  if (i >= 12) i = 0;\
  switch (i) { \
    case 0: var = telemetry_mode_Ap; break;\
    case 1: var = telemetry_mode_Fbw; break;\
    case 2: var = flight_altitude; break;\
    case 3: var = nav_course; break;\
    case 4: var = nav_shift; break;\
    case 5: var = autopilot_flight_time; break;\
    case 6: var = nav_radius; break;\
    case 7: var = pprz_mode; break;\
    case 8: var = launch; break;\
    case 9: var = kill_throttle; break;\
    case 10: var = gps.reset; break;\
    case 11: var = gps_ubx_gps_ubx_ucenter_periodic_status; break;\
    default: var = 0.; break;\
  }\
  pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\
  i++;\
}
static inline float settings_get_value(uint8_t i) {
  switch (i) { \
    case 0: return telemetry_mode_Ap;
    case 1: return telemetry_mode_Fbw;
    case 2: return flight_altitude;
    case 3: return nav_course;
    case 4: return nav_shift;
    case 5: return autopilot_flight_time;
    case 6: return nav_radius;
    case 7: return pprz_mode;
    case 8: return launch;
    case 9: return kill_throttle;
    case 10: return gps.reset;
    case 11: return gps_ubx_gps_ubx_ucenter_periodic_status;
    default: return 0.;
    }
  }

/* Persistent Settings */
struct PersistentSettings {
};

extern struct PersistentSettings pers_settings;

static inline void persistent_settings_store( void ) {
}

static inline void persistent_settings_load( void ) {
}

#endif // SETTINGS_H
