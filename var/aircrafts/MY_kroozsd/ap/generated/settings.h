/* This file has been generated by gen_settings from /home/lsk/paparazzi/var/aircrafts/kroozsd_lsk/settings_modules.xml /home/lsk/paparazzi/conf/modules/air_data.xml /home/lsk/paparazzi/conf/modules/gps_ubx_ucenter.xml /home/lsk/paparazzi/conf/settings/control/rotorcraft_guidance.xml /home/lsk/paparazzi/conf/settings/control/stabilization_rate.xml /home/lsk/paparazzi/conf/settings/control/stabilization_att_int.xml /home/lsk/paparazzi/conf/settings/estimation/ahrs_int_cmpl_quat.xml /home/lsk/paparazzi/var/aircrafts/kroozsd_lsk/settings_telemetry.xml */
/* Please DO NOT EDIT */

#ifndef SETTINGS_H
#define SETTINGS_H

#define RCSettings(mode_changed) { \
}

#include "air_data/air_data.h"
#include "generated/periodic_telemetry.h"
#include "gps/gps_ubx_ucenter.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "navigation.h"
#include "stabilization/stabilization_attitude_common_int.h"
#include "stabilization/stabilization_rate.h"
#include "subsystems/ahrs/ahrs_int_cmpl_quat.h"
#include "generated/modules.h"

#define SETTINGS_NAMES { \
 { "telemetry_mode_Main" }, \
 { "ahrs_icq.gravity_heuristic_factor" }, \
 { "ahrs_icq.accel_omega" }, \
 { "ahrs_icq.accel_zeta" }, \
 { "ahrs_icq.mag_omega" }, \
 { "ahrs_icq.mag_zeta" }, \
 { "stabilization_gains.p.x" }, \
 { "stabilization_gains.d.x" }, \
 { "stabilization_gains.i.x" }, \
 { "stabilization_gains.dd.x" }, \
 { "stabilization_gains.p.y" }, \
 { "stabilization_gains.d.y" }, \
 { "stabilization_gains.i.y" }, \
 { "stabilization_gains.dd.y" }, \
 { "stabilization_gains.p.z" }, \
 { "stabilization_gains.d.z" }, \
 { "stabilization_gains.i.z" }, \
 { "stabilization_gains.dd.z" }, \
 { "stabilization_rate_gain.p" }, \
 { "stabilization_rate_gain.q" }, \
 { "stabilization_rate_gain.r" }, \
 { "stabilization_rate_igain.p" }, \
 { "stabilization_rate_igain.q" }, \
 { "stabilization_rate_igain.r" }, \
 { "stabilization_rate_ddgain.p" }, \
 { "stabilization_rate_ddgain.q" }, \
 { "stabilization_rate_ddgain.r" }, \
 { "guidance_v_kp" }, \
 { "guidance_v_kd" }, \
 { "guidance_v_ki" }, \
 { "guidance_v_nominal_throttle" }, \
 { "guidance_v_adapt_throttle_enabled" }, \
 { "guidance_v_z_sp" }, \
 { "guidance_h_use_ref" }, \
 { "gh_ref.max_speed" }, \
 { "guidance_h_approx_force_by_thrust" }, \
 { "gh_ref.tau" }, \
 { "gh_ref.omega" }, \
 { "gh_ref.zeta" }, \
 { "guidance_h_pgain" }, \
 { "guidance_h_dgain" }, \
 { "guidance_h_igain" }, \
 { "guidance_h_vgain" }, \
 { "guidance_h_again" }, \
 { "guidance_h_pos_sp.x" }, \
 { "guidance_h_pos_sp.y" }, \
 { "flight_altitude" }, \
 { "nav_heading" }, \
 { "nav_radius" }, \
 { "gps_ubx_ucenter.sw_ver_h" }, \
 { "gps_ubx_ucenter.sw_ver_l" }, \
 { "gps_ubx_ucenter.hw_ver_h" }, \
 { "gps_ubx_ucenter.hw_ver_l" }, \
 { "gps_ubx_ucenter.baud_init" }, \
 { "gps_ubx_ucenter.baud_run" }, \
 { "air_data.qnh" }, \
 { "air_data.tas_factor" }, \
 { "air_data.calc_qnh_once" }, \
 { "air_data.calc_airspeed" }, \
 { "air_data.calc_tas_factor" }, \
 { "air_data.calc_amsl_baro" }, \
 { "gps_ubx_gps_ubx_ucenter_periodic_status" }, \
};
#define SETTINGS_NAMES_SHORT { \
 "tel_mod_Mai" , \
 "ahr_icq_gra_heu_" , \
 "ahr_icq_acc_ome" , \
 "ahr_icq_acc_zet" , \
 "ahr_icq_mag_ome" , \
 "ahr_icq_mag_zet" , \
 "sta_gai_p_x" , \
 "sta_gai_d_x" , \
 "sta_gai_i_x" , \
 "sta_gai_dd_x" , \
 "sta_gai_p_y" , \
 "sta_gai_d_y" , \
 "sta_gai_i_y" , \
 "sta_gai_dd_y" , \
 "sta_gai_p_z" , \
 "sta_gai_d_z" , \
 "sta_gai_i_z" , \
 "sta_gai_dd_z" , \
 "sta_rat_gai_p" , \
 "sta_rat_gai_q" , \
 "sta_rat_gai_r" , \
 "sta_rat_iga_p" , \
 "sta_rat_iga_q" , \
 "sta_rat_iga_r" , \
 "sta_rat_ddg_p" , \
 "sta_rat_ddg_q" , \
 "sta_rat_ddg_r" , \
 "gui_v_kp" , \
 "gui_v_kd" , \
 "gui_v_ki" , \
 "gui_v_nom_thr" , \
 "gui_v_ada_thr_en" , \
 "gui_v_z_sp" , \
 "gui_h_use_ref" , \
 "gh_ref_max_spe" , \
 "gui_h_app_for_by" , \
 "gh_ref_tau" , \
 "gh_ref_ome" , \
 "gh_ref_zet" , \
 "gui_h_pga" , \
 "gui_h_dga" , \
 "gui_h_iga" , \
 "gui_h_vga" , \
 "gui_h_aga" , \
 "gui_h_pos_sp_x" , \
 "gui_h_pos_sp_y" , \
 "fli_alt" , \
 "nav_hea" , \
 "nav_rad" , \
 "gps_ubx_uce_sw_v" , \
 "gps_ubx_uce_sw_v" , \
 "gps_ubx_uce_hw_v" , \
 "gps_ubx_uce_hw_v" , \
 "gps_ubx_uce_bau_" , \
 "gps_ubx_uce_bau_" , \
 "air_dat_qnh" , \
 "air_dat_tas_fac" , \
 "air_dat_cal_qnh_" , \
 "air_dat_cal_air" , \
 "air_dat_cal_tas_" , \
 "air_dat_cal_ams_" , \
 "gps_ubx_gps_ubx_" , \
};
#define NB_SETTING 62
#define DlSetting(_idx, _value) { \
  switch (_idx) { \
    case 0: telemetry_mode_Main = _value; break;\
    case 1: ahrs_icq.gravity_heuristic_factor = _value; break;\
    case 2: ahrs_int_cmpl_quat_SetAccelOmega( _value ); _value = ahrs_icq.accel_omega; break;\
    case 3: ahrs_int_cmpl_quat_SetAccelZeta( _value ); _value = ahrs_icq.accel_zeta; break;\
    case 4: ahrs_int_cmpl_quat_SetMagOmega( _value ); _value = ahrs_icq.mag_omega; break;\
    case 5: ahrs_int_cmpl_quat_SetMagZeta( _value ); _value = ahrs_icq.mag_zeta; break;\
    case 6: stabilization_gains.p.x = _value; break;\
    case 7: stabilization_gains.d.x = _value; break;\
    case 8: stabilization_gains.i.x = _value; break;\
    case 9: stabilization_gains.dd.x = _value; break;\
    case 10: stabilization_gains.p.y = _value; break;\
    case 11: stabilization_gains.d.y = _value; break;\
    case 12: stabilization_gains.i.y = _value; break;\
    case 13: stabilization_gains.dd.y = _value; break;\
    case 14: stabilization_gains.p.z = _value; break;\
    case 15: stabilization_gains.d.z = _value; break;\
    case 16: stabilization_gains.i.z = _value; break;\
    case 17: stabilization_gains.dd.z = _value; break;\
    case 18: stabilization_rate_gain.p = _value; break;\
    case 19: stabilization_rate_gain.q = _value; break;\
    case 20: stabilization_rate_gain.r = _value; break;\
    case 21: stabilization_rate_igain.p = _value; break;\
    case 22: stabilization_rate_igain.q = _value; break;\
    case 23: stabilization_rate_igain.r = _value; break;\
    case 24: stabilization_rate_ddgain.p = _value; break;\
    case 25: stabilization_rate_ddgain.q = _value; break;\
    case 26: stabilization_rate_ddgain.r = _value; break;\
    case 27: guidance_v_kp = _value; break;\
    case 28: guidance_v_kd = _value; break;\
    case 29: guidance_v_SetKi( _value ); _value = guidance_v_ki; break;\
    case 30: guidance_v_nominal_throttle = _value; break;\
    case 31: guidance_v_adapt_throttle_enabled = _value; break;\
    case 32: guidance_v_z_sp = _value; break;\
    case 33: guidance_h_SetUseRef( _value ); _value = guidance_h_use_ref; break;\
    case 34: guidance_h_SetMaxSpeed( _value ); _value = gh_ref.max_speed; break;\
    case 35: guidance_h_approx_force_by_thrust = _value; break;\
    case 36: guidance_h_SetTau( _value ); _value = gh_ref.tau; break;\
    case 37: guidance_h_SetOmega( _value ); _value = gh_ref.omega; break;\
    case 38: guidance_h_SetZeta( _value ); _value = gh_ref.zeta; break;\
    case 39: guidance_h_pgain = _value; break;\
    case 40: guidance_h_dgain = _value; break;\
    case 41: guidance_h_SetKi( _value ); _value = guidance_h_igain; break;\
    case 42: guidance_h_vgain = _value; break;\
    case 43: guidance_h_again = _value; break;\
    case 44: guidance_h_pos_sp.x = _value; break;\
    case 45: guidance_h_pos_sp.y = _value; break;\
    case 46: navigation_SetFlightAltitude( _value ); _value = flight_altitude; break;\
    case 47: nav_heading = _value; break;\
    case 48: nav_radius = _value; break;\
    case 49: gps_ubx_ucenter.sw_ver_h = _value; break;\
    case 50: gps_ubx_ucenter.sw_ver_l = _value; break;\
    case 51: gps_ubx_ucenter.hw_ver_h = _value; break;\
    case 52: gps_ubx_ucenter.hw_ver_l = _value; break;\
    case 53: gps_ubx_ucenter.baud_init = _value; break;\
    case 54: gps_ubx_ucenter.baud_run = _value; break;\
    case 55: air_data.qnh = _value; break;\
    case 56: air_data.tas_factor = _value; break;\
    case 57: air_data.calc_qnh_once = _value; break;\
    case 58: air_data.calc_airspeed = _value; break;\
    case 59: air_data.calc_tas_factor = _value; break;\
    case 60: air_data.calc_amsl_baro = _value; break;\
    case 61: gps_ubx_gps_ubx_ucenter_periodic_status = _value; break;\
    default: break;\
  }\
}
#define PeriodicSendDlValue(_trans, _dev) { \
  static uint8_t i;\
  float var;\
  if (i >= 62) i = 0;\
  switch (i) { \
    case 0: var = telemetry_mode_Main; break;\
    case 1: var = ahrs_icq.gravity_heuristic_factor; break;\
    case 2: var = ahrs_icq.accel_omega; break;\
    case 3: var = ahrs_icq.accel_zeta; break;\
    case 4: var = ahrs_icq.mag_omega; break;\
    case 5: var = ahrs_icq.mag_zeta; break;\
    case 6: var = stabilization_gains.p.x; break;\
    case 7: var = stabilization_gains.d.x; break;\
    case 8: var = stabilization_gains.i.x; break;\
    case 9: var = stabilization_gains.dd.x; break;\
    case 10: var = stabilization_gains.p.y; break;\
    case 11: var = stabilization_gains.d.y; break;\
    case 12: var = stabilization_gains.i.y; break;\
    case 13: var = stabilization_gains.dd.y; break;\
    case 14: var = stabilization_gains.p.z; break;\
    case 15: var = stabilization_gains.d.z; break;\
    case 16: var = stabilization_gains.i.z; break;\
    case 17: var = stabilization_gains.dd.z; break;\
    case 18: var = stabilization_rate_gain.p; break;\
    case 19: var = stabilization_rate_gain.q; break;\
    case 20: var = stabilization_rate_gain.r; break;\
    case 21: var = stabilization_rate_igain.p; break;\
    case 22: var = stabilization_rate_igain.q; break;\
    case 23: var = stabilization_rate_igain.r; break;\
    case 24: var = stabilization_rate_ddgain.p; break;\
    case 25: var = stabilization_rate_ddgain.q; break;\
    case 26: var = stabilization_rate_ddgain.r; break;\
    case 27: var = guidance_v_kp; break;\
    case 28: var = guidance_v_kd; break;\
    case 29: var = guidance_v_ki; break;\
    case 30: var = guidance_v_nominal_throttle; break;\
    case 31: var = guidance_v_adapt_throttle_enabled; break;\
    case 32: var = guidance_v_z_sp; break;\
    case 33: var = guidance_h_use_ref; break;\
    case 34: var = gh_ref.max_speed; break;\
    case 35: var = guidance_h_approx_force_by_thrust; break;\
    case 36: var = gh_ref.tau; break;\
    case 37: var = gh_ref.omega; break;\
    case 38: var = gh_ref.zeta; break;\
    case 39: var = guidance_h_pgain; break;\
    case 40: var = guidance_h_dgain; break;\
    case 41: var = guidance_h_igain; break;\
    case 42: var = guidance_h_vgain; break;\
    case 43: var = guidance_h_again; break;\
    case 44: var = guidance_h_pos_sp.x; break;\
    case 45: var = guidance_h_pos_sp.y; break;\
    case 46: var = flight_altitude; break;\
    case 47: var = nav_heading; break;\
    case 48: var = nav_radius; break;\
    case 49: var = gps_ubx_ucenter.sw_ver_h; break;\
    case 50: var = gps_ubx_ucenter.sw_ver_l; break;\
    case 51: var = gps_ubx_ucenter.hw_ver_h; break;\
    case 52: var = gps_ubx_ucenter.hw_ver_l; break;\
    case 53: var = gps_ubx_ucenter.baud_init; break;\
    case 54: var = gps_ubx_ucenter.baud_run; break;\
    case 55: var = air_data.qnh; break;\
    case 56: var = air_data.tas_factor; break;\
    case 57: var = air_data.calc_qnh_once; break;\
    case 58: var = air_data.calc_airspeed; break;\
    case 59: var = air_data.calc_tas_factor; break;\
    case 60: var = air_data.calc_amsl_baro; break;\
    case 61: var = gps_ubx_gps_ubx_ucenter_periodic_status; break;\
    default: var = 0.; break;\
  }\
  pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\
  i++;\
}
static inline float settings_get_value(uint8_t i) {
  switch (i) { \
    case 0: return telemetry_mode_Main;
    case 1: return ahrs_icq.gravity_heuristic_factor;
    case 2: return ahrs_icq.accel_omega;
    case 3: return ahrs_icq.accel_zeta;
    case 4: return ahrs_icq.mag_omega;
    case 5: return ahrs_icq.mag_zeta;
    case 6: return stabilization_gains.p.x;
    case 7: return stabilization_gains.d.x;
    case 8: return stabilization_gains.i.x;
    case 9: return stabilization_gains.dd.x;
    case 10: return stabilization_gains.p.y;
    case 11: return stabilization_gains.d.y;
    case 12: return stabilization_gains.i.y;
    case 13: return stabilization_gains.dd.y;
    case 14: return stabilization_gains.p.z;
    case 15: return stabilization_gains.d.z;
    case 16: return stabilization_gains.i.z;
    case 17: return stabilization_gains.dd.z;
    case 18: return stabilization_rate_gain.p;
    case 19: return stabilization_rate_gain.q;
    case 20: return stabilization_rate_gain.r;
    case 21: return stabilization_rate_igain.p;
    case 22: return stabilization_rate_igain.q;
    case 23: return stabilization_rate_igain.r;
    case 24: return stabilization_rate_ddgain.p;
    case 25: return stabilization_rate_ddgain.q;
    case 26: return stabilization_rate_ddgain.r;
    case 27: return guidance_v_kp;
    case 28: return guidance_v_kd;
    case 29: return guidance_v_ki;
    case 30: return guidance_v_nominal_throttle;
    case 31: return guidance_v_adapt_throttle_enabled;
    case 32: return guidance_v_z_sp;
    case 33: return guidance_h_use_ref;
    case 34: return gh_ref.max_speed;
    case 35: return guidance_h_approx_force_by_thrust;
    case 36: return gh_ref.tau;
    case 37: return gh_ref.omega;
    case 38: return gh_ref.zeta;
    case 39: return guidance_h_pgain;
    case 40: return guidance_h_dgain;
    case 41: return guidance_h_igain;
    case 42: return guidance_h_vgain;
    case 43: return guidance_h_again;
    case 44: return guidance_h_pos_sp.x;
    case 45: return guidance_h_pos_sp.y;
    case 46: return flight_altitude;
    case 47: return nav_heading;
    case 48: return nav_radius;
    case 49: return gps_ubx_ucenter.sw_ver_h;
    case 50: return gps_ubx_ucenter.sw_ver_l;
    case 51: return gps_ubx_ucenter.hw_ver_h;
    case 52: return gps_ubx_ucenter.hw_ver_l;
    case 53: return gps_ubx_ucenter.baud_init;
    case 54: return gps_ubx_ucenter.baud_run;
    case 55: return air_data.qnh;
    case 56: return air_data.tas_factor;
    case 57: return air_data.calc_qnh_once;
    case 58: return air_data.calc_airspeed;
    case 59: return air_data.calc_tas_factor;
    case 60: return air_data.calc_amsl_baro;
    case 61: return gps_ubx_gps_ubx_ucenter_periodic_status;
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
