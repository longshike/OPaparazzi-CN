/* This file has been generated by gen_flight_plan from /home/dino/paparazzi/conf/flight_plans/MY_fly_plan.xml */
/* Please DO NOT EDIT */

#ifndef FLIGHT_PLAN_H
#define FLIGHT_PLAN_H

#include "std.h"
#include "generated/modules.h"
#include "subsystems/datalink/datalink.h"
#define FLIGHT_PLAN_NAME "Basic"
#define NAV_UTM_EAST0 440259
#define NAV_UTM_NORTH0 4439214
#define NAV_UTM_ZONE0 50
#define NAV_LAT0 401010990 /* 1e7deg */
#define NAV_LON0 1162990961 /* 1e7deg */
#define NAV_ALT0 50000 /* mm above msl */
#define NAV_MSL0 -10050 /* mm, EGM96 geoid-height (msl) over ellipsoid */
#define QFU 0.0
#define WP_dummy 0
#define WP_HOME 1
#define WP_STDBY 2
#define WP_1 3
#define WP_2 4
#define WP_MOB 5
#define WP_S1 6
#define WP_S2 7
#define WP_AF 8
#define WP_TD 9
#define WP__BASELEG 10
#define WP_CLIMB 11
#define WAYPOINTS { \
 {42.0, 42.0, 100},\
 {-0.2, -0.5, 100},\
 {-36.1, 28.8, 100},\
 {-44.1, 68.6, 100},\
 {-42.7, -19.2, 100},\
 {-54.1, 10.0, 100},\
 {-103.3, 99.2, 100},\
 {-8.2, -52.7, 100},\
 {-4.2, 120.8, 35},\
 {-4.2, 100.8, 5},\
 {-65.6, 134.2, 100},\
 {-37.8, 4.2, 100},\
};
#define WAYPOINTS_ENU { \
 {41.68, 42.35, 50.00}, /* ENU in meters  */ \
 {-0.20, -0.50, 50.00}, /* ENU in meters  */ \
 {-36.34, 28.53, 50.00}, /* ENU in meters  */ \
 {-44.66, 68.28, 50.00}, /* ENU in meters  */ \
 {-42.56, -19.54, 50.00}, /* ENU in meters  */ \
 {-54.20, 9.58, 50.00}, /* ENU in meters  */ \
 {-104.12, 98.42, 50.00}, /* ENU in meters  */ \
 {-7.79, -52.78, 50.00}, /* ENU in meters  */ \
 {-5.15, 120.81, -15.00}, /* ENU in meters  */ \
 {-5.00, 100.80, -45.00}, /* ENU in meters  */ \
 {-66.68, 133.73, 50.00}, /* ENU in meters  */ \
 {-37.85, 3.90, 50.00}, /* ENU in meters  */ \
};
#define WAYPOINTS_LLA { \
 {.lat=401014803, .lon=1162995849, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401010944, .lon=1162990937, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401013558, .lon=1162986699, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401017138, .lon=1162985723, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401009229, .lon=1162985969, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401011852, .lon=1162984604, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401019853, .lon=1162978750, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401006236, .lon=1162990047, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401021869, .lon=1162990356, .alt=35000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401020068, .lon=1162990375, .alt=5000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401023033, .lon=1162983140, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
 {.lat=401011341, .lon=1162986522, .alt=100000}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=-10.05m) */ \
};
#define WAYPOINTS_LLA_WGS84 { \
 {.lat=401014803, .lon=1162995849, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401010944, .lon=1162990937, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401013558, .lon=1162986699, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401017138, .lon=1162985723, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401009229, .lon=1162985969, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401011852, .lon=1162984604, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401019853, .lon=1162978750, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401006236, .lon=1162990047, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401021869, .lon=1162990356, .alt=24950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401020068, .lon=1162990375, .alt=-5050}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401023033, .lon=1162983140, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
 {.lat=401011341, .lon=1162986522, .alt=89950}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \
};
#define WAYPOINTS_GLOBAL { \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
 FALSE, \
};
#define NB_WAYPOINT 12
#define FP_BLOCKS { \
 { "Wait GPS" }, \
 { "Geo init" }, \
 { "Holding point" }, \
 { "Takeoff" }, \
 { "Standby" }, \
 { "Figure 8 around wp 1" }, \
 { "Oval 1-2" }, \
 { "MOB" }, \
 { "Survey S1-S2" }, \
 { "Path 1,2,S1,S2,STDBY" }, \
 { "Land Right AF-TD" }, \
 { "Land Left AF-TD" }, \
 { "land" }, \
 { "final" }, \
 { "flare" }, \
 { "HOME" }, \
};
#define NB_BLOCK 16
#define GROUND_ALT 50.
#define GROUND_ALT_CM 5000
#define SECURITY_HEIGHT 25.
#define SECURITY_ALT 75.
#define HOME_MODE_HEIGHT 25.
#define MAX_DIST_FROM_HOME 300.

#ifdef NAV_C


static inline void auto_nav(void) {
  switch (nav_block) {
    Block(0) // Wait GPS
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        kill_throttle = 1;
        NextStage();
      Label(while_1)
      Stage(1)
        if (! (!(GpsFixValid()))) Goto(endwhile_2) else NextStageAndBreak();
        Stage(2)
          Goto(while_1)
        Label(endwhile_2)
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(1) // Geo init
    ; // pre_call
    switch(nav_stage) {
      Label(while_3)
      Stage(0)
        if (! (LessThan(NavBlockTime(),10))) Goto(endwhile_4) else NextStageAndBreak();
        Stage(1)
          Goto(while_3)
        Label(endwhile_4)
      Stage(2)
        if (! (NavSetGroundReferenceHere())) {
          NextStage();
        } else {
          break;
        }
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(2) // Holding point
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        kill_throttle = 1;
        NextStage();
      Stage(1)
        {
          NavAttitude(RadOfDeg(0));
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalThrottleMode(9600*(0));
        }
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(3) // Takeoff
    ; // pre_call
    if ((nav_block != 4) && (GetPosAlt()>(GetAltRef()+25))) { GotoBlock(4); return; }
    switch(nav_stage) {
      Stage(0)
        kill_throttle = 0;
        NextStage();
      Stage(1)
        autopilot_flight_time = 0;
        NextStage();
      Stage(2)
        if (NavApproachingFrom(11,1,CARROT)) NextStageAndBreakFrom(11) else {
          NavGotoWaypoint(11);
          NavVerticalAutoThrottleMode(RadOfDeg(15));
          NavVerticalThrottleMode(9600*(1.000000));
        }
        break;
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(4) // Standby
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(2), 0.);
        NavCircleWaypoint(2, nav_radius);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(5) // Figure 8 around wp 1
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_eight_init();
        NextStageAndBreak();
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        Eight(3, 4, nav_radius);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(6) // Oval 1-2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_oval_init();
        NextStageAndBreak();
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        Oval(3, 4, nav_radius);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(7) // MOB
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (! (NavSetWaypointHere(WP_MOB))) {
          NextStage();
        } else {
          break;
        }
      Stage(1)
        nav_radius = DEFAULT_CIRCLE_RADIUS;
        NextStage();
      Stage(2)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(5), 0.);
        NavCircleWaypoint(5, nav_radius);
        break;
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(8) // Survey S1-S2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavSurveyRectangleInit(6, 7, 150, NS);
        NextStageAndBreak();
      Stage(1)
        NavSurveyRectangle(6, 7);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(9) // Path 1,2,S1,S2,STDBY
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(4,3,CARROT)) NextStageAndBreakFrom(4) else {
          NavSegment(3, 4);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        }
        break;
      Stage(1)
        if (NavApproachingFrom(6,4,CARROT)) NextStageAndBreakFrom(6) else {
          NavSegment(4, 6);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(6), 0.);
        }
        break;
      Stage(2)
        if (NavApproachingFrom(7,6,1)) NextStageAndBreakFrom(7) else {
          NavSegment(6, 7);
          NavVerticalAutoPitchMode(9600*(0.400000));
          NavVerticalAltitudeMode(WaypointAlt(7), 0.);
        }
        break;
      Stage(3)
        if (NavApproachingFrom(2,7,1)) NextStageAndBreakFrom(2) else {
          NavSegment(7, 2);
          NavVerticalAutoPitchMode(9600*(0.400000));
          NavVerticalAltitudeMode(WaypointAlt(2), 0.);
        }
        break;
      Stage(4)
        GotoBlock(4);
        break;
      default:
      Stage(5)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(10) // Land Right AF-TD
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_radius = DEFAULT_CIRCLE_RADIUS;
        NextStage();
      Stage(1)
        GotoBlock(12);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(11) // Land Left AF-TD
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_radius = -(DEFAULT_CIRCLE_RADIUS);
        NextStage();
      Stage(1)
        GotoBlock(12);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(12) // land
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (! (nav_compute_baseleg(WP_AF, WP_TD, WP__BASELEG, nav_radius))) {
          NextStage();
        } else {
          break;
        }
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(10), 0.);
        NavCircleWaypoint(10, nav_radius);
        if ((NavCircleCount()>0.500000)) NextStageAndBreak();
        break;
      Stage(2)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(10), 0.);
        NavCircleWaypoint(10, nav_radius);
        if (And(NavQdrCloseTo((DegOfRad(baseleg_out_qdr)-((nav_radius/fabs(nav_radius))*10))),(10>fabs((GetPosAlt()-WaypointAlt(WP__BASELEG)))))) NextStageAndBreak();
        break;
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(13) // final
    ; // pre_call
    if ((nav_block != 14) && ((GetAltRef()+10)>GetPosAlt())) { GotoBlock(14); return; }
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(9,8,CARROT)) NextStageAndBreakFrom(9) else {
          NavSegment(8, 9);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavGlide(8, 9);
        }
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(14) // flare
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(9,8,0)) NextStageAndBreakFrom(9) else {
          NavSegment(8, 9);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalThrottleMode(9600*(0.000000));
        }
        break;
      Stage(1)
        if (FALSE) NextStageAndBreak() else {
          NavAttitude(RadOfDeg(0.000000));
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalThrottleMode(9600*(0.000000));
        }
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(15) // HOME
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_home();
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    default: break;
  }
}
#endif // NAV_C

#endif // FLIGHT_PLAN_H
