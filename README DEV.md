Main Dev 2.0

BUS 01: 

SEND: 

BO_ 1570 LIGHT_STALK: 8 SCM
 SG_ AUTO_HIGH_BEAM : 37|1@0+ (1,0) [0|1] "" XXX
 SG_ FRONT_FOG : 27|1@0+ (1,0) [0|1] "" XXX
 SG_ PARKING_LIGHT : 28|1@0+ (1,0) [0|1] "" XXX
 SG_ LOW_BEAM : 29|1@0+ (1,0) [0|1] "" XXX
 SG_ HIGH_BEAM : 30|1@0+ (1,0) [0|1] "" XXX
 SG_ DAYTIME_RUNNING_LIGHT : 31|1@0+ (1,0) [0|1] "" XXX

BO_ 1556 BLINKERS_STATE: 8 XXX
 SG_ BLINKER_BUTTON_PRESSED : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ HAZARD_LIGHT : 27|1@0+ (1,0) [0|1] "" XXX
 SG_ TURN_SIGNALS : 29|2@0+ (1,0) [0|3] "" XXX

BO_ 1568 BODY_CONTROL_STATE: 8 XXX
 SG_ METER_DIMMED : 38|1@0+ (1,0) [0|1] "" XXX
 SG_ PARKING_BRAKE : 60|1@0+ (1,0) [0|1] "" XXX
 SG_ SEATBELT_DRIVER_UNLATCHED : 62|1@0+ (1,0) [0|1] "" XXX
 SG_ DOOR_OPEN_FL : 45|1@0+ (1,0) [0|1] "" XXX
 SG_ DOOR_OPEN_RL : 42|1@0+ (1,0) [0|1] "" XXX
 SG_ DOOR_OPEN_RR : 43|1@0+ (1,0) [0|1] "" XXX
 SG_ DOOR_OPEN_FR : 44|1@0+ (1,0) [0|1] "" XXX

BO_ 1552 BODY_CONTROL_STATE_2: 8 XXX
 SG_ UI_SPEED : 23|8@0+ (1,0) [0|255] "" XXX
 SG_ METER_SLIDER_BRIGHTNESS_PCT : 30|7@0+ (1,0) [12|100] "%" XXX
 SG_ METER_SLIDER_LOW_BRIGHTNESS : 37|1@0+ (1,0) [0|1] "" XXX
 SG_ METER_SLIDER_DIMMED : 38|1@0+ (1,0) [0|1] "" XXX
 SG_ UNITS : 63|3@0+ (1,0) [1|4] "" XXX

BO_ 951 ESP_CONTROL: 8 ESP
 SG_ TC_DISABLED : 13|1@0+ (1,0) [0|1] "" XXX
 SG_ VSC_DISABLED : 12|2@0+ (1,0) [0|1] "" XXX
 SG_ BRAKE_LIGHTS_ACC : 18|1@0+ (1,0) [0|1] "" XXX
 SG_ BRAKE_HOLD_ENABLED : 33|1@1+ (1,0) [0|1] "" XXX
 SG_ BRAKE_HOLD_ACTIVE : 36|1@0+ (1,0) [0|1] "" XXX

BO_ 548 BRAKE_MODULE: 8 XXX
 SG_ BRAKE_PRESSURE : 43|12@0+ (1,0) [0|4047] "" XXX
 SG_ BRAKE_PRESSED : 5|1@0+ (1,0) [0|1] "" XXX

BO_ 170 WHEEL_SPEEDS: 8 XXX
 SG_ WHEEL_SPEED_FR : 7|16@0+ (0.01,-67.67) [0|250] "km/h" XXX
 SG_ WHEEL_SPEED_FL : 23|16@0+ (0.01,-67.67) [0|250] "km/h" XXX
 SG_ WHEEL_SPEED_RR : 39|16@0+ (0.01,-67.67) [0|250] "km/h" XXX
 SG_ WHEEL_SPEED_RL : 55|16@0+ (0.01,-67.67) [0|250] "km/h" XXX

BO_ 466 PCM_CRUISE: 8 XXX
 SG_ GAS_RELEASED : 4|1@0+ (1,0) [0|1] "" XXX
 SG_ CRUISE_ACTIVE : 5|1@0+ (1,0) [0|1] "" XXX
 SG_ ACC_BRAKING : 12|1@0+ (1,0) [0|1] "" XXX
 SG_ ACCEL_NET : 23|16@0- (0.0009765625,0) [-20|20] "m/s^2" XXX
 SG_ NEUTRAL_FORCE : 39|16@0- (2,0) [-65536|65534] "N" XXX
 SG_ CRUISE_STATE : 55|4@0+ (1,0) [0|15] "" XXX
 SG_ CANCEL_REQ : 49|1@1+ (1,0) [0|1] "" XXX
 SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX

BO_ 921 PCM_CRUISE_SM: 8 XXX    Main ECU simulated. UI Set Speed sendet festen wert. Vielleicht Set speed übernehmen.
 SG_ MAIN_ON : 4|1@0+ (1,0) [0|1] "" XXX
 SG_ CRUISE_CONTROL_STATE : 11|4@0+ (1,0) [0|15] "" XXX
 SG_ DISTANCE_LINES : 14|2@0+ (1,0) [0|3] "" XXX
 SG_ TEMP_ACC_FAULTED : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ UI_SET_SPEED : 31|8@0+ (1,0) [0|255] "" XXX

BO_ 800 VSC1S07: 8 CGW
 SG_ FBKRLY : 6|1@0+ (1,0) [0|0] "" DS1
 SG_ FVSCM : 4|1@0+ (1,0) [0|0] "" DS1
 SG_ FVSCSFT : 3|1@0+ (1,0) [0|0] "" DS1
 SG_ FABS : 2|1@0+ (1,0) [0|0] "" DS1,FCM
 SG_ TSVSC : 1|1@0+ (1,0) [0|0] "" DS1
 SG_ FVSCL : 0|1@0+ (1,0) [0|0] "" DS1
 SG_ RQCSTBKB : 15|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PSBSTBY : 14|1@0+ (1,0) [0|0] "" DS1
 SG_ P2BRXMK : 13|1@0+ (1,0) [0|0] "" DS1
 SG_ MCC : 11|1@0+ (1,0) [0|0] "" DS1
 SG_ RQBKB : 10|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ BRSTOP : 9|1@0+ (1,0) [0|0] "" DS1,FCM
 SG_ BRKON : 8|1@0+ (1,0) [0|0] "" DS1,FCM
 SG_ ASLP : 23|8@0- (1,0) [0|0] "deg" DS1
 SG_ BRTYPACC : 31|2@0+ (1,0) [0|0] "" DS1
 SG_ BRKABT3 : 26|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ BRKABT2 : 25|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ BRKABT1 : 24|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ GVC : 39|8@0- (0.04,0) [0|0] "m/s^2" DS1
 SG_ XGVCINV : 43|1@0+ (1,0) [0|0] "" DS1
 SG_ S07CNT : 52|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PCSBRSTA : 50|2@0+ (1,0) [0|0] "" DS1
 SG_ VSC07SUM : 63|8@0+ (1,0) [0|0] "" DS1,FCM

BO_ 452 ENGINE_RPM: 8 CGW
 SG_ RPM : 7|16@0- (0.78125,0) [0|0] "rpm" SCS
 SG_ ENGINE_RUNNING : 27|1@0+ (1,0) [0|1] "" XXX

BO_ 956 GEAR_PACKET: 8 XXX
 SG_ SPORT_ON : 2|1@0+ (1,0) [0|1] "" XXX
 SG_ GEAR : 13|6@0+ (1,0) [0|63] "" XXX
 SG_ SPORT_GEAR_ON : 33|1@0+ (1,0) [0|1] "" XXX
 SG_ SPORT_GEAR : 38|3@0+ (1,0) [0|7] "" XXX
 SG_ ECON_ON : 40|1@0+ (1,0) [0|1] "" XXX
 SG_ B_GEAR_ENGAGED : 41|1@0+ (1,0) [0|1] "" XXX
 SG_ DRIVE_ENGAGED : 47|1@0+ (1,0) [0|1] "" XXX

BO_ 467 PCM_CRUISE_2: 8 XXX
 SG_ BRAKE_PRESSED : 3|1@0+ (1,0) [0|1] "" XXX
 SG_ PCM_FOLLOW_DISTANCE : 12|2@0+ (1,0) [0|3] "" XXX
 SG_ LOW_SPEED_LOCKOUT : 14|2@0+ (1,0) [0|3] "" XXX
 SG_ MAIN_ON : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ SET_SPEED : 23|8@0+ (1,0) [0|255] "km/h" XXX
 SG_ ACC_FAULTED : 47|1@0+ (1,0) [0|1] "" XXX
 SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX

BO_ 836 PRE_COLLISION_2: 8 DSU
 SG_ DSS1GDRV : 7|10@0- (0.1,0) [0|0] "m/s^2" Vector__XXX
 SG_ PCSALM : 17|1@0+ (1,0) [0|0] "" FCM
 SG_ IBTRGR : 27|1@0+ (1,0) [0|0] "" FCM
 SG_ PBATRGR : 30|2@0+ (1,0) [0|0] "" Vector__XXX
 SG_ PREFILL : 33|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ AVSTRGR : 36|1@0+ (1,0) [0|0] "" SCS
 SG_ CHECKSUM : 63|8@0+ (1,0) [0|0] "" XXX


________________________________________________________________________________________________________________________________________________________________________________________________
READ:

BO_ 610 EPS_STATUS: 5 EPS
 SG_ IPAS_STATE : 3|4@0+ (1,0) [0|15] "" XXX
 SG_ LKA_STATE : 31|7@0+ (1,0) [0|127] "" XXX
 SG_ TYPE : 24|1@0+ (1,0) [0|1] "" XXX
 SG_ CHECKSUM : 39|8@0+ (1,0) [0|255] "" XXX

BO_ 37 STEER_ANGLE_SENSOR: 8 XXX
 SG_ STEER_ANGLE : 3|12@0- (1.5,0) [-500|500] "deg" XXX
 SG_ STEER_FRACTION : 39|4@0- (0.1,0) [-0.7|0.7] "deg" XXX
 SG_ STEER_RATE : 35|12@0- (1,0) [-2000|2000] "deg/s" XXX

BO_ 608 STEER_TORQUE_SENSOR: 8 XXX
 SG_ STEER_TORQUE_EPS : 47|16@0- (1,0) [-32768|32767] "" XXX
 SG_ STEER_TORQUE_DRIVER : 15|16@0- (1,0) [-32768|32767] "" XXX
 SG_ STEER_ANGLE : 31|16@0- (0.0573,0) [-500|500] "" XXX
 SG_ STEER_ANGLE_INITIALIZING : 3|1@0+ (1,0) [0|1] "" XXX
 SG_ STEER_OVERRIDE : 0|1@0+ (1,0) [0|1] "" XXX
 SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX
