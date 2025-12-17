#ifndef INITH
#define INITH

#include <Wire.h>
#include <VL53L0X.h>
#include <PinChangeInterrupt.h>
#include "MAPnSOLVE.h"
#include "Compass.h"

extern VL53L0X sensor1;
extern VL53L0X sensor2;
extern VL53L0X sensor3;
extern VL53L0X sensor4;
extern VL53L0X sensor5;
extern VL53L0X sensor6;

extern float tofa, tofb, tofc, tofd, tofe, toff;
extern long temp;
extern int tof;
/*
//speed of 150  
extern double kp = 10;
extern double ki = 0.5;
extern double kd = 0.9;
*/
//PID Wall
extern float KP_NEGITIVE;    //15
extern float KI_NEGITIVE;   //0.5
extern float KD_NEGITIVE;  //0.5
extern float KP_POSITIVE;    //15
extern float KI_POSITIVE;     //0.5
extern float KD_POSITIVE;


extern double dt;
extern double last_time;
extern float integral_left, previous_left, output_left;
extern float integral_right, previous_right, output_right;
extern int INT_MEMORY;
extern int MAX_LIMIT;
extern int MIN_LIMIT;

//PID motor sync
extern float kp_fwd;     //1
extern float ki_fwd;   //0.05
extern float kd_fwd;  //0.1

extern double integral_fwd, previous_fwd;
extern int MAX_LIMIT_FWD;
extern int MIN_LIMIT_FWD;

//PID turn
extern float kp_turn;
extern float ki_turn;
extern float kd_turn;
extern float ff_turn;//.65

extern float integral_turn, previous_turn;
extern int MAX_LIMIT_TURN;
extern int MIN_LIMIT_TURN;

//Encoder
extern volatile long enc1;
extern volatile long enc2;
extern volatile long lastEnc1;
extern volatile long lastEnc2;

//Robot Parameters
extern int MOTOR_SPEED;
extern float SIDE_WALL_DIST;  //for PID cm 2cm
extern float FRONT_DIST;                //cm pre 10
extern float SIDE_WALL_CHECK_DIST;     //14cm
extern float NEXT_SIDE_WALL_CHECK_DIST;
extern float COUNT_PER_DEGREE;           //408 encoder counts per rotation of wheel || wheel dia = 33mm
extern float COUNT_PER_DEGREE_BOT_TURN;  //actual 2.868 encoder counts per robot turn
extern int FRONT_TERMINATE_DIST;            // cm
extern float IDEAL_SIDE_DIST;             // Ideal distance of side TOF when bot at center.
extern float IDEAL_SIDE_TOLERANCE;        // Tolerance of above.
extern int TOF_A_D_DIST;                    //cm

//Arina Parameters
extern int end[1][2];
extern int start[2];
extern bool thisCellFront, thisCellLeft, thisCellRight, thisCellBack;

//Global Variables for function internals
extern int motor_speed;
extern volatile long initState1;
extern volatile long initState2;
extern int intm_left;
extern int intm_right;
extern int intm_fwd;
extern int intm_turn;
extern long wheel1_initial_pos;
extern long wheel2_initial_pos;
extern bool turnFlag;
extern bool frontFlag;
extern char prevFwdOrient;
extern volatile bool haltFlag;
extern int scanCount;



#endif