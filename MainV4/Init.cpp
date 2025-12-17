
#include <Wire.h>
#include <VL53L0X.h>
#include <PinChangeInterrupt.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;
VL53L0X sensor6;

float tofa, tofb, tofc, tofd, tofe, toff;
long temp = 0;
int tof;
/*
//speed of 150  
double kp = 10;
double ki = 0.5;
double kd = 0.9;
*/
//PID Wall
float KP_NEGITIVE = 10;    //15
float KI_NEGITIVE = 0.1;   //0.5
float KD_NEGITIVE = 0.25;  //0.5
float KP_POSITIVE = 25;    //15
float KI_POSITIVE = 1;     //0.5
float KD_POSITIVE = 2;


double dt;
double last_time = 0;
float integral_left, previous_left, output_left = 0;
float integral_right, previous_right, output_right = 0;
int INT_MEMORY = 10;
int MAX_LIMIT = 355;
int MIN_LIMIT = -355;

//PID motor sync
float kp_fwd = 1;     //1
float ki_fwd = 0.5;   //0.05
float kd_fwd = 0.01;  //0.1

double integral_fwd, previous_fwd;
int MAX_LIMIT_FWD = 50;
int MIN_LIMIT_FWD = -50;

//PID turn
float kp_turn = 2.2;
float ki_turn = 0.5;
float kd_turn = 0.075;
float ff_turn = 0.05;//.65

float integral_turn, previous_turn;
int MAX_LIMIT_TURN = 120;
int MIN_LIMIT_TURN = 90;

//Encoder
volatile long enc1 = 0;
volatile long enc2 = 0;
volatile long lastEnc1 = 0;
volatile long lastEnc2 = 0;

//Robot Parameters
int MOTOR_SPEED = 200;
float SIDE_WALL_DIST = 6.5 * 1.414;  //for PID cm 2cm
float FRONT_DIST = 10;                //cm pre 10
float SIDE_WALL_CHECK_DIST = 10;     //14cm
float NEXT_SIDE_WALL_CHECK_DIST = 14;
float COUNT_PER_DEGREE = 1.13;           //408 encoder counts per rotation of wheel || wheel dia = 33mm
float COUNT_PER_DEGREE_BOT_TURN = 2.87;  //actual 2.868 encoder counts per robot turn
int FRONT_TERMINATE_DIST = 5;            // cm
float IDEAL_SIDE_DIST = 4.7;             // Ideal distance of side TOF when bot at center.
float IDEAL_SIDE_TOLERANCE = 0.3;        // Tolerance of above.
int TOF_A_D_DIST = 7;                    //cm

//Arina Parameters
bool thisCellFront = false, thisCellLeft = false, thisCellRight = false, thisCellBack = true;


//Global Variables for function internals
int motor_speed = MOTOR_SPEED;
volatile long initState1;
volatile long initState2;
int intm_left = INT_MEMORY;
int intm_right = INT_MEMORY;
int intm_fwd = INT_MEMORY;
int intm_turn = INT_MEMORY;
long wheel1_initial_pos;
long wheel2_initial_pos;
bool turnFlag = true;
bool frontFlag = true;
char prevFwdOrient = 'X';
volatile bool haltFlag = false;
int scanCount = 0;



