/*
   This software is intended to control joint movement in order to test the joint mechanics.
   It will run on four Arduino Megas, one for each of the SL, UR, EL and LR joints.

   The software will know on which joint it is running by checking the state of certain pins
   upon startup. Each joint’s Mega will have different pins tied to ground for identification. On the Minion shields, this is accomplished with DIP switches.

   It is similar to the BR_Demo_Joint_Controller software for the Arduino Due used for
   the BR joint. Because of the limited RAM and processing speed of the Mega, a simpler
   scheme will be used for accel and decel. Each of the four Megas and the single Due
   will publish and subscribe x ROS topics via USB to the laptop running roscore.

   IMPORTANT - DO NOT REUSE char miscMsgs[20] for nh.loginfo. Create a new micsMsgs for each use.
   Otherwise reusing somehow changes the value of my global variables which is VERY hard to debug!

   (C) 2017 William Dalton
*/

/*
   Minion Identification
      minion identification pins allow each minion instance to identify which joint it is controlling
      minion identification pins are pulled up to HIGH in software, then selected combinations set to GND
      with DIP switch on new minion shield to provide LOW signal. SOftware reads combination of minion
      identification pins to idnetify which minion.
           IDENT_PIN_8 IDENT_PIN_4 IDENT_PIN_2 IDENT_PIN_1 minion_ident
      pin->   42          40          38          36
      BR      HIGH        HIGH        HIGH        LOW           14
      SL      HIGH        HIGH        LOW         HIGH          13
      UR      HIGH        HIGH        LOW         LOW           12
      EL      HIGH        LOW         HIGH        HIGH          11
      LR      HIGH        LOW         HIGH        LOW           10
      WL      HIGH        LOW         LOW         HIGH           9
      WR      HIGH        LOW         LOW         LOW            8
      EE1     LOW         HIGH        HIGH        HIGH           7
      EE2     LOW         HIGH        HIGH        LOW            6

   Note since DIP switch ON position connects to Ground, the above table translates into the following
          IDENT_PIN_8 IDENT_PIN_4 IDENT_PIN_2 IDENT_PIN_1 minion_ident
      pin->   42          40          38          36
      BR      OFF         OFF         OFF         ON            14
      SL      OFF         OFF         ON          OFF           13
      UR      OFF         OFF         ON          ON            12
      EL      OFF         ON          OFF         OFF           11
      LR      OFF         ON          OFF         ON            10
      WL      OFF         ON          ON          OFF            9
      WR      OFF         ON          ON          ON             8
      EE1     ON          OFF         OFF         OFF            7
      EE2     ON          OFF         OFF         ON             6

      Minion Ident Great To Do Idea! 13Nov2017:
      Redo minion_ident DIP switch assignments so that minion_ident is
      identical to joint index for ROS stuff like JointTrajectoryPoint,
      BR_IDENT = 0, SL_IDENT = 1, and so on ...
*/

/*
   modifications 7/20/18 to run cycloidal drive test rig as simulated WL joint
   1. increase number of joints to add WL
   2.
*/

/*
   TO DO
   done - @ident & switch case for joint identification
   done - @commanded_joint_positions method
   done - @test 5x Arduino on USB Hub with rqt - Tests good!
   done - @implement commanded_joint_positions first term is time in seconds in which to complete motion
   done - @calc my LUT from my params during setup()? Yes!
        would making changin accel params MUCH easier!
   done - @change timer in setup() from Due timer to Mega
   done - @implent setting Timer1 for next pulse
   done - implement endstop functionality in stepOnce()
   @fix problem of runaway for commanded motion > 10 degrees
      somehow steps_remaining become negative on second command
        problems is in planMovement() current_pos is some large negative number
           suspect encoder_1_pos is getting corriupted from ISR even though it is declared volatile
      &&Idea: this might work better if encoder counting was in a class with getter and setter
   @get rid of 1 pulse per second when stationary
      never getting to running_state = HOLDING_POSITION;
   @replace joint_encoder_pos with stepper_count_pos as controlling variable?
      to get around problems with reading encoder_1_pos
   @implement makeHoldPositionAdjustment() from BR_demo_joint_controller
   @implement faster alternative to digitalRead/digitalWrite ref: http://masteringarduino.blogspot.com/2013/10/fastest-and-smallest-digitalread-and.html
      attempted 12/6/17 but it did not work
      esp. for end stop
      also esp. for encoder ISRs
   @add readSensors() method detail
   @implement something to set positions to zero
   done - implement stuff to actually move steppers
   @experiment with JointTrajectoryPoint
      .cpp to read a line typed in terminal, then publish
      this program subscribes, then acts on JTP
   @get correct ratios for UR_JOINT_ENCODER_GEAR_RATIO etc
      done for SL only
   @after true joint encoders actually installed, change joint_encoder_pos.data to use that
   @implement minion_ident renumbering to match joint_ident
*/

#include <ros.h>
#include <ArduinoHardware.h>
#include <TimerOne.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//Defines to replace digitalWrite and digitalRead from: http://masteringarduino.blogspot.com/2013/10/fastest-and-smallest-digitalread-and.html
#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

//states
enum {UNPOWERED, HOLDING_POSITION, MOVING, FINAL_APPROACH, CW_ENDSTOP_ACTIVATED, CCW_ENDSTOP_ACTIVATED, CALIBRATING } running_state;
enum {GREEN, YELLOW, RED} torque_state;

//states string messages
char states_msg[100];
char empty_states_msg[100] = "States: ";
char unpowered_msg[] = "unpowered ";
char holding_position_msg[] = "holding_position ";
char moving_msg[] = "moving ";
char final_approach_msg[] = "final_approach ";
char cw_endstop_activated_msg[] = "cw_endstop_activated ";
char ccw_endstop_activated_msg[] = "ccw_endstop_activated ";
char calibrating_msg[] = "calibrating ";
char green_torque_msg[] = "green";
char yellow_torque_msg[] = "yellow";
char red_torque_msg[] = "red";

//flags
volatile bool new_plan = false;

//debug messages
char miscMsgs[20];
char miscMsgs2[20];
char miscMsgs3[20];
char miscMsgs4[20];
char miscMsgs5[20];
char miscMsgs6[20];
char miscMsgs7[20];
char miscMsgs8[20];
char miscMsgs9[20];
char miscMsgs10[20];
char miscMsgs11[20];
char miscMsgs12[20];
char miscMsgs13[20];
char miscMsgs14[20];
char miscMsgs15[20];
char miscMsgs16[20];
char miscMsgs17[20];
char miscMsgs18[20];
char miscMsgs19[20];
char miscMsgs20[20];
char miscMsgs21[20];
char miscMsgs22[20];
char miscMsgs23[20];
char miscMsgs24[20];
char miscMsgs25[20];
char miscMsgs26[20];
char miscMsgs27[20];
char miscMsgs28[20];
char miscMsgs29[20];
char miscMsgs30[20];
char inbound_message[40];

//internal position and direction variables
volatile long encoder_1_pos = 0;  //position of doe in encoder clicks
volatile long encoder_2_pos = 0;  //position of stepper doe in encoder clicks
volatile long stepper_counts = 0; //stepper position in stepper steps

//stepper system constants
const int STEPPER_STEPS_PER_REV = 200;  //stepper motor specific
const float STEPPER_DEGREES_PER_STEP = (float) (1 / STEPPER_STEPS_PER_REV);//s.b. 1.8 degrees
const int MICROSTEPS = 8;               //set on stepper driver
float gear_ratio = 65.0;  //NOT USED?

//encoder constants - ONLY USED TO CREATE OTHER CONSTANTS
//const int ENCODER_1_PPR = 2400;
//const float ENCODER_1_GEAR_RATIO = 18.055;            //ratio final joint motion to doe encoder motion = 65/3.6
//const float ENCODER_1_DEGREES_PER_COUNT = 0.0083;       //=360 / ENCODER_1_PPR / ENCODER_1_GEAR_RATIO;//360/2400/5.0=0.03degrees
//const int ENCODER_2_PPR = 2400;                       //in encoder counts, used for stepper_doe
//const float ENCODER_2_GEAR_RATIO = 15.0;              //ratio final joint motion to stepper doe encoder motion
//const float ENCODER_2_DEGREES_PER_COUNT = 0.01;       //=360 / ENCODER_2_PPR / ENCODER_2_GEAR_RATIO;//360/2400/15.0=0.01degrees
//const float ENCODER_1_COUNTS_PER_STEPPER_COUNT = 36.0;//= GEAR_REDUCTION_RATIO_STEPPER_TO_SERIES_ELASTIC * ENCODER_1_PPR / (STEPPER_STEPS_PER_REV * MICROSTEPS);
/*
   special case encoder constants for AndyMark encoder mounted on S-E assembly.
   gear ratio for AndyMark encoder to SL joint = 19.741 = (59T/23T)*(59T/23T)*(6"/2")
   Normally and eventuall, an encoder (analog or digital) will be mounted directly on joint itself.
   The following are ratios between the joint itself and the shaft on which the joint encoder is located
   SL_JOINT_ENCODER_GEAR_RATIO = 19.741 = (59T/23T)*(59T/23T)*(6"/2")
   SL_JOINT_ENCODER_GEAR_RATIO = 17.754 = (59T/25T)*(59T/23T)*(6.1"/2.08")
   UR_JOINT_ENCODER_GEAR_RATIO = 13.825 = (59T/25T)*(59T/23T)*(4.75"/2.08")
   EL_JOINT_ENCODER_GEAR_RATIO = 11.787 = (71T/13T)*(59T/23T)*()*(1.75"/2.08")
   LR_JOINT_ENCODER_GEAR_RATIO =  = ( / )*( /)*(1.5"/2")*( /)
   SL_JOINT_ENCODER_PPR = 1440;                   //for AndyMark encoder
   UR_JOINT_ENCODER_PPR = 1440;                   //for AndyMark encoder
   EL_JOINT_ENCODER_PPR = 1440;                   //for AndyMark encoder
   LR_JOINT_ENCODER_PPR = 1440;                   //for AndyMark encoder
*/
//const float SL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.01284; //0.01284=360/1440/19.741=360/SL_JOINT_ENCODER_PPR/SL_JOINT_ENCODER_GEAR_RATIO
const float SL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.01408; //0.01284=360/1440/17.754=360/SL_JOINT_ENCODER_PPR/SL_JOINT_ENCODER_GEAR_RATIO
const float UR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.01808; //0.01284=360/1440/13.825
const float EL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.005698; //0.01284=360/1440/43.8749
const float LR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.01284; //0.01284=360/1440/19.741
const float WL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.15; //direct drive optical encoder 360degrees/2400quadppr=0.15

//encoder variables
/*
   eventually =zero when encoder directly on joint.
   For now, this varies for each joint depending on gear ratio between joint and
   encoder on S-E assembly. This is set in setup() for each joint
*/
volatile float joint_degrees_per_encoder_count = 1.0;

//motion constants
const float MAX_VEL = 15.0;   //degrees/sec //was 25.0 before WL test rig
const float MAX_ACCEL = 5.0; //degrees/sec^2 //was 10.0 before WL test rig
const float MIN_TIME_TO_REACH_MAX_VEL = MAX_VEL / MAX_ACCEL;//in seconds
const float MIN_TRAVEL_TO_REACH_MAX_VEL = MIN_TIME_TO_REACH_MAX_VEL * ((MAX_VEL + MAX_ACCEL) / 2);
//const float DEGREES_PER_MICROSTEP = 0.006923;// = 0.03; //= 360 / (STEPPER_STEPS_PER_REV * MICROSTEPS) / GEAR_RATIO;
const float POSITION_HOLD_TOLERANCE = 0.2;    //in degrees
const long POSITION_ADJUSTMENT_SPEED = 5000;  //stepper period interval speed at which to make position adjustments
const float RETREAT_DISTANCE = 1.0;           //degrees to retreat
const long QUEUE_SIZE = 100;//100*8bytes=800 bytes
static const float SL_DEGREES_PER_MICROSTEP = 0.00352;  //63.915:1 gear ratio April 2019 arm configuration
static const float UR_DEGREES_PER_MICROSTEP = 0.00452;  //49.770:1 gear ratio April 2019 arm configuration
static const float EL_DEGREES_PER_MICROSTEP = 0.001246; //180.51:1 gear ratio April 2019 arm configuration
static const float LR_DEGREES_PER_MICROSTEP = 0.01350;  //16.675:1 gear ratio April 2019 arm configuration
static const float WL_DEGREES_PER_MICROSTEP = 0.01875;  //12.000:1 gear ratio April 2019 arm configuration
//const float WL_DEGREES_PER_MICROSTEP = 0.075;  //cycloidal test rig 1st try 12:1 reduction, microsteps=2, =360/(200*2)/12=0.075
//const float WL_DEGREES_PER_MICROSTEP = 0.15;  //cycloidal test rig 1st try 12:1 reduction, microsteps=1, =360/(200*1)/12=0.075
/*
  this is a symetrical n x n array where n is the number of joints
  The joints are in order BR, SL, UR, EL, LR; wherein in BR is the 0th element, SL the 1st and so on.
  These compensation factors corrects for change in cable path length for driving cables due
  to change in upstream joints positions as cables wrap and unwrap around pulley on axis
  of upstream joints.
  For example below the 0.438 factor in position [1][2] says that UR joint will change position
  by 0.438 times the change in the SL position
*/

const float JOINT_POSITION_CORRECTION_FACTORS[6][6] = {
  //modified 7/20/18 to add WL with zero correction factors becasue not cable driven in WL_cycloidal test rig
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, -0.438, 0.319, 0.0, 0.0},
  {0.0, -0.438, 0.0, 0.09, 0.0, 0.0},
  {0.0, 0.319, 0.09, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
};

//torque limits - To Do constants for now, but eventually variables that are function of MoveIt motion plan output
const float YELLOW_LIMIT = 10.0;//torque yellow limit in degrees of joint travel - To Do make function of MoveIt motion plan output
const float RED_LIMIT = 25.0;

//motion variables
volatile float required_duration;    //required duration in seconds for joint movement
volatile float commanded_stepper_position = 0.0; //joint commanded_stepper_position in degrees
volatile int cruise_plateau_index = 0;  //index of fastest plateau to be used in a particular movement
volatile int current_plateau_index = 0;
volatile bool direction_CW = true;   //movement in positive joint direction is defined as CW
volatile float current_pos = 0.0;    //joint position in degrees
volatile float pos_error = 0.0;    //difference in degrees between joint commanded_stepper_position and joint current position
volatile long dtg_stepper_counts = 0;//in absolute value of stepper counts
volatile long accel_until_stepper_counts = 0;//in absolute value of stepper counts
volatile long decel_after_stepper_counts = 0;//in absolute value of stepper counts
volatile long cruising_stepper_counts = 0;
volatile long steps_remaining = 0; //stepper steps remaining to move. uses in final_approach and holding_position
volatile long current_plateau_steps_remaining = 0;
volatile long close_enough = 100L;  //reset in setup() to value for particular joint
volatile float degrees_per_microstep = 0.007; //reset to particulr joint in setup()

//my queue
volatile long my_queue[QUEUE_SIZE];
volatile unsigned int head = 0; //index to front of my_queue
volatile unsigned int tail = 0; //index to rear of my_queue

//linear acceleration Lookup Table (LUT)
const int NUM_PLATEAUS = 50;//  number of steps (plateaus) in acceleration LUT
const float PLATEAU_DURATION = MAX_VEL / MAX_ACCEL / NUM_PLATEAUS;
volatile long accel_LUT[NUM_PLATEAUS][3];

//position variables
long max_stepper_counts = 0;      //position of stepper at CW endstop
long min_stepper_counts = 0;      //position of stepper at CCW endstop
long max_stepper_doe_counts = 0;  //position of stepper doe at CW endstop
long min_stepper_doe_counts = 0;  //position of stepper doe at CCW endstop
long max_doe_counts = 0;         //position of joint doe at CW endstop
long min_doe_counts = 0;          //position of joint doe at CCW endstop

//variables to be published
//std_msgs::Float32 joint_aoe_pos; //joint position in +/- degrees, to three decimal places. Stored internally as a long, then converted when published to std_msgs/Float32
std_msgs::Float32 joint_encoder_pos; //joint position in +/- degrees, to three decimal places. Stored internally as a long, then converted when published to std_msgs/Float32
std_msgs::Float32 stepper_count_pos; //Equivalent joint position in +/- degrees, to three decimal places. Store internally as long as stepper counts
std_msgs::Float32 stepper_encoder_pos; //Equivalent joint position in +/- degrees, to three decimal places.
std_msgs::Float32 torque;  ////torque joint rotation in +/- degrees, to three decimal places. std_msgs/Float32.
std_msgs::String state; //may be in multiple states simultaneously
std_msgs::String state2; //may be in multiple states simultaneously
trajectory_msgs::JointTrajectoryPoint jtp;

//endstop pins
const int CW_ENDSTOP_PIN = 2;  //pin # corrected 12/6/17
const int CCW_ENDSTOP_PIN = 3; //pin # corrected 12/6/17

//encoder pins
const int ENCODER_1_PIN_A = 18;
const int ENCODER_1_PIN_B = 19;
const int ENCODER_2_PIN_A = 20;
const int ENCODER_2_PIN_B = 21;

//stepper pins
const int STEP_PIN = A0;
const int DIR_PIN = A1;
const int ENABLE_PIN = A2;//note this enables both stepper 1 and stepper 2 on a joint
const int STEP2_PIN = A3; //for second stepper motor on same joint e.g. SL
const int DIR2_PIN = A4;  //Note DIR2 pin for SL must always be set opposite DIR_PIN as steppers motors installed opposite each other

//minion identification pins
const int IDENT_PIN_1 = 36;  //correct for new minion shield  V0.1 APr2016
const int IDENT_PIN_2 = 38;
const int IDENT_PIN_4 = 40;
const int IDENT_PIN_8 = 42;
int minion_ident = 0;//stores minion identification number
const int BR_IDENT = 14;
const int SL_IDENT = 13;
const int UR_IDENT = 12;
const int EL_IDENT = 11;
const int LR_IDENT = 10;
const int WL_IDENT = 9;
const int WR_IDENT = 8;
const int EE1_IDENT = 7;
const int EE2_IDENT = 6;

//ROS node handler
//ros::NodeHandle  nh;
//added this next line to increase size of sub and pub buffers. Defaults were 512bytes. Ref: https://github.com/tonybaltovski/ros_arduino/issues/10
//ros::NodeHandle_<ArduinoHardware, 2, 10, 4096, 4096> nh;//this uses too much dynamic memory for Mega
//ros::NodeHandle_<ArduinoHardware, 2, 8, 280, 280> nh;
ros::NodeHandle_<ArduinoHardware, 2, 8, 512, 1024> nh;

////loop timing variables
volatile unsigned long next_update = 0L;//used to time loop updates
const unsigned long UPDATE_INTERVAL = 10L;//in milliseconds, 500 is 2Hz, 50 is 20Hz, 10 is 100Hz
volatile unsigned long loop_counter = 0L;
volatile int int_loop_counter = 0;

//debug stuff
float test_var = 56.66;

//subscriber call backs
//void commandedPositionCallback(const std_msgs::Float32& the_command_msg_) {
//  //DEPRECATED - action to be taken when msg received on topic BR_cmd. SL_cmd, etc
//  static float previous_commanded_stepper_position_;
//  float the_commanded_position_ = the_command_msg_.data;
//
//  /*
//    check for special commands. Normal range of commands is +/- 179.999 degrees,
//    therefore anything outside of that range is considered a special command
//    commanded_stepper_position ~=400 -> means set all values to zero
//  */
//  if (the_commanded_position_ > -359.99 && the_commanded_position_ < 359.99) {
//    //this is a regular command
//    //record commanded position as a global
//    commanded_stepper_position = the_command_msg_.data;
//
//    //check this is not a repeated command to the same position
//    if (commanded_stepper_position != previous_commanded_position_) {
//      //this is a new command. Plan and execute motion
//      //set previous commanded position
//      previous_commanded_position_ = commanded_stepper_position;
//      //set flag
//      new_plan = true;
//    }
//  }
//}//end commandedPositionCallback()

void commandedJointPositionsCallback(const std_msgs::String& the_command_msg_) {
  //
  /*
    action to be taken when msg received on topic commanded_joint_positions.
    First token in command message is a message number (int) to determine if command is duplicate that already been processed
    Second token is required duration of movement in (float) seconds
    Subsequent tokens represent commanded position in (float) degrees for each joint
      in order BR, SL, UR, EL, LR, WL
  */
  int current_command_number_ = 0;
  static int previous_command_number_ = 0;//unique integer identify this command
  float commanded_joint_positions_[7];//array holding req'd duration and commanded position in degrees in order: BR,SL,UR,EL,LR, WL

  //parse command
  inbound_message[0] = (char)0;  //"empties inbound_message by setting first char in string to 0
  strcat(inbound_message, the_command_msg_.data);//this finally works

  //debug only
  nh.loginfo("InbdMsg=");
  nh.loginfo(inbound_message);//this now works
  nh.spinOnce();

  //  //debug load state.data for eventual publishing by updateStatus()
  //  miscMsgs2[0] = (char)0;  //"empties msg by setting first char in string to 0
  //  strcat(miscMsgs2, inbound_message);
  //  state.data = miscMsgs2;

  //process first token in inbound_message which is unique command id number
  //http://jhaskellsblog.blogspot.com/2011/06/parsing-quick-guide-to-strtok-there-are.html
  //this whole test section works and returns torque of 5.230000
  //char test_message[] = "6.23,4.56,7.89";
  char* command;
  //  command = strtok(test_message, ",");//works
  command = strtok(inbound_message, ",");//works
  //commanded_joint_positions_[0] = atof(command);
  commanded_joint_positions_[0] = atoi(command);//retrieves unique command message number

  //check this is not a repeated command
  current_command_number_ = commanded_joint_positions_[0];
  if (current_command_number_ != previous_command_number_) {
    //this is a new command. Plan and execute motion
    //set previous commanded position
    previous_command_number_ = current_command_number_;
    //set flag
    new_plan = true;//new_plan flag is polled in loop() to initiate a new plan and motion
  }
  else {
    //this is a duplicate plan, exit
    return;
  }

  //  //debug loginfo
  //  char result2[8]; // Buffer big enough for 7-character float
  //  dtostrf(commanded_joint_positions_[0], 6, 2, result2); // Leave room for too large numbers!
  //  nh.loginfo("commanded_joint_positions_[0]=");//this works
  //  nh.loginfo(result2);//this works

  //  //debug pub to state2 topic
  //  miscMsgs3[0] = (char)0;  //"empties msg by setting first char in string to 0
  //  strcat(miscMsgs3, command);
  //  state2.data = miscMsgs3;

  //process remaining tokens in inbound_message
  int joint_index = 1;//note starts at 1 because required duration has already been parsed as joint_index=0
  while (command != 0)
  {
    command = strtok(0, ",");//note "0" i.e. null as input for subsequent calls to strtok()
    commanded_joint_positions_[joint_index] = atof(command);
    joint_index++;
  }

  //apply to only this joint
  switch (minion_ident) {
    case BR_IDENT:
      joint_index = 2;
      break;
    case SL_IDENT:
      joint_index = 3;
      break;
    case UR_IDENT:
      joint_index = 4;
      break;
    case EL_IDENT:
      joint_index = 5;
      break;
    case LR_IDENT:
      joint_index = 6;
      break;
    case WL_IDENT:
      joint_index = 7;
    break;    default:
      break;
  }//end switch case

  /* determine commanded_stepper_position by applying JOINT_POSITION_CORRECTION_FACTORS to
     commanded_joint_positions_.
     start with command_joint_position for this joint, then add compensations from
     upsstream joints
  */
  commanded_stepper_position = commanded_joint_positions_[joint_index];
  for (int n = 2; n < joint_index; n++) {
    commanded_stepper_position += commanded_joint_positions_[n] * JOINT_POSITION_CORRECTION_FACTORS[joint_index - 2][n - 2];
  }

  //assign required duration and commanded position
  required_duration = commanded_joint_positions_[1];
  //commanded_stepper_position = commanded_joint_positions_[joint_index];//deprecated by use of JOINT_POSITION_CORRECTION_FACTORS

  //debug loginfo commanded position
  char result5[8]; // Buffer big enough for 7-character float
  dtostrf(required_duration, 6, 2, result5); // Leave room for too large numbers!
  nh.loginfo("required_duration=");//this works
  nh.loginfo(result5);//this works

  //debug loginfo commanded position
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(commanded_stepper_position, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo("commanded_stepper_position=");//this works
  nh.loginfo(result);//this works

  //check for Command 400 to set all values to zero
  if (commanded_stepper_position > 399.5 && commanded_stepper_position < 400.5) {
    //Command 400 - set all values to zero
    encoder_1_pos = 0;
    encoder_2_pos = 0;
    stepper_counts = 0;
    current_pos = 0.0;
    dtg_stepper_counts = 0;
    steps_remaining = 0;
    //    previous_commanded_position_ = 0.0;
    commanded_stepper_position = 0.0;
    new_plan = false;

    //log
    nh.loginfo("reset all vars to zero");
  }// end of command 400

}// end commandedJointPositionsCallback()

//subscribers
ros::Subscriber<std_msgs::String> commanded_joint_positions_sub("commanded_joint_positions", commandedJointPositionsCallback);

//publishers - need to create all possible publishers, but will only advertise ones called for by minion_ident
//ros::Publisher joint_aoe_pos_pub("BR_aoe_pos", &joint_aoe_pos);     //BR joint position in +/- degrees, to three decimal places. std_msgs/Float32. This is result of lookup function.
ros::Publisher SL_joint_encoder_pos_pub("SL_joint_encoder_pos", &joint_encoder_pos);     //BR joint position in +/- degrees, to three decimal places. Stored internally as a long, then converted when published to Float32
ros::Publisher SL_stepper_count_pos_pub("SL_stepper_count_pos", &stepper_count_pos); //Equivalent BR joint position in +/- degrees, to three decimal places. Store internally as long as stepper_counts
ros::Publisher SL_stepper_encoder_pos_pub("SL_stepper_encoder_pos", &stepper_encoder_pos); //Equivalent BR joint position in +/- degrees, to three decimal places.
ros::Publisher SL_torque_pub("SL_torque", &torque);       //BR joint rotation in +/- degrees, to three decimal places. std_msgs/Float32.
//ros::Publisher BR_joint_state_pub("BR_joint_state", &BR_joint_state);  //sensor_msgs/JointState
ros::Publisher SL_state_pub("SL_state", &state);        //std_msgs/String. States such cw_endstop_activated, holding_position, moving, unpowered, yellow_torque, red_torque
ros::Publisher SL_state2_pub("SL_state2", &state2);        //std_msgs/String. States such cw_endstop_activated, holding_position, moving, unpowered, yellow_torque, red_torque

//ros::Publisher joint_aoe_pos_pub("BR_aoe_pos", &joint_aoe_pos);     //BR joint position in +/- degrees, to three decimal places. std_msgs/Float32. This is result of lookup function.
ros::Publisher UR_joint_encoder_pos_pub("UR_joint_encoder_pos", &joint_encoder_pos);     //BR joint position in +/- degrees, to three decimal places. Stored internally as a long, then converted when published to Float32
ros::Publisher UR_stepper_count_pos_pub("UR_stepper_count_pos", &stepper_count_pos); //Equivalent BR joint position in +/- degrees, to three decimal places. Store internally as long as stepper_counts
ros::Publisher UR_stepper_encoder_pos_pub("UR_stepper_encoder_pos", &stepper_encoder_pos); //Equivalent BR joint position in +/- degrees, to three decimal places.
ros::Publisher UR_torque_pub("UR_torque", &torque);       //BR joint rotation in +/- degrees, to three decimal places. std_msgs/Float32.
//ros::Publisher BR_joint_state_pub("BR_joint_state", &BR_joint_state);  //sensor_msgs/JointState
ros::Publisher UR_state_pub("UR_state", &state);        //std_msgs/String. States such cw_endstop_activated, holding_position, moving, unpowered, yellow_torque, red_torque

//ros::Publisher joint_aoe_pos_pub("BR_aoe_pos", &joint_aoe_pos);     //BR joint position in +/- degrees, to three decimal places. std_msgs/Float32. This is result of lookup function.
ros::Publisher EL_joint_encoder_pos_pub("EL_joint_encoder_pos", &joint_encoder_pos);     //BR joint position in +/- degrees, to three decimal places. Stored internally as a long, then converted when published to Float32
ros::Publisher EL_stepper_count_pos_pub("EL_stepper_count_pos", &stepper_count_pos); //Equivalent BR joint position in +/- degrees, to three decimal places. Store internally as long as stepper_counts
ros::Publisher EL_stepper_encoder_pos_pub("EL_stepper_encoder_pos", &stepper_encoder_pos); //Equivalent BR joint position in +/- degrees, to three decimal places.
ros::Publisher EL_torque_pub("EL_torque", &torque);       //BR joint rotation in +/- degrees, to three decimal places. std_msgs/Float32.
//ros::Publisher BR_joint_state_pub("BR_joint_state", &BR_joint_state);  //sensor_msgs/JointState
ros::Publisher EL_state_pub("EL_state", &state);        //std_msgs/String. States such cw_endstop_activated, holding_position, moving, unpowered, yellow_torque, red_torque

//ros::Publisher joint_aoe_pos_pub("BR_aoe_pos", &joint_aoe_pos);     //BR joint position in +/- degrees, to three decimal places. std_msgs/Float32. This is result of lookup function.
ros::Publisher LR_joint_encoder_pos_pub("LR_joint_encoder_pos", &joint_encoder_pos);     //BR joint position in +/- degrees, to three decimal places. Stored internally as a long, then converted when published to Float32
ros::Publisher LR_stepper_count_pos_pub("LR_stepper_count_pos", &stepper_count_pos); //Equivalent BR joint position in +/- degrees, to three decimal places. Store internally as long as stepper_counts
ros::Publisher LR_stepper_encoder_pos_pub("LR_stepper_encoder_pos", &stepper_encoder_pos); //Equivalent BR joint position in +/- degrees, to three decimal places.
ros::Publisher LR_torque_pub("LR_torque", &torque);       //BR joint rotation in +/- degrees, to three decimal places. std_msgs/Float32.
//ros::Publisher BR_joint_state_pub("BR_joint_state", &BR_joint_state);  //sensor_msgs/JointState
ros::Publisher LR_state_pub("LR_state", &state);        //std_msgs/String. States such cw_endstop_activated, holding_position, moving, unpowered, yellow_torque, red_torque

ros::Publisher WL_joint_encoder_pos_pub("WL_joint_encoder_pos", &joint_encoder_pos);     //WL joint position in +/- degrees, to three decimal places. Stored internally as a long, then converted when published to Float32
ros::Publisher WL_stepper_count_pos_pub("WL_stepper_count_pos", &stepper_count_pos); //Equivalent WL joint position in +/- degrees, to three decimal places. Store internally as long as stepper_counts
ros::Publisher WL_stepper_encoder_pos_pub("WL_stepper_encoder_pos", &stepper_encoder_pos); //Equivalent WL joint position in +/- degrees, to three decimal places.
ros::Publisher WL_torque_pub("WL_torque", &torque);       //WL joint rotation in +/- degrees, to three decimal places. std_msgs/Float32.
ros::Publisher WL_state_pub("WL_state", &state);        //std_msgs/String. States such cw_endstop_activated, holding_position, moving, unpowered, yellow_torque, red_torque


void doEncoder1A() {
  /*  encoder rotation causes encoder pins to change state. this monitors sequence of
      pins changes to determine direction and counts pulses to determine position change.
      A Series-Elastic joint and gear reduction connect the stepper motor to the encoder.
      THe difference between the encoder position and the stepper position is proportional
      to the torque across the Series-Elastic joint. Integer math can be used for torque
      measurement in real-time by selecting a gear reduction ratio to be an integer multiple
      of the ratio between encoder counts per revolution and stepper counts per revolution.
      At each encoder pulse, the encoder count is incremented by this integer multiple, thus
      at any time, the stepper position and the encoder position can be directly compared to determine torque.
      For example, in this incarnation:
      Stepper steps per revolution: 200
      ENcoder counts per revolution: 360
      Ration encoder counts to stepper steps: 360/200 = 1.8:1
      Planetary gear reduction ratio: 3.6:1
      Integer multiple: 3.6/1.8 = 2
      Increment/decrement encoder count by integer multiple of 2 at each encoder pulse
  */
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_1_PIN_A) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_B) == LOW) {
      //      encoder_1_pos += 1;         // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;         // CCW
      encoder_1_pos += 1;         // CW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_B) == HIGH) {
      //      encoder_1_pos += 1;          // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;          // CCW
      encoder_1_pos += 1;         // CW
    }
  }
} //end doEncoder1A()

void doEncoder1B() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_1_PIN_B) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_A) == HIGH) {
      //      encoder_1_pos += 1;         // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;         // CCW
      encoder_1_pos += 1;         // CW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_A) == LOW) {
      //      encoder_1_pos += 1;          // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;          // CCW
      encoder_1_pos += 1;         // CW
    }
  }
} //end doEncoder1B()

void doEncoder2A() {
  /*  encoder rotation causes encoder pins to change state. this monitors sequence of
      pins changes to determine direction and counts pulses to determine position change.
      A Series-Elastic joint and gear reduction connect the stepper motor to the encoder.
      THe difference between the encoder position and the stepper position is proportional
      to the torque across the Series-Elastic joint. Integer math can be used for torque
      measurement in real-time by selecting a gear reduction ratio to be an integer multiple
      of the ratio between encoder counts per revolution and stepper counts per revolution.
      At each encoder pulse, the encoder count is incremented by this integer multiple, thus
      at any time, the stepper position and the encoder position can be directly compared to determine torque.
      For example, in this incarnation:
      Stepper steps per revolution: 200
      ENcoder counts per revolution: 360
      Ration encoder counts to stepper steps: 360/200 = 1.8:1
      Planetary gear reduction ratio: 3.6:1
      Integer multiple: 3.6/1.8 = 2
      Increment/decrement encoder count by integer multiple of 2 at each encoder pulse
  */
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_2_PIN_A) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_B) == LOW) {
      //      encoder_2_pos += 1;         // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;         // CCW
      encoder_2_pos += 1;         // CW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_B) == HIGH) {
      //      encoder_2_pos += 1;          // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;          // CCW
      encoder_2_pos += 1;         // CW
    }
  }
} //end doEncoder2A()

void doEncoder2B() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_2_PIN_B) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_A) == HIGH) {
      //      encoder_2_pos += 1;         // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;         // CCW
      encoder_2_pos += 1;         // CW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_A) == LOW) {
      //      encoder_2_pos += 1;          // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;          // CCW
      encoder_2_pos += 1;         // CW
    }
  }
} //end doEncoder2B()

void pulseStepper() {
  //pulses stepper during normal motion.
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP2_PIN, HIGH);
  //    delayMicroseconds(5);  //increase this if pulses too fast for accurate step or jitters
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(STEP2_PIN, LOW);
  //    delayMicroseconds(5);  //increase this if pulses too fast for accurate step or jitters
  if (direction_CW) {
    stepper_counts++;
  }
  else {
    stepper_counts--;
  }
  //  nh.loginfo("pulseStepper");
  //  nh.spinOnce();

} // end pulseStepper()

void stepOnce() {
  /*
     determines, based on state, whether or not to pulse stepper, and how fast to pulse
  */
  //  nh.loginfo("stepOnce()");
  //  nh.spinOnce();

  //check for endstop activation - note endstop pins are set as INPUT_PULLUP so a LOW means switch has been activated
  if (digitalRead(CW_ENDSTOP_PIN) == LOW) {
    if (direction_CW) {
      nh.loginfo("CW endstop + dir CW, stop!");
      //WORKING HERE 12/6/17
      //NEED TO CHECK DIRECTIONS ARE CORRECT
      //NEED TO UNCOMMENT NEXT LINE
      running_state = CW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
    else {
      nh.loginfo("CW endstop + dir CCW");
      running_state = CW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
  }
  if (digitalRead(CCW_ENDSTOP_PIN) == LOW) {
    if (!direction_CW) {
      nh.loginfo("CCW endstop + dir CCW, stop!");
      running_state = CCW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
    else {
      nh.loginfo("CCW endstop + dir CW");
      running_state = CCW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
  }

  switch (running_state) {
    case UNPOWERED:
      //      nh.loginfo("UNP");
      break;
    case HOLDING_POSITION:
      //      nh.loginfo("HOLDING_POSITION");
      if (steps_remaining > close_enough) {
        //pulse stepper
        pulseStepper();
        //decrements steps remaining
        steps_remaining--;
      }
      break;
    case MOVING:
      //      nh.loginfo("MOVING");
      if (current_plateau_steps_remaining > 0) {
        current_plateau_steps_remaining--;
      }
      else {
        //used up steps to pulse in this plateau, need to see if accelerating or decelerating
        if (steps_remaining > accel_until_stepper_counts) {
          //still accelerating, so move to next fastest plateau
          current_plateau_index++;
          Timer1.setPeriod(accel_LUT[current_plateau_index][0]);
          //debug
          sprintf(miscMsgs, "steps_remaining=%ld", steps_remaining);
          nh.loginfo(miscMsgs);
          nh.spinOnce();
          nh.loginfo("cpi++");
          nh.spinOnce();
          //have we entered cruise phase?
          if (current_plateau_index == cruise_plateau_index) {
            //Yes, set up for cruise phase
            current_plateau_steps_remaining = cruising_stepper_counts;
          }
          else {
            //No, still more acceleration to come
            current_plateau_steps_remaining = accel_LUT[current_plateau_index][1];
          }
        }
        else if (steps_remaining < decel_after_stepper_counts) {
          //Decelerating, do we still have steps to move?
          if (steps_remaining > 0) {
            //yes, still decelerating, so move to next slowest plateau
            current_plateau_index--;
            Timer1.setPeriod(accel_LUT[current_plateau_index][0]);
            //debug
            sprintf(miscMsgs2, "steps_remaining=%ld", steps_remaining);
            nh.loginfo(miscMsgs2);
            nh.spinOnce();
            nh.loginfo("cpi--");
            nh.spinOnce();
            current_plateau_steps_remaining = accel_LUT[current_plateau_index][1];
          }
          else {
            //no, movement is complete. setup HOLDING_POSITION state and return
            running_state = HOLDING_POSITION;
            Timer1.setPeriod(POSITION_ADJUSTMENT_SPEED);
            //WORKING HERE 12/14/17 8:33AM - not getting here!
            Timer1.stop();
            nh.loginfo("stopping timer");
            return;
          }
        }
        else {
          //cruising 0- no action required
        }
      }
      //step and decrement
      pulseStepper();
      steps_remaining--;
      break;

    //    case FINAL_APPROACH:
    //      //      nh.loginfo("FINAL_APPROACH");
    //      if (steps_remaining > 0) {
    //        //pulse stepper
    //        pulseStepper();
    //        //decrements steps remaining
    //        steps_remaining--;
    //      }
    //      else {
    //        //final_approach phase is complete, transition to holding_position
    //        running_state = HOLDING_POSITION;
    //      }
    //      break;
    case CW_ENDSTOP_ACTIVATED:
      break;
    case CCW_ENDSTOP_ACTIVATED:
      break;
    case CALIBRATING:
      break;
    default:
      break;
  }//end switch running_state
}// end stepOnce()

float getCurrentPos ( long encoder_position_ ) {
  /*
    Converts doe encoder_1_pos in doe encoder counts into degrees of joint position and centers appropriately from calibration()
    Note encoder position zero is defined as CCW endstop, while joint position zero is defined as center of travel
    Note that degrees of joint position can therefore be positive or negative
    Conversion factor is half of fullscale doe travel
  */
  /*
     note there is a spceical temporary case for WL Cycloidal Test Rig to use stepper count position instead of encoder count position unitl encoder is connecte)
  */
  //To Do - reinstate after calibration routine finished
  //  float float_pos_ = ENCODER_1_DEGREES_PER_COUNT * (encoder_1_pos - (max_doe_counts / 2));
  //float float_pos_ = (float) (ENCODER_1_DEGREES_PER_COUNT * encoder_1_pos);
  float float_pos_;

  noInterrupts();

  nh.loginfo("entering getCurrentPos()");
  nh.spinOnce();

  sprintf(miscMsgs3, "long encoder_1_pos=%ld", encoder_1_pos);
  nh.loginfo(miscMsgs3);
  nh.spinOnce();

  sprintf(miscMsgs4, "long encoder_position_=%ld", encoder_position_);
  nh.loginfo(miscMsgs4);
  nh.spinOnce();

  //WORKING HERE 1/1/2018 11:49am miscMsgs in this next section changes value of volatile long encoder_1_pos
  sprintf(miscMsgs5, "getCurrentPos() encoder_position_=%ld", encoder_position_);
  nh.loginfo(miscMsgs5);
  //  float float_pos_ = -1.0 * ((float) encoder_1_pos) * joint_degrees_per_encoder_count;
  if (minion_ident == WL_IDENT) {
    //SPECIAL CASE OF WL CYCLOIDAL TEST RIG 7/20/18
    float_pos_ =  stepper_count_pos.data;
  }
  else {
    float_pos_ = -1.0 * ((float) encoder_position_) * joint_degrees_per_encoder_count;
  }

  //debug to do remove
  //  float float_pos_ = 1.01;

  sprintf(miscMsgs6, "long encoder_1_pos=%ld", encoder_1_pos);
  nh.loginfo(miscMsgs6);
  nh.spinOnce();

  interrupts();

  nh.loginfo("exiting getCurrentPos()");
  nh.spinOnce();

  return float_pos_;
}

void planMovement(float the_commanded_position_) {
  /*
     determines variables for movement controlled by accel_LUT
     accel_until_stepper_counts and accel_until_stepper_counts
  */
  float dtg_ = 0.0;  //joint distance to travel in signed degrees. Sign indicates direction of movement
  long dtg_stepper_counts_ = 0; //joint distance to travel in steps in absolute value as long
  float min_travel_time_ = 0.1; //in seconds using MAX_VEL and MAX_ACCEL and accel_LUT
  long steps_to_reach_max_vel_ = 0;//from accel_LUT
  long steps_to_reach_reduced_cruise_vel_ = 0;//from accel_LUT
  long steps_to_reach_cruise_ = 0;
  long cruise_steps_at_max_vel_ = 0;
  long cruise_steps_at_reduced_vel_ = 0;
  long cruise_steps_ = 0;

  nh.loginfo("entering planMovement()");
  nh.spinOnce();

  //stop Timer1 from firing pulses while plan is made
  //  Timer1.stop();

  sprintf(miscMsgs7, "long encoder_1_pos=%ld", encoder_1_pos);
  nh.loginfo(miscMsgs7);
  nh.spinOnce();

  //get the distance to go
  current_pos = getCurrentPos(encoder_1_pos); // uses doe as joint reference for now. Eventually will be aoe.
  dtg_ = the_commanded_position_ - current_pos;  //joint distance to travel in signed degrees. Sign indicates direction of movement

  //debug loginfo dtg_
  char result[16]; // Buffer big enough for 15-character float
  dtostrf(current_pos, 14, 2, result); // Leave room for too large numbers!
  nh.loginfo("current_pos=");//this works
  nh.loginfo(result);//this works
  nh.spinOnce();
  dtostrf(dtg_, 14, 2, result); // Leave room for too large numbers!
  nh.loginfo("dtg_=");//this works
  nh.loginfo(result);//this works

  //convert dtg_ in degrees to dtg_stepper_counts_
  dtg_stepper_counts_ = (long) abs(dtg_ / degrees_per_microstep);

  //debug only
  sprintf(miscMsgs8, "dtg_stepper_counts_=%ld", dtg_stepper_counts_);
  nh.loginfo(miscMsgs8);

  //  //debug
  //  nh.loginfo("finished populating LUT");
  //  nh.spinOnce();

  //debug loginfo
  char result3[8]; // Buffer big enough for 7-character float
  dtostrf(PLATEAU_DURATION, 6, 2, result3); // Leave room for too large numbers!
  nh.loginfo("PLATEAU_DURATION=");//this works
  nh.loginfo(result3);//this works
  nh.spinOnce();

  //debug print out accel_LUT stuff
  for (int j = 0; j < NUM_PLATEAUS; j++) {
    //debug only
    //    sprintf(miscMsgs, "accel_LUT[1][0]=%ld", accel_LUT[j][0]);
    sprintf(miscMsgs9, "accel_LUT[%ld", j);
    sprintf(miscMsgs10, "][0]=%ld", accel_LUT[j][0]);
    strcat(miscMsgs9, miscMsgs10);
    nh.loginfo(miscMsgs9);
    nh.spinOnce();
  }
  nh.loginfo("");
  nh.spinOnce();

  //debug print out accel_LUT stuff
  for (int j = 0; j < NUM_PLATEAUS; j++) {
    //debug only
    //    sprintf(miscMsgs, "accel_LUT[1][0]=%d", accel_LUT[j][0]);
    sprintf(miscMsgs11, "accel_LUT[%ld", j);
    sprintf(miscMsgs12, "][1]=%ld", accel_LUT[j][1]);
    strcat(miscMsgs11, miscMsgs12);
    nh.loginfo(miscMsgs11);
    nh.spinOnce();
  }
  nh.loginfo("");
  nh.spinOnce();

  //debug print out accel_LUT stuff
  for (int j = 0; j < NUM_PLATEAUS; j++) {
    //debug only
    //    sprintf(miscMsgs, "accel_LUT[1][0]=%ld", accel_LUT[j][0]);
    sprintf(miscMsgs13, "accel_LUT[%ld", j);
    sprintf(miscMsgs14, "][2]=%ld", accel_LUT[j][2]);
    strcat(miscMsgs13, miscMsgs14);
    nh.loginfo(miscMsgs13);
    nh.spinOnce();
  }

  //  //debug
  //  nh.loginfo("finished printing LUT");
  //  nh.spinOnce();

  //calculate minimum number of steps to accel to max velocity
  steps_to_reach_max_vel_ = accel_LUT[NUM_PLATEAUS - 1][2];//correct

  //debug only
  sprintf(miscMsgs15, "steps_to_reach_max_vel_=%ld", steps_to_reach_max_vel_);
  nh.loginfo(miscMsgs15);
  nh.spinOnce();

  //determine min_travel_time for this movement
  if (dtg_stepper_counts_ > (2 * steps_to_reach_max_vel_)) {
    //in this case, can accel to max vel, hence there will be a cruise phase at max velocity
    //calculate accel and decel time at max accel
    min_travel_time_ = (float)(2 * NUM_PLATEAUS * PLATEAU_DURATION);

    //debug only
    sprintf(miscMsgs16, "2nd reading dtg_stepper_counts_=%ld", dtg_stepper_counts_);
    nh.loginfo(miscMsgs16);

    //calc cruise steps
    cruise_steps_at_max_vel_ = (dtg_stepper_counts_ - ( 2 * steps_to_reach_max_vel_));//correct
    //    cruise_steps_ = (dtg_stepper_counts_ - ( 2 * steps_to_reach_max_vel_));//correct

    //calc cruise time and add to accel+decel time. Note LUT value is period (ie time) between steps in microseconds
    min_travel_time_ += (float)(cruise_steps_at_max_vel_ * accel_LUT[NUM_PLATEAUS - 1][0]) / 1000000.0;
    //    min_travel_time_ += (float)(cruise_steps_ * accel_LUT[NUM_PLATEAUS - 1][0]) / 1000000.0;

    //calc cruise_plateau_index - for this case it will be max vel
    cruise_plateau_index = NUM_PLATEAUS - 1;
  }
  else {
    //else cannot accel to max vel within steps to be moved, hence must figure short cruise phase at lower velocity
    //iterate through LUT to find first plateau that will move enough steps
    for (int j = 1; j < NUM_PLATEAUS; j++) {
      if (dtg_stepper_counts_ < 2 * accel_LUT[j][2]) {
        //this plateau will move more steps than are required - use next slowest plateau with short cruise segment
        cruise_plateau_index = j - 1;
        break;
      }
    }

    //calc cruise steps
    steps_to_reach_reduced_cruise_vel_ = accel_LUT[cruise_plateau_index][2];
    cruise_steps_at_reduced_vel_ = (dtg_stepper_counts_ - ( 2 * steps_to_reach_reduced_cruise_vel_));
    //    steps_to_reach_cruise_ = accel_LUT[cruise_plateau_index][2];
    //    cruise_steps_ = (dtg_stepper_counts_ - ( 2 * steps_to_reach_cruise_));

    //calculate accel and decel time
    min_travel_time_ = (float)(2 * cruise_plateau_index * PLATEAU_DURATION);

    //calc cruise time and add to accel+decel time. Note LUT value is period (ie time) between steps in microseconds
    min_travel_time_ += (float)(cruise_steps_at_reduced_vel_ * accel_LUT[cruise_plateau_index][0]) / 1000000.0;
    //    min_travel_time_ += (float)(cruise_steps_ * accel_LUT[cruise_plateau_index][0]) / 1000000.0;

  }

  //debug only
  sprintf(miscMsgs17, "cruise_plateau_index=%d", cruise_plateau_index);
  nh.loginfo(miscMsgs17);
  nh.spinOnce();

  //debug only
  sprintf(miscMsgs18, "cruise_steps_at_reduced_vel_=%ld", cruise_steps_at_reduced_vel_);
  nh.loginfo(miscMsgs18);
  nh.spinOnce();

  //debug loginfo
  char result6[8]; // Buffer big enough for 7-character float
  dtostrf(min_travel_time_, 6, 2, result6); // Leave room for too large numbers!
  nh.loginfo("min_travel_time_=");//this works
  nh.loginfo(result6);//this works
  nh.spinOnce();

  //now determine if min_travel_time_ is more or less than required_duration
  if (required_duration < min_travel_time_) {
    //operate as fast as possible - this joint will take longer than required
    //math has already been done above for:
    //min_travel_time_
    //cruise_plateau_index
    //cruise_steps_at_max_vel_
  }
  else {
    /*
       this joint will operate at slower speed than max to complete movement in as
       close to required time as possible
       iterate through accel_LUT lookup table to determine lowest plateau speed level
       that meets required duration of movement
    */
    float calculated_duration_ = 0.0;
    for (int i = 1; i < NUM_PLATEAUS; i++) {
      //starts at i=1 because first plateau has zeros in it
      //first, calculate time to accel to, then decel from, the ith plateau
      calculated_duration_ = 2 * i * PLATEAU_DURATION;
      //then add the cruising time
      steps_to_reach_reduced_cruise_vel_ = accel_LUT[i][2];
      cruise_steps_at_reduced_vel_ = dtg_stepper_counts_ - (2 * steps_to_reach_reduced_cruise_vel_);
      calculated_duration_ += (float)(cruise_steps_at_reduced_vel_ * accel_LUT[i][0]) / 1000000.0;
      //now find the first index when this calculated duration is less than the required duration.
      if (calculated_duration_ < required_duration) {
        //this will be the cruise plateau
        cruise_plateau_index = i;
        break;
      }
    }

    //debug loginfo
    char result4[8]; // Buffer big enough for 7-character float
    dtostrf(calculated_duration_, 6, 2, result4); // Leave room for too large numbers!
    nh.loginfo("calculated_duration_=");//this works
    nh.loginfo(result4);//this works
    nh.spinOnce();

    //debug only
    sprintf(miscMsgs19, "cruise_plateau_index=%d", cruise_plateau_index);
    nh.loginfo(miscMsgs19);
    nh.spinOnce();

    //  nh.loginfo("finished iterating LUT");
  }

  //outputs needed
  //cruise_plateau_index
  //calculated_duration_ or global version thereof
  //accel_until_stepper_counts
  //decel_after_stepper_counts
  //cruising_stepper_counts


  //calculate accelerate until and decelerate after points in terms of stepper counts
  accel_until_stepper_counts = dtg_stepper_counts_ - accel_LUT[cruise_plateau_index][2];
  decel_after_stepper_counts = accel_LUT[cruise_plateau_index][2];

  //debug only
  sprintf(miscMsgs20, "accel_until_stepper_counts=%ld", accel_until_stepper_counts);
  nh.loginfo(miscMsgs20);
  nh.spinOnce();
  //debug only
  sprintf(miscMsgs21, "decel_after_stepper_counts=%ld", decel_after_stepper_counts);
  nh.loginfo(miscMsgs21);
  nh.spinOnce();

  //  nh.loginfo("finished accel_until");
  //  nh.spinOnce();

  //calculate exact number of cruise stepper counts
  cruising_stepper_counts = dtg_stepper_counts_ - (2 * accel_LUT[cruise_plateau_index][2]);
  //debug only
  sprintf(miscMsgs22, "cruising_stepper_counts=%ld", cruising_stepper_counts);
  nh.loginfo(miscMsgs22);
  nh.spinOnce();

  //get direction of motion. positive distance defined as CW, negative as CCW
  if (dtg_ > 0) {
    direction_CW = true;
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(DIR2_PIN, HIGH);
    nh.loginfo("in planMovement(), dir is CW");
    nh.spinOnce();
  } else {
    direction_CW = false;
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(DIR2_PIN, LOW);
    nh.loginfo("in planMovement(), dir is CCW");
    nh.spinOnce();
  }
  nh.spinOnce();

  //set Timer1 to period specified in first plateau
  //  Timer1.initialize(500000L);  // initialize timer1, and set a 1/2 second period
  //  Timer1.attachInterrupt(stepOnce);
  Timer1.setPeriod(accel_LUT[1][0]);
  //    Timer1.setPeriod(1000000L);

  //  Timer1.restart();
  //  Timer1.start();
  nh.loginfo("setPeriod");
  nh.spinOnce();

  //initilaize first plateau parameters
  steps_remaining = dtg_stepper_counts_;
  current_plateau_index = 1;
  current_plateau_steps_remaining = accel_LUT[current_plateau_index][1];

  //debug only
  sprintf(miscMsgs23, "3rd reading dtg_stepper_counts_=%ld", dtg_stepper_counts_);
  nh.loginfo(miscMsgs23);
  nh.spinOnce();

  sprintf(miscMsgs24, "long encoder_1_pos=%ld", encoder_1_pos);
  nh.loginfo(miscMsgs24);
  nh.spinOnce();

  nh.loginfo("exiting planMovement()");
  nh.spinOnce();
}//end planMovement()

void readSensors() {
  //report state
  strcpy(states_msg, empty_states_msg);      //overwrites previous states_msg with empty msg
  if (running_state == UNPOWERED) strcat(states_msg, unpowered_msg);
  if (running_state == HOLDING_POSITION) strcat(states_msg, holding_position_msg);
  if (running_state == MOVING) strcat(states_msg, moving_msg);
  if (running_state == FINAL_APPROACH) strcat(states_msg, final_approach_msg);
  if (running_state == CW_ENDSTOP_ACTIVATED) strcat(states_msg, cw_endstop_activated_msg);
  if (running_state == CCW_ENDSTOP_ACTIVATED) strcat(states_msg, ccw_endstop_activated_msg);
  if (running_state == CALIBRATING) strcat(states_msg, calibrating_msg);
  if (torque_state == GREEN) strcat(states_msg, green_torque_msg);
  if (torque_state == YELLOW) strcat(states_msg, yellow_torque_msg);
  if (torque_state == RED) strcat(states_msg, red_torque_msg);

  //place readings in ros messages to be published
  //WORKING HERE 12/13/17
  //gear ratio SL joint to AndyMark encoder on S-E joint = 19.741 = (6/2)*(59T/23T)*(59T/23T)
  joint_encoder_pos.data = -1.0 * ((float) encoder_1_pos) * joint_degrees_per_encoder_count;//TO DO change this to true joint encoder after joint encoder actually installed
  stepper_count_pos.data = ((float) stepper_counts) * degrees_per_microstep;
  stepper_encoder_pos.data = 9.99;//stepper_doe_pos TO DO placeholder value for now
  torque.data = joint_encoder_pos.data - stepper_count_pos.data;//TO DO convert to joint_encoder_pos.data -stepper_encoder_pos.data

}//end readSensors()

void publishAll() {
  //publish stuff for this joint
  //  torque.data = commanded_stepper_position;

  //debug only
  //      static int publish_counter = 0;
  //    sprintf(miscMsgs, "publish_counter=%d", publish_counter);
  //    publish_counter++;
  //    nh.loginfo(miscMsgs);
  //  nh.spinOnce();

  //populate data
  state.data = states_msg;

  //debug only
  //  sprintf(miscMsgs, "encoder_1_pos=%ld", encoder_1_pos);
  //  nh.loginfo(miscMsgs);
  //  nh.spinOnce();
  //  sprintf(miscMsgs, "encoder_2_pos=%ld", encoder_2_pos);
  //  nh.loginfo(miscMsgs);
  //  nh.spinOnce();
  //  sprintf(miscMsgs, "current_pos=%d", current_pos);
  //  nh.loginfo(miscMsgs);
  //  nh.spinOnce();

  switch (minion_ident) {
    case SL_IDENT:
      SL_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      SL_stepper_count_pos_pub.publish( &stepper_count_pos );
      nh.spinOnce();
      SL_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      SL_torque_pub.publish( &torque );
      //      SL_joint_state_pub.publish( &BR_joint_state );
      SL_state_pub.publish( &state );
      SL_state2_pub.publish( &state2 );
      nh.spinOnce();
      break;
    case UR_IDENT:
      UR_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      UR_stepper_count_pos_pub.publish( &stepper_count_pos );
      nh.spinOnce();
      UR_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      UR_torque_pub.publish( &torque );
      //      UR_joint_state_pub.publish( &BR_joint_state );
      UR_state_pub.publish( &state );
      nh.spinOnce();
      break;
    case EL_IDENT:
      EL_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      EL_stepper_count_pos_pub.publish( &stepper_count_pos );
      nh.spinOnce();
      EL_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      EL_torque_pub.publish( &torque );
      //      EL_joint_state_pub.publish( &BR_joint_state );
      EL_state_pub.publish( &state );
      nh.spinOnce();
      break;
    case LR_IDENT:
      LR_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      LR_stepper_count_pos_pub.publish( &stepper_count_pos );
      nh.spinOnce();
      LR_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      LR_torque_pub.publish( &torque );
      //      LR_joint_state_pub.publish( &BR_joint_state );
      LR_state_pub.publish( &state );
      nh.spinOnce();
      break;
    case WL_IDENT:
      WL_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      WL_stepper_count_pos_pub.publish( &stepper_count_pos );
      nh.spinOnce();
      WL_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      WL_torque_pub.publish( &torque );
      //      WL_joint_state_pub.publish( &WL_joint_state );
      WL_state_pub.publish( &state );
      nh.spinOnce();
      break;
    default:
      break;
  }//end switch case
}//end publishAll()

void setup() {
  // setup, then read, minion identification pins
  pinMode(IDENT_PIN_1, INPUT_PULLUP);
  pinMode(IDENT_PIN_2, INPUT_PULLUP);
  pinMode(IDENT_PIN_4, INPUT_PULLUP);
  pinMode(IDENT_PIN_8, INPUT_PULLUP);
  bitWrite(minion_ident, 0, digitalRead(IDENT_PIN_1));
  bitWrite(minion_ident, 1, digitalRead(IDENT_PIN_2));
  bitWrite(minion_ident, 2, digitalRead(IDENT_PIN_4));
  bitWrite(minion_ident, 3, digitalRead(IDENT_PIN_8));

  //init encoder pins
  pinMode(ENCODER_1_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_1_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_B, INPUT_PULLUP);

  //init endstop pins
  pinMode(CW_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(CCW_ENDSTOP_PIN, INPUT_PULLUP);

  //init digital encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_A), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_B), doEncoder1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_A), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_B), doEncoder2B, CHANGE);

  //init stepper driver pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  //enable stepper by default
  digitalWrite(ENABLE_PIN, LOW);


  //setup publishers and subscribers
  //  nh.getHardware()->setBaud(230400);  //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  nh.getHardware()->setBaud(115200);//06Jan2018 lowered to 115200 for compatiblity with simultaneous running of Arduino Due //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
//  nh.getHardware()->setBaud(57600);  //reset to this 1/22/19 for sevon-minio comaptibility - baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  //    nh.getHardware()->setBaud(9600);  //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  nh.initNode();
  nh.subscribe(commanded_joint_positions_sub);

  /*
    use switch to setup each joint's needs:
      joint_degrees_per_encoder_count
      degrees per microstep
      encoders, pubs and subs
  */

  // only encoders, pubs and subs needed for this minion's joint
  switch (minion_ident) {
    case SL_IDENT:
      // SL joint
      joint_degrees_per_encoder_count = SL_JOINT_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = SL_DEGREES_PER_MICROSTEP;
      nh.advertise(SL_joint_encoder_pos_pub);
      nh.advertise(SL_stepper_count_pos_pub);
      nh.advertise(SL_stepper_encoder_pos_pub);
      nh.advertise(SL_torque_pub);
      nh.advertise(SL_state_pub);
      nh.advertise(SL_state2_pub);
      //      nh.subscribe(SL_plan_sub);
      nh.loginfo("SetupSL");
      break;
    case UR_IDENT:
      // UR joint
      joint_degrees_per_encoder_count = UR_JOINT_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = UR_DEGREES_PER_MICROSTEP;
      nh.advertise(UR_joint_encoder_pos_pub);
      nh.advertise(UR_stepper_count_pos_pub);
      nh.advertise(UR_stepper_encoder_pos_pub);
      nh.advertise(UR_torque_pub);
      nh.advertise(UR_state_pub);
      //      nh.subscribe(UR_plan_sub);
      nh.loginfo("SetupUR");
      break;
    case EL_IDENT:
      // EL joint
      joint_degrees_per_encoder_count = EL_JOINT_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = EL_DEGREES_PER_MICROSTEP;
      nh.advertise(EL_joint_encoder_pos_pub);
      nh.advertise(EL_stepper_count_pos_pub);
      nh.advertise(EL_stepper_encoder_pos_pub);
      nh.advertise(EL_torque_pub);
      nh.advertise(EL_state_pub);
      //      nh.subscribe(EL_plan_sub);
      nh.loginfo("SetupEL");
      break;
    case LR_IDENT:
      // LR joint
      joint_degrees_per_encoder_count = LR_JOINT_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = LR_DEGREES_PER_MICROSTEP;
      nh.advertise(LR_joint_encoder_pos_pub);
      nh.advertise(LR_stepper_count_pos_pub);
      nh.advertise(LR_stepper_encoder_pos_pub);
      nh.advertise(LR_torque_pub);
      nh.advertise(LR_state_pub);
      //      nh.sscribe(LR_plan_sub);
      nh.loginfo("SetupLR");
      break;
    case WL_IDENT:
      // WL joint
      joint_degrees_per_encoder_count = WL_JOINT_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = WL_DEGREES_PER_MICROSTEP;
      nh.advertise(WL_joint_encoder_pos_pub);
      nh.advertise(WL_stepper_count_pos_pub);
      nh.advertise(WL_stepper_encoder_pos_pub);
      nh.advertise(WL_torque_pub);
      nh.advertise(WL_state_pub);
      //      nh.subscribe(WL_plan_sub);
      nh.loginfo("SetupWL");
    break;    default:
      break;
  }//end switch case

  //set position hold tolerance as function of degrees_per_microstep set in switch case above
  close_enough = POSITION_HOLD_TOLERANCE * degrees_per_microstep;//position holding tolerance in stepper steps

  nh.spinOnce();

  //setup timer
  //  Timer7.attachInterrupt(stepOnce).setPeriod(0).start(); //delay(50);      //fires steppers at freqs specified in queue
  //  Timer7.attachInterrupt(stepOnce).setPeriod(0).start(); //delay(50);      //fires steppers at freqs specified in queue
  //ref: https://playground.arduino.cc/Code/Timer1
  Timer1.initialize(5000000);  // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(stepOnce);

  //polpulate acceleration LookUp Table accel_LUT
  long period_;           //time between stepper pulses in microseconds
  float vel_deg_per_sec_; //velocity at a particular plateau in joint degrees per second
  float vel_steps_per_sec_;//
  long num_pulses_;       //number of pulses in a particular plateau
  long cum_num_pulses_ = 0;   //cumulative number of pulses
  for (int i = 0; i < NUM_PLATEAUS; i++) {
    //first column is period
    vel_deg_per_sec_ = i * MAX_ACCEL * PLATEAU_DURATION;
    vel_steps_per_sec_ = vel_deg_per_sec_ / degrees_per_microstep;
    period_ = (long) ((1 / vel_steps_per_sec_ ) * 1000000);//works
    accel_LUT[i][0] = period_;
    //second column is number of pulses in this plateau
    num_pulses_ = (long) (PLATEAU_DURATION * vel_steps_per_sec_);
    accel_LUT[i][1] = num_pulses_;
    //third column is cumlative number of pulses
    if (i == 0) {
      cum_num_pulses_ = 0;
    }
    else {
      cum_num_pulses_ = accel_LUT[i - 1][2] + num_pulses_;
    }
    accel_LUT[i][2] = cum_num_pulses_;
  }

  nh.loginfo("V0.0.23 7/20/18");

  nh.spinOnce();
}// end setup()


void loop() {
  //wait until you are actually connected. Ref: http://wiki.ros.org/rosserial_arduino/Tutorials/Logging
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  //  determine motion needed and begin moving
  if (new_plan)     {
    planMovement(commanded_stepper_position);
    //    nh.loginfo("finished if new_plan 1");
    new_plan = false;
    //    nh.loginfo("finished if new_plan 2");
    running_state = MOVING;
    //    nh.loginfo("finished if new_plan 3");
  }

  //log updates
  if (millis() > next_update) {
    // to do ?Add a if (!nh.connected()){spinOnce()} else {updateStatus()};
    //    nh.loginfo("updating");

    //    sprintf(miscMsgs, "steps_remaining=%ld", steps_remaining);
    //    nh.loginfo(miscMsgs);
    //    nh.spinOnce();

    //  //debug
    //    sprintf(miscMsgs25, "encoder_1_pos=%ld", encoder_1_pos);
    //    nh.loginfo(miscMsgs25);
    //    nh.spinOnce();
    //    sprintf(miscMsgs26, "encoder_2_pos=%ld", encoder_2_pos);
    //    nh.loginfo(miscMsgs26);
    //    nh.spinOnce();

    //  //debug get the distance to go
    //  current_pos = getCurrentPos(encoder_1_pos); // uses doe as joint reference for now. Eventually will be aoe.
    //
    //  //debug loginfo
    //  char result[16]; // Buffer big enough for 15-character float
    //  dtostrf(current_pos, 14, 2, result); // Leave room for too large numbers!
    //  nh.loginfo("current_pos=");//this works
    //  nh.loginfo(result);//this works
    //  nh.spinOnce();

    readSensors();
    publishAll();

    //    noInterrupts();
    next_update = millis() + UPDATE_INTERVAL;
    //    interrupts();

  }// end if next_update

  //loop still alive indication
  //  int_loop_counter++;
  //  if (int_loop_counter % 1000 == 0) {
  //    nh.loginfo("looping");
  //    sprintf(miscMsgs27, "millis()=%ld", millis());
  //    nh.loginfo(miscMsgs27);
  //    nh.spinOnce();
  //    sprintf(miscMsgs28, "next_update=%ld", next_update);
  //    nh.loginfo(miscMsgs28);
  //    nh.spinOnce();
  //    int_loop_counter = 0;
  //  }//end if int_loop_counter

  nh.spinOnce();
}
