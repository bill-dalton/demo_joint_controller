/*
   This software is intended to control joint movement in order to test the joint mechanics.
   It will run on four Arduino Megas, one for each of the SL, UR, EL and LR joints.

   The software will know on which joint it is running by checking the state of certain pins
   upon startup. Each joint’s Mega will have different pins tied to ground for identification. On the Minion shields, this is accomplished with DIP switches.

   It is similar to the BR_Demo_Joint_Controller software for the Arduino Due used for
   the BR joint. Because of the limited RAM and processing speed of the Mega, a simpler
   scheme will be used for accel and decel. Each of the four Megas and the single Due
   will publish and subscribe ROS topics via USB to the laptop running roscore.



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
*/

/*
   TO DO
   1/2 - @ident & switch case for joint identification
   doesn't work - @commanded_joint_positions method
   @test simple, 4x command terminals BR_cmd, SL_cmd etc.
   @experiment with JointTrajectoryPoint
      .cpp to read a line tped in terminal, then publish
      this program subscribes, then acts on JTP
   @test pub/sub 4X Arduinos on USB hub
   @lookup table for 100 rows for accel created from .xls Lieb Ramp
*/

#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//states
enum {UNPOWERED, HOLDING_POSITION, MOVING, FINAL_APPROACH, CW_ENDSTOP_ACTIVATED, CCW_ENDSTOP_ACTIVATED, CALIBRATING } running_state;
enum {GREEN, YELLOW, RED} torque_state;

//flags
bool new_plan = false;

//debug messages
char miscMsgs[20];
char miscMsgs2[20];
char miscMsgs3[20];
char inbound_message[40];

//internal position and direction variables
volatile long encoder_1_pos = 0;  //position of doe in encoder clicks
volatile long encoder_2_pos = 0;  //position of stepper doe in encoder clicks
volatile long stepper_counts = 0; //stepper position in stepper steps

//stepper system constants
const int STEPPER_STEPS_PER_REV = 200;  //stepper motor specific
const int MICROSTEPS = 4;               //set on stepper driver
const float GEAR_RATIO = 65.0;
const float STEPPER_DEGREES_PER_STEP = 0.03;//=360/200/15/4

//encoder constants
const int ENCODER_1_PPR = 2400;
const float ENCODER_1_GEAR_RATIO = 5.0;               //ratio final joint motion to doe encoder motion
const float ENCODER_1_DEGREES_PER_COUNT = 0.03;       //=360 / ENCODER_1_PPR / ENCODER_1_GEAR_RATIO;//360/2400/5.0=0.03degrees
const int ENCODER_2_PPR = 2400;                       //in encoder counts, used for stepper_doe
const float ENCODER_2_GEAR_RATIO = 15.0;              //ratio final joint motion to stepper doe encoder motion
const float ENCODER_2_DEGREES_PER_COUNT = 0.01;       //=360 / ENCODER_2_PPR / ENCODER_2_GEAR_RATIO;//360/2400/15.0=0.01degrees
const float ENCODER_1_COUNTS_PER_STEPPER_COUNT = 36.0;//= GEAR_REDUCTION_RATIO_STEPPER_TO_SERIES_ELASTIC * ENCODER_1_PPR / (STEPPER_STEPS_PER_REV * MICROSTEPS);

//motion constants
const float DEGREES_PER_MICROSTEP = 0.03; //= 360 / (STEPPER_STEPS_PER_REV * MICROSTEPS) / GEAR_RATIO;
const float POSITION_HOLD_TOLERANCE = 0.2;    //in degrees
const long POSITION_ADJUSTMENT_SPEED = 5000;  //stepper period interval speed at which to make position adjustments
const float RETREAT_DISTANCE = 1.0;           //degrees to retreat
const long QUEUE_SIZE = 100;//100*8bytes=800 bytes

//torque limits - To Do constants for now, but eventually variables that are function of MoveIt motion plan output
const float YELLOW_LIMIT = 10.0;//torque yellow limit in degrees of joint travel - To Do make function of MoveIt motion plan output
const float RED_LIMIT = 25.0;

//motion variables
volatile float commanded_position = 0.0; //joint commanded_position in degrees
volatile bool direction_CW = true;   //movement in positive joint direction is defined as CW
volatile float current_pos = 0.0;    //joint position in degrees
volatile float pos_error = 0.0;    //difference in degrees between joint commanded_position and joint current position
volatile long dtg_stepper_counts = 0;//in absolute value of stepper counts
volatile long steps_remaining = 0; //stepper steps remaining to move. uses in final_approach and holding_position
volatile long close_enough = POSITION_HOLD_TOLERANCE * DEGREES_PER_MICROSTEP;//position holding tolerance in stepper steps

//my queue
volatile long my_queue[QUEUE_SIZE];
volatile unsigned int head = 0; //index to front of my_queue
volatile unsigned int tail = 0; //index to rear of my_queue

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
const int CW_ENDSTOP_PIN = 16;  //TO DO UPDATE TO CORRECT PIN!
const int CCW_ENDSTOP_PIN = 17;  //TO DO UPDATE TO CORRECT PIN!

//encoder pins
const int ENCODER_1_PIN_A = 18;
const int ENCODER_1_PIN_B = 19;
const int ENCODER_2_PIN_A = 20;
const int ENCODER_2_PIN_B = 21;

//stepper pins
const int STEP_PIN = A0;
const int DIR_PIN = A1;
const int ENABLE_PIN = A2;

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
ros::NodeHandle  nh;
//added this next line to increase size of sub and pub buffers. Defaults were 512bytes. Ref: https://github.com/tonybaltovski/ros_arduino/issues/10
//ros::NodeHandle_<ArduinoHardware, 2, 10, 4096, 4096> nh;//this uses too much dynamic memory for Mega

////loop timing variables
volatile unsigned long next_update = 0;//used to time loop updates
const unsigned long UPDATE_INTERVAL = 1000;//in milliseconds

//debug stuff
float test_var = 56.66;

//subscriber call backs
void commandedPositionCallback(const std_msgs::Float32& the_command_msg_) {
  //action to be taken when msg received on topic BR_cmd. SL_cmd, etc
  static float previous_commanded_position_;
  float the_commanded_position_ = the_command_msg_.data;

  /*
    check for special commands. Normal range of commands is +/- 179.999 degrees,
    therefore anything outside of that range is considered a special command
    commanded_position ~=400 -> means set all values to zero
  */
  if (the_commanded_position_ > -359.99 && the_commanded_position_ < 359.99) {
    //this is a regular command
    //record commanded position as a global
    commanded_position = the_command_msg_.data;

    //check this is not a repeated command to the same position
    if (commanded_position != previous_commanded_position_) {
      //this is a new command. Plan and execute motion
      //set previous commanded position
      previous_commanded_position_ = commanded_position;
      //set flag
      new_plan = true;
    }
  }
}//end commandedPositionCallback()

void commandedJointPositionsCallback(const std_msgs::String& the_command_msg_) {
  //action to be taken when msg received on topic commanded_joint_positions
  float joint_commands_[5];//array holding commanded position in degrees in order: BR,SL,UR,EL,LR

  //parse command
  inbound_message[0] = (char)0;  //"empties inbound_message by setting first char in string to 0
  strcat(inbound_message, the_command_msg_.data);//this finally works

  //debug only
  nh.loginfo("InbdMsg=");
  nh.loginfo(inbound_message);//this now works

  //load state.data for eventual publishing by updateStatus()
  miscMsgs2[0] = (char)0;  //"empties msg by setting first char in string to 0
  strcat(miscMsgs2, inbound_message);
  state.data = miscMsgs2;

//http://jhaskellsblog.blogspot.com/2011/06/parsing-quick-guide-to-strtok-there-are.html
//this whole test section works and returns torque of 5.230000
  char test_message[] = "6.23,4.56,7.89";
  char* command;
//  command = strtok(test_message, ",");
//  command = strtok(inbound_message, ",");//explodes
  command = strtok(miscMsgs2, ",");//works
  float test_float_cmd;
  joint_commands_[0] = atof(command);
  torque.data = joint_commands_[0];

  
  
  nh.loginfo("command=");
  nh.loginfo(command);
  joint_commands_[0] = atof(command);

      char result2[8]; // Buffer big enough for 7-character float
      dtostrf(joint_commands_[0], 6, 2, result2); // Leave room for too large numbers!
      nh.loginfo("joint_commands_[0]=");//this works
      nh.loginfo(result2);//this works

  //debug pub to state2 topic 
    miscMsgs3[0] = (char)0;  //"empties msg by setting first char in string to 0
    strcat(miscMsgs3, command);
//    strcat(miscMsgs3, inbound_message);
    state2.data = miscMsgs3;

  int joint_index = 1;//note starts at 1 because BR joint has already been parsed as j0int_index=0
  while (command != 0)
  {
    command = strtok(0, ",");//note "0" i.e. null as input for subsequent calls to strtok()
    joint_commands_[joint_index] = atof(command);
    joint_index++;
  }

  //apply to only this joint
  switch (minion_ident) {
    case SL_IDENT:
      joint_index = 1;
      commanded_position = joint_commands_[joint_index];
      
      char result[8]; // Buffer big enough for 7-character float
      dtostrf(commanded_position, 6, 2, result); // Leave room for too large numbers!
      nh.loginfo("commanded_position=");//this works
      nh.loginfo(result);//this works
      
      break;
    case UR_IDENT:
      joint_index = 2;
      commanded_position = joint_commands_[joint_index];
      sprintf(miscMsgs, "UR cmd %3.2f", commanded_position);
      nh.loginfo(miscMsgs);
      break;
    case EL_IDENT:
      joint_index = 3;
      commanded_position = joint_commands_[joint_index];
      sprintf(miscMsgs, "EL cmd %3.2f", commanded_position);
      nh.loginfo(miscMsgs);
      break;
    case LR_IDENT:
      joint_index = 4;
      commanded_position = joint_commands_[joint_index];
      sprintf(miscMsgs, "LR cmd %3.2f", commanded_position);
      nh.loginfo(miscMsgs);
      break;
    default:
      break;
  }//end switch case

}// end commandedPositionCallback()

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

void updateStatus() {
  //publish stuff for this joint
//  torque.data = commanded_position;
  SL_torque_pub.publish( &torque );
  SL_state_pub.publish( &state );
  switch (minion_ident) {
    case SL_IDENT:
      SL_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      SL_stepper_count_pos_pub.publish( &stepper_count_pos );
      SL_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      SL_torque_pub.publish( &torque );
      //      SL_joint_state_pub.publish( &BR_joint_state );
      SL_state_pub.publish( &state );
      SL_state2_pub.publish( &state2 );
      break;
    case UR_IDENT:
      UR_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      UR_stepper_count_pos_pub.publish( &stepper_count_pos );
      UR_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      UR_torque_pub.publish( &torque );
      //      UR_joint_state_pub.publish( &BR_joint_state );
      UR_state_pub.publish( &state );
      break;
    case EL_IDENT:
      EL_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      EL_stepper_count_pos_pub.publish( &stepper_count_pos );
      EL_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      EL_torque_pub.publish( &torque );
      //      EL_joint_state_pub.publish( &BR_joint_state );
      EL_state_pub.publish( &state );
      break;
    case LR_IDENT:
      LR_joint_encoder_pos_pub.publish( &joint_encoder_pos );
      LR_stepper_count_pos_pub.publish( &stepper_count_pos );
      LR_stepper_encoder_pos_pub.publish( &stepper_encoder_pos );
      LR_torque_pub.publish( &torque );
      //      LR_joint_state_pub.publish( &BR_joint_state );
      LR_state_pub.publish( &state );
      break;
    default:
      break;
  }//end switch case
}//end updateStatus()

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

  //setup publishers and subscribers
  //  nh.getHardware()->setBaud(230400);  //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  nh.getHardware()->setBaud(115200);  //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  //  nh.getHardware()->setBaud(57600);  //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  nh.initNode();
  nh.subscribe(commanded_joint_positions_sub);

  //use swtich to only setup pubs and subs needed for this minion's joint
  switch (minion_ident) {
    case SL_IDENT:
      // SL joint
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
      nh.advertise(LR_joint_encoder_pos_pub);
      nh.advertise(LR_stepper_count_pos_pub);
      nh.advertise(LR_stepper_encoder_pos_pub);
      nh.advertise(LR_torque_pub);
      nh.advertise(LR_state_pub);
      //      nh.sscribe(LR_plan_sub);
      nh.loginfo("SetupLR");
      break;
    default:
      break;
  }//end switch case

  nh.loginfo("V0.0.16");

  nh.spinOnce();
}

void loop() {
  //determine motion needed
  //  if (new_plan)     {
  //    planMovement(commanded_position);
  //    new_plan = false;
  //  }

  //log updates
  if (millis() > next_update) {
    updateStatus();
    next_update = millis() + UPDATE_INTERVAL;
  }

  nh.spinOnce();
}
