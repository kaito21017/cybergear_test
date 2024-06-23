#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include "cybergear_driver.hh"
#include <cybergear_can_interface_Teensy4.hh>

uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID = 0x7E;

// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
CybergearDriver driver = CybergearDriver(MASTER_CAN_ID, MOT_CAN_ID);
// CybergearController controller = CybergearController(MASTER_CAN_ID);

MotorStatus motor_status;

CybergearCanInterfaceTeensy interface;
CAN_message_t msg;



#define INC_POSITION  20.0
#define INC_VELOCITY  0.4
#define INC_TORQUE    0.04

// CAN_message_t msg;
uint8_t mode = MODE_POSITION;   //!< current mode
float target_pos = 0.0;         //!< motor target position
float target_vel = 0.0;         //!< motor target velocity
float target_torque = 0.0;      //!< motor target torque
float dir = 1.0f;               //!< direction for motion mode
float default_kp = 50.0f;       //!< default kp for motion mode
float default_kd = 1.0f;        //!< default kd for motion mode
float init_speed = 30.0f;       //!< initial speed
float slow_speed = 1.0f;        //!< slow speed
bool state_change_flag = false; //!< state change flag


void setup(void) {
  Serial.begin(115200); delay(400);
//   pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  // can1.begin();
  // can1.setBaudRate(1000000);
//   // can1.setMaxMB(16);
//   // can1.onReceive(canSniff);
//   // can1.mailboxStatus();
  // can2.begin();
  // can2.setBaudRate(1000000);

  can3.begin();
  can3.setBaudRate(1000000);
  interface.init();
  driver.init(&interface);
  driver.init_motor(mode);
  driver.set_limit_speed(init_speed);
  driver.enable_motor();
}


void loop() {
  // can1.events();
  // can2.events();
  // can3.events();
  String str = Serial.readString();
  if(str == "c") {
    // mode = (mode + 1) % MODE_CURRENT + 1;
    mode = MODE_SPEED;
    state_change_flag = true;
    driver.init_motor(mode);
    driver.enable_motor();
    target_pos = 0.0;
    target_vel = 0.0;
    target_torque = 0.0;
    // Serial.println(mode);
    // print_can_packet();
    // draw_display(mode, true);

  } else if (str == "d") {
    if (mode == MODE_POSITION) {
      target_pos += INC_POSITION / 180.0f * M_PI;
      Serial.print("target_pos:");
      Serial.println(target_pos);

    } else if (mode == MODE_SPEED) {
      target_vel += INC_VELOCITY;
      driver.set_speed_ref(target_vel);
      Serial.print("target_vel:");
      Serial.println(target_vel);

    } else if (mode == MODE_CURRENT) {
      target_torque += INC_TORQUE;
      Serial.print("target_torque:");
      Serial.println(target_torque);
    }
   

  } else if (str == "a") {
    if (mode == MODE_POSITION) {
      target_pos -= INC_POSITION / 180.0f * M_PI;
      Serial.print("target_pos:");
       Serial.println(target_pos);
       Serial.print("current_pos:");
       Serial.println(motor_status.position);


    } else if (mode == MODE_SPEED) {
      target_vel -= INC_VELOCITY;
      driver.set_speed_ref(target_vel);
      Serial.print("target_vel:");
       Serial.println(target_vel);

    } else if (mode == MODE_CURRENT) {
      target_torque -= INC_TORQUE;
      Serial.print("target_torque:");
      Serial.println(target_torque);
    }
  }

  // if (driver.get_run_mode() == MODE_POSITION) {
  //   // set limit speed when state changed
  //   if (state_change_flag) {
  //     driver.set_limit_speed(slow_speed);
  //     state_change_flag = false;
  //   }
  //   if (std::fabs(motor_status.position - target_pos) < 10.0 / 180.0 * M_PI) {
  //     driver.set_limit_speed(init_speed);
  //   }

  //   driver.set_position_ref(target_pos);
  //   // Serial.println(target_torque);
  // }
  // else if (driver.get_run_mode() == MODE_SPEED) {
  //   driver.set_speed_ref(target_vel);
  //   Serial.println("speed_mode");
  // }
  // else if (driver.get_run_mode() == MODE_CURRENT) {
  //   driver.set_current_ref(target_torque);
  // // }
  // else {
  //   target_pos += dir * 10.0 / 180.0 * M_PI;
  //   if (target_pos > P_MAX) { dir = -1.0; target_pos = P_MAX; }
  //   else if (target_pos < P_MIN) { dir = 1.0; target_pos = P_MIN; }
  //   driver.motor_control(target_pos, dir * target_vel, dir * target_torque, default_kd, default_kd);
  // }

  // update and get motor data
  if ( driver.process_packet()) {
    motor_status = driver.get_motor_status();
    Serial.println(mode);
  }

  delay(2);
  // if ( can1.read(msg) ) {
  //   Serial.print("CAN1 "); 
  //   Serial.print("MB: "); Serial.print(msg.mb);
  //   Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
  //   Serial.print("  EXT: "); Serial.print(msg.flags.extended );
  //   Serial.print("  LEN: "); Serial.print(msg.len);
  //   Serial.print(" DATA: ");
  //   for ( uint8_t i = 0; i < 8; i++ ) {
  //     Serial.print(msg.buf[i]); Serial.print(" ");
  //   }
  //   Serial.print("  TS: "); Serial.println(msg.timestamp);
  // }

  // if ( can2.read(msg) ) {
  //   Serial.print("CAN2 "); 
  //   Serial.print("MB: "); Serial.print(msg.mb);
  //   Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
  //   Serial.print("  EXT: "); Serial.print(msg.flags.extended );
  //   Serial.print("  LEN: "); Serial.print(msg.len);
  //   Serial.print(" DATA: ");
  //   for ( uint8_t i = 0; i < 8; i++ ) {
  //     Serial.print(msg.buf[i]); Serial.print(" ");
  //   }
  //   Serial.print("  TS: "); Serial.println(msg.timestamp);
  // }

  if ( can3.read(msg) ) {
    Serial.print("CAN3 "); 
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, DEC);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended );
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for ( uint8_t i = 0; i < 8; i++ ) {
      Serial.print(msg.buf[i]); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);
  }

  

  // if(Serial.read()>0){
  //   CAN_message_t test;
  //   test.id = random(0x1,0x7FE);
  //   for ( uint8_t i = 0; i < 8; i++ ) test.buf[i] = i + 1;
  //   can1.write(test);
  //   Serial.print("CAN1test ");
  // }
  
}

