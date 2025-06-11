// #include <Arduino.h>

// #define PWM_PIN 5   // Sends PWM signal to control lift speed
// #define CNT_1 12    // Used to control direction of lift
// #define CNT_2 13    // Used to control direction of lift
// #define ENC1 2      // Encoder 1 in lift
// #define ENC2 7      // Encoder 2 in lift

// #define BASE_SPEED 100    // Range from 0 to 255. Above 200 not recommended due to high current

// volatile float max_height = 0;   // Lower than actual encoder max ticks
// volatile float min_height = 0;   // Cannot be below 0
// volatile float cur_height = 0;

// volatile bool height_changed = false;
// volatile unsigned long last_resp_time;

// bool dir_changed = false;
// unsigned long start_dir_change_time;

// volatile int pwm = 0;

// // Enumerates commands/communication receivable by the lift
// enum Node2ArduinoCommand {
//   SAVED_LAST_HEIGHT = 97,
//   SAVED_MAX_HEIGHT,
//   MOVE_DOWN,
//   MOVE_UP,
//   STOP_MOVE,
//   CALIBRATE,
//   ROS_MOVE
// };

// // Enumerates commands/communication that can be sent by the lift
// enum Arduino2NodeCommand {
//   OPTICAL_SENSOR_BLOCKED = 97,  // a
//   OPTICAL_SENSOR_UNBLOCKED,     // b
//   CURRENT_LIMIT_TRIGGERED,      // c
//   CURRENT_LIMIT_UNTRIGGERED,    // d
//   ROS_CMD_COMPLETE,             // e
//   CUR_HEIGHT,                   // f
//   MAX_HEIGHT,                   // g
//   GOAL_HEIGHT,                  // h
//   OPTICAL_SENSOR_OFF,           // i
//   END_CALIBRATION               // j
// };

// // Defines the signal sent to the lift, should it go up or go down or stop
// enum Signal {
//   DOWN, UP, STOP
// };
// volatile Signal dir_signal = Signal::STOP;

// enum ROSCommandState {
//   NO_COMMAND,
//   RUNNING,
//   COMPLETED
// };
// volatile ROSCommandState ros_cmd_state = ROSCommandState::NO_COMMAND;

// enum CalibrationState {
//   NO_CALIB,
//   CALIB_DOWN,
//   CALIB_UP,
//   RESETTING_CALIB
// };
// volatile CalibrationState calib_state = CalibrationState::NO_CALIB;

// int goal = 0;

// // Overcurrent detection 
// int current_draw_avg_counter = 0;
// int motor_current_draw = 0;

// // Flags to ensure status messages are only sent back to the ROS node once
// bool optical_sensor_update_sent = false;
// // bool optical_sensor_off_sent = false;
// bool current_limit_warning_sent = false;


// // Lift keeps moving down continuously
// // Check for reaching minimum height or lowest physical height is done in encoder_isr()
// // If the lift reaches its lowest physical height, this function behaves equivalent to downUntilReset();
// void continuousDown() {
//   // Handle case where lift starts/changes direction and there is a momentary
//   // current spike. In this we want to skip the overcurrent
//   // detection check. 
//   if (dir_signal != Signal::DOWN) {
//     dir_changed = true;
//     start_dir_change_time = millis();
//   }

//   dir_signal = Signal::DOWN;
//   pwm = BASE_SPEED;
//   last_resp_time = millis();
// }


// // Lift keeps moving up continuously
// // Check for reaching maximum height is done in encoder_isr()
// void continuousUp() {
//   if (dir_signal != Signal::UP) {
//     dir_changed = true;
//     start_dir_change_time = millis();
//   }

//   dir_signal = Signal::UP;
//   pwm = BASE_SPEED;
//   last_resp_time = millis();
// }


// void stopMotor() {
//   dir_signal = Signal::STOP;
//   pwm = 0;
//   last_resp_time = millis();
// }


// // Interrupt service triggered when ENCODER_1 pin rises
// // Used to update how far the lift has moved
// void encoder_isr() {
//     last_resp_time = millis();
//     height_changed = true;
//     int enc_val = digitalRead(ENC2);

//     if (!enc_val) {
//       // Handle case where encoder values lag behind newly sent
//       // command in opposite direction
//       if (dir_signal == Signal::DOWN) {
//         return;
//       }
      
//       // Encoder reports going up
//       cur_height++;

//       // Prevent checks using max and min height during calibration
//       if (calib_state == CalibrationState::CALIB_UP) {
//         return;
//       }

//       bool should_consider_goal = 
//         ros_cmd_state == ROSCommandState::RUNNING &&
//         calib_state == CalibrationState::NO_CALIB &&
//         cur_height == goal;
    
//       if (cur_height > max_height || should_consider_goal) {
//         // Stop lift from going higher
//         stopMotor();
//         cur_height = min(max_height, cur_height);
        
//         if (should_consider_goal) {
//           ros_cmd_state = ROSCommandState::COMPLETED;
//         }
//       }

//     } else {
//       // Handle case where encoder values lag behind newly sent
//       // command in opposite direction
//       if (dir_signal == Signal::UP) {
//         return;
//       }

//       // Encoder reports going down
//       cur_height--;

//       // Prevent checks using max and min height during calibration
//       if (calib_state == CalibrationState::CALIB_DOWN || calib_state == CalibrationState::RESETTING_CALIB) {
//         return;
//       }  

//       bool should_consider_goal = 
//         ros_cmd_state == ROSCommandState::RUNNING &&
//         calib_state == CalibrationState::NO_CALIB &&
//         cur_height == goal;

//       if (cur_height < min_height || should_consider_goal) {
//         // Stop lift from going any lower
//         stopMotor();
//         cur_height = max(min_height, cur_height);
//         calib_state = CalibrationState::NO_CALIB;
        
//         if (should_consider_goal) {
//           ros_cmd_state = ROSCommandState::COMPLETED;
//         }
//       }
//     }
// }


// // Function that actually controls the motor based on the set params from the other funcs
// void motorControl() {
//   // Write the directional signals to the IN1 and IN2 pins on the motor controller
//   if (dir_signal == Signal::UP) {
//     digitalWrite(CNT_1, HIGH);
//     digitalWrite(CNT_2, LOW);
//   } else if (dir_signal == Signal::DOWN) {
//     digitalWrite(CNT_1, LOW);
//     digitalWrite(CNT_2, HIGH);
//   } else {
//     digitalWrite(CNT_1, LOW);
//     digitalWrite(CNT_2, LOW);
//   }

//   // Send speed signal to the motor controller via PWM
//   analogWrite(PWM_PIN, pwm);

//   // Don't send back current value during down calibration since this value could be wrong
//   if (calib_state == CalibrationState::CALIB_DOWN) {
//     return;
//   }

//   // Output the current height value when it changes
//   if (height_changed) {
//     height_changed = false;
//     Serial.print((char)Arduino2NodeCommand::CUR_HEIGHT);
//     Serial.println(cur_height);  
//   }
// }


// void setup() {
// //   pinMode(ENC1, INPUT);
// //   pinMode(ENC2, INPUT);
//   pinMode(PWM_PIN, OUTPUT);
//   pinMode(CNT_1, OUTPUT);
//   pinMode(CNT_2, OUTPUT);
  
//   // Setup interrupt service routine
//   attachInterrupt(digitalPinToInterrupt(ENC1), encoder_isr, RISING);
//   Serial.begin(115200);

//   Serial.println("xLift booting up");
// }


// void handleROSInput(float raw_goal) {
//   // Limiting raw_goal to 1 maximum
//   raw_goal = min(1.0f, raw_goal);
  
//   // Getting goal relative to the current min and maximum height setting
//   goal = ((max_height - min_height) * raw_goal) + min_height;
//   // Send this parsed goal height back to ROS
//   Serial.print((char)Arduino2NodeCommand::GOAL_HEIGHT);
//   Serial.println(goal);
  
//   if (goal > cur_height) {
//     continuousUp();
//   } else if (goal < cur_height) {
//     continuousDown();
//   } else {
//     stopMotor();
//     ros_cmd_state = ROSCommandState::COMPLETED;
//   }
// }


// void handleSerialInput(String input) {
//   int cmd = input[0];

//   calib_state = CalibrationState::NO_CALIB;
//   ros_cmd_state = ROSCommandState::NO_COMMAND;

//   // ROS node sending last saved height to controller
//   if (cmd == Node2ArduinoCommand::SAVED_LAST_HEIGHT) {  
//     cur_height = input.substring(1).toInt();
  
//   // ROS node sending max saved height to controller
//   } else if (cmd == Node2ArduinoCommand::SAVED_MAX_HEIGHT) {
//     max_height = input.substring(1).toInt();

//   } else if (cmd == Node2ArduinoCommand::MOVE_DOWN) {
//     continuousDown();

//   } else if (cmd == Node2ArduinoCommand::MOVE_UP) {
//     continuousUp();
  
//   } else if (cmd == Node2ArduinoCommand::STOP_MOVE) {
//     stopMotor();
  
//   } else if (cmd == Node2ArduinoCommand::CALIBRATE) {
//     calib_state = CalibrationState::CALIB_DOWN;
//     Serial.println("xStarting calibration");
//     // Start by moving lift down until we hit the optical sensor
//     continuousDown();      

//   } else if (cmd == Node2ArduinoCommand::ROS_MOVE) {
//     ros_cmd_state = ROSCommandState::RUNNING;
//     float raw_goal = input.substring(1).toFloat();
//     handleROSInput(raw_goal);
  
//   } else {
//     Serial.println("xUNKNOWN SERIAL INPUT");
//   }
// }


// void loop() {
//   // Handle input from Serial port
//   if (Serial.available()) {
//     String input = Serial.readStringUntil('\n');
//     handleSerialInput(input);
//   }
  
//   // If we have finished a movement sent out by ROS, inform the lift controller
//   // ROS node that the movement is complete
//   if (ros_cmd_state == ROSCommandState::COMPLETED) {
//     Serial.println((char)Arduino2NodeCommand::ROS_CMD_COMPLETE);
//     ros_cmd_state = ROSCommandState::NO_COMMAND;
//   }
  
//   motorControl(); // Execute motor control signal sending
// }
