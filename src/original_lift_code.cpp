// Use a rosservice to send a command to the lift_node, and receive a status back from it.
// 

// If I start the lift_node and lift_height.save file is blank or negative, current height is unknown
// so prompt user to do the height reset routine. Do not move the lift until the user performs reset.
  // But if the optical sensor is already being triggered, height is 0. So we can continue
  // In this case however, still give out a warning of

// If lift_height.save file is not blank, send this height to the Arduino.
  // If Arduino reports the optical sensor is being triggered, set height to 0 and give user this
  // information. 



// Issue where outputs go to negative when lift is moving up. Could this be because of how heavy
// the load is?

#include <Arduino.h>

#define PWM_PIN 5 // Sends PWM signal to control lift speed
#define CNT_1 8   // Used to control direction of lift
#define CNT_2 9   // Used to control direction of lift
#define ENC1 2    // Encoder 1 in lift
#define ENC2 7    // Encoder 2 in lift
#define CURRENT_SENSE_PIN A0

#define OPTIC_INPUT_NC 3
#define OPTIC_INPUT_NO 4
#define BLOCK_LED_PIN 13

#define BASE_SPEED 200    // Range from 0 to 255. Above 200 not recommended due to high current
#define DIR_CHANGE_THRESH 500

// This corresponds to a voltage of about 0.1V. If this is detected, lift is stopped to prevent overload
// There is also some margin included for a current spike in the beginning of movement
#define CURRENT_LIMIT 60
#define MAX_HEIGHT_MARGIN 100

volatile float max_height = 0;   // Lower than actual encoder max ticks
volatile float min_height = 0;   // Cannot be below 0
volatile float cur_height = 0;

volatile bool height_changed = false;
volatile unsigned long last_resp_time;

bool dir_changed = false;
unsigned long start_dir_change_time;

volatile int pwm = 0;

// Enumerates commands/communication receivable by the lift
enum Node2ArduinoCommand {
  SAVED_LAST_HEIGHT = 97,
  SAVED_MAX_HEIGHT,
  MOVE_DOWN,
  MOVE_UP,
  STOP_MOVE,
  CALIBRATE,
  ROS_MOVE
};

// Enumerates commands/communication that can be sent by the lift
enum Arduino2NodeCommand {
  OPTICAL_SENSOR_BLOCKED = 97,  // a
  OPTICAL_SENSOR_UNBLOCKED,     // b
  CURRENT_LIMIT_TRIGGERED,      // c
  CURRENT_LIMIT_UNTRIGGERED,    // d
  ROS_CMD_COMPLETE,             // e
  CUR_HEIGHT,                   // f
  MAX_HEIGHT,                   // g
  GOAL_HEIGHT,                  // h
  OPTICAL_SENSOR_OFF,           // i
  END_CALIBRATION               // j
};

// Defines the signal sent to the lift, should it go up or go down or stop
enum Signal {
  DOWN, UP, STOP
};
volatile Signal dir_signal = Signal::STOP;

enum ROSCommandState {
  NO_COMMAND,
  RUNNING,
  COMPLETED
};
volatile ROSCommandState ros_cmd_state = ROSCommandState::NO_COMMAND;

enum CalibrationState {
  NO_CALIB,
  CALIB_DOWN,
  CALIB_UP,
  RESETTING_CALIB
};
volatile CalibrationState calib_state = CalibrationState::NO_CALIB;

int goal = 0;

// Overcurrent detection 
int current_draw_avg_counter = 0;
int motor_current_draw = 0;

// Flags to ensure status messages are only sent back to the ROS node once
bool optical_sensor_update_sent = false;
// bool optical_sensor_off_sent = false;
bool current_limit_warning_sent = false;


// Lift keeps moving down continuously
// Check for reaching minimum height or lowest physical height is done in encoder_isr()
// If the lift reaches its lowest physical height, this function behaves equivalent to downUntilReset();
void continuousDown() {
  // Handle case where lift starts/changes direction and there is a momentary
  // current spike. In this we want to skip the overcurrent
  // detection check. 
  if (dir_signal != Signal::DOWN) {
    dir_changed = true;
    start_dir_change_time = millis();
  }

  dir_signal = Signal::DOWN;
  pwm = BASE_SPEED;
  last_resp_time = millis();
}


// Lift keeps moving up continuously
// Check for reaching maximum height is done in encoder_isr()
void continuousUp() {
  if (dir_signal != Signal::UP) {
    dir_changed = true;
    start_dir_change_time = millis();
  }

  dir_signal = Signal::UP;
  pwm = BASE_SPEED;
  last_resp_time = millis();
}


void stopMotor() {
  dir_signal = Signal::STOP;
  pwm = 0;
  last_resp_time = millis();
}


// Interrupt service triggered when ENCODER_1 pin rises
// Used to update how far the lift has moved
void encoder_isr() {
    last_resp_time = millis();
    height_changed = true;
    int enc_val = digitalRead(ENC2);

    if (!enc_val) {
      // Handle case where encoder values lag behind newly sent
      // command in opposite direction
      if (dir_signal == Signal::DOWN) {
        return;
      }
      
      // Encoder reports going up
      cur_height++;

      // Prevent checks using max and min height during calibration
      if (calib_state == CalibrationState::CALIB_UP) {
        return;
      }

      bool should_consider_goal = 
        ros_cmd_state == ROSCommandState::RUNNING &&
        calib_state == CalibrationState::NO_CALIB &&
        cur_height == goal;
    
      if (cur_height > max_height || should_consider_goal) {
        // Stop lift from going higher
        stopMotor();
        cur_height = min(max_height, cur_height);
        
        if (should_consider_goal) {
          ros_cmd_state = ROSCommandState::COMPLETED;
        }
      }

    } else {
      // Handle case where encoder values lag behind newly sent
      // command in opposite direction
      if (dir_signal == Signal::UP) {
        return;
      }

      // Encoder reports going down
      cur_height--;

      // Prevent checks using max and min height during calibration
      if (calib_state == CalibrationState::CALIB_DOWN || calib_state == CalibrationState::RESETTING_CALIB) {
        return;
      }  

      bool should_consider_goal = 
        ros_cmd_state == ROSCommandState::RUNNING &&
        calib_state == CalibrationState::NO_CALIB &&
        cur_height == goal;

      if (cur_height < min_height || should_consider_goal) {
        // Stop lift from going any lower
        stopMotor();
        cur_height = max(min_height, cur_height);
        calib_state = CalibrationState::NO_CALIB;
        
        if (should_consider_goal) {
          ros_cmd_state = ROSCommandState::COMPLETED;
        }
      }
    }
}


// Function that actually controls the motor based on the set params from the other funcs
void motorControl() {
  // Write the directional signals to the IN1 and IN2 pins on the motor controller
  if (dir_signal == Signal::UP) {
    digitalWrite(CNT_1, HIGH);
    digitalWrite(CNT_2, LOW);
  } else if (dir_signal == Signal::DOWN) {
    digitalWrite(CNT_1, LOW);
    digitalWrite(CNT_2, HIGH);
  } else {
    digitalWrite(CNT_1, LOW);
    digitalWrite(CNT_2, LOW);
  }

  // Send speed signal to the motor controller via PWM
  analogWrite(PWM_PIN, pwm);

  // Don't send back current value during down calibration since this value could be wrong
  if (calib_state == CalibrationState::CALIB_DOWN) {
    return;
  }

  // Output the current height value when it changes
  if (height_changed) {
    height_changed = false;
    Serial.print((char)Arduino2NodeCommand::CUR_HEIGHT);
    Serial.println(cur_height);  
  }
}


void setup() {
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(CNT_1, OUTPUT);
  pinMode(CNT_2, OUTPUT);
  pinMode(CURRENT_SENSE_PIN, INPUT);

  // Setup optical sensor related pins
  pinMode(OPTIC_INPUT_NC, INPUT);
  pinMode(OPTIC_INPUT_NO, INPUT);
  pinMode(BLOCK_LED_PIN, OUTPUT);

  // Setup interrupt service routine
  attachInterrupt(digitalPinToInterrupt(ENC1), encoder_isr, RISING);
  Serial.begin(115200);

  Serial.println("xLift booting up");
}


void handleROSInput(float raw_goal) {
  // Limiting raw_goal to 1 maximum
  raw_goal = min(1, raw_goal);
  
  // Getting goal relative to the current min and maximum height setting
  goal = ((max_height - min_height) * raw_goal) + min_height;
  // Send this parsed goal height back to ROS
  Serial.print((char)Arduino2NodeCommand::GOAL_HEIGHT);
  Serial.println(goal);
  
  if (goal > cur_height) {
    continuousUp();
  } else if (goal < cur_height) {
    continuousDown();
  } else {
    stopMotor();
    ros_cmd_state = ROSCommandState::COMPLETED;
  }
}


void handleSerialInput(String input) {
  int cmd = input[0];

  calib_state = CalibrationState::NO_CALIB;
  ros_cmd_state = ROSCommandState::NO_COMMAND;

  // ROS node sending last saved height to controller
  if (cmd == Node2ArduinoCommand::SAVED_LAST_HEIGHT) {  
    cur_height = input.substring(1).toInt();
  
  // ROS node sending max saved height to controller
  } else if (cmd == Node2ArduinoCommand::SAVED_MAX_HEIGHT) {
    max_height = input.substring(1).toInt();

  } else if (cmd == Node2ArduinoCommand::MOVE_DOWN) {
    continuousDown();

  } else if (cmd == Node2ArduinoCommand::MOVE_UP) {
    continuousUp();
  
  } else if (cmd == Node2ArduinoCommand::STOP_MOVE) {
    stopMotor();
  
  } else if (cmd == Node2ArduinoCommand::CALIBRATE) {
    calib_state = CalibrationState::CALIB_DOWN;
    Serial.println("xStarting calibration");
    // Start by moving lift down until we hit the optical sensor
    continuousDown();      

  } else if (cmd == Node2ArduinoCommand::ROS_MOVE) {
    ros_cmd_state = ROSCommandState::RUNNING;
    float raw_goal = input.substring(1).toFloat();
    handleROSInput(raw_goal);
  
  } else {
    Serial.println("xUNKNOWN SERIAL INPUT");
  }
}


void checkOpticalSensor() {
  // Check if the optical sensor was triggered, this means lift
  // is definitely at the bottom
  bool nc_status = digitalRead(OPTIC_INPUT_NC);
  bool no_status = digitalRead(OPTIC_INPUT_NO);

  /*** Code below is disabled because during some direction changes or lift stops
   * I hypothesize that a current spike causes the NC and NO on the optical sensor to 
   * go to an invalid state for 1 frame. Not sure why this happens exactly.
   ***/
  // Optical sensor is switched off. Do not allow user to move the 
  // lift until the sensor is turned on again
  // if (nc_status == LOW && no_status == LOW) {
    //   Serial.println("xWarning! Optical sensor was detected to be off. Please verify");
    // if (!optical_sensor_off_sent) {
    //   Serial.println((char)Arduino2NodeCommand::OPTICAL_SENSOR_OFF);
    //   optical_sensor_off_sent = true;
    // }
    // stopMotor();

  // Impossible state. Give a critical warning in this case
  // } else if (nc_status == HIGH && no_status == HIGH) {
  //   Serial.println("Invalid State! Optical sensor NC & NO both HIGH");

  // Nothing is blocking the optical sensor
  if (nc_status == LOW && no_status == HIGH) {
    // optical_sensor_off_sent = false;
    
    // Inform via serial that lift just exited the optical sensor
    if (optical_sensor_update_sent) {
      optical_sensor_update_sent = false;
      digitalWrite(BLOCK_LED_PIN, LOW);
      Serial.println((char)Arduino2NodeCommand::OPTICAL_SENSOR_UNBLOCKED);
    }

  // Optical sensor is blocked
  } else { // if (nc_status == HIGH && no_status == LOW)
    // optical_sensor_off_sent = false;
    
    // Inform via serial that lift was reset back to 0
    if (!optical_sensor_update_sent) {
      optical_sensor_update_sent = true;
      digitalWrite(BLOCK_LED_PIN, HIGH);
      Serial.println((char)Arduino2NodeCommand::OPTICAL_SENSOR_BLOCKED);
    }

    // If calibration is being reset and the optical sensor is triggered
    // Set the calibration to no calib state
    if (calib_state == CalibrationState::RESETTING_CALIB) {
      Serial.println((char)Arduino2NodeCommand::END_CALIBRATION);
      calib_state = CalibrationState::NO_CALIB;
      Serial.println("xCalib ended");
    } 

    if (calib_state == CalibrationState::NO_CALIB) {
      // When moving up from 0, optical sensor would be normally triggered. 
      // We ignore such cases
      if (dir_signal != Signal::UP) {
        cur_height = 0;
        stopMotor();

        if (ros_cmd_state == ROSCommandState::RUNNING) {
          ros_cmd_state = ROSCommandState::COMPLETED;
        }
      }

    // Inside calibration
    } else {
      // Start moving the lift up
      calib_state = CalibrationState::CALIB_UP;
      min_height = 0;
      cur_height = 0;
      continuousUp();
    }
  }
}


void checkLiftCurrentDraw() {
  // If the motor current draw is too high, it is possible we are colliding with something
  // or have reached the physical limits. This acts as a nice failsafe in gthe case that 
  // the optical sensor and encoder data fails 
  motor_current_draw += analogRead(CURRENT_SENSE_PIN);
  if (current_draw_avg_counter++ % 10 == 0) {
    motor_current_draw /= 10;

    if (motor_current_draw >= CURRENT_LIMIT) {
      if (calib_state == CalibrationState::CALIB_UP) {
        // Set max_height with some margin
        max_height = cur_height - MAX_HEIGHT_MARGIN;
        Serial.print((char)Arduino2NodeCommand::MAX_HEIGHT);
        Serial.println(max_height);
        calib_state = CalibrationState::RESETTING_CALIB;
        Serial.println("xOvercurrent protection triggered during calibration");
        Serial.println("xEnding calibration mode");
        // Start moving down to reset the lift position
        continuousDown();

      } else {
        // If this function is triggered outside of calibration, it means
        // our lift has gone past the margin. Inform ROS of this failure
        // and request the user to re-calibrate the lift.
        // Ensures we only send the warning once
        if (!current_limit_warning_sent) {
          Serial.println((char)Arduino2NodeCommand::CURRENT_LIMIT_TRIGGERED);
          current_limit_warning_sent = true;
        }
        stopMotor();
      }

    } else {
      // When there is no more current_limit warning we only want to inform
      // the lift controller node once that the current is safe again
      if (current_limit_warning_sent) {
        Serial.println((char)Arduino2NodeCommand::CURRENT_LIMIT_UNTRIGGERED);
        current_limit_warning_sent = false;
      }
    }

    motor_current_draw = 0;
  }
}


void loop() {
  // Handle input from Serial port
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    handleSerialInput(input);
  }

  // Lift safety checks
  checkOpticalSensor();
  
  // Don't check for overcurrent right after a change of direction
  if (!dir_changed) {
    checkLiftCurrentDraw();
  } else {
    if (millis() - start_dir_change_time >= DIR_CHANGE_THRESH) {
      dir_changed = false;
    }
  }

  // If we have finished a movement sent out by ROS, inform the lift controller
  // ROS node that the movement is complete
  if (ros_cmd_state == ROSCommandState::COMPLETED) {
    Serial.println((char)Arduino2NodeCommand::ROS_CMD_COMPLETE);
    ros_cmd_state = ROSCommandState::NO_COMMAND;
  }
  
  motorControl(); // Execute motor control signal sending
}
