#include <Arduino.h>

#define PWM_PIN 5   // Sends PWM signal to control lift speed
#define CNT_1 13    // Used to control direction of lift
#define CNT_2 12    // Used to control direction of lift
#define ENC1 4      // Encoder 1 in lift

#define UP_SPEED 100    // Range from 0 to 255. Above 125 not recommended due to high current
#define DOWN_SPEED 60  

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
  SAVED_LAST_HEIGHT = 97,   // a
  SAVED_MAX_HEIGHT,         // b
  MOVE_DOWN,                // c
  MOVE_UP,                  // d
  STOP_MOVE,                // e
  CALIBRATE,                // f
  ROS_MOVE                  // g
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

// Flags to ensure status messages are only sent back to the ROS node once
bool current_limit_warning_sent = false;

// Needed for to properly use ISR with ESP8266
void IRAM_ATTR encoder_isr();


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
  pwm = DOWN_SPEED;
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
  pwm = UP_SPEED;
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

    if (dir_signal == Signal::DOWN) {
        // Encoder treated as going down
        cur_height--;

        // Prevent checks using max and min height during calibration
        // if (calib_state == CalibrationState::CALIB_DOWN || calib_state == CalibrationState::RESETTING_CALIB) {
        //     return;
        // }  

        // bool should_consider_goal = 
        //     ros_cmd_state == ROSCommandState::RUNNING &&
        //     calib_state == CalibrationState::NO_CALIB &&
        //     cur_height == goal;

        // if (cur_height < min_height || should_consider_goal) {
        //     // Stop lift from going any lower
        //     stopMotor();
        //     cur_height = max(min_height, cur_height);
        //     calib_state = CalibrationState::NO_CALIB;
            
        //     if (should_consider_goal) {
        //         ros_cmd_state = ROSCommandState::COMPLETED;
        //     }
        // }

    } else if (dir_signal == Signal::UP) {
        // Encoder treated as going up
        cur_height++;

        // Prevent checks using max and min height during calibration
        // if (calib_state == CalibrationState::CALIB_UP) {
        //     return;
        // }

        // bool should_consider_goal = 
        //     ros_cmd_state == ROSCommandState::RUNNING &&
        //     calib_state == CalibrationState::NO_CALIB &&
        //     cur_height == goal;
        
        // if (cur_height > max_height || should_consider_goal) {
        //     // Stop lift from going higher
        //     stopMotor();
        //     cur_height = min(max_height, cur_height);
            
        //     if (should_consider_goal) {
        //         ros_cmd_state = ROSCommandState::COMPLETED;
        //     }
        // }
    } else {
        // Serial.println("xERROR: Encoder ISR called while lift is not moving");
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
  pinMode(PWM_PIN, OUTPUT);
  pinMode(CNT_1, OUTPUT);
  pinMode(CNT_2, OUTPUT);
  
  // Setup interrupt service routine
  attachInterrupt(digitalPinToInterrupt(ENC1), encoder_isr, RISING);
  Serial.begin(115200);

  Serial.println("xLift booting up");
}


void handleROSInput(float raw_goal) {
  // Limiting raw_goal to 1 maximum
  raw_goal = min(1.0f, raw_goal);
  
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


void loop() {
    // Handle input from Serial port
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        handleSerialInput(input);
    }
    
    // If we have finished a movement sent out by ROS, inform the lift controller
    // ROS node that the movement is complete
    if (ros_cmd_state == ROSCommandState::COMPLETED) {
        Serial.println((char)Arduino2NodeCommand::ROS_CMD_COMPLETE);
        ros_cmd_state = ROSCommandState::NO_COMMAND;
    }
    
    if (dir_signal != Signal::STOP) {
        // If the lift is moving, check if it has been a while since the last response
        // If so, stop the lift and send a warning to the ROS node
        if (millis() - last_resp_time > 500) {
            Serial.println("xERROR: Lift not responding, stopping motor");
            stopMotor();
            Serial.println((char)Arduino2NodeCommand::CURRENT_LIMIT_TRIGGERED);
            // current_limit_warning_sent = true;
        }
        
        // else if (current_limit_warning_sent) {
        //     // If we have sent a warning before, we can now send an untriggered message
        //     Serial.println((char)Arduino2NodeCommand::CURRENT_LIMIT_UNTRIGGERED);
        //     current_limit_warning_sent = false;
        // }
    }

    motorControl(); // Execute motor control signal sending
}
