#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define PWM_PIN 5   // Sends PWM signal to control lift speed
#define CNT_1 13    // Used to control direction of lift
#define CNT_2 12    // Used to control direction of lift
#define LIFT_ENC1 4      // Encoder 1 in lift

#define LIGHT_ENC1 15
#define LIGHT_ENC2 2
#define LIGHT_CONTROL 14

// Remaining OUT GPIOs
// 0, 16

#define UP_SPEED 100    // Range from 0 to 255. Above 125 not recommended due to high current
#define DOWN_SPEED 60  

// The encoder when going down seems to be much more sensitive than when going up
// The scale seems to be around 1500 == 31117
volatile float max_height = 1500;   // Lower than actual encoder max ticks 1500 seems right.
volatile float min_height = 0;      // Cannot be below 0
volatile float cur_height = 0;

volatile bool height_changed = false;
volatile unsigned long last_resp_time;

bool dir_changed = false;
unsigned long start_dir_change_time;

volatile int pwm = 0;
bool light_toggled = false;

// Enumerates commands/communication receivable by the lift
enum Node2ArduinoCommand {
  SAVED_LAST_HEIGHT = 97,   // a
  SAVED_MAX_HEIGHT,         // b
  MOVE_DOWN,                // c
  MOVE_UP,                  // d
  STOP_MOVE,                // e
  CALIBRATE,                // f
  ROS_MOVE,                 // g
  LIGHT_TOGGLE,             // h
  LIGHT_BRIGHT_UP,          // i
  LIGHT_BRIGHT_DOWN,        // j
  LIGHT_TEMP_UP,            // k
  LIGHT_TEMP_DOWN           // l
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

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
const char* html_page = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>Lift Controller</title></head>
<body>
  <h2>Lift Control Panel</h2>
  <button onclick="ws.send('up')">Up</button>
  <button onclick="ws.send('down')">Down</button>
  <button onclick="ws.send('stop')">Stop</button>

  <h2>Light Controls</h2>
  <button onclick="ws.send('light:toggle')">Toggle Light</button>
  <button onclick="ws.send('light:bright_up')">Brightness +</button>
  <button onclick="ws.send('light:bright_down')">Brightness -</button>
  <button onclick="ws.send('light:temp_up')">Temp +</button>
  <button onclick="ws.send('light:temp_down')">Temp -</button>

  <p id="status">Status: --</p>
<script>
let ws = new WebSocket(`ws://${location.hostname}/ws`);

ws.onopen = () => console.log("WebSocket open");
ws.onmessage = (e) => {
  document.getElementById("status").textContent = "Status: " + e.data;
  console.log("Server:", e.data);
};
</script>
</body>
</html>
)rawliteral";


// Needed for to properly use ISR with ESP8266
void IRAM_ATTR encoderISR();


// Lift keeps moving down continuously
// Check for reaching minimum height or lowest physical height is done in encoderISR()
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
// Check for reaching maximum height is done in encoderISR()
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
void encoderISR() {
    last_resp_time = millis();
    height_changed = true;

    if (dir_signal == Signal::DOWN) {
        // Encoder treated as going down
        cur_height--;

    } else if (dir_signal == Signal::UP) {
        // Encoder treated as going up
        cur_height++;
        
        if (cur_height > max_height) {
            // Stop lift from going higher
            stopMotor();
            cur_height = min(max_height, cur_height);
            ros_cmd_state = ROSCommandState::COMPLETED;
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


void lightToggle() {
  digitalWrite(LIGHT_CONTROL, LOW);
  delay(100);
  digitalWrite(LIGHT_CONTROL, HIGH);
}


void lightBrightUp() {
  digitalWrite(LIGHT_ENC1, HIGH);
  digitalWrite(LIGHT_ENC2, LOW);
  delay(10);
  digitalWrite(LIGHT_ENC1, LOW);
}


void lightBrightDown() {
  digitalWrite(LIGHT_ENC1, LOW);
  digitalWrite(LIGHT_ENC2, HIGH);
  delay(10);
  digitalWrite(LIGHT_ENC2, LOW);
}


void lightTempUp() {
  digitalWrite(LIGHT_CONTROL, LOW);
  delay(100);
  lightBrightUp();
  delay(100);
  digitalWrite(LIGHT_CONTROL, HIGH);
}


void lightTempDown() {
  digitalWrite(LIGHT_CONTROL, LOW);
  delay(100);
  lightBrightDown();
  delay(100);
  digitalWrite(LIGHT_CONTROL, HIGH);
}


void onWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;

  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    String msg = "";
    for (size_t i = 0; i < len; i++) {
      msg += (char)data[i];  // Build the String manually
    }
    Serial.println("WS Message: " + msg);

    if (msg == "up") {
      continuousUp();
    } else if (msg == "down") {
      continuousDown();
    } else if (msg == "stop") {
      stopMotor();
    } else if (msg == "light:toggle") {
      lightToggle();
    } else if (msg == "light:bright_up") {
      lightBrightUp();
    } else if (msg == "light:bright_down") {
      lightBrightDown();
    } else if (msg == "light:temp_up") {
      lightTempUp();
    } else if (msg == "light:temp_down") {
      lightTempDown();
    }
    // else if (msg.startsWith("goal:")) {
    //   float val = msg.substring(5).toFloat();
    //   handleROSInput(val);
    // }

    // Send back confirmation
    ws.textAll("ack:" + msg);
  }
}


void setupServer() {
  // Set your desired static IP, gateway, and subnet
  IPAddress local_IP(192, 168, 18, 42);      // IP you want for the ESP
  IPAddress gateway(192, 168, 18, 1);        // Usually your router's IP
  IPAddress subnet(255, 255, 255, 0);       // Standard for home networks
  IPAddress dns(8, 8, 8, 8);                // Optional: Google DNS
  
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("WiFi failed to configure");
    return;
  }
  WiFi.begin("TheMatrix", "asdasdasd");
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", html_page);  // Serve inline HTML
  });

  ws.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client,
                AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      Serial.println("WebSocket client connected");
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.println("WebSocket client disconnected");
    } else if (type == WS_EVT_DATA) {
      onWebSocketMessage(arg, data, len);
    }
  });

  server.addHandler(&ws);
  server.begin();
}


void setup() {
  // Setup lift stuf
  pinMode(LIFT_ENC1, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(CNT_1, OUTPUT);
  pinMode(CNT_2, OUTPUT);  
  attachInterrupt(digitalPinToInterrupt(LIFT_ENC1), encoderISR, RISING);
  
  // Setup light stuff
  pinMode(LIGHT_CONTROL, OUTPUT);
  pinMode(LIGHT_ENC1, OUTPUT);
  pinMode(LIGHT_ENC2, OUTPUT);
  digitalWrite(LIGHT_CONTROL, HIGH);
  digitalWrite(LIGHT_ENC1, HIGH);
  digitalWrite(LIGHT_ENC2, HIGH);

  Serial.begin(115200);
  Serial.println("xLift booting up");

  setupServer();
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
  
  } else if (cmd == Node2ArduinoCommand::LIGHT_TOGGLE) {
    lightToggle();
  
  } else if (cmd == Node2ArduinoCommand::LIGHT_BRIGHT_UP) {
    lightBrightUp();
  
  } else if (cmd == Node2ArduinoCommand::LIGHT_BRIGHT_DOWN) {
    lightBrightDown();

  } else if (cmd == Node2ArduinoCommand::LIGHT_TEMP_UP) {
    lightTempUp();
  
  } else if (cmd == Node2ArduinoCommand::LIGHT_TEMP_DOWN) {
    lightTempDown();

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
            stopMotor();
            cur_height = max(min_height, cur_height); // Ensure we don't go below min height
            Serial.println((char)Arduino2NodeCommand::CURRENT_LIMIT_TRIGGERED);
            // current_limit_warning_sent = true;
        }

        ws.textAll("height:" + String(cur_height));
        
        // else if (current_limit_warning_sent) {
        //     // If we have sent a warning before, we can now send an untriggered message
        //     Serial.println((char)Arduino2NodeCommand::CURRENT_LIMIT_UNTRIGGERED);
        //     current_limit_warning_sent = false;
        // }
    }

    motorControl(); // Execute motor control signal sending
}
