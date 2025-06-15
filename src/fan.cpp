// #include <Arduino.h>

// // Filled in missing ones via https://www.youtube.com/watch?list=TLGGmwQb26PIWdkxNDA2MjAyNQ&v=ni9TII6xymQ
// // 0000 - Do nothing
// // 0001 - 4
// // 0010 - 3
// // 0011 - 4h
// // 0100 - 2
// // 0101 - 8h
// // 0110 - 2h
// // 0111 - ???
// // 1000 - 1
// // 1001 - Toggle light
// // 1010 - 1h
// // 1011 - (Pairing)
// // 1100 - Toggle on/off
// // 1101 - ???
// // 1110 - ???
// // 1111 - ???

// // No reason fan remote can't work using 4.6V. It's a super dumb circuit.

// #define K0 15 // D8
// #define K1 13 // D7
// #define K2 12 // D6
// #define K3 14 // D5

// void setup() {
//   Serial.begin(115200);
  
//   pinMode(K0, INPUT);
//   pinMode(K1, INPUT);
//   pinMode(K2, INPUT);
//   pinMode(K3, INPUT);

//   Serial.println("Started!");
// }

// void loop() {
//   // if (Serial.available() > 0) {
//     // char input = Serial.read();

//     // if (input == 'r') {
//       Serial.print("Reading: ");
//       int k0 = digitalRead(K0);
//       int k1 = digitalRead(K1);
//       int k2 = digitalRead(K2);
//       int k3 = digitalRead(K3);
      
//       Serial.print(k0);
//       Serial.print(" ");
//       Serial.print(k1);
//       Serial.print(" ");
//       Serial.print(k2);
//       Serial.print(" ");
//       Serial.print(k3);
//       Serial.print("\n");
//     // }
//   // }
// }
