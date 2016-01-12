/*
        Microstrain IMU library for Arduino
        ===================================
    This routine interacts with microstrain IMU. 
    
    a) ping
*/


// 100 ms wait for serial signal
#define MAX_MILLIS_TO_WAIT 100


// Ping the IMU to check if it is "alive"
byte ping() {
    byte cmd[] = {0x75,0x65,0x01,0x02,0x02,0x01,0xE0,0xC6};
    byte reply[10];  
    
    Serial.write(cmd, sizeof(cmd));         // send ping request
      
    delay(1);                               // 1 ms delay

    starttime = millis();                   // start the clock

    // Wait till all 10 bytes are recieved or 100 ms has gone by
    while ((Serial.available() < 10) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT)) {
    }

    // If 10 bytes of reply is not available send \x01 error code else send 
    // \x00 success code
    if (Serial.available() < 10) {
       return byte(1);
    }
    else {
       for (int i = 0; i < 10; i++) {
           reply[i] = Serial.read();
       }
       return reply[7];
    }
}


// Entry point for arduino code
void setup() {
    // Start Serial transmission from/to IMU
    Serial.begin(115200);
    
    // Delay of 10 ms
    delay(10);
}


// Loop through following code
void loop() {
    // Check if IMU is "alive"
    if (ping() == '\x00') {
        Serial.println("IMU: ping successful!");
    }
    else {
        Serial.println("IMU: ping unsuccessful");
    }

    // Delay os 5 ms (200 Hz) i.e. get IMU data at 200Hz rate
    delay(5);
}
