/*
        Microstrain IMU library for Arduino
        ===================================
    This routine interacts with microstrain IMU. 
    
    a) ping
*/

// Define value of pi
#define PI 3.141592653589793

// 100 ms wait for serial signal
#define MAX_MILLIS_TO_WAIT 100

// Global variables
unsigned long starttime;
float euler[] = {0.0, 0.0, 0.0};
float gyro[] = {0.0, 0.0, 0.0};
float accl[] = {0.0, 0.0, 0.0};


// Function to convert angles in radians to degree
float rad2deg(float angle) {
    return (180/PI) * angle;
}


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


// Get Euler angles from the IMU
byte get_euler_angles(float* euler) {
    byte cmd[] = {0x75,0x65,0x0C,0x07,0x07,0x01,0x00,0x01,0x0C,0x00,0x00,0x02,0xFC};
    byte reply[30];
    
    // Variables to receive roll, pitch and yaw angles
    union {
     float val;
     unsigned char b[4];   
    } roll_angle, pitch_angle, yaw_angle;
    
    // Send serial command
    Serial.write(cmd, sizeof(cmd));
   
    // Delay of 1 ms
    delay(1);
 
    starttime = millis();

    // Wait till all 30 bytes are recieved or 100 ms has gone bye
    while ((Serial.available() < 30) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT)) {
    }

    // If less than 30 bytes are available on serial register, return \x01 error
    // code else send \x00 success code and populate euler angles   
    if (Serial.available() < 30) {
       return byte(1);
    }
    else {
       for (int n = 0; n < 30; n++) {
           reply[n] = Serial.read();
       }

       // Roll angle
       roll_angle.b[0] = reply[19];
       roll_angle.b[1] = reply[18];
       roll_angle.b[2] = reply[17];
       roll_angle.b[3] = reply[16];
       euler[0] = roll_angle.val;
       
       //Serial.println(euler[0]);
 	
       // Pitch  angle
       pitch_angle.b[0] = reply[23];
       pitch_angle.b[1] = reply[22];
       pitch_angle.b[2] = reply[21];
       pitch_angle.b[3] = reply[20];
       euler[1] = pitch_angle.val;
       
       // Yaw angle
       yaw_angle.b[0] = reply[27];
       yaw_angle.b[1] = reply[26];
       yaw_angle.b[2] = reply[25];
       yaw_angle.b[3] = reply[24];
       euler[2] = yaw_angle.val;                
       
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

    if (get_euler_angles(euler) == '\x00') {
	Serial.println("IMU: Euler angle successful!");
        Serial.println(rad2deg(euler[0]));
    	Serial.println(rad2deg(euler[1]));
    	Serial.println(rad2deg(euler[2])); 
    }
    else {
	Serial.println("IMU: Euler angle unsuccessful!");
    }

    delay(5);
}
