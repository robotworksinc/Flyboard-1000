/*
        Microstrain IMU library for Arduino
        ===================================
    This routine interacts with microstrain IMU and send/get IMU data. 
    Following function are implemented (the one's used for UAV):
    
    a) AHRS: Euler, Gyro and Accelerometer values    
*/

// Define value of pi
#define PI 3.141592653589793
#define g  9.80665

// 20 ms wait for serial signal
#define MAX_MILLIS_TO_WAIT 20

// Global variables
unsigned long starttime;
float euler[] = {0.0, 0.0, 0.0};
float gyro[] = {0.0, 0.0, 0.0};
float accl[] = {0.0, 0.0, 0.0};

// Function to convert angles in radians to degree
float rad2deg(float angle) {
    return (180/PI) * angle;
}


// Get Euler angles, gyro an accelerometer values from the IMU
byte get_ahrs(float* euler, float* gyro, float* accl) {
    byte cmd[] = {0x75,0x65,0x0C,0x0D,0x0D,0x01,0x00,0x03,0x0C,0x00,0x00,0x04,0x00,0x00,0x05,0x00,0x00,0x19,0xE5};
    byte reply[58];
    
    // Variables to receive euler angles, gyro and accelerometer values
    union {
     float val;
     unsigned char b[4];   
    } uav_value;
    
    // Send serial command
    Serial.write(cmd, sizeof(cmd));
   
    // Delay of 1 ms
    delay(1);
 
    starttime = millis();

    // Wait till all 58 bytes are recieved or 100 ms has gone bye
    while ((Serial.available() < 58) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT)) {
    }

    // If less than 58 bytes are available on serial register, return \x01 error
    // code else send \x00 success code and populate euler angles   
    if (Serial.available() < 58) {
       return byte(1);
    }
    else {
       for (int n = 0; n < 58; n++) {
           reply[n] = Serial.read();
       }

       // Roll angle
       uav_value.b[0] = reply[19];
       uav_value.b[1] = reply[18];
       uav_value.b[2] = reply[17];
       uav_value.b[3] = reply[16];
       euler[0] = uav_value.val;
       
       // Pitch  angle
       uav_value.b[0] = reply[23];
       uav_value.b[1] = reply[22];
       uav_value.b[2] = reply[21];
       uav_value.b[3] = reply[20];
       euler[1] = uav_value.val;
       
       // Yaw angle
       uav_value.b[0] = reply[27];
       uav_value.b[1] = reply[26];
       uav_value.b[2] = reply[25];
       uav_value.b[3] = reply[24];
       euler[2] = uav_value.val;                

       // X Acceleration
       uav_value.b[0] = reply[33];
       uav_value.b[1] = reply[32];
       uav_value.b[2] = reply[31];
       uav_value.b[3] = reply[30];
       accl[0] = uav_value.val;
          
       // Y Acceleration
       uav_value.b[0] = reply[37];
       uav_value.b[1] = reply[36];
       uav_value.b[2] = reply[35];
       uav_value.b[3] = reply[34];
       accl[1] = uav_value.val;
       
       // Z Acceleration
       uav_value.b[0] = reply[41];
       uav_value.b[1] = reply[40];
       uav_value.b[2] = reply[39];
       uav_value.b[3] = reply[38];
       accl[2] = uav_value.val;               
              
       // X gyro
       uav_value.b[0] = reply[47];
       uav_value.b[1] = reply[46];
       uav_value.b[2] = reply[45];
       uav_value.b[3] = reply[44];
       gyro[0] = uav_value.val;
          
       // Y gyro
       uav_value.b[0] = reply[51];
       uav_value.b[1] = reply[50];
       uav_value.b[2] = reply[49];
       uav_value.b[3] = reply[48];
       gyro[1] = uav_value.val;
       
       // Z gyro
       uav_value.b[0] = reply[55];
       uav_value.b[1] = reply[54];
       uav_value.b[2] = reply[53];
       uav_value.b[3] = reply[52];
       gyro[2] = uav_value.val;                
                            
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
   Serial.flush();
   
   if (get_ahrs(euler, gyro, accl)== '\x00') {
      Serial.println("IMU: AHRS values  successful!");
      Serial.println("Euler angles:");
      Serial.println(rad2deg(euler[0]));
      Serial.println(rad2deg(euler[1]));
      Serial.println(rad2deg(euler[2]));
      Serial.println("Gyro values:");
      Serial.println(rad2deg(gyro[0]));
      Serial.println(rad2deg(gyro[1]));
      Serial.println(rad2deg(gyro[2]));
      Serial.println("Acceleration values"); 
      Serial.println(accl[0]*g);
      Serial.println(accl[1]*g);
      Serial.println(accl[2]*g);
    }
    else {
        Serial.println("IMU: AHRS values unsuccessful!");
    }   

    // Delay os 5 ms (200 Hz) i.e. get IMU data at 200Hz rate
    delay(5);
}
