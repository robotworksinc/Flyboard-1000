/*
        Microstrain IMU library for Arduino
        ===================================
    This routine interacts with microstrain IMU and send/get IMU data. 
    Following function are implemented (the one's used for UAV):
    
    a) only gyro values
*/

// Define value of pi
#define PI 3.141592653589793

// 20 ms wait for serial signal
#define MAX_MILLIS_TO_WAIT 20

// Global variables
unsigned long starttime;
float gyro[] = {0.0, 0.0, 0.0};

// Function to convert angles in radians to degree
float rad2deg(float angle) {
    return (180/PI) * angle;
}


// Get 3D gyro values from the IMU
byte get_gyro(float* gyro) {
    byte cmd[] = {0x75,0x65,0x0C,0x07,0x07,0x01,0x00,0x01,0x05,0x00,0x00,0xFB,0xE7};
    byte reply[30];
    
    // Variables to receive x,y and z gyro values
    union {
     float val;
     unsigned char b[4];   
    } x_gyro, y_gyro, z_gyro;
    
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
           delay(1);
       }

       // X gyro
       x_gyro.b[0] = reply[19];
       x_gyro.b[1] = reply[18];
       x_gyro.b[2] = reply[17];
       x_gyro.b[3] = reply[16];
       gyro[0] = x_gyro.val;
          
       // Y gyro
       y_gyro.b[0] = reply[23];
       y_gyro.b[1] = reply[22];
       y_gyro.b[2] = reply[21];
       y_gyro.b[3] = reply[20];
       gyro[1] = y_gyro.val;
       
       // Z gyro
       z_gyro.b[0] = reply[27];
       z_gyro.b[1] = reply[26];
       z_gyro.b[2] = reply[25];
       z_gyro.b[3] = reply[24];
       gyro[2] = z_gyro.val;                
     
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
    if (get_gyro(gyro) == '\x00') {
       Serial.println(rad2deg(gyro[0]));
       Serial.println(rad2deg(gyro[1]));
       Serial.println(rad2deg(gyro[2])); 
    }
    else {
       Serial.println("IMU: Gyro values unsuccessful!");
    } 

    // Delay os 5 ms (200 Hz) i.e. get IMU data at 200Hz rate
    delay(5);
}
