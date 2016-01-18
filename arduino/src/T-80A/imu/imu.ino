/*
        Microstrain IMU library for Arduino
        ===================================
    This routine interacts with microstrain IMU and send/get IMU data. 
    Following function are implemented (the one's used for UAV):
    
    a) ping
    b) euler + gyro values
    c) only euler angles
    d) only gyro values
    e) only accelerometer values    
*/

// Define value of pi
#define PI 3.141592653589793
#define g  9.80665

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


// Get 3D acclerometer values from the IMU
byte get_acceleration(float* accl) {
    byte cmd[] = {0x75,0x65,0x0C,0x07,0x07,0x01,0x00,0x01,0x04,0x00,0x00,0xFA,0xE4};
    byte reply[30];
    
    // Variables to receive x,y and z accelerometer values
    union {
     float val;
     unsigned char b[4];   
    } x_accl, y_accl, z_accl;
    
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

       // X Acceleration
       x_accl.b[0] = reply[19];
       x_accl.b[1] = reply[18];
       x_accl.b[2] = reply[17];
       x_accl.b[3] = reply[16];
       accl[0] = x_accl.val;
        	
       // Y Acceleration
       y_accl.b[0] = reply[23];
       y_accl.b[1] = reply[22];
       y_accl.b[2] = reply[21];
       y_accl.b[3] = reply[20];
       accl[1] = y_accl.val;
       
       // Z Acceleration
       z_accl.b[0] = reply[27];
       z_accl.b[1] = reply[26];
       z_accl.b[2] = reply[25];
       z_accl.b[3] = reply[24];
       accl[2] = z_accl.val;                
       
       return reply[7];
    }
}


// Get Euler angles, gyro an accelerometer values from the IMU
byte get_uav(float* euler, float* accl, float* gyro) {
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
    
    // Check if IMU is "alive"
    if (ping() == '\x00') {
        Serial.println("IMU: ping successful!");
    }
    else {
        Serial.println("IMU: ping unsuccessful");
    }
        
    // Delay of 10 ms
    delay(10);
}


// Loop through following code
void loop() {
    //if (get_gyro(gyro) == '\x00') {
	//Serial.println("IMU: Euler angle successful!");
        //Serial.println(rad2deg(gyro[0]));
    	//Serial.println(rad2deg(gyro[1]));
    	//Serial.println(rad2deg(gyro[2])); 
    //}
    //else {
	//Serial.println("IMU: Euler angle unsuccessful!");
    //}	

    //if (get_euler_angles(euler) == '\x00') {
	//Serial.println("IMU: Euler angle successful!");
        //Serial.println(rad2deg(euler[0]));
    	//Serial.println(rad2deg(euler[1]));
    	//Serial.println(rad2deg(euler[2])); 
    //}
    //else {
	//Serial.println("IMU: Euler angle unsuccessful!");
    //}

    if (get_uav(euler, accl, gyro)== '\x00') {
	Serial.println("IMU: UAV successful!");
	Serial.println(rad2deg(euler[0]));
    	Serial.println(rad2deg(euler[1]));
    	Serial.println(rad2deg(euler[2]));
        Serial.println(rad2deg(gyro[0]));
    	Serial.println(rad2deg(gyro[1]));
    	Serial.println(rad2deg(gyro[2])); 
	Serial.println(accl[0]*g);
    	Serial.println(accl[1]*g);
    	Serial.println(accl[2]*g);
    }
    else {
        Serial.println("IMU: Euler angle unsuccessful!");
    } 	

    // Delay os 5 ms (200 Hz) i.e. get IMU data at 200Hz rate
    delay(5);
}
