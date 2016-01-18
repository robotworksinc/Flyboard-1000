/*
        Microstrain IMU library for Arduino
        ===================================
    This routine interacts with microstrain IMU. 
    
    a) ping
*/

#include <PinChangeInt.h>
#include <Servo.h>

// Define value of pi
#define PI 3.141592653589793
#define g  9.80665

// 100 ms wait for serial signal
#define MAX_MILLIS_TO_WAIT 100

// Define RC channels pin numbers
#define RC_THROTTLE      2
#define RC_PITCH         4
#define RC_ROLL          7
#define RC_YAW           8
#define RC_IN_PIN_5     12
#define RC_IN_PIN_6     13 

// Define drone motor pin numbers
#define MOTOR_FL         9
#define MOTOR_FR        10
#define MOTOR_BL         6
#define MOTOR_BR        11

// Radio min/max for each stick
#define RC_MIN       1050
#define RC_MAX       1800
#define ESC_MIN      1400
#define ESC_MAX      1750

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG   1
#define YAW_FLAG        2
#define PITCH_FLAG      4
#define ROLL_FLAG       8

// Global variables
unsigned long starttime;

float euler[] = {0.0, 0.0, 0.0};
float gyro[] = {0.0, 0.0, 0.0};
float accl[] = {0.0, 0.0, 0.0};

volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared, unYawInShared, unPitchInShared, unRollInShared;
uint32_t ulThrottleStart, ulYawStart, ulPitchStart, ulRollStart;
uint16_t fast, slow, rcthr, rcyaw, rcpit, rcroll;

// Define ESC servo objects
Servo ESC1, ESC2, ESC3, ESC4;

// ===========================================================================================
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

// =================================================================
// simple interrupt service routine
void calcThrottle()
{
  if(digitalRead(RC_THROTTLE) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}


void calcYaw()
{
  if(digitalRead(RC_YAW) == HIGH)
  {
    ulYawStart = micros();
  }
  else
  {
    unYawInShared = (uint16_t)(micros() - ulYawStart);
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void calcPitch()
{
  if(digitalRead(RC_PITCH) == HIGH)
  {
    ulPitchStart = micros();
  }
  else
  {
    unPitchInShared = (uint16_t)(micros() - ulPitchStart);
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}


void calcRoll()
{
  if(digitalRead(RC_ROLL) == HIGH)
  {
    ulRollStart = micros();
  }
  else
  {
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}


void rcmotors()
{
  // local copy of channel inputs
  static uint16_t unThrottleIn;
  static uint16_t unYawIn;
  static uint16_t unPitchIn;
  static uint16_t unRollIn;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & YAW_FLAG)
    {
      unYawIn = unYawInShared;
    }

    if(bUpdateFlags & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;
    }
    
    if(bUpdateFlags & ROLL_FLAG)
    {
      unRollIn = unRollInShared;
    }
    
    bUpdateFlagsShared = 0;
   
    interrupts(); 
  }
 
  if(bUpdateFlags & THROTTLE_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(rcthr);
    ESC2.writeMicroseconds(rcthr);
    ESC3.writeMicroseconds(rcthr);
    ESC4.writeMicroseconds(rcthr);
  }
 
  if(bUpdateFlags & YAW_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    rcyaw = map(unYawIn, RC_MIN, RC_MAX, -200, 200);   
    fast = constrain(rcthr + rcyaw, ESC_MIN, ESC_MAX);
    slow = constrain(rcthr - rcyaw, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(fast);
    ESC4.writeMicroseconds(fast);
    ESC2.writeMicroseconds(slow);
    ESC3.writeMicroseconds(slow);
  } 

  if(bUpdateFlags & PITCH_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    rcpit = map(unPitchIn, RC_MIN, RC_MAX, -170, 170);   
    fast = constrain(rcthr + rcpit, ESC_MIN, ESC_MAX);
    slow = constrain(rcthr - rcpit, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(fast);
    ESC2.writeMicroseconds(fast);
    ESC3.writeMicroseconds(slow);
    ESC4.writeMicroseconds(slow);
  } 

  if(bUpdateFlags & ROLL_FLAG)
  {
    rcthr = map(unThrottleIn, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
    rcroll = map(unRollIn, RC_MIN, RC_MAX, -170, 170);    
    fast = constrain(rcthr + rcroll, ESC_MIN, ESC_MAX);
    slow = constrain(rcthr - rcroll, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(fast);
    ESC3.writeMicroseconds(fast);
    ESC2.writeMicroseconds(slow);
    ESC4.writeMicroseconds(slow);
  }
              
  bUpdateFlags = 0;
}

// =============================================================================

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
    
    ESC1.attach(MOTOR_FL);
    ESC2.attach(MOTOR_FR);
    ESC3.attach(MOTOR_BL);
    ESC4.attach(MOTOR_BR);

    PCintPort::attachInterrupt(RC_THROTTLE,calcThrottle,CHANGE);
    PCintPort::attachInterrupt(RC_YAW,calcYaw,CHANGE);
    PCintPort::attachInterrupt(RC_PITCH,calcPitch,CHANGE);
    PCintPort::attachInterrupt(RC_ROLL,calcRoll,CHANGE);

    // Delay of 10 ms
    delay(10);
}


// Loop through following code
void loop() {
    //if (get_euler_angles(euler) == '\x00') {
	//Serial.println("IMU: Euler values successful!");
	//Serial.println(rad2deg(euler[0]));
	//Serial.println(rad2deg(euler[1]));
	//Serial.println(rad2deg(euler[2]));
    //}
    //else {
	//Serial.println("IMU: Euler angle unsuccessful!");
    //}

    //delay(10);

    //if (get_gyro(gyro) == '\x00') {
	//Serial.println("IMU: gyro values successful!");
	//Serial.println(rad2deg(gyro[0]));
	//Serial.println(rad2deg(gyro[1]));
	//Serial.println(rad2deg(gyro[2]));
    //}
    //else {
	//Serial.println("IMU: gyro unsuccessful!");
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

    delay(5);

    rcmotors();

    delay(5);
}
