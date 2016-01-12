#include <PinChangeInt.h>

#include <Servo.h>

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

// Define ESC servo objects
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

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

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulYawStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;

// Intermediate variables to hold values
uint16_t fast, slow, rcthr, rcyaw, rcpit, rcroll;

// 100 ms wait for serial signal
#define MAX_MILLIS_TO_WAIT 100

// Define value of pi
#define PI 3.141592653589793

// Global variables
unsigned long starttime;
float euler[] = {0.0, 0.0, 0.0};
float gyro[] = {0.0, 0.0, 0.0};
float accl[] = {0.0, 0.0, 0.0};

// Function to convert angles in radians to degree
float rad2deg(float angle) {
    return (180/PI) * angle;
}


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


void run()
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
    rcyaw = map(unYawIn, RC_MIN, RC_MAX, -200, 200)    
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
    rcpit = map(unPitchIn, RC_MIN, RC_MAX, -170, 170)    
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
    rcroll = map(unRollIn, RC_MIN, RC_MAX, -170, 170)    
    fast = constrain(rcthr + rcroll, ESC_MIN, ESC_MAX);
    slow = constrain(rcthr - rcroll, ESC_MIN, ESC_MAX);
    ESC1.writeMicroseconds(fast);
    ESC3.writeMicroseconds(fast);
    ESC2.writeMicroseconds(slow);
    ESC4.writeMicroseconds(slow);
  }
              
  bUpdateFlags = 0;
}

void setup()
{
  Serial.begin(115200);
 
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

void loop()
{
  // Operate motors using RC
  run()
  
  // Check if IMU is "alive"
  if (ping() == '\x00') {
    Serial.println("IMU: ping successful!");
  }
  else {
    Serial.println("IMU: ping unsuccessful");
    
  // Get Euler angles, acceleration and gyro values
  if (get_uav(euler, accl, gyro) == '\x00') {
	Serial.println("IMU: IMU values are successful!");
        Serial.println(rad2deg(euler[0]));
    	Serial.println(rad2deg(euler[1]));
    	Serial.println(rad2deg(euler[2]));
	Serial.println(rad2deg(gyro[0]));
    	Serial.println(rad2deg(gyro[1]));
    	Serial.println(rad2deg(gyro[2]));
    }
    else {
	Serial.println("IMU: IMU values are unsuccessful!");
    }	
  
  }
}
