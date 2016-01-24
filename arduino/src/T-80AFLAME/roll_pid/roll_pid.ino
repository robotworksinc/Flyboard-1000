#include <PinChangeInt.h>
#include <PID_v1.h>
#include <Servo.h>


// Define value of pi
#define PI 3.141592653589793
#define g  9.80665
#define NITER  5

// 20 ms wait for serial signal
#define MAX_MILLIS_TO_WAIT 20

// Define RC channels pin numbers
#define RC_THROTTLE      2
#define RC_ROLL          4
#define RC_PITCH         7
#define RC_YAW           8
#define RC_IN_PIN_5     12
#define RC_IN_PIN_6     13 

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG   1
#define ROLL_FLAG       2
#define PITCH_FLAG      4
#define YAW_FLAG        8

// Define drone motor pin numbers
#define MOTOR_FL        11
#define MOTOR_FR         6
#define MOTOR_BL        10
#define MOTOR_BR         9

// RC Radio min/max for each stick
#define RC_MIN       1100
#define RC_MID       1500
#define RC_MAX       1900
#define ESC_MIN      1450
#define ESC_MAX      1830

// Define ESC servo objects
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

// Various global variables
int channels[4];
volatile uint8_t bUpdateFlagsShared;
long rcthr, rcyaw, rcpit, rcroll, pwm1, pwm2, pwm3, pwm4;
uint32_t ulThrottleStart, ulYawStart, ulPitchStart, ulRollStart;
volatile uint16_t unThrottleInShared, unYawInShared, unPitchInShared, unRollInShared;

unsigned long starttime;
float euler[] = {0.0, 0.0, 0.0};
float gyro[] = {0.0, 0.0, 0.0};
float accl[] = {0.0, 0.0, 0.0};
double roll, pitch, yaw, x_gyro, y_gyro, z_gyro;
float roll_val[NITER], pitch_val[NITER], yaw_val[NITER], x_gyro_val[NITER], y_gyro_val[NITER], z_gyro_val[NITER]; 

// PID variables and arrays
double roll_out, pitch_out, yaw_out;
double roll_stab_out, pitch_stab_out, yaw_stab_out;

double roll_stab_param[] = {0.5, 0.0, 0.0};

double roll_rate_param[] = {0.2, 0.0, 0.0};

PID roll_stab_pid((double *)&rcroll, &roll_stab_out, &roll, roll_stab_param[0], roll_stab_param[1], roll_stab_param[2], DIRECT);

PID roll_rate_pid((double *)&roll_stab_out, &roll_out, &x_gyro, roll_rate_param[0], roll_rate_param[1], roll_rate_param[2], DIRECT);


// ==================  IMU control code  ==============================================
// Determine mean value
float mean(float* val) {
  float temp = 0.0;
  for(int i = 0; i < NITER; i++){
    temp += val[i];
  }
  return temp/NITER;
}

// Function to convert angles in radians to degree
float rad2deg(float angle) {
    return (180/PI) * angle;
}

// Get Euler angles, gyro an accelerometer values from the IMU
byte readIMU(float* euler, float* gyro, float* accl) {
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
           delay(1);  
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


// =================== RC/MOTOR control code ===========================================
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


void readRC(int channels[]) {
   // local copy of channel inputs
   static uint16_t unThrottleIn, unYawIn, unPitchIn, unRollIn;
  
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
     
     if(bUpdateFlags & ROLL_FLAG)
     {
       unRollIn = unRollInShared;
     }

     if(bUpdateFlags & PITCH_FLAG)
     {
       unPitchIn = unPitchInShared;
     }
           
     if(bUpdateFlags & YAW_FLAG)
     {
       unYawIn = unYawInShared;
     }
    
     bUpdateFlagsShared = 0;
   
     interrupts(); 
   }
 
   if(bUpdateFlags & THROTTLE_FLAG)
   {
     channels[0] = unThrottleIn;
   }

   if(bUpdateFlags & ROLL_FLAG)
   {
     channels[1] = unRollIn;
   }

   if(bUpdateFlags & PITCH_FLAG)
   {
     channels[2] = unPitchIn;

   }    

   if(bUpdateFlags & YAW_FLAG)
   {
     channels[3] =  unYawIn;
   } 

   bUpdateFlags = 0;
}


void setup()
{
  // Start serial communication channel
  Serial.begin(115200);

  // Attach interrupts to RC arduino pins
  PCintPort::attachInterrupt(RC_THROTTLE,calcThrottle,CHANGE);
  PCintPort::attachInterrupt(RC_ROLL,calcRoll,CHANGE);
  PCintPort::attachInterrupt(RC_PITCH,calcPitch,CHANGE);
  PCintPort::attachInterrupt(RC_YAW,calcYaw,CHANGE);

  // Attach ESC's to arduino pins
  ESC1.attach(MOTOR_FL);
  ESC2.attach(MOTOR_FR);
  ESC3.attach(MOTOR_BL);
  ESC4.attach(MOTOR_BR);

  // Set RC channel value to "rest"
  channels[0] = RC_MIN;
  channels[1] = RC_MID;
  channels[2] = RC_MID; 
  channels[3] = RC_MID;

  // Read "rest" euler, gyro and accl values
  if (readIMU(euler, gyro, accl) == '\x00') {
       roll = rad2deg(euler[0]);
       pitch = rad2deg(euler[1]);
       yaw = rad2deg(euler[2]);
       x_gyro = rad2deg(gyro[0]);
       y_gyro = rad2deg(gyro[1]);
       z_gyro = rad2deg(gyro[2]);
  }
  else {
       roll = 0.0;
       pitch = 0.0;
       yaw = 0.0;
       x_gyro = 0.0;
       y_gyro = 0.0;
       z_gyro = 0.0;
  } 

  // Set PID mode
  roll_stab_pid.SetMode(AUTOMATIC);
  roll_rate_pid.SetMode(AUTOMATIC);
  
  // Delay of 10 ms
  delay(10);
}

void loop()
{
  // Read IMU 10-15 times and average the Euler angle and rotational speed values
  for (int i = 0; i < NITER; i++){
    if (readIMU(euler, gyro, accl) == '\x00'){
        roll_val[i] = rad2deg(euler[0]);
        pitch_val[i] = rad2deg(euler[1]);
        yaw_val[i] = rad2deg(euler[2]);
        x_gyro_val[i] = rad2deg(gyro[0]);
        y_gyro_val[i] = rad2deg(gyro[1]);
        z_gyro_val[i] = rad2deg(gyro[2]);
    }
  }

  // Mean value of Euler angles and rotational speed
  roll = mean(roll_val);

  // Display IMU values
  Serial.println(roll);
    
  // Read RC channel values
  readRC(channels);

  if (channels[1] == 0 & channels[2] == 0 & channels[3] == 0) {
    channels[0] = RC_MIN;
    channels[1] = RC_MID;
    channels[2] = RC_MID; 
    channels[3] = RC_MID;
  }

  // Apart from throttle, map roll, pitch and yaw to angles in degrees
  rcthr = channels[0];
  rcroll = map(channels[1], RC_MIN, RC_MAX, -15, 15);
  rcpit = map(channels[2], RC_MIN, RC_MAX, -15, 15);
  rcyaw = map(channels[3], RC_MIN, RC_MAX, -150, 150);
  
  Serial.println("RC constrained values:");
  Serial.println(rcroll);

  // Stablization PID
  roll_stab_pid.Compute();
  //pitch_stab_pid.Compute();
  //yaw_stab_pid.Compute();

  Serial.println("Stab PID output");
  Serial.println(roll_stab_out);

  roll_stab_out = constrain(roll_stab_out, -100, 100);
  
  // Rate PID
  roll_rate_pid.Compute();

  Serial.println("Rate PID output");
  Serial.println(roll_out);
  
  roll_out = constrain(roll_out, -150, 150);

  pwm1 = constrain(rcthr - rcroll - rcpit - rcyaw, RC_MIN, RC_MAX);
  pwm2 = constrain(rcthr + rcroll - rcpit + rcyaw, RC_MIN, RC_MAX);
  pwm3 = constrain(rcthr - rcroll + rcpit + rcyaw, RC_MIN, RC_MAX);
  pwm4 = constrain(rcthr + rcroll + rcpit - rcyaw, RC_MIN, RC_MAX);

  //pwm1 = constrain(rcthr - roll_output - pitch_output - yaw_output, RC_MIN, RC_MAX);
  //pwm2 = constrain(rcthr + roll_output - pitch_output + yaw_output, RC_MIN, RC_MAX);
  //pwm3 = constrain(rcthr - roll_output + pitch_output + yaw_output, RC_MIN, RC_MAX);
  //pwm4 = constrain(rcthr + roll_output + pitch_output - yaw_output, RC_MIN, RC_MAX);

  pwm1 = map(pwm1, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
  pwm2 = map(pwm2, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
  pwm3 = map(pwm3, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);
  pwm4 = map(pwm4, RC_MIN, RC_MAX, ESC_MIN, ESC_MAX);

  Serial.println("ESC pwm values:");
  Serial.println(pwm1);
  Serial.println(pwm2);
  Serial.println(pwm3);
  Serial.println(pwm4);
  
  ESC1.writeMicroseconds(pwm1);
  ESC2.writeMicroseconds(pwm2);
  ESC3.writeMicroseconds(pwm3);
  ESC4.writeMicroseconds(pwm4);
}
