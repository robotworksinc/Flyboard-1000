#include <PinChangeInt.h>

#include <Servo.h>

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

// Define ESC servo objects
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

// Define drone motor pin numbers
#define MOTOR_FL         9
#define MOTOR_FR        10
#define MOTOR_BL         6
#define MOTOR_BR        11

// RC Radio min/max for each stick
#define RC_MIN       1050
#define RC_MID       1500
#define RC_MAX       1800
#define ESC_MIN      1400
#define ESC_MAX      1750

// Various global variables
int channels[4];
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared, unYawInShared, unPitchInShared, unRollInShared;
uint32_t ulThrottleStart, ulYawStart, ulPitchStart, ulRollStart;
uint16_t rcthr, rcyaw, rcpit, rcroll, pwm1, pwm2, pwm3, pwm4;

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
  Serial.begin(115200);

  PCintPort::attachInterrupt(RC_THROTTLE,calcThrottle,CHANGE);
  PCintPort::attachInterrupt(RC_ROLL,calcRoll,CHANGE);
  PCintPort::attachInterrupt(RC_PITCH,calcPitch,CHANGE);
  PCintPort::attachInterrupt(RC_YAW,calcYaw,CHANGE);
  
  ESC1.attach(MOTOR_FL);
  ESC2.attach(MOTOR_FR);
  ESC3.attach(MOTOR_BL);
  ESC4.attach(MOTOR_BR);

  delay(10);

}

void loop()
{
  readRC(channels);

  // map(channels[0], RC_MIN, RC_MAX, ESC_MIN, ESC_MAX)
  rcthr = channels[0];
  rcroll = map(channels[1], RC_MIN, RC_MAX, -60, 60);
  rcpit = map(channels[2], RC_MIN, RC_MAX, -60, 60);
  rcyaw = map(channels[3], RC_MIN, RC_MAX, -300, 300);
  
  Serial.println("RC constrained values:");
  Serial.println(rcthr);
  Serial.println(rcroll);
  Serial.println(rcpit);
  Serial.println(rcyaw);

  Serial.println("Motor values:");
  Serial.println(rcthr - rcroll - rcpit - rcyaw);
  Serial.println(rcthr + rcroll - rcpit + rcyaw);
  Serial.println(rcthr - rcroll + rcpit + rcyaw);
  Serial.println(rcthr + rcroll + rcpit - rcyaw); 

  pwm1 = constrain(rcthr - rcroll - rcpit - rcyaw, RC_MIN, RC_MAX);
  pwm2 = constrain(rcthr + rcroll - rcpit + rcyaw, RC_MIN, RC_MAX);
  pwm3 = constrain(rcthr - rcroll + rcpit + rcyaw, RC_MIN, RC_MAX);
  pwm4 = constrain(rcthr + rcroll + rcpit - rcyaw, RC_MIN, RC_MAX);

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
