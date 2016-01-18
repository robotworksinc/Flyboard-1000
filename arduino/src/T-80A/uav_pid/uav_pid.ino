#include <PID_v1.h>

// Radio min/max for each stick
#define RC_MIN       1050
#define RC_MAX       1800
#define ESC_MIN      1400
#define ESC_MAX      1750

// Global variables
int channels[4];
double yaw, pitch, roll;
double rcthr, rcyaw, rcpit, rcroll;
double gyro_pitch, gyro_roll, gyro_yaw;
double pitch_stab_output, roll_stab_output, yaw_stab_output;
double pitch_output, roll_output, yaw_output;

//Define the Kp, Ki and Kd Tuning Parameters
double pitch_stab_param[] = {0.1, 0.0, 0.0};
double roll_stab_param[] = {2.0, 0.0, 0.0};
double yaw_stab_param[] = {4.0, 0.0, 0.0};

double pitch_rate_param[] = {0.2, 0.0, 0.0};
double roll_rate_param[] = {0.2, 0.0, 0.0};
double yaw_rate_param[] = {1.0, 0.0, 0.0};


//Specify the links and initial tuning parameters
PID pitch_stab_PID(&rcpit, &pitch_stab_output, &pitch, pitch_stab_param[0], pitch_stab_param[1], pitch_stab_param[2], DIRECT);
PID roll_stab_PID(&rcroll, &roll_stab_output, &roll, roll_stab_param[0], roll_stab_param[1], roll_stab_param[2], DIRECT);
PID yaw_stab_PID(&rcyaw, &yaw_stab_output, &yaw, yaw_stab_param[0], yaw_stab_param[1], yaw_stab_param[2], DIRECT);

PID pitch_rate_PID(&pitch_stab_output, &pitch_output, &gyro_pitch, pitch_rate_param[0], pitch_rate_param[1], pitch_rate_param[2], DIRECT);
PID roll_rate_PID(&roll_stab_output, &roll_output, &gyro_roll, roll_rate_param[0], roll_rate_param[1], roll_rate_param[2], DIRECT);
PID yaw_rate_PID(&yaw_stab_output, &yaw_output, &gyro_yaw, yaw_rate_param[0], yaw_rate_param[1], yaw_rate_param[2], DIRECT);


void setup()
{
  Serial.begin(115200);
 
  pitch_stab_PID.SetMode(AUTOMATIC);

  pitch_rate_PID.SetMode(AUTOMATIC);

  // Delay of 10 ms
  delay(10);
}

void loop()
{
  // Get values from 4 RC channels (Throttle, Yaw, Pitch and Roll)
  channels[0] = 1200;
  channels[1] = 1100;
  channels[2] = 1100;
  channels[3] = 1100;

  // Constrain RC channel values - yaw is contrained between +/-150 degrees
  // Pitch and roll and contrained between +/- 15 degrees
  rcthr = 1410;

  // Get Euler angles and Gyro values
  roll = -4.0;
  pitch = 0.6;
  yaw = 6.0;
      
  gyro_roll = 0.1;
  gyro_pitch = 0.2;
  gyro_yaw = 0.3;
  
  // Stablizing PID
  pitch_stab_PID.Compute(); 
  roll_stab_PID.Compute();
  yaw_stab_PID.Compute();

  Serial.println(pitch_stab_output);
  
  pitch_stab_output = constrain(pitch_stab_output, -50, 50);
  roll_stab_output = constrain(roll_stab_output, -50, 50);
  yaw_stab_output = constrain(yaw_stab_output, -100, 100);

  pitch_rate_PID.Compute();

  pitch_output = constrain(pitch_output, -175, 175);

  rcpit = rcpit + 0.1;

  Serial.flush();    
}
