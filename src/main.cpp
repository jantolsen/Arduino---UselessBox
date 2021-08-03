// Useless Box
// ------------------------
// 
// Version:
// 0.1 - Initial Version
//       [02.08.2021] Jan T. Olsen


#include "Arduino.h"
#include "Servo.h"

// Initialization of pins
  
  // Input(s)
  int pin_switch_A = 2;     // Digital-Pin (D2) for Toggle-Switch (Position A)
  int pin_switch_B = 3;     // Digital-Pin (D3) for Toggle-Switch (Position B)
  int pin_pot = 0;          // Analog-Pin (A0) for Potentiometer
  
  // Output(s)
  int pin_servo = 9;        // Servo-Motor-Pin (D9)

// Internal Variable(s)
  Servo servo;        // Create Servo-Object to control a servo motor
  int servo_position; // Servo-Motor Position [deg]
  int servo_time;     // Servo-Motor Speed [ms]
  int state;      // State-ID used for State-Machine
  bool flag = false;  // Help Memory

// Setup
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Servo
  servo.attach(pin_servo);        // Attaches the servo on Pin D9 to the Servo-object

  // Digital Pin assignment
  pinMode(pin_switch_A, INPUT);   // Assign as Digital-Input
  pinMode(pin_switch_B, INPUT);   // Assign as Digital-Input
}

// Servo Control with Potentiometer
void ServoControl_Potmeter(int pin_potmeter, Servo servo_object)
{
  // Local Variable(s)
  int angle_setpoint;                                     // Angle Setpoint variable for Servo-Motor

  // Setpoint computation
  angle_setpoint = analogRead(pin_potmeter);              // reads the value of the potentiometer (value between 0 and 1023)
  angle_setpoint = map(angle_setpoint, 0, 1023, 0, 180);  // scale it to use it with the servo (value between 0 and 180)

  // Servo Control
  servo.write(angle_setpoint);                            // Set Servo-Motor position according to angle setpoint

  // Publish Servo-Motor angle setpoint
  Serial.println(angle_setpoint);
}

// Servo Control - Speed Control
// -------------------------
// Input(s)
//  Angle   - Angle Setpoint [deg]
//  Time    - Time Setpoint [ms]
//  Pin_potmeter - Digital Pin used for Servo-Motor
//  Servo_Object - Servo Object used to communicate with the Servo-Motor
void ServoControl_Speed(int angle, int time, int pin_potmeter, Servo servo_object)
{
  // Local Variable(s)
  // -------------------------
    int angle_setpoint = angle;               // Angle Setpoint value for Servo-Motor [deg]
    int angle_current = servo_object.read();  // Angle current value for Servo-Motor [deg]
    int pos;                                  // Servo-Motor Angle Position (used in for-loop) [deg]                         
    int wait_delay;                           // Increment Wait time delay [ms]
    int angle_diff;                           // Angle Difference between current and setpoint value
    // int angle_min = 0;                        // Minimum Angle for Servo-Motor [deg]
    // int angle_max = 180;                      // Maximum Angle for Servo-Motor [deg]

  // Speed Control
  // -------------------------

    // Angle Difference
    angle_diff = angle_setpoint - angle_current;

    // Positive Speed
    if(angle_diff > 0)
    {
      // Positive Sweep
      // Sweep for Current Angle to Setpoint Angle
      // For-Loop from current to setpoint angle 
      for (pos = angle_current; pos <= angle_setpoint; pos += 1)
      {
        //Serial.println(pos);
        servo_object.write(pos);                    // Set Servo-Motor position
        wait_delay = round(time / abs(angle_diff)); // Calculate increment wait delay
        //Serial.println(wait_delay);
        delay(wait_delay);                          // Wait [ms] for the Servo-Motor to reach the target position
      }
      
    }
    // Negative Speed
    else if(angle_diff < 0)
    {
      // Negative Sweep
      // Sweep for Current Angle to Setpoint Angle
      // For-Loop from current to setpoint angle 
      for (pos = angle_current; pos >= angle_setpoint; pos -= 1)
      {
        //Serial.println(pos);
        servo_object.write(pos);                    // Set Servo-Motor position
        wait_delay = round(time / abs(angle_diff)); // Calculate increment wait delay
        //Serial.println(wait_delay);
        delay(wait_delay);                          // Wait [ms] for the Servo-Motor to reach the target position
      }
    } 
    // Zero Speed
    else
    {
      // Motor already at angle-setpoint
      // Nothing should happend
    }
    Serial.println(angle_diff);

}

// State Machine
// -------------------------
void StateMachine(int state)
{

  int state_id = state;


  switch (state_id)
  {
    
    // Case 1
    case 1:

      // Pre-Trigger Push
      // -------------------------
      servo_position = 140;
      servo_time = 1000;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(1000);

      // Trigger Push
      // -------------------------
      servo.write(180);
      delay(15);

      // Break Case-statement
      if(digitalRead(pin_switch_A) == LOW)
      {
        servo.write(1);
        delay(15);
        break;
      }

    // Case 2
    case 2:

      // Pre-Trigger Push
      // -------------------------
      servo_position = 140;
      servo_time = 1000;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(2000);

      servo_position = 10;
      servo_time = 3000;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(500);

      // Trigger Push
      // -------------------------
      servo.write(180);
      delay(15);

      // Break Case-statement
      if(digitalRead(pin_switch_A) == LOW)
      {
        servo.write(1);
        delay(15);
        break;
      }

    // Case 3
    case 3:

      // Pre-Trigger Push
      // -------------------------
      servo_position = 110;
      servo_time = 2000;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(500);

      servo_position = 10;
      servo_time = 100;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(1000);

      servo_position = 110;
      servo_time = 2000;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(500);

      servo_position = 10;
      servo_time = 100;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(1000);

      // Trigger Push
      // -------------------------
      servo.write(180);
      delay(15);

      // Break Case-statement
      if(digitalRead(pin_switch_A) == LOW)
      {
        servo.write(1);
        delay(15);
        break;
      }

    // Case 4
    case 4:

      // Pre-Trigger Push
      // -------------------------
      servo_position = 140;
      servo_time = 500;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      delay(50);

      // Trigger Push
      // -------------------------
      servo.write(180);
      delay(15);

      // Break Case-statement
      if(digitalRead(pin_switch_A) == LOW)
      {
        servo.write(1);
        delay(15);
        break;
      }
    
    // Case 5
    case 5:
    
      delay(4000);

      // Trigger Push
      // -------------------------
      servo.write(180);
      delay(15);

      // Break Case-statement
      if(digitalRead(pin_switch_A) == LOW)
      {
        servo.write(1);
        delay(15);
        break;
      }

    // Case 6 to 14
    case 6 ... 14:

      delay(500);

      // Trigger Push
      // -------------------------
      servo.write(180);
      delay(15);

      // Break Case-statement
      if(digitalRead(pin_switch_A) == LOW)
      {
        servo.write(1);
        delay(15);
        break;
      }

    // Case 15 to 30
    case 15 ... 20:

      delay(500);

      // Pre-Trigger Push
      // -------------------------
      servo_position = 140;
      servo_time = 250;
      ServoControl_Speed(servo_position, servo_time, pin_pot, servo);
      //delay(50);

      // Trigger Push
      // -------------------------
      servo.write(180);
      delay(15);

      // Break Case-statement
      if(digitalRead(pin_switch_A) == LOW)
      {
        servo.write(1);
        delay(15);
        break;
      }

    default:
      servo.write(1);
      delay(15);
      break;
  }
  
}

void loop()
{
  

  // Servo Control - Potentiometer
  // -------------------------
  //ServoControl_Potmeter(pin_pot, servo);

  // Servo Control - State Machine
  // -------------------------
  if(digitalRead(pin_switch_A) == HIGH)
  {
    int state_min = 1;
    int state_max = 20 + 1;
    
    StateMachine(random(state_min, state_max));

  }
  else
  {
    servo.write(1);
    delay(15);
  }
  
  delay(1);        // delay in between reads for stability
}