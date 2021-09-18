//in case we want to fine-tune this program (for example new battery) , this "variables" need to be change:
//                                                                                                         speed/delay defines
//                                                                                                         distance defines
//                                                                                                     ??? distance_L , distance_R , distance_LL , distance_RR ???

#include <AFMotor.h>
#include <IRremote.h>

/////__SENSORS NAMES DEFINITION__//////
#define FORWARD_L 0 //Sensor forward left number 0
#define FORWARD_R 1 //Sensor forward right number 1
#define RIGHT 2 //Sensor right side number 2
#define LEFT 3//Sensor left side number 3

/////__SPEED/DELAY DEFINES__/////
#define FORWARD_SPEED 80 //Set forward speed
#define REVERSE_SPEED 65 //Set reverse speed
#define FORWARD_LOW_SPEED 50 //Set forward low speed
#define TURN_LEFT_SPEED 150 //Set turn left speed
#define TURN_RIGHT_SPEED 150 //Set turn right speed
#define TURN_LEFT_DELAY 155 //Set delay for turning left 
#define TURN_RIGHT_DELAY 180 //Set delay for turning right
//#define STEP_RIGHT_TURN_DELAY 80 //Set delay time for turning steps to the right
#define STEP_RIGHT_TURN_SPEED 60 //Set turn right steps speed                           //when ir recieved
//#define STEP_LEFT_TURN_DELAY 150 //Set delay time for turning steps to the left
#define STEP_LEFT_TURN_SPEED 60 //Set turn left steps speed                             //when ir recieved 
#define DELAY_FORWARD_AFTER_TURN 200


/////__DISTANCES DEFINES__/////
#define CLEAR_DISTANCE_FOR_SIDE_SENSOR 45
#define STOP_DISTANCE_LIMIT 40
#define CLEAR_DISTANCE_LIMIT 90

/////__DEFINE SENSORS PINS__//////
uint8_t trigPin[] = { 18 , 16 , 14 , 20 };
uint8_t echoPin[] = { 19 , 17 , 15 , 22 };

/////__DEFINE LEDS PINS__//////
uint8_t led_g_vcc = 37; // Green led (MEGA)
uint8_t led_r_vcc = 35; // Red led (MEGA)
uint8_t led_y_vcc_right = 33; // Yellow right led (MEGA)
uint8_t led_y_vcc_left = 31; // Yellow left led (MEGA)
uint8_t side; // Side legend: 1=RIGHT , 2=LEFT, 3=REVERSE

uint8_t Speed = 0;
long duration, cm;
long distance_L = 50;
long distance_R = 50;
long distance_RR = 50;
long distance_LL = 50;
int interrupt_pin = 21;
int interrupt_data_msb_pin = 25;
int interrupt_data_lsb_pin = 24;
int near_to_transmmiter_pin = 26;
int ir_dir; // flag for ir direction : 0=NO IR , 1=LEFT , 2=RIGHT , 3=FORWARD

AF_DCMotor FR(3);
AF_DCMotor FL(4);
AF_DCMotor RL(1);
AF_DCMotor RR(2);

void ir(void);                                // ISR
void choose_dir(void);                        //
int check_sides(void);                        //
void forward_routine(void);                   //
void turn_step_right(void);                   //
void turn_step_left(void);                    //
void turn_left(void);                         //
void turn_right(void);                        //
void forward_after_right(void);               //
void forward_after_left(void);                //
void Stop(void);                              //
void forward(uint8_t Speed);                  //
void reverse(void);                           //
void update_position_L(void);                 // ultrasonic FL
void update_position_R(void);                 // ultrasonic FR
void update_position_RR(void);                // ultrasonic RR
void update_position_LL(void);                // ultrasonic LL
long read_sensor(uint8_t sensor_num);         //
void trigger_sensor(uint8_t sensor_num);      //

void setup()
{

  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(trigPin[i], OUTPUT);// set the trig pin to output (Send sound waves)
    pinMode(echoPin[i], INPUT);// set the echo pin to input (recieve sound waves)
  }

  pinMode(led_r_vcc, OUTPUT);
  pinMode(led_g_vcc, OUTPUT);
  pinMode(led_y_vcc_right, OUTPUT);
  pinMode(led_y_vcc_left, OUTPUT);
  digitalWrite(led_r_vcc, HIGH);
  digitalWrite(led_g_vcc, HIGH);
  digitalWrite(led_y_vcc_right, HIGH);
  digitalWrite(led_y_vcc_left, HIGH);

  attachInterrupt(digitalPinToInterrupt(interrupt_pin), ir, RISING);
  pinMode(interrupt_data_msb_pin, INPUT);
  pinMode(interrupt_data_lsb_pin, INPUT);
  pinMode(near_to_transmmiter_pin, INPUT);
  delay(1000);
  
}

void ir()// Side legend: 0=NO IR , 1=LEFT , 2=RIGHT , 3=FORWARD .
{
  Stop();
  if (digitalRead(near_to_transmmiter_pin) == HIGH)
  {
    Stop();
    while (1)
    {
      cli();  //ignoring more interrupts
      digitalWrite(led_y_vcc_right, HIGH);
      digitalWrite(led_y_vcc_left, HIGH);
    }
  }
  ir_dir = digitalRead(interrupt_data_msb_pin) * 2 + digitalRead(interrupt_data_lsb_pin);
  if (ir_dir == 1)
  {
    while (ir_dir != 3 && ir_dir != 2)
    {
      digitalWrite(led_y_vcc_left, HIGH);
      turn_step_left();
      ir_dir = digitalRead(interrupt_data_msb_pin) * 2 + digitalRead(interrupt_data_lsb_pin);
    }
  }
  if (ir_dir == 2)
  {
    while (ir_dir != 3 && ir_dir != 1)
    {
      digitalWrite(led_y_vcc_right, HIGH);
      turn_step_right();
      ir_dir = digitalRead(interrupt_data_msb_pin) * 2 + digitalRead(interrupt_data_lsb_pin);
    }
  }
  Stop();
  digitalWrite(led_r_vcc, HIGH);
  digitalWrite(led_y_vcc_left, LOW);
  forward(FORWARD_SPEED);

}


void loop()
{
  forward_routine();
  choose_dir();
}

void choose_dir()
{
  side = check_sides();
  if (side == 1)
  {
    turn_left();
    forward_after_left();
  }
  if (side == 2)
  {
    turn_right();
    forward_after_right();
  }
  if (side == 3)
  {
    digitalWrite(led_y_vcc_right, HIGH);
    digitalWrite(led_y_vcc_left, HIGH);
    reverse();
    do {
      update_position_LL();
      update_position_RR();
      delay(10);
      update_position_LL();
      update_position_RR();
      delay(10);
      update_position_LL();
      update_position_RR();
      delay(10);
      if (distance_RR > CLEAR_DISTANCE_FOR_SIDE_SENSOR)
      {
        delay(DELAY_FORWARD_AFTER_TURN);
        Stop();
        delay(100);
        turn_right();
        forward_after_right();
        break;
      }
      if (distance_LL > CLEAR_DISTANCE_FOR_SIDE_SENSOR)
      {
        delay(DELAY_FORWARD_AFTER_TURN);
        Stop();
        delay(100);
        turn_left();
        forward_after_left();
        break;
      }
    } while (1);
  }
}
int check_sides()
{ //1=LEFT ,2=RIGHT, 3=REVERSE

  update_position_LL();
  delay(50);
  update_position_LL();
  delay(50);
  update_position_LL();
  delay(50);
  if (distance_LL > CLEAR_DISTANCE_FOR_SIDE_SENSOR)
  {
    return 1;
  }
  update_position_RR();
  delay(50);
  update_position_RR();
  delay(50);
  update_position_RR();
  delay(50);
  if (distance_RR > CLEAR_DISTANCE_FOR_SIDE_SENSOR)
  {
    return 2;
  }
  else return 3;
}
void forward_routine()
{
  digitalWrite(led_r_vcc, LOW);
  digitalWrite(led_g_vcc, LOW);
  digitalWrite(led_y_vcc_right, LOW);
  digitalWrite(led_y_vcc_left, LOW);
  update_position_R();
  update_position_L();
  update_position_R();
  update_position_L();
  forward(FORWARD_SPEED);
  while (distance_R >= CLEAR_DISTANCE_LIMIT && distance_L >= CLEAR_DISTANCE_LIMIT)
  {
    update_position_R();
    update_position_L();
    update_position_R();
    update_position_L();
    digitalWrite(led_g_vcc, HIGH);
    digitalWrite(led_r_vcc, LOW);

  }
  //  Stop();
  //  delay(100);
  forward(FORWARD_LOW_SPEED);
  update_position_R();
  update_position_L();
  update_position_R();
  update_position_L();

  while (distance_R > STOP_DISTANCE_LIMIT && distance_L > STOP_DISTANCE_LIMIT)
  {
    update_position_R();
    update_position_L();
    update_position_R();
    update_position_L();
    digitalWrite(led_r_vcc, HIGH);
  }
  Stop();
  delay(200);
  digitalWrite(led_g_vcc, LOW);
  digitalWrite(led_r_vcc, LOW);
}

void turn_step_right()
{
  FR.run(BACKWARD);
  FL.run(FORWARD);
  RL.run(FORWARD);
  RR.run(BACKWARD);
  FR.setSpeed(STEP_RIGHT_TURN_SPEED);
  FL.setSpeed(STEP_RIGHT_TURN_SPEED);
  RL.setSpeed(STEP_RIGHT_TURN_SPEED);
  RR.setSpeed(STEP_RIGHT_TURN_SPEED);
}
void turn_step_left()
{
  FR.run(FORWARD);
  FL.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(FORWARD);
  FR.setSpeed(STEP_LEFT_TURN_SPEED);
  FL.setSpeed(STEP_LEFT_TURN_SPEED);
  RL.setSpeed(STEP_LEFT_TURN_SPEED);
  RR.setSpeed(STEP_LEFT_TURN_SPEED);
}
void turn_left()//Function need fix
{
  digitalWrite(led_y_vcc_right, LOW);
  digitalWrite(led_y_vcc_left, HIGH);
  FR.run(FORWARD);
  FL.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(FORWARD);
  FR.setSpeed(TURN_LEFT_SPEED + 20);
  FL.setSpeed(TURN_LEFT_SPEED);
  RL.setSpeed(TURN_LEFT_SPEED);
  RR.setSpeed(TURN_LEFT_SPEED + 20);
  delay(TURN_LEFT_DELAY);
  delay(50);
  Stop();
  delay(50);

  digitalWrite(led_y_vcc_left, LOW);
  return;
}
void turn_right()//Function need fix
{
  digitalWrite(led_y_vcc_right, HIGH);
  digitalWrite(led_y_vcc_left, LOW);

  FR.run(BACKWARD);
  FL.run(FORWARD);
  RL.run(FORWARD);
  RR.run(BACKWARD);
  FR.setSpeed(TURN_RIGHT_SPEED + 20);
  FL.setSpeed(TURN_RIGHT_SPEED);
  RL.setSpeed(TURN_RIGHT_SPEED);
  RR.setSpeed(TURN_RIGHT_SPEED + 20);
  delay(TURN_RIGHT_DELAY);
  delay(50);
  Stop();
  delay(50);
  digitalWrite(led_y_vcc_right, LOW);
  return;
}
void forward_after_right()
{
  forward(FORWARD_LOW_SPEED);
  do
  {
    update_position_R();
    update_position_L();
    update_position_LL();
    delay(10);
    update_position_R();
    update_position_L();
    update_position_LL();
    delay(10);

    if ((distance_L < STOP_DISTANCE_LIMIT) || (distance_R < STOP_DISTANCE_LIMIT))
    {

      Stop();
      if (distance_LL >= CLEAR_DISTANCE_FOR_SIDE_SENSOR)
      {
        turn_left();
        delay(100);
        Stop();
      }
      return;
    }

  } while (distance_LL <= CLEAR_DISTANCE_FOR_SIDE_SENSOR);
  delay(DELAY_FORWARD_AFTER_TURN);
  Stop();
  delay(50);
  //left_after_right();
  turn_left();
  delay(100);
  Stop();
}
void forward_after_left()
{
  forward(FORWARD_LOW_SPEED);
  do
  {
    update_position_R();
    update_position_L();
    update_position_RR();
    delay(10);
    update_position_R();
    update_position_L();
    update_position_RR();
    delay(10);

    if ((distance_L < STOP_DISTANCE_LIMIT) || (distance_R < STOP_DISTANCE_LIMIT))
    {
      Stop();
      if (distance_RR >= CLEAR_DISTANCE_FOR_SIDE_SENSOR)
      {
        turn_right();
        delay(100);
        Stop();
      }
      return;
    }

  } while (distance_RR <= CLEAR_DISTANCE_FOR_SIDE_SENSOR);
  delay(DELAY_FORWARD_AFTER_TURN);
  Stop();
  delay(50);
  //left_after_right();
  turn_right();
  delay(100);
  Stop();
}

void Stop()
{
  FR.setSpeed(0);
  FL.setSpeed(0);
  RL.setSpeed(0);
  RR.setSpeed(0);

  FR.run(RELEASE);
  FL.run(RELEASE);
  RL.run(RELEASE);
  RR.run(RELEASE);
}

void forward(uint8_t Speed)
{
  FR.run(FORWARD);
  FL.run(FORWARD);
  RL.run(FORWARD);
  RR.run(FORWARD);

  for (int speedSet = 0; speedSet < Speed; speedSet += 1) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    FR.setSpeed(speedSet + 20);
    FL.setSpeed(speedSet);
    RL.setSpeed(speedSet);
    RR.setSpeed(speedSet + 20);
    delay(10);
  }
}
void reverse()
{
  FR.run(BACKWARD);
  FL.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(BACKWARD);
  for (int speedSet = 0; speedSet < REVERSE_SPEED; speedSet += 1) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    FR.setSpeed(speedSet + 20);
    FL.setSpeed(speedSet);
    RL.setSpeed(speedSet);
    RR.setSpeed(speedSet + 20);
    delay(10);
  }
}
void update_position_L(void)
{
  distance_L = read_sensor(FORWARD_L);
//  Serial.print("FORWARD_LEFT=");
//  Serial.println(distance_L, DEC);
}
void update_position_R(void)
{
  distance_R = read_sensor(FORWARD_R);
//  Serial.print("FORWARD_RIGHT=");
//  Serial.println(distance_R, DEC);
}
void update_position_RR(void)
{
  distance_RR = read_sensor(RIGHT);
//  Serial.print("RIGHT=");
//  Serial.println(distance_RR, DEC);
}
void update_position_LL(void)
{
  distance_LL = read_sensor(LEFT);
//  Serial.print("LEFT=");
//  Serial.println(distance_LL, DEC);
}

long read_sensor(uint8_t sensor_num)
{
  delay(30);//Opition: echo delay (20)
  trigger_sensor(sensor_num);
  duration = pulseIn(echoPin[sensor_num], HIGH);
  cm = (duration / 2) / 29.1;
  return cm;
}

void trigger_sensor(uint8_t sensor_num)
{
  digitalWrite(trigPin[sensor_num], LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin[sensor_num], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin[sensor_num], LOW);
}
