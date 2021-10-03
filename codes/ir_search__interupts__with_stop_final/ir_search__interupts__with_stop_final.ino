//in case we want to fine-tune this program , this "variables" need to be change:
//for near_distance_stop we need to change:
//                                                         ir1_sensor_stop_distance , and ir2_sensor_stop_distance (theoretically need to be the same)
//and for adjusting our frequency range we need to change :
//                                                         timeout in pulseln function , and the frequency range in the "if" functions

#define ir1_sensor_stop_distance_min 30     //in case they are in good place ir1 and ir2 need to be the same
#define ir1_sensor_stop_distance_max 300
#define ir2_sensor_stop_distance_min 30
#define ir2_sensor_stop_distance_max 300

//#define timeout 2500
//#define frequency_min 990
//#define frequency_max 1100

#define timeout 1500
#define frequency_min 1650
#define frequency_max 1850

//#define timeout 1500
//#define frequency_min 1650
//#define frequency_max 1850

int ir1_pin = A1;   //ir1,and ir2 are simple ir recievers for short distance.
int ir2_pin = A2;
int ir1_value=0;
int ir2_value=0;
int result = 0;  //result is a flag - if HIGH we are near the transmmiters , and need to inform mega to stop.    //i have changed it from loop 
int near_to_transmmiter_pin = 9; // nano's pin 9 -->mega's pin 26 , if HIGH so the car is near to the transmmiters and need to stop.

int _5v_with_reset_pin = 13;
int ir_left_pin = 12;
int ir_forward_pin = 7;
int ir_right_pin = 3;
int data_msb_pin = 11;  //to the mega (pin 25) ,and to the green led
int data_lsb_pin = 10;   //to the mega (pin 24) ,and to the yellow led
//data = 01(yellow) ----> left  ,10(green) ----> right   ,11(yellow and green) ----> forward
int interrupt_pin = 4;  //to the mega (pin 21) ,and to the red led

//Variables for calculating the received frequency
float Htime ;
float Ltime ;
float Ttime ;
float frequency_left ;
float frequency_right ;
float frequency_forward ;

void setup()
{
  // put your setup code here, to run once:

  //Serial.begin(9600);

  //ir sensors for short distance
  pinMode(near_to_transmmiter_pin, OUTPUT);
  digitalWrite(near_to_transmmiter_pin, LOW);                       //i have changed it from loop

  //ir sensors for long distance
  pinMode (_5v_with_reset_pin , OUTPUT);
  pinMode (ir_left_pin , INPUT);
  pinMode (ir_forward_pin , INPUT);
  pinMode (ir_right_pin , INPUT);
  pinMode (data_msb_pin, OUTPUT);
  pinMode (data_lsb_pin , OUTPUT);
  pinMode (interrupt_pin , OUTPUT);
  //digitalWrite (_5v_with_reset_pin, LOW);
  digitalWrite (data_msb_pin, LOW);
  digitalWrite (data_lsb_pin, LOW);
  digitalWrite (interrupt_pin, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:

  //checking if the car is near the transmmiters.
  ir1_value = analogRead(ir1_pin);
  ir2_value = analogRead(ir2_pin);
  //if the car near to the transmmiters , near_to_transmmiter_pin (pin 9) go HIGH ,interrupt_pin go HIGH (for 10 milly seconds) , and yellow&green leds are blinking (for 1 second)
  if (ir1_value > ir1_sensor_stop_distance_min && ir1_value < ir1_sensor_stop_distance_max || ir2_value > ir2_sensor_stop_distance_min && ir2_value < ir2_sensor_stop_distance_max)
  {
    result = 1; //flag for saving the state we are near the transmmiters.
    digitalWrite (near_to_transmmiter_pin , HIGH);
    digitalWrite (interrupt_pin , HIGH);
    delay(10);
    digitalWrite (interrupt_pin , LOW);
    while (1)
    {
      digitalWrite (data_msb_pin , LOW);
      digitalWrite (data_lsb_pin , LOW);
      delay(1000);
      digitalWrite (data_msb_pin , HIGH);
      digitalWrite (data_lsb_pin , HIGH);
      delay(1000);
    }
  }

  //reset the ir reciever ("tsop 34838f") for 10 milly seconds.
  digitalWrite (_5v_with_reset_pin, LOW);
  delay(10);
  digitalWrite (_5v_with_reset_pin, HIGH);
  delay(10);

  // IR SEARCH LEFT (data out=01 ,interrupt pin go high (for 10 milly seconds) , yellow led go on , and red led blink)
  Htime = pulseIn(ir_left_pin, HIGH, timeout); //read high time
  Ltime = pulseIn(ir_left_pin, LOW, timeout);   //read low time
  Ttime = Htime + Ltime;
  frequency_left = 1000000 / Ttime; //getting frequency with Ttime is in Micro seconds
  //Serial.print (frequency_left);
  //Serial.print ("  ");
  if (frequency_left > frequency_min && frequency_left < frequency_max)
  {
    digitalWrite (data_msb_pin , LOW);
    digitalWrite (data_lsb_pin , HIGH);
    digitalWrite (interrupt_pin , HIGH);
    delay(10);
    digitalWrite (interrupt_pin , LOW);
    delay(500);
  }

  // IR SEARCH RIGHT (data out=10 ,interrupt pin go high (for 10 milly seconds) , green led go on , and red led blink)
  Htime = pulseIn(ir_right_pin, HIGH, timeout); //read high time
  Ltime = pulseIn(ir_right_pin, LOW, timeout);   //read low time
  Ttime = Htime + Ltime;
  frequency_right = 1000000 / Ttime; //getting frequency with Ttime is in Micro seconds
  //Serial.print (frequency_right);
  //Serial.print ("  ");
  if (frequency_right > frequency_min && frequency_right < frequency_max)
  {
    digitalWrite (data_msb_pin , HIGH);
    digitalWrite (data_lsb_pin , LOW);
    digitalWrite (interrupt_pin , HIGH);
    delay(10);
    digitalWrite (interrupt_pin , LOW);
    delay(500);
  }

  // IR SEARCH FORWARD (data out=11 and yellow&green leds go on)
  Htime = pulseIn(ir_forward_pin, HIGH, timeout); //read high time
  Ltime = pulseIn(ir_forward_pin, LOW, timeout);   //read low time
  Ttime = Htime + Ltime;
  frequency_forward = 1000000 / Ttime; //getting frequency with Ttime is in Micro seconds
  //Serial.println (frequency_forward);
  if (frequency_forward > frequency_min && frequency_forward < frequency_max)
  {
    digitalWrite (data_msb_pin , HIGH);
    digitalWrite (data_lsb_pin , HIGH);
  }

}
