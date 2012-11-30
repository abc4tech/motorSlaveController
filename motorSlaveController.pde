/****************************************************************************
*  Super Amazing Motor Controller 
*    by Bill Porter
*
*   Copyright (c) 2009 billporter.info
*   (please link back if you use this code!)
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*   
*  credit for Interupt code:
*              http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
*
**************************************************************************/
/* IO pins
Digital
0  Serial TX
1  Serial RX
2  Left Encoder for pulse
3  Right Encoder for pulse
4  
5  Left Motor PWM
6  Right Motor PWM
7  Left Encoder for Direction   PortD 10000000
8  Right Encoder for direction   PortB 00000001
9  
10  Left Motor Direction
11  Right Motor Direction
12
13  LED on board

Analog
0  
1  Battery Voltage
2  Battery Current
3  


Serial command structure

Sent from motorcontroller

  string structure ::  S distance_traveled battery_voltage battery_current*CS
  distance is two byte, voltage and current is 2 byte

Sent to Motorcontroller
    string structure ::  C Left_speed_req right_speed_req*CS
    
The idea is the master uC will send speed commands, and the motorcontroller
    will respond with sensor data. If no data is recieved for 2 seconds, all stop is given


*/
#define LEFT_ENC_DIR 7 //  PortD 10000000
#define RIGHT_ENC_DIR 8 //  PortB 00000001

#define LEFT_MOTOR_DIR 10 //PortB 00000100
#define RIGHT_MOTOR_DIR 11  //PortB 00001000

#define LEFT_MOTOR_PWM 5
#define RIGHT_MOTOR_PWM 6

//#define TOGGLE_IO        13  //Arduino pin to toggle in timer ISR   used for showing freq
#define K_P .4  //porpotional control Gain
#define K_D 4  //Derivitave control gain
#define TIMER_CLOCK_FREQ 7812.5 //2MHz for /8 prescale from 16MHz

#define CURRENT_LIMIT 3000  //start cutting power at this limit. 
#define SEND_SENTENCE_LENGTH  8 //9 with checksum
#define RECEIVE_SENTENCE_LENGTH 4 //5 with checksum
#define TIMEOUT_LENGTH 70 //one second timeout equals 35 clicks, so 70 clicks is 2 seconds. 


//On my Robot it's exactly 39cm per rotation 
// 30:1 Gearbox
// 100 clicks per encoder revolution
// = 3000 clicks per wheel rotation
//  = 76.923 clicks per cm

unsigned int latency;
unsigned int latencySum;
unsigned int sampleCount;
unsigned char timerLoadValue;
int timerflag=0;


signed int tempclicks_l=0;
signed int power_l=0;
signed int count_l=0;
signed char speed_req_l = 0;  // in centimeters per second
signed int clicks_req_l =0;
signed int distance_trav_l =0;
signed int temp_dist_l = 0;
signed int error_l =0;
signed int lasterror_l=0;
signed int derror_l=0;

signed int tempclicks_r=0;
signed int power_r=0;
signed int count_r=0;
signed char speed_req_r = 0;    // in centimeters per second
signed int clicks_req_r =0;
signed int distance_trav_r =0;
signed int temp_dist_r = 0;
signed int error_r =0;
signed int lasterror_r=0;
signed int derror_r=0;

//int check=0;
signed int voltage=0;  //in mV
signed int current=0;  //in mA
signed int distance_moved = 0; 


struct 
{
  char msgHeader;
  signed int data[3];
  char star;
  byte CS;
} msg;

signed char speed_l_temp = 0;
signed char speed_r_temp = 0;
int no_command_count = 0;

void setup(void) {
  pinMode(LEFT_ENC_DIR,INPUT);
  pinMode(RIGHT_ENC_DIR,INPUT);
 // pinMode(TOGGLE_IO,OUTPUT);//Set the pin we want the ISR to toggle for output to show freq
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);

  attachInterrupt(0, countpulse_l, RISING); //pin 2   encoder interupts
  attachInterrupt(1, countpulse_r, RISING); //pin3
 
  //Start up the serial port
  Serial.begin(57600);
  
  //Signal the program start
  Serial.println("Motor Control Program v1.0");
  
  //Start the control loop timer and get the timer reload value.
  timerLoadValue=SetupTimer2(35);
  
  //Output the timer reload value
 // Serial.print("Timer2 Load:");
 //Serial.println(timerLoadValue,HEX);
  msg.msgHeader = 'S';
  msg.star = '*';
}

void loop(void) {
 
  
  while (timerflag == 0) //loop until I need to recalculate motor control
  {
    
  delay(10); //not stable unless this delay is in here. No idea why

//stuff to do outside of control loop function

//calculate clicks_req from speed_req
clicks_req_l = ( speed_req_l * 77 ) / 35;
clicks_req_r = ( speed_req_r * 77 ) / 35;

//read temp_dist for distance calcs and convert to cm while leaving remainder to avoid rounding error over time
distance_trav_l += temp_dist_l / 77;
distance_trav_r += temp_dist_r / 77;
temp_dist_l = temp_dist_l % 77;
temp_dist_r = temp_dist_r % 77;

distance_moved = (distance_trav_l / 2) + (distance_trav_r / 2);

 // ++check; //for diag
 
 if (Serial.available() >= RECEIVE_SENTENCE_LENGTH) //if a full sentence is waiting for processing
     if (Serial.read() == 'C')  //nested failsafe, check if command char is present and at front. If not should resync by trashing byte
 {
		//first revision will not have checksum, will have to rethink if
                //i decide i want checksum
                 //   string structure ::  C Left_speed_req right_speed_req*CS
                
                 // read the incoming byte:
		speed_l_temp = Serial.read();
                speed_r_temp = Serial.read();  
                
                
                if (Serial.read() == '*')//one last check to confirm good command (without checksum)
                  {
                    speed_req_l = speed_l_temp; //set requested speeds. 
                    speed_req_r = speed_r_temp;
                    no_command_count = 0;
                  
                  
                  //now that received command has been Santafied (checked twice :-), send sensor data
                  
                   //Sensor data to send out
                   //distance traveled; battery voltage and current
                   //   string structure ::  S,distance_traveled,battery_voltage,battery_current*CS
                   //distance is two byte, voltage and current is 2 byte
                  //AVR's store int's backwards, had to do some fancy footwork to get high byte first
                       msg.data[0] = ((distance_moved << 8) | ((unsigned int)distance_moved >>8)) ;
                       msg.data[1] = ((voltage << 8) | ((unsigned int)voltage >>8)) ;
                       msg.data[2] = ((current << 8) | ((unsigned int)current >>8)) ;
                         msg.CS = 0;
 
                       //calculate checksum optional
                       byte *p = (byte*)&msg;
                       /*
                         for (int i = 0; i < msg_size - 2; i++)
                          {
	                      msg.CS ^= (byte)p[i];
                            }
                            */
                       //send sensor packet
                        Serial.write(p, SEND_SENTENCE_LENGTH);
                        
                        //Reset distance traveled values.
                        distance_trav_l=0;
                        distance_trav_r=0;

                  }
          }
 }

  //stuff to do after control loop timer interupts and sets flag
  
  //clear timer flag
  timerflag=0;
  
  // Computer error functions
   error_l = clicks_req_l - tempclicks_l;
   error_r = clicks_req_r - tempclicks_r;
   temp_dist_l += tempclicks_l;   //capture clicks for distance traveled calcs
   temp_dist_r += tempclicks_r;
   derror_l = ((error_l - lasterror_l) / 2);
   derror_r = ((error_r - lasterror_r) / 2);
   lasterror_l = error_l;
   lasterror_r = error_r;
   
   
    //check failsafe condition
  if (no_command_count++ >= (TIMEOUT_LENGTH - 1)) //failsafe for no commands, kill power after so long of no commands
     {
       speed_req_l = 0;
       speed_req_r = 0;
     }
       
     
  
  voltage=analogRead(1);    //read battery voltage and current
  current=analogRead(2);
  
  voltage = (((voltage + 1) / 13.039) +.6) * 1000;   //compute values from ADC
  current = ((current - 3.0128 ) / 6.9627) * 1000;
  
  if (current > CURRENT_LIMIT)      //limit power if current is too high
    {
       if (power_l < 0)
           power_l += 30;
       else
           power_l -= 30;
           
       if (power_r < 0)
           power_r += 30;
       else
           power_r -= 30;
    }
    
  else   //compute PD control equations if within current limit
    {
      power_l+= ((K_P * error_l) + (K_D * derror_l));
      power_r+= ((K_P * error_r) + (K_D * derror_r));
  
      if (power_l >255)      //check for out of range values
          power_l = 255;
      
      else if (power_l < -255)
          power_l = -255;
      
      if (power_r >255)
          power_r = 255;
      
      else if (power_r < -255)
          power_r = -255;
          
      if (clicks_req_l == 0  && tempclicks_l == 0) //check for all stop requested and met, then cut power. Fixes humming when stoped.
         power_l =0;
 
      if (clicks_req_r == 0  && tempclicks_r == 0)
         power_r =0;  
     }  
    
    
  if (power_l < 0)    //write direction and speed to H-bridge
      {
        digitalWrite(LEFT_MOTOR_DIR, LOW);
        analogWrite(LEFT_MOTOR_PWM,(0 - power_l));
      }
  else 
      {
      digitalWrite(LEFT_MOTOR_DIR, HIGH);
      analogWrite(LEFT_MOTOR_PWM,power_l);
      }
      
   if (power_r < 0)
      {
        digitalWrite(RIGHT_MOTOR_DIR, LOW);
        analogWrite(RIGHT_MOTOR_PWM,(0 - power_r));
      }
  else 
      {
      digitalWrite(RIGHT_MOTOR_DIR, HIGH);
      analogWrite(RIGHT_MOTOR_PWM,power_r);
      }
      
  //Debug stuff
  /*
  Serial.print(tempclicks_l);
  Serial.print(",");
  Serial.print(power_l);
  Serial.print(",");
   Serial.print(tempclicks_r);
  Serial.print(",");
  Serial.print(power_r);
  Serial.print(";   ");
  
  
  Serial.print(distance_trav_l);
  Serial.print(";   ");
  
   Serial.print(check);
  Serial.print(";   ");
  check=0;
  
  Serial.print(voltage);
  Serial.print(";");
  Serial.println(current);
  */
  

}

//Setup Timer2.
//Configures the ATMegay168 8-Bit Timer2 to generate an interrupt at the specified frequency.
//Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
//See the example usage below.
unsigned char SetupTimer2(float timeoutFrequency){
  unsigned char result; //The value to load into the timer to control the timeout interval.

  //Calculate the timer load value
  result=(int)((257.0-(TIMER_CLOCK_FREQ/timeoutFrequency))+0.5); //the 0.5 is for rounding;
  //The 257 really should be 256 but I get better results with 257, dont know why.

  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timmer clock = 16MHz/8 = 2Mhz or 0.5us
  //The /8 prescale gives us a good range to work with so we just hard code this for now.
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20; 

  //Timer2 Overflow Interrupt Enable   
  TIMSK2 = 1<<TOIE2;

  //load the timer for its first cycle
  TCNT2=result; 
  
  return(result);
}

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {

  //Toggle the IO pin to the other state.
  //digitalWrite(TOGGLE_IO,!digitalRead(TOGGLE_IO));
   tempclicks_l=count_l;
   tempclicks_r=count_r;
   timerflag=1;
   count_l=0;
   count_r=0;
   
  //Capture the current timer value. This is how much error we have
  //due to interrupt latency and the work in this function
  latency=TCNT2;

  //Reload the timer and correct for latency.  //Reload the timer and correct for latency.  //Reload the timer and correct for latency.
  TCNT2=latency+timerLoadValue; 
}
void countpulse_l(){
  //count pulse in the correct direction using direct port reading to save cycles
  if (PIND & 0b10000000)
  count_l--;
  else
  count_l++;
  
}

void countpulse_r(){
  //count pulse in the correct direction using direct port reading to save cycles
   if (PINB & 0b00000001)
  count_r--;
  else
  count_r++;
}
