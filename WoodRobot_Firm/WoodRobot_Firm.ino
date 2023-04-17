#include <Stepper.h>
#include <Servo.h>
#include <stdlib.h>

#define ENGAGE 1
#define DISENGAGE 0

int relay_vcc = 14;
int heater = 5;     // 15 for relay, 5 for heater 
int relay_gnd = 17;
int frontstop = A4;
int ThermistorPin = A3;   // Thermistor (analog pin 0)

int steppers_homes[3] = {50, 52, A5}; 

long steps_per_rot = 12800;

float cyc_rad = 336.36/2;

//{r, z, theta, press, Feed}
float stage_calib[4] = {steps_per_rot/5, steps_per_rot/10, round(((steps_per_rot+200)*95/32)/(2*360)), round((400*50/(62.8318530))*(1-0.05))} ;
float theta_feed_prp = round(cyc_rad*stage_calib[3]/(stage_calib[2])*0.0174533);

long stage_desired_position[4] = {0, 0, 0, 0};
long stage_max[2] = {round(107*(stage_calib[0])), round(490*(stage_calib[1]))};

long stage_positions[3] = {stage_max[0]-1, stage_max[1]-1, 0}; //Starts high so it can home to zero from greater than max

//mm, mm, deg
long jog_val[4] = {1, 1, 3, 1};

// Thermistor parameters
int Vo = 0;
float R1 = 100000;
float logR2, R2;
double c1 = 0.00042;
double c2 = 0.000253;

// Heater parameters
int driver, T;
int setpoint = 0;
int setpoint_saver = 150;
int P = 15;
int temps[5] = {0,0,0,0,0};
int tempsum = 0;

boolean manage_temp = false;
boolean joint_attached = false;

int n = 0;
String parse_val;

// Stepper parameters
const int stepsPerRevolution = 32767;

const long STEPS_TO_CLAMP = 60000;
const long Press_dig_stop = 90000;
int stepper_cmd = 0;

int arm_up = 90;
int arm_down = 40;

Stepper radial(steps_per_rot, 10,9); // 10 --> pulse, 9 --> dir
Stepper theta(steps_per_rot/2, 13,12); 
Stepper vertical(steps_per_rot, 7,6); 
Stepper clamp(32767, 3,2); 

Servo arm;  // create servo object to control arm
Servo pawl;  // create servo object to control pawl
Servo cutter; 

int cut_ang = 40;
int open_ang = 180;

void setup() {
  pinMode(heater, OUTPUT);  // set PWM output
  
  radial.setSpeed(50); //speed is in RPM;
  theta.setSpeed(20); //speed is in RPM;
  vertical.setSpeed(50); //speed is in RPM;
  clamp.setSpeed(3); //speed is in RPM;

  pinMode(frontstop, INPUT_PULLUP );    // sets the digital pin 7 as input

  //limit switches
  pinMode(steppers_homes[0],INPUT_PULLUP);
  pinMode(steppers_homes[1],INPUT_PULLUP);
  pinMode(steppers_homes[2],INPUT_PULLUP);

  pinMode(8,OUTPUT); //Set Pin 8 as dir
  pinMode (4,OUTPUT); //Set Pin 4 as pul

  pinMode(relay_vcc,OUTPUT);
  pinMode(relay_gnd,OUTPUT);

  digitalWrite(relay_vcc,1);
  digitalWrite(relay_gnd,0);
  
  // servo setup routine
  arm.attach(A1);
  pawl.attach(A0); 
  cutter.attach(A2);

  delay(1000);
  arm.write(arm_up);
  delay(1000);
  pawl.write(70);
  delay(1000);
  cutter.write(open_ang);


  Serial.begin(9600);
  Serial.println("waiting for input");
 
  while(Serial.available() == 0){
    
  }
  int input = Serial.parseInt();
  homing();

  Serial.println("Done Homing. Jog Stages:");

}

void loop() {

  //Read Commands
  serial_Check();

  //Adjust current temperature
  if(manage_temp){
  setTemp();
  }else{digitalWrite(heater,0);}

  //Move stages if there is currently error in system
  check_stages();
}

void serial_Check() {
 if (Serial.available() > 0) {
  parse_val = Serial.readString();
  Serial.println(parse_val);

   if(parse_val.indexOf("jog") >= 0){
    int dir = 1;
    if(parse_val.indexOf("b")>= 0){
       dir = -1;
     }
     Serial.println(jog_val[0]*dir);
      
       if(parse_val.indexOf("r") >= 0){
        stage_desired_position[0] = stage_desired_position[0] + mm_to_steps(jog_val[0]*dir, 0);
       }else if(parse_val.indexOf("z") >= 0){
        stage_desired_position[1] = stage_desired_position[1] + mm_to_steps(jog_val[1]*dir, 1);
       }else if(parse_val.indexOf("t") >= 0){
        stage_desired_position[2] = stage_desired_position[2] + mm_to_steps(jog_val[2]*dir, 2);
       }else if(parse_val.indexOf("f") >= 0){
        stage_desired_position[3] = stage_desired_position[3] + mm_to_steps(jog_val[3]*dir, 3);
       }
   }

   else if(parse_val.indexOf("move") >= 0){  
    if(parse_val.indexOf(",")<=0){Serial.println("Include comma before move value");}
    else{
      String amount = parse_val.substring(parse_val.indexOf(",")+1);
      int int_amount = amount.toInt();
     
      
      if(parse_val.indexOf("r") >= 0){
        if(int_amount > 107){Serial.println("Input value less than 107");} else{
        stage_desired_position[0] = stage_desired_position[0] + mm_to_steps(int_amount, 0);
        }
       }else if(parse_val.indexOf("z") >= 0){
        if(int_amount > 490){Serial.println("Input value less than 490");} else{
        stage_desired_position[1] = stage_desired_position[1] + mm_to_steps(int_amount, 1);
        }
       }else if(parse_val.indexOf("t") >= 0){
        stage_desired_position[2] = stage_desired_position[2] + mm_to_steps(int_amount, 2);
       }else if(parse_val.indexOf("f") >= 0){
        if(int_amount<0){digitalWrite(8,1);}
        else{digitalWrite(8,0);}
        for (int x=0; x<mm_to_steps(int_amount, 3);x++) //Repeat 400 times a revolution when setting is 400 on driver
          {
          digitalWrite(4,HIGH); //Output high
          delayMicroseconds(100); //Set rotate speed
          digitalWrite(4,LOW); //Output low
          delayMicroseconds(100); //Set rotate speed
          }
       }
    }
   }
   else if(parse_val.indexOf("joint") >= 0){ 
      make_joint();
   }
   else if(parse_val.indexOf("arm") >= 0){ 
      move_arm(ENGAGE);
   }
   else if(parse_val.indexOf("pawl") >= 0){ 
      move_pawl(ENGAGE);
   }
   else if(parse_val.indexOf("home") >= 0){ 
      homing();
   }
   else if(parse_val.indexOf("cut") >= 0){ 
      cut();
   }
   else if(parse_val.indexOf("read") >= 0){ 
      setTemp();
   }
   else if(parse_val.indexOf("circ") >= 0){ 
      make_joint();
      stage_desired_position[2] = stage_desired_position[2] + mm_to_steps(-30, 2); 
   }
   else if(parse_val.indexOf("layer") >= 0){ 
      stage_desired_position[0] = stage_desired_position[0] + mm_to_steps(-1, 0);
      stage_desired_position[1] = stage_desired_position[1] + mm_to_steps(18, 1);
   }
   else if(parse_val.indexOf("pos") >= 0){ 
      Serial.println("R:");
      Serial.println(stage_positions[0]/stage_calib[0]);
      Serial.println("Z:");
      Serial.println(stage_positions[1]/stage_calib[1]);
      Serial.println("T:");
      Serial.println(stage_positions[2]/stage_calib[2]);
   }
   else if(parse_val.indexOf("ex") >= 0){ //STOP ALL MOTORS
      stage_desired_position[0] = stage_positions[0];
      stage_desired_position[1] = stage_positions[1];
      stage_desired_position[2] = 0;
      stage_desired_position[3] = 0;
   }
   else if(parse_val.indexOf("temp") >= 0){
    if(parse_val.indexOf("on") >= 0){manage_temp = true; setpoint = 150;}
    else if(parse_val.indexOf("off") >= 0){manage_temp = false; setpoint = 0;}
   }
  }
}

void setTemp() {
       // Read temp + do moving average
      Vo = analogRead(ThermistorPin);
      R2 = R1 * (1023.0 / (float)Vo - 1.0);
      logR2 = log(R2);
      T = (int) (1.0 / (c1 + c2*logR2));
      T = abs(T - 273);
    
      tempsum = 0;
      for(int i = 0; i < 5; i++){
        if(i == 4){
          temps[4] = T;
        }else{
          temps[i] = temps[i + 1];
        }
        tempsum += temps[i];
      }
      T = abs(tempsum/sizeof(temps)); // do the moving average

      if(T > 200){
        setpoint = 0;
      }else{setpoint = setpoint_saver;}

     
    
      // Set heater output based on temp
      
//      driver = (setpoint - T)*P;
//      
//      if(driver > 255){
//        driver = 255;
//      }
//      else if (driver < 0){
//        driver = 0;
//      }
//
//      if((temps[4] < setpoint - 5 && T>setpoint-5) || temps[4] < setpoint - 10){
//        digitalWrite(heater,1);
//      }
      if(T < setpoint - 10){
        digitalWrite(heater,1);
      }
      else{
        digitalWrite(heater,0);
      }
      
      //analogWrite(heater, driver); // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
      Serial.println(T);
    
}

void make_joint(){
  if(!joint_attached){
    joint_attached = true;
  }
  
  move_arm(ENGAGE);
  delay(500);
  move_pawl(ENGAGE);

  clamp.setSpeed(10); 
  long steps = 0;
  while (steps < STEPS_TO_CLAMP+25000) {
    setTemp();
    
    clamp.step(100);
    steps += 100;
    // slow down the press speed when we are applying force
    if(steps >= STEPS_TO_CLAMP){
      clamp.setSpeed(2); 
    }
    if(steps > Press_dig_stop){return;}
    if(!digitalRead(frontstop)){break;}
   }

   for(int timer = 0; timer < 30; timer++){
     delay(1000);
     setTemp();
     if(timer%5 == 0){
      clamp.step(500); // try to clamp a bit more
     }
   }
 
  clamp.setSpeed(10); 
  while (digitalRead(steppers_homes[2])) {  
    setTemp();
    clamp.step(-100);
  }  

   move_pawl(DISENGAGE);
   delay(500);
   move_arm(DISENGAGE);
}

void cut(){
  if(joint_attached){
    joint_attached = false;
  }
}



// dir = 0 for down, dir = 1 for up
void move_arm(bool dir){
  int pos_arm = 0;
  
  if(dir == ENGAGE){
      if(arm.read() != arm_up){
        Serial.println("arm not in position to be engaged");
        return;
      }
      for (pos_arm = arm_up; pos_arm >= arm_down; pos_arm -= 1) {
        arm.write(pos_arm);             
        delay(15);                      
      }
  }
  else{
      if(arm.read() != arm_down){
        Serial.println("arm not in position to be disengaged");
        return;
      }
      for (pos_arm = arm_down; pos_arm <= arm_up; pos_arm += 1) {
        arm.write(pos_arm);              
        delay(15);                       
      }    
  }
}

// dir = 0 for down, dir = 1 for up
void move_pawl(bool dir){
  int pos_pawl = 0;
  if(dir == DISENGAGE){
      if(pawl.read() != 140){
        Serial.println("pawl not in position to be engaged");
        return;
      }
      for (pos_pawl = 140; pos_pawl >= 70; pos_pawl -= 1) {
        pawl.write(pos_pawl);             
        delay(15);                      
      }
  }
  else{
      if(pawl.read() != 70){
        Serial.println("pawl not in position to be disengaged");
        return;
      }
      for (pos_pawl = 70; pos_pawl <= 140; pos_pawl += 1) {
        pawl.write(pos_pawl);              
        delay(15);                       
      }    
  }
}

void check_stages(){
  long step_val = 1;
  
  for(int i=0; i<4; i++){
    long err = stage_desired_position[i]- stage_positions[i];
    if(err != 0){
        if(i == 0){
        if((digitalRead(steppers_homes[i]) != 0 || sign(err)>0)){
          if(stage_positions[i] < stage_max[i] || sign(err)<0){
              radial.step(step_val*sign(err));
              stage_positions[i] = stage_positions[i]+step_val*sign(err); 
          }else{
            stage_desired_position[i] = 490;
          }
        }
        }
        else if(i == 1){
        if ((digitalRead(steppers_homes[i]) != 0 || sign(err)>0)){
          if(stage_positions[i] < stage_max[i] || sign(err)<0){
            vertical.step(step_val*sign(err));
            stage_positions[i] = stage_positions[i]+step_val*sign(err);
          }else{
            stage_desired_position[i] = 107;
          }
        }
        } 
    }
    if(i == 2 && abs(stage_desired_position[i]) > 0){
          theta.step(step_val*sign(stage_desired_position[i]));
          stage_desired_position[i] = stage_desired_position[i]-step_val*sign(stage_desired_position[i]);

          if(joint_attached){
            if(sign(stage_desired_position[i])<0){digitalWrite(8,0);}
            else{digitalWrite(8,1);}
                    for (int x=0; x<step_val*theta_feed_prp;x++)//Repeat 400 times a revolution when setting is 400 on driver
                      {
                      digitalWrite(4,HIGH); //Output high
                      delayMicroseconds(50); //Set rotate speed
                      digitalWrite(4,LOW); //Output low
                      delayMicroseconds(50); //Set rotate speed
                      }
          }
    }
  }
}

void homing(){
  radial.setSpeed(70); //speed is in RPM;
  vertical.setSpeed(70); //speed is in RPM;
  
    clamp.setSpeed(10);
    stage_desired_position[0] = 0;
    stage_desired_position[1] = 0;
  while (digitalRead(steppers_homes[2])) {  
    setTemp();
    clamp.step(-500);
  }
  while(digitalRead(steppers_homes[0]) || digitalRead(steppers_homes[1])){
    check_stages();
  }
          stage_positions[0] = 0;
          stage_desired_position[0] = 0;
          stage_positions[1] = 0;
          stage_desired_position[1] = 0;

  //radial.setSpeed(20); //speed is in RPM;
  //vertical.setSpeed(20); //speed is in RPM;
  }

long mm_to_steps(long mm_val, int stage){
    return mm_val*stage_calib[stage];
  }

int sign(long value) {
return int((value>0)-(value<0));
}

void circle(){
  for(int i=0; i<12; i++){
    make_joint();
    stage_desired_position[2] = stage_desired_position[2] + mm_to_steps(-30, 2);
  }
  
  cut(); //prolly wont cut
  stage_desired_position[2] = stage_desired_position[2] + mm_to_steps(-30, 2);
  stage_desired_position[2] = stage_desired_position[2] + mm_to_steps(-90, 2);
  

  
}
