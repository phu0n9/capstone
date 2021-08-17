#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>


const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

/*--------------Declare PIN of each wheels---------------*/
#define encodPinA1      2          //Encoder front right                  
#define encodPinA2      28         //Encoder front right                    
#define encodPinB1      3          //Encoder rear right                  
#define encodPinB2      29         //Encoder rear right
#define encodPinC1      19         //Encoder front left                   
#define encodPinC2      30         //Encoder front left
#define encodPinD1      18         //Encoder rear left                   
#define encodPinD2      31         //Encoder rear left

int enA = 8, fwdA = 50, revA = 51;  //Direction and Speed pin front right wheel
int enB = 9, fwdB = 52, revB = 53;  //Direction and Speed pin front left wheel
int enC = 10, fwdC = 22, revC = 23; //Direction and Speed pin rear right wheel
int enD = 11, fwdD = 24, revD = 25; //Direction and Speed pin rear left wheel
/*------------------------END---------------------------*/

/*------------------------Decalre PID parameter for each wheels----------*/
double kp_frontright = 5, ki_frontright = 1, kd_frontright = 0.01;
double kp_frontleft = 5, ki_frontleft = 1, kd_frontleft = 0.01;
double kp_rearright = 5, ki_rearright = 1, kd_rearright = 0.01;
double kp_rearleft = 5, ki_rearleft = 1, kd_rearleft = 0.01;

double input_frontright = 0, output_frontright = 0, setpoint_frontright = 0;
double input_frontleft = 0, output_frontleft = 0, setpoint_frontleft = 0;
double input_rearright = 0, output_rearright = 0, setpoint_rearright = 0;
double input_rearleft = 0, output_rearleft = 0, setpoint_rearleft = 0;
/*---------------------------------END-----------------------------------*/

/*--------------------------PID object for each wheels-------------------*/
PID front_right(&input_frontright, &output_frontright, &setpoint_frontright, kp_frontright, ki_frontright, kd_frontright, DIRECT);
PID front_left(&input_frontleft, &output_frontleft, &setpoint_frontleft, kp_frontleft, ki_frontleft, kd_frontleft, DIRECT);
PID rear_right(&input_rearright, &output_rearright, &setpoint_rearright, kp_rearright, ki_rearright, kd_rearright, DIRECT); 
PID rear_left(&input_rearleft, &output_rearleft, &setpoint_rearleft, kp_rearleft, ki_rearleft, kd_rearleft, DIRECT);    
/*---------------------------------END-----------------------------------*/

/*-----------------Setting parameter to take sampling time---------------*/
unsigned long previous_time, current_time;
volatile long encoderA = 0, encoderB = 0, encoderC = 0, encoderD = 0;
volatile long last_encoderA = 0, last_encoderB = 0, last_encoderC = 0, last_encoderD = 0;
float encoderAdiff = 0, encoderBdiff = 0, encoderCdiff = 0, encoderDdiff = 0;
/*---------------------------------END-----------------------------------*/



/*-----------------Setting parameter from Software System---------------*/
#define LOOPTIME 10
float demandx = 0;
float demandz = 0;
double demand_speed_left = 0;
double demand_speed_right = 0;
double speed_act_left = 0;
double speed_act_right = 0;
/*---------------------------------END-----------------------------------*/

ros::NodeHandle nh;

/*-----------------Function will called receive from host---------------*/
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  demandx = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  demandz = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
}
/*---------------------------------END-----------------------------------*/

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

void setup() {
  /*----------------Declare setting pinmode of the wheels----------------*/
  pinMode(encodPinA1, INPUT_PULLUP);                  
  pinMode(encodPinA2, INPUT_PULLUP);         
  pinMode(encodPinB1, INPUT_PULLUP);                  
  pinMode(encodPinB2, INPUT_PULLUP);
  pinMode(encodPinC1, INPUT_PULLUP);                  
  pinMode(encodPinC2, INPUT_PULLUP);
  pinMode(encodPinD1, INPUT_PULLUP);                  
  pinMode(encodPinD2, INPUT_PULLUP);        

  attachInterrupt(digitalPinToInterrupt(encodPinA1), encoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinB1), encoder_B, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinC1), encoder_C, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinD1), encoder_D, RISING);

  pinMode(enA, OUTPUT);  // Front right
  pinMode(enC, OUTPUT);  // Front left
  pinMode(enB, OUTPUT); // Rear right
  pinMode(enD, OUTPUT); // Rear left

  pinMode(fwdA, OUTPUT); // Front right - IN1 
  pinMode(revA, OUTPUT); // Front right - IN2
  pinMode(revC, OUTPUT); // Front left  - IN1
  pinMode(revC, OUTPUT); // Front left  - IN2

  pinMode(fwdB, OUTPUT); // Rear right - IN1 
  pinMode(revB, OUTPUT); // Rear right - IN2 
  pinMode(fwdD, OUTPUT); // Rear left - IN1 
  pinMode(fwdD, OUTPUT); // Rear left - IN2
  /*------------------------END---------------------------------------*/ 

  /*----------------Setting PID object 4 wheels-----------------------*/
  front_right.SetMode(AUTOMATIC);
  front_right.SetSampleTime(1);
  front_right.SetOutputLimits(-255, 255);

  front_left.SetMode(AUTOMATIC);
  front_left.SetSampleTime(1);
  front_left.SetOutputLimits(-255, 255);

  rear_right.SetMode(AUTOMATIC);
  rear_right.SetSampleTime(1);
  rear_right.SetOutputLimits(-255, 255);

  rear_left.SetMode(AUTOMATIC);
  rear_left.SetSampleTime(1);
  rear_left.SetOutputLimits(-255, 255);
  /*------------------------END---------------------------------------*/

  /*----------------Creating Node for ROS-----------------------*/
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  /*------------------------END---------------------------------------*/
}

void loop() {
  nh.spinOnce();
  current_time = millis();
  int timeChange = (current_time - previous_time);
  if(timeChange >= 10){
    previous_time = current_time; 
    demand_speed_left = demandx - (demandz*0.145);
    demand_speed_right = demandx + (demandz*0.145);

    encoderAdiff = encoderA - last_encoderA;
    encoderBdiff = encoderB - last_encoderB;
    encoderCdiff = encoderC - last_encoderC;
    encoderDdiff = encoderD - last_encoderD;

    speed_act_left = encoderCdiff/39.65;
    speed_act_right = encoderAdiff/39.65;

    last_encoderA = encoderA;
    last_encoderB = encoderB;
    last_encoderC = encoderC;
    last_encoderD = encoderD;

    setpoint_frontleft = demand_speed_left*39.65;
    setpoint_rearleft = demand_speed_left*39.65;
    setpoint_frontright = demand_speed_right*39.65;
    setpoint_rearright = demand_speed_right*39.65;

    front_left.Compute();
    rear_left.Compute();
    front_right.Compute();
    rear_right.Compute();

    pwmOut(enA, fwdA, revA, output_frontright);
    pwmOut(enB, fwdB, revB, output_rearright);
    pwmOut(enC, fwdC, revC, output_frontleft);
    pwmOut(enD, fwdD, revD, output_rearleft);
    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
}


/*----------------------------------Interrupt function to count pusle of 4 wheels-----------------------------------------*/
void encoder_A()  {                                     // pulse and direction, direct port reading to save cycles front right wheels 
  if (digitalRead(encodPinA2) == HIGH)    encoderA++;             
  else                      encoderA--;             
}

void encoder_B()  {                                     // pulse and direction, direct port reading to save cycles rear right wheels 
  if (digitalRead(encodPinB2) == HIGH)    encoderB++;             
  else                      encoderB--;            
}

void encoder_C()  {                                     // pulse and direction, direct port reading to save cycles front left wheels
  if (digitalRead(encodPinC2) == HIGH)    encoderC++;            
  else                      encoderC--;             
}

void encoder_D()  {                                     // pulse and direction, direct port reading to save cycles rear left wheels
  if (digitalRead(encodPinD2) == HIGH)    encoderD++;             
  else                      encoderD--;             
}
/*-----------------------------------------------------END-----------------------------------------------------------------*/

void pwmOut(int en_pin, int fwd_pin, int back_pin, int out){
  if(out > 0){
    analogWrite(en_pin, out);
    digitalWrite(fwd_pin, HIGH);
    digitalWrite(back_pin, LOW);
  }
  else{
    analogWrite(en_pin, abs(out));
    digitalWrite(fwd_pin, LOW);
    digitalWrite(back_pin, HIGH);
  }
}

void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
