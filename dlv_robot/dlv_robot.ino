
//libraries
  #include <PID_v1.h>
  #include <DigiPotX9Cxxx.h>// потенциометр
  #include <ros.h>
  #include <geometry_msgs/Twist.h>
  #include "GyverEncoder.h"
  
//pins
  #define MotEnable 3 //пин регулировки скорости мотора
  #define MotFwd  5  // пин мотор вперед
  #define MotRev  6 // пин мотор назад
  #define encoderPin1 18
  #define encoderPin2 19
  #define crashSensor 2

//variables
  //Serial
    String readString; //для ввода числа в консоль ардуино - пока не используется
    
  //encoder
    
    volatile long encoderValue = 0; // 
    void updateEncoder();
  //PID
    double kp = 5 , ki = 1 , kd = 0.009;             // PID, коэф для подсчета нужного угла. Там интергалы, все дела - 
    double input = 0, output = 0, setpoint = 0;      //#коэффициенты не используются из за введения константы +-80
    
  //crashSensor
    bool flagForRotateMotor = false;

  //rear wheels
    int speedForward = 20; // 0 макс скорость вперед
    int speedBackward = 70; // 99 макс скорость назад
    int speedStop = 45;// стоп!
    
  //front wheels
    int User_Input = 0; // то самое значения угла поворта - не понятно, зачем нужна.
    void clockwise();
    void counterclockwise();
    void stop_motor();
    
//external classes
  DigiPot pot(8,9,10);// пины потенциометра
  PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
  Encoder enc1(encoderPin1, encoderPin2);

//ros initialisation
  void onTwist(const geometry_msgs::Twist& msg);
  ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",onTwist); 
  ros::NodeHandle nh;

void setup() {
  //start serial
    Serial.begin(9600); 
    
  //front wheels motor setup
    pinMode(MotEnable, OUTPUT);
    pinMode(MotFwd, OUTPUT); 
    pinMode(MotRev, OUTPUT); 
    
 
  //attach encoder to interrupt service
    attachInterrupt(4, updateEncoder, CHANGE); 
    attachInterrupt(5, updateEncoder, CHANGE);
    


  //PID
    myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
    myPID.SetSampleTime(1);  // refresh rate of PID controller
    myPID.SetOutputLimits(-125, 125); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  //ros setup
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
  enc1.tick();
  //ROS loop
    nh.spinOnce();

  //rotate motor untill button is clicked
    while(!flagForRotateMotor) 
    {
      if(digitalRead(crashSensor)) 
      {
      
        clockwise(127);
      }
      else
      {
        
        stop_motor();
        flagForRotateMotor = true; // меням флаг на  true, тем самым выходим из цикла
        User_Input = 0; // устанавливаем значение угла энкодера на ноль, теперь нулевое положение будет, там, где датчик касания
        encoderValue = 0;
    } 
  }
  
    Serial.print("this is UI - "); 
Serial.println(User_Input);
    setpoint = User_Input; 
    input = encoderValue ; 
     Serial.print("encoderValue - ");
 Serial.println(encoderValue);// 
    myPID.Compute(); 
    pwmOut(output);
    
}

//front wheels motor - поменяйте название, если не направления не соответствуют
  void clockwise (int angular_speed) 
  {
    digitalWrite(MotFwd, HIGH); 
    digitalWrite(MotRev, LOW); 
    analogWrite(MotEnable,angular_speed);
  }
  void counterclockwise (int angular_speed) 
  {
    digitalWrite(MotFwd, LOW); 
    digitalWrite(MotRev, HIGH); 
    analogWrite(MotEnable,angular_speed);
  }
  void stop_motor(){
    digitalWrite(MotFwd, LOW);  // Устанавливаем логический 0 на входе драйвера L_PWM, значит на выходе драйвера M- будет установлен потенциал S-
    digitalWrite(MotRev, LOW);  // Устанавливаем логическую 1 на входе драйвера R_PWM, значит на выходе драйвера M+ будет установлен потенциал S+
    analogWrite (MotEnable,    0); 
  }
  
void pwmOut(int out) 
{                               
  if (out > 0) 
  {                         
    clockwise(out);
  }
  else 
  {
    
    counterclockwise(-out);// 
  }
}

// фунция прерывания, не сработала корректно для моего энкодера.
void updateEncoder() 
{

  if(enc1.isTurn())
  {
    if (enc1.isRight())
    { 
    encoderValue++;  
    
    }   
  
  if (enc1.isLeft())
    {
    encoderValue--;
    }
  }
}


//ros callback function 
//в качестве сигнала на задние колеса подаются константы, что не дает возможности контролировать скорость
void onTwist(const geometry_msgs::Twist& msg)
{
  //forward
  if(msg.linear.x > 0) pot.set(speedForward);
  
  //backward
  else if(msg.linear.x < 0) pot.set(speedBackward); 
  
  //right 
  // Разве число доступных оборотов энкодера не меньше чем 34?
  else if(msg.angular.z < 0)
  {
    if(User_Input >=0 && User_Input <=34) User_Input-= 1;
    
  }

  else if(msg.angular.z > 0)
  {
    if(User_Input >=0 && User_Input <=34) User_Input+= 1;
    
  }
  else pot.set(speedStop);
 
}
