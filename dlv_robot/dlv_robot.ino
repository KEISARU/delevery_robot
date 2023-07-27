
//libraries

  #include <DigiPotX9Cxxx.h>// потенциометр
  #include <ros.h>
  #include <geometry_msgs/Twist.h>
  #include <Servo.h>
  

  Servo servo;
  DigiPot pot(8,9,10);// пины потенциометра

  
int speedForward = 20; // 0 макс скорость вперед
int speedBackward = 70; // 99 макс скорость назад
int speedStop = 45;// стоп!
  int angle = 50;



//ros initialisation
  void onTwist(const geometry_msgs::Twist& msg);
  ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",onTwist); 
  ros::NodeHandle nh;

void setup() {
  //start serial
    Serial.begin(9600); 
    
    servo.attach(5);
    servo.write(angle);

  
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
 
  //ROS loop
    nh.spinOnce();

 
}




//ros callback function 
//в качестве сигнала на задние колеса подаются константы, что не дает возможности контролировать скорость
void onTwist(const geometry_msgs::Twist& msg)
{
  //forward
  if(msg.linear.x > 0)
  {
     pot.set(speedForward);
  }
  
  //backward
  else if(msg.linear.x < 0)
  {
    pot.set(speedBackward); 
  }

  else if(msg.linear.x ==0)
  {
    pot.set(speedStop);
  }
  
  //right
  //l
  if(msg.angular.z < 0 && angle <= 70)
  {
    angle+=3;
    servo.write(angle);
    
    
    
    
  }
  //j left
  else if(msg.angular.z > 0 && angle >= 32)
  {
    angle-=3;
    servo.write(angle);
    
    
   
  }
  else if(msg.angular.z == 0) 
  {
     angle = 50;
    servo.write(angle);
    
   
  }
  
 
}
