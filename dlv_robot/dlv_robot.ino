#include <PID_v1.h>
#define MotEnable 3 //пин регулировки скорости мотора
#define MotFwd  5  // пин мотор вперед
#define MotRev  6 // пин мотор назад
#define encoderPin1 18 // пины энкодера
#define encoderPin2 19

#include <DigiPotX9Cxxx.h>// потенциометр

#include <ros.h> // 
#include <geometry_msgs/Twist.h>//



int speedForward = 20; // 0 макс скорость вперед
int speedBackward = 70; // 99 макс скорость назад
int speedStop = 45;// стоп!

DigiPot pot(8,9,10);// пины потенциометра


int User_Input = 0; // то самое значения угла поворта
int crashSensor = 2; // Датчик касания


volatile int lastEncoded = 0; // для обновления значения энкодера
volatile long encoderValue = 0; // 



bool flagForRotateMotor = false;

double kp = 5 , ki = 1 , kd = 0.009;             // PID, коэф для подсчета нужного угла. Там интергалы, все дела
double input = 0, output = 0, setpoint = 0; 
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  //



void onTwist(const geometry_msgs::Twist& msg)//
{
  //forward
  if(msg.linear.x > 0)
  {
    
      pot.set(speedForward);//кнопка u .отправляем на потенциометр значение чтоб ехал вперед
    
  }
  //backward
  else if(msg.linear.x < 0)
  {
     
      pot.set(speedBackward); // кнопка m, назад
    
  }
  //right
  else if(msg.angular.z < 0)//Для поворота на право
  {
    if(User_Input >= 0)//делаем так, чтобы он не крутился за нулевую позицию энокдера, ограничиваем угол поворота мотора
    {
      User_Input-= 1;
    }
    else
    {
      User_Input = 0;
    }
  }

  else if(msg.angular.z > 0)// тоже самое что и выше, но за позицию 34. Их всего 34
  {
    if(User_Input <= 34)//
    {
      User_Input+= 1;
    }
    else
    {
      User_Input = 34;
    }
  }
  else 
  {
   pot.set(speedStop); // кнопка к. остновка заднего привода
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",onTwist); //

ros::NodeHandle nh;//

void setup() {
  pinMode(MotEnable, OUTPUT); // передний привод
  pinMode(MotFwd, OUTPUT); 
  pinMode(MotRev, OUTPUT); 
  Serial.begin(9600); //открываем пины мотора

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP); // пин энкодера 

  digitalWrite(encoderPin1, HIGH); //
  digitalWrite(encoderPin2, HIGH); //включаем энкодер

  //
  //
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  

    
  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   // ставим автомат на пид контроль.
  myPID.SetSampleTime(1);  // таймер обновления расчетов
  myPID.SetOutputLimits(-125, 125); // ограничиваем скорость мотора
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();

  // вращаем мотор пока не коснется датчика

    while(!flagForRotateMotor) // мотор вращается пока этот флаг false
  {
    int buttonState = digitalRead(crashSensor); // Датчик касания
    //Serial.println(buttonState);
    if(buttonState == 1) // если сработала кнопка дачика
    {
    digitalWrite(MotFwd  , HIGH);  // Устанавливаем логический 0 на входе драйвера L_PWM, значит на выходе драйвера M- будет установлен потенциал S-
    digitalWrite(MotRev , LOW);  // Устанавливаем логическую 1 на входе драйвера R_PWM, значит на выходе драйвера M+ будет установлен потенциал S+
    analogWrite (MotEnable,   127); 
    }else
    {
      // останавливаем передний привод. Все значения обнуляем. Теперь эта точка нулевая.
      digitalWrite(MotFwd, LOW);  // Устанавливаем логический 0 на входе драйвера L_PWM, значит на выходе драйвера M- будет установлен потенциал S-
      digitalWrite(MotRev, LOW);  // Устанавливаем логическую 0 на входе драйвера R_PWM, значит на выходе драйвера M+ будет установлен потенциал S-
      analogWrite (MotEnable,    0); 
      flagForRotateMotor = true; // меням флаг на  true, тем самым выходим из цикла
      
      User_Input = 0; // устанавливаем значение угла энкодера на ноль, теперь нулевое положение будет, там, где датчик касания
      encoderValue = 0;
    } 
  }


  
  
              

  setpoint = User_Input;                    //вычисление угла
  input = encoderValue ;           // 

  myPID.Compute(); 
  //ограничиваем скорость мотора. Иначе он не вращает если скорость ниже 80 или -80.
  if (output< 0 && output > -80)
  {
    output = -80;
    if(User_Input == encoderValue)
        output = 0; // ставлю скорость на ноль, чтобы остановился
  }
  if( output> 0 && output < 80)
  {
    output = 80;
    if(User_Input == encoderValue)
        output = 0;
  }
  //
  pwmOut(output); 
  
}
void pwmOut(int out) 
{                               
  if (out > 0) 
  {                         // включаем мотор вперед, назад, взависимости от указанного значения. Передний привод   
    analogWrite(MotEnable, out);         // 
    reverse();// 
  }
  else 
  {
    analogWrite(MotEnable, abs(out));          //                       
    forward();                            // 
  }
}

// фунция прерывания, она работает всегда, в любой момент времени. Она делает так, чтобы экнкодер считал каждый поворот, а не через два.
void updateEncoder()
{
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time

}

// функции для мотора, чтоб проще было
void forward () 
{
  digitalWrite(MotFwd, HIGH); 
  digitalWrite(MotRev, LOW); 
}

void reverse () 
{
  digitalWrite(MotFwd, LOW); 
  digitalWrite(MotRev, HIGH); 
}
