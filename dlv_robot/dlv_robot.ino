
#include <PID_v1.h>
#define MotEnable 3 //пин регулировки скорости мотора
#define MotFwd  5  // пин мотор вперед
#define MotRev  6 // пин мотор назад

                      
String readString; //для ввода числа в консоль ардуино
int User_Input = 0; // то самое значения угла поворта
int crashSensor = 2; // Датчик касания

int encoderPin1 = 18; //
int encoderPin2 = 19; //пины энокодра
volatile int lastEncoded = 0; // для обновления значения энкодера
volatile long encoderValue = 0; // 
int PPR = 1600;  // для расчета угла
int angle = 360; // макс градус.
int REV = 0;          // для значения угла поворота
int lastMSB = 0;
int lastLSB = 0;

bool flagForRotateMotor = false;

double kp = 5 , ki = 1 , kd = 0.01;             // PID, коэф для подсчета нужного угла. Там интергалы, все дела
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  




void setup() {
  pinMode(MotEnable, OUTPUT);
  pinMode(MotFwd, OUTPUT); 
  pinMode(MotRev, OUTPUT); 
  Serial.begin(9600); //открываем пины мотора

   pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP); // пин энкодера 

  digitalWrite(encoderPin1, HIGH); //
  digitalWrite(encoderPin2, HIGH); //включаем энкодер

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  // вращаем мотор пока не коснется датчика

    while(!flagForRotateMotor) // мотор вращается пока этот флаг false
  {
    int buttonState = digitalRead(crashSensor); // Датчик касания
    //Serial.println(buttonState);
    if(buttonState == 1) // если сработала кнопка дачика
    {
    digitalWrite(MotFwd  , HIGH);  // Устанавливаем логический 0 на входе драйвера L_PWM, значит на выходе драйвера M- будет установлен потенциал S-
    digitalWrite(MotRev , LOW);  // Устанавливаем логическую 1 на входе драйвера R_PWM, значит на выходе драйвера M+ будет установлен потенциал S+
    analogWrite (MotEnable,  255 ); 
    }else{

      digitalWrite(MotFwd, LOW);  // Устанавливаем логический 0 на входе драйвера L_PWM, значит на выходе драйвера M- будет установлен потенциал S-
    digitalWrite(MotRev, LOW);  // Устанавливаем логическую 1 на входе драйвера R_PWM, значит на выходе драйвера M+ будет установлен потенциал S+
      analogWrite (MotEnable,    0); 
      flagForRotateMotor = true; // меням флаг на  true, тем самым выходим из цикла
      
      User_Input = 0; // устанавливаем значение угла энкодера на ноль, теперь нулевое положение будет, там, где датчик касания
    } 
  }

    
  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-125, 125); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
}

void loop() {
  while (Serial.available()) { // выводим значения экнодера в терминал (сериал принт)
    delay(3);                  // 
    char c = Serial.read();  // 
    readString += c;         // 
  }
 
  if (readString.length() >0) {
  
   Serial.println(readString.toInt());  //printing the input data in integer form
    User_Input = readString.toInt();   // here input data is store in integer form
    
  }

 REV = User_Input;//map (User_Input, 0, 360, 0, 1600); // mapping degree into pulse
  

Serial.print("this is REV - "); 
Serial.println(REV);               

setpoint = REV;                    //вычисление угла
  input = encoderValue ;           // 
 Serial.print("encoderValue - ");
 Serial.println(encoderValue);
  myPID.Compute();                 // 
  pwmOut(output);  
}
void pwmOut(int out) {                               
  if (out > 0) {                         // включаем мотор вперед, назад, взависимости от указанного значения   
    analogWrite(MotEnable, out);         // 
    forward();                           // 
  }
  else {
    analogWrite(MotEnable, abs(out));          //                       
    reverse();                            // 
  
  
  readString=""; // для инпута


}


 
}


// фунция прерывания, она работает всегда, в любой момент времени. Она делает так, чтобы экнкодер считал каждый поворот, а не через два.
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time

}

// функции для мотора, чтоб проще было
void forward () {
  digitalWrite(MotFwd, HIGH); 
 digitalWrite(MotRev, LOW); 
  
}

void reverse () {
  digitalWrite(MotFwd, LOW); 
 digitalWrite(MotRev, HIGH); 
  
}
void finish () {
  digitalWrite(MotFwd, LOW); 
 digitalWrite(MotRev, LOW); 
  
}
