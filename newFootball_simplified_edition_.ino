// danek 2022

// Упрощённая версия программы
                        // ВСЕ НАСТРОЙКИ(ПИНЫ, СКОРОСТИ И Т.Д.) МОЖНО ИЗМЕНИТЬ В НАСТРОЙКАХ! см. строки 233, 234, 235
uint8_t actuatorPin = 15; // соленоид
uint16_t actuatorDelays[2] = {35,125}; //задержки для соленоида в мс
     //    actuatorDelays[0] = 35; //ждём 35мс перед тем, как включить соленоид
       //   actuatorDelays[1] = 125;// ждём 125мс перед тем, как выключить соленоид
uint8_t dribblerPin = 17; // дрибблер  
bool dribblerState = LOW; //состояние дрибблера. По умолчанию выключен
uint8_t l1 = 10, l2 = 11; // левый мотор
uint8_t r1 = 5, r2 = 6;   // правый мотор
int16_t defaultSpeed = 230; // скорость по умолчанию
float spdK = 0.6;//коэффициент, позволяющий уменьшить скорость на поворотах и сделать робота более управляемым
uint8_t selectedProg = 2; // выбор программы. По умолчанию rc car
// Моторы в класс объявлять не будем. Неудобно. Лучше в класс соберём светодиоды
// Задержку с помощью delay() в некоторых случаях лучше не использовать
// Программа останавливается, в буфер Serial продолжают приходить новые данные
// Но мы их не читаем. Поэтому вместо задержек мы создадим класс, 
// который будет возвращать число, показывающее, сколько мс прошло с его последнего вызова
class Timer
{
  public:
  unsigned long tm = 0; //время вызова в миллисекундах
  bool timeFlag = true;// переменная, показывающая состояние таймера: false - запущен, изменять запрещено
  uint16_t deltaT = 0;// время, на которое был вызван таймер
  void setTimer(uint16_t t){deltaT = t;tm = (timeFlag)? millis():tm;timeFlag = false;}//Устанавливаем таймер. Запрещаем менять значение времени вызова.
  bool timeIsOver(){if((tm+deltaT) < millis()){return true;}return false;}// Проверяем, вышло ли время
  void enableTimer(){timeFlag = true;}// Разрешаем обновить таймер
  
};
Timer del;
class Leds      // класс светодиодов
{
  public:
  uint8_t settingLed;
  uint8_t actuatorLed;
  uint8_t dribblerLed;
  uint8_t beginLed;
  uint16_t deltaCounter = 1;
  long deltaT = 500;
  uint16_t timer = 0;
  void start(uint8_t s,uint8_t b,uint8_t a,uint8_t d)
  {
    settingLed = s;
    beginLed = b;
    actuatorLed = a;
    dribblerLed = d;
    pinMode(settingLed,OUTPUT);
    pinMode(actuatorLed,OUTPUT);
    pinMode(dribblerLed,OUTPUT);
    pinMode(beginLed,OUTPUT);
    }
    void turnOn(uint8_t &tled){digitalWrite(tled,HIGH);}
     void turnOff(uint8_t &tled){digitalWrite(tled,LOW);}
};
class DefaultSettings // класс стандартных настроек
{
  public:
  float sdK = 0.55; // коэффициент скорости
  uint16_t dfSpeed = 230;  // выбрать 255 скорость
  uint8_t dfProg = 2;      // программа - RC BLUETOOTH CAR
  void start(){defaultSpeed = dfSpeed;selectedProg = dfProg;spdK = sdK;}
  void changeParameters(uint8_t maxSpeed,float mspeedK,uint8_t progOption){defaultSpeed = maxSpeed;spdK = mspeedK;selectedProg = progOption;}
  void changeMotorPins(uint8_t lt1,uint8_t lt2,uint8_t rt1,uint8_t rt2){l1 = lt1;l2 = lt2;r1 = rt1;r2 = rt2;}
  void changeOtherPins(uint8_t aP,uint8_t dP){actuatorPin = aP;dribblerPin = dP;}
  void changeActuatorDelays(uint16_t nD1, uint16_t nD2){actuatorDelays[0] = nD1;actuatorDelays[1] = nD2;}
};
class ControlJoystickCommands // список комманд программы ControlJoystick
{
  public:
 uint8_t stopCode = 0xF3; // код остановки
 uint8_t forwardCode = 0xF1; //код для движения вперёд
 uint8_t backwardCode = 0xF2;//код для движения назад
 uint8_t speedValue = 0x00; // по умолчанию стоим
 float angleValue = 0x5A; // по умолчаниию 90(начальное положение)
 uint8_t actuatorCode = 0x04; //включаем соленоид при получении кода 4
 uint8_t dribblerCode = 0x80; //включаем дриблер при получении кода 128  
 int16_t deltaFourthByte = 0x00;//необходимо для правильного управления соленоидом и дрибблером
};
ControlJoystickCommands jCommands; //объявляем класс с командами Control Joystick
class MotorShield // если использовать motor shield, нужно менять настройки
{
  public:
  uint8_t lDir, rDir;
  uint8_t lSpeed, rSpeed;
  void init(uint8_t lD,uint8_t lS,uint8_t rD,uint8_t rS){lDir = lD;rDir = rD;lSpeed = lS;rSpeed = rS;}
  void leftMotorWrite(int16_t spd){bool dir = (spd >= 0)? HIGH : LOW;digitalWrite(lDir,dir);analogWrite(lSpeed,abs(spd));}
   void rightMotorWrite(int16_t spd){bool dir = (spd >= 0)? HIGH : LOW;digitalWrite(rDir,dir);analogWrite(rSpeed,abs(spd));}
};
MotorShield motorshield;// обЪявляем класс MotorShield
void setupMotorSystem(uint8_t L1,uint8_t L2,uint8_t R1,uint8_t R2)
{
  pinMode(L1,OUTPUT);
    pinMode(R1,OUTPUT);
      pinMode(L2,OUTPUT);
        pinMode(R2,OUTPUT);   }
void leftMotor(int16_t mSpeed) //функция управления левым мотором 
{
  if(mSpeed >= 0){analogWrite(l1,mSpeed);analogWrite(l2,0);} // Закомментировать, если motor shield
  else{analogWrite(l2,(mSpeed*(-1)));analogWrite(l1,0);}  // Закомментировать, если motor shield
 // motorshield.leftMotorWrite(mSpeed);                   // Раскомментировать, если motor shield
}
  void rightMotor(int16_t mSpeed) //функция управления правым мотором 
{
  if(mSpeed >= 0){analogWrite(r1,mSpeed);analogWrite(r2,0);} // Закомментировать, если motor shield
  else{analogWrite(r2,(mSpeed*(-1)));analogWrite(r1,0);}    // Закомментировать, если motor shield
  // motorshield.rightMotorWrite(mSpeed);                   // Раскомментировать, если motor shield
}
void delays()
{
  // Если мы поставили таймер,
  if(!del.timeIsOver())// Смотрим, вышло ли время
    {                     // Если нет, то определяем, что мы должны делать, пока чтоит таймер
      if(del.deltaT == actuatorDelays[0]){dribblerState = LOW;digitalWrite(dribblerPin,dribblerState);} // просто ждём 35мс. Выключаем дрибблер
      else if(del.deltaT == actuatorDelays[1]){digitalWrite(actuatorPin,HIGH);} //держим соленоид включенным 125 мс
      else{;}
    }
    else
    { // Если таймер вышел,то выполняем заданные действия
       if(del.deltaT == actuatorDelays[0]){digitalWrite(actuatorPin,HIGH);del.enableTimer();del.setTimer(actuatorDelays[1]);} // таймер на 35мс закончился - включаем соленоид и ставим таймер на 125мс
         else if(del.deltaT == actuatorDelays[1]){digitalWrite(actuatorPin,LOW);del.enableTimer();}// таймер на 125мс закончился - выключаем соленоид и разрешаем ставить таймер снова
    }
}
// Для Arduino Bluetooth RC Car
void bluetoothRcCar()
{
  if(!del.timeFlag) {delays();}                    // Если есть таймер
   if(Serial.available())
{
  char value = Serial.read();
  if(value == 'F'){rightMotor(defaultSpeed);leftMotor(defaultSpeed);}
  else if(value == 'R'){rightMotor(0);leftMotor(round(defaultSpeed/2));}
  else if(value == 'L'){rightMotor(round(defaultSpeed/2));leftMotor(0);}
  else if(value == 'B'){rightMotor(-defaultSpeed);leftMotor(-defaultSpeed);}
  else if(value == 'S'){rightMotor(0);leftMotor(0);}
  else if(value == 'I'){leftMotor(defaultSpeed);rightMotor(round(defaultSpeed/2));}
  else if(value == 'G'){rightMotor(defaultSpeed);leftMotor(round(defaultSpeed/2));}
  else if(value == 'H'){rightMotor(-defaultSpeed);leftMotor(-round(defaultSpeed/2));}
  else if(value == 'J'){leftMotor(-defaultSpeed);rightMotor(-round(defaultSpeed/2));}
    else if((value == 'X')||(value == 'x'))
    {del.setTimer(35);}
     else if(value == 'W'){dribblerState = !dribblerState;digitalWrite(dribblerPin,dribblerState);}
   else if(value == 'w'){dribblerState = !dribblerState;digitalWrite(dribblerPin,dribblerState);}

}
}
// для Control Joystick
void bluetoothControlJoystick()
{
  if(!del.timeFlag) {delays();}                    // Если есть таймер
  if(Serial.available() >= 4) // ждём, пока в буфере будет 4 и больше байт
  {
    uint8_t firstByte = Serial.read(); // читаем первый байт. Отвечает за направление
    uint8_t secondByte = Serial.read(); // читаем второй байт. Отвечает за скорость
    uint8_t thirdByte = Serial.read(); // читаем третий байт. Отвечает за угол поворота
    uint8_t fourthByte = Serial.read(); // читаем четвёртый байт. Отвечает за состояние кнопок
    if(firstByte == jCommands.stopCode){rightMotor(0);leftMotor(0);} // останавливаемся
    else // иначе смотрим, что мы получили. Да, можно обьявить функцию типа int(или любого другого) и возвращать значение, чтобы избавиться от if..else 
    {
      float speedK = spdK; // Для удобства можно добавить дополнительный множитель, понижающий скорость на поворотах
      float deltaSpeed = 0;// для расчета изменения скорости при повороте
      deltaSpeed = thirdByte/jCommands.angleValue;//делим наше значение за значение по умолчанию, получаем множитель для скорости
      // Да, можно менять скорость одного мотора пропорционально скорости другого, но на практике роботов становится сложно управлять
      //Поэтому мы не будем увеличивать скорость второго мотора, а только уменьшать скорость первого
      int8_t dir = (firstByte == jCommands.forwardCode)? 1 : (-1);// сначала узнаём направление
      if(deltaSpeed > 1){leftMotor(speedK*dir*secondByte);rightMotor(speedK*dir*secondByte*(2-deltaSpeed));}// поворачиваем вправо
      else if(deltaSpeed < 1){leftMotor(speedK*secondByte*deltaSpeed*dir);rightMotor(speedK*dir*secondByte);}// поворачиваем влево
      else if(deltaSpeed == 1){leftMotor(dir*secondByte);rightMotor(dir*secondByte);}// специально вынесено от других случаев, чтобы робот мог ехать прямо на полной скорости
     }
      if(((fourthByte-jCommands.deltaFourthByte) == jCommands.actuatorCode)||((fourthByte-jCommands.deltaFourthByte) == (-jCommands.actuatorCode)))
     {del.setTimer(35);}
     else if((fourthByte-jCommands.deltaFourthByte) == jCommands.dribblerCode){dribblerState = !dribblerState;digitalWrite(dribblerPin,dribblerState);}
     else if((fourthByte-jCommands.deltaFourthByte) == -(jCommands.dribblerCode)){dribblerState = !dribblerState;digitalWrite(dribblerPin,dribblerState);}
     jCommands.deltaFourthByte = fourthByte;
  }
}
// Для Arduino Joystick
uint8_t bluetoothJoystickAdvansed() //улучшенный джойстик
{
  if(!del.timeFlag) {delays();}                    // Если есть таймер
  if(Serial.available())
{
    float angle, motorSpeed;uint8_t pressedButtonNum; // переменные, получаемые с джойстика. Угол, скорость и номер нажатой кнопки
  long value = Serial.parseInt(); // читаем число, которое получили
  pressedButtonNum = value%10;value/= 10; // получили номер нажатой кнопки. Убрали его из value
  if(pressedButtonNum == 1){dribblerState = !dribblerState;digitalWrite(dribblerPin,dribblerState);}
  else if(pressedButtonNum == 2){del.setTimer(35);}
  angle = float(value/1000.0);motorSpeed = value%1000;//переводим код в значения угла и скорости
  angle =(angle > 180)?(angle-360):angle; // углы больше 180 делаем отрицательными для удобства
  int8_t direct = (angle >= 0)? 1 : -1; // выбираем направление вращения моторов
  float k = angle/90; //коэффициет показывает, какую часть составляет угол от угла 90
  k = (k >= 0)? k : (-k); // коэффициент не может быть отрицательным. Учитываем
  float sK = motorSpeed/100;// перевод скорости из процентов в десятичную дробь
  float speedK = spdK;// Для удобства можно добавить дополнительный множитель, понижающий скорость на поворотах
  int8_t remoteArea = -50; // промежутки от -180 до -180-remoteArea и от -remoteArea до 0 будут убраны. Сделано для удобства
  if((angle < 0) && (angle > remoteArea)){leftMotor(defaultSpeed*sK*speedK); rightMotor(0);return 0;}
  if((angle > -180)&&(angle < (-180-remoteArea))){leftMotor(0); rightMotor(defaultSpeed*sK*speedK);return 0;}
  //в промежутке от 75 до 105 будет считать, что робот едет прямо без уменьшения скорости для поворота. Сделано для удобства
  if((angle < 105) && (angle > 75)){leftMotor(defaultSpeed*sK); rightMotor(defaultSpeed*sK);return 0; }
  float rightMotorSpeed = defaultSpeed*k*sK*speedK;rightMotorSpeed = (rightMotorSpeed > defaultSpeed)? defaultSpeed:rightMotorSpeed;//считаем скорость правого мотора
  float leftMotorSpeed = defaultSpeed*(2-k)*sK*speedK;leftMotorSpeed = (leftMotorSpeed > defaultSpeed)? defaultSpeed:leftMotorSpeed;//считаем скорость левого мотора
  leftMotorSpeed = direct*round(leftMotorSpeed);  rightMotorSpeed = direct*round(rightMotorSpeed); // скорости округляем и выбираем направление
  
  rightMotor((int16_t)rightMotorSpeed);
  leftMotor((int16_t)leftMotorSpeed); //пишем скорости
  return 0;
}
}
Leds leds;        // объявляем класс светодиодов
DefaultSettings dfS; //объявляем класс с базовыми настройками
void setup() {
   dfS.changeParameters(230,0.5,3); // Изменить максимальную скорость(0..255) | уменьшение скорости на поворотах(0..1)float, 1 - не уменьшать | выбранную программу(1,2,3)
   dfS.changeMotorPins(10,11,5,6);// Изменить пины моторов левый_1 | левый_2 | правый_1 | правый_2
   dfS.changeOtherPins(15,17); // Изменить пины соленоида | дрибблера
   dfS.changeActuatorDelays(35,125);// Изменить задержки перед включением | выключением соленоида
  //motorshield.init(левый_направление,левый_скорость,правый_направление,правый_скорость); //РАСКОММЕНТИРОВАТЬ, ЕСЛИ ИСПОЛЬЗУЕТСЯ MOTOR SHIELD!!! 
    setupMotorSystem(l1,l2,r1,r2);   //  закомментировать, если используется motor shield

    
    leds.start(13,13,13,13);             // поставить все пины одинаковые(любой, который не используется), если светодиодов нет
  Serial.begin(9600);
  pinMode(actuatorPin,OUTPUT);
  pinMode(dribblerPin,OUTPUT);
  leds.turnOn(leds.beginLed); // готовы к запуску
}
void loop() {
if(selectedProg == 1){bluetoothJoystickAdvansed();} // Arduino Joystick
else if(selectedProg == 2){bluetoothRcCar();}       // Обычный rc Car. Выбран по умолчанию
else if(selectedProg == 3){bluetoothControlJoystick();} // Control Joystick
  
}


//danek 2022
