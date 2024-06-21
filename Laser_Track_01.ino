#include <Servo.h>
#include "SR04.h"
#include <Stepper.h>
#include <LCD_I2C.h>

#define SERVO_PIN A0
#define TRIG_PIN_1 13
#define ECHO_PIN_1 A1
#define TRIG_PIN_2 12
#define ECHO_PIN_2 A2
#define TRIG_PIN_3 7
#define ECHO_PIN_3 A3
#define LASER_PIN 2

const int stepsPerRevolution = 200;

LCD_I2C lcd(0x27, 16, 2);
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

int pos_Laser = 0;  // number of steps per revolution

int min = 5;
int max = 30;
int _delay = 1;
int step = 5;

SR04 sr04_ = SR04(ECHO_PIN_1, TRIG_PIN_1, ECHO_PIN_2, TRIG_PIN_2, ECHO_PIN_3, TRIG_PIN_3);
long d1;
int l1 = 4;

long d2;
int l2 = 4;

long d3;
int l3 = 4;

double x_obj1, y_obj1, theta_obj1;
int start1 = -1;
int end1 = -1;
int center1 = -1;
bool obj_Detected = false;

Servo servo;

int pos = 0;

bool dist_v[72];
int dist[72];

void UltraRead_1()
{
  sr04_.Distance(&d1, &d2, &d3);
  if(d1>max)
    d1=max;
  else if(d1<min)
    d1=max;
  if(d2>max)
    d2=max;
  else if(d2<min)
    d2=max;
  if(d3>max)
    d3=max;
  else if(d3<min)
    d3=max;  
  // Serial.print("d1: ");
  // Serial.print(d1);
  // Serial.print("cm\t");
  // Serial.print("d2: ");
  // Serial.print(d2);
  // Serial.print("cm\t");
  // Serial.print("d3: ");
  // Serial.print(d3);
  // Serial.println("cm");
}

void ServoStep1()
{
  for(; pos <= 120 ; pos += step)
  {
    servo.write(pos);
    UltraRead_1();
    // delay(5);

    dist[((pos + 300) % 360) / step] = d1;
    if (d1 < max)
      dist_v[((pos + 300) % 360) / step] = true;
    else
      dist_v[((pos + 300) % 360) / step] = false;

    dist[(pos + 60) / step] = d2;
    if (d2 < max)
      dist_v[(pos + 60) / step] = true;
    else
      dist_v[(pos + 60)/ step] = false;

    dist[(pos + 180) / step] = d3;
    if (d3 < max)
      dist_v[(pos + 180) / step] = true;
    else
      dist_v[(pos + 180)/ step] = false;
  }
}
void ServoStep2()
{
  for(; pos >=0 ; pos -= step)
  {
    servo.write(pos);
    UltraRead_1();
    dist[((pos + 300) % 360) / step] = d1;
    if (d1 < max)
      dist_v[((pos + 300) % 360) / step] = true;
    else
      dist_v[((pos + 300) % 360) / step] = false;

    dist[(pos + 60) / step] = d2;
    if (d2 < max)
      dist_v[(pos + 60) / step] = true;
    else
      dist_v[(pos + 60)/ step] = false;

    dist[(pos + 180) / step] = d3;
    if (d3 < max)
      dist_v[(pos + 180) / step] = true;
    else
      dist_v[(pos + 180)/ step] = false;
  }
}

void detectObject()
{
  int i, pos_New;
  double x1 = 4 ,y1 = 0, x2 = -2 , y2 = 3.464, x3 = -2, y3 = - 3.464;
  for(i = 0; i < 72; i++)
  {
    if(dist_v[i])
    {
      start1 = i;
      while(dist_v[i]) {i++;}
      end1 = i-1;
      if(end1 - start1 < 2)
      {
        // i = 72;
        // break;
        continue;
      }
      center1 = 0.5 * (start1 + end1);
      // if(center1 >= 60 && center1 < 12)
      // {
      //   x_obj1 = x1 + dist[center1] * cos(center1 * 5 * 3.1416 / 180);
      //   y_obj1 = dist[center1] * sin(center1 * 5 * 3.1416 / 180);
      // }
      // else if(center1 < 36)
      // {
      //   x_obj1 = x2 + dist[center1] * cos(center1 * 5 * 3.1416 / 180 - 60);
      //   y_obj1 = y2 + dist[center1] * sin(center1 * 5 * 3.1416 / 180 - 60);
      // }
      // x_obj1 = l1 + dist[center1] * cos(center1 * 5 * 3.1416 / 180); // Calculate x coordinate of point C
      // y_obj1 = dist[center1] * sin(center1 * 5 * 3.1416 / 180);
      // theta_obj1 = atan2(y_obj1, x_obj1) * 180 / 3.1416; 
      // Serial.print("x: ");
      // Serial.print(x_obj1);
      // Serial.print("      y: ");
      // Serial.print(y_obj1);
      theta_obj1 = center1 * 5;
      // Serial.print("      theta: ");
      // Serial.print(theta_obj1);
      // if (theta_obj1 < 0)
      //   theta_obj1 = 360 + theta_obj1;
      theta_obj1 = theta_obj1 * 2048 / 360;
      // Serial.print("      theta_s1: ");
      // Serial.print(theta_obj1);
      if (theta_obj1 > 1024)
        pos_New = theta_obj1 -2048;
      else
        pos_New = theta_obj1;
      // Serial.print("      pos_old: ");
      // Serial.print(pos_Laser);
      // Serial.print("      pos_New: ");
      // Serial.print(pos_New);
      theta_obj1 = pos_New - pos_Laser;
      if (theta_obj1 > 1024)
        theta_obj1 = theta_obj1 -2048; 
      pos_Laser = pos_New;
      // Serial.print("      theta_s: ");
      // Serial.print(theta_obj1);
      // Serial.println("");
      obj_Detected = true;
      break;
    }
  }
  if(i == 72)
    obj_Detected = false;
}

void setup() {
  servo.attach(SERVO_PIN);
  Serial.begin(9600);
  for (int i = 0; i < 72; i++) 
    dist_v[i] = false;
  pinMode(LASER_PIN, OUTPUT);
  myStepper.setSpeed(150);
  digitalWrite(LASER_PIN, LOW);

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("    No Object"); // You can make spaces using well... spaces
  lcd.setCursor(4, 1); // Or setting the cursor in the desired position.
  lcd.print("Detected!");

  delay(1000);
}

void loop() {
  ServoStep1();
  detectObject();
  // Serial.println();
  // Serial.println();
  // Serial.print("start: ");
  // Serial.print(start1);
  // Serial.print("   end: ");
  // Serial.print(end1);
  // Serial.print("   center: ");
  // Serial.print(center1);
  // Serial.println();
  // Serial.print("pos: ");
  // Serial.println(pos_Laser);
  
  // for (int i = 0; i < 12; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // for (int i = 60; i < 72; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 12; i < 36; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 36; i < 60; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // Serial.println();

  // for (int i = 0; i < 24; i++) 
  // {
  //   Serial.print(dist_v[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 24; i < 48; i++) 
  // {
  //   Serial.print(dist_v[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 48; i < 72; i++) 
  // {
  //   Serial.print(dist_v[i]);
  //   Serial.print("   ");
  // }
  //   Serial.println();


 if( obj_Detected)
  {
    lcd.clear();
    lcd.print("     Object"); // You can make spaces using well... spaces
    lcd.setCursor(4, 1); // Or setting the cursor in the desired position.
    lcd.print("Detected!");
    myStepper.step(theta_obj1);
    digitalWrite(LASER_PIN, HIGH);
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("    No Object"); // You can make spaces using well... spaces
    lcd.setCursor(4, 1); // Or setting the cursor in the desired position.
    lcd.print("Detected!");
    Serial.println();
    Serial.println();
    digitalWrite(LASER_PIN, LOW);
    // delay(10000);
  }
  // delay(3000);
  ServoStep2();
  detectObject();


  // Serial.println();
  // Serial.println();
  // Serial.print("start: ");
  // Serial.print(start1);
  // Serial.print("   end: ");
  // Serial.print(end1);
  // Serial.print("   center: ");
  // Serial.print(center1);
  // Serial.println();
  // Serial.print("pos: ");
  // Serial.println(pos_Laser);
  // for (int i = 0; i < 12; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // for (int i = 60; i < 72; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 12; i < 36; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 36; i < 60; i++) 
  // {
  //   Serial.print(dist[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // Serial.println();

  // for (int i = 0; i < 24; i++) 
  // {
  //   Serial.print(dist_v[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 24; i < 48; i++) 
  // {
  //   Serial.print(dist_v[i]);
  //   Serial.print("   ");
  // }
  // Serial.println();
  // for (int i = 48; i < 72; i++) 
  // {
  //   Serial.print(dist_v[i]);
  //   Serial.print("   ");
  // }
  //   Serial.println();



  if( obj_Detected)
  {
    lcd.clear();
    lcd.print("     Object"); // You can make spaces using well... spaces
    lcd.setCursor(4, 1); // Or setting the cursor in the desired position.
    lcd.print("Detected!");
    myStepper.step(theta_obj1);
    digitalWrite(LASER_PIN, HIGH);
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("    No Object"); // You can make spaces using well... spaces
    lcd.setCursor(4, 1); // Or setting the cursor in the desired position.
    lcd.print("Detected!");
    Serial.println();
    Serial.println();
    digitalWrite(LASER_PIN, LOW);
    // delay(10000);
  }


  // delay(3000);
}
