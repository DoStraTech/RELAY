/*
   RELAY: Robotic EyeLink AnalYsis

   Copyright (C) 2022  Dominykas.strazdas [at] ovgu.de

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Libraries Used:
// waspinator/AccelStepper@^1.61

#include <AccelStepper.h>
#include <math.h>
#define XMIN -260.0         //Motor Values
#define XMAX 260.0  
#define YMIN -240.0  
#define YMAX 240.0
#define X_SIZE 531.36       //Screen Values
#define Y_SIZE 298.89
#define DIST 927
#define X_Resolution 1920
#define Y_Resolution 1080
#define DEBUG false  
#define POINTS 160
#define TSPEED 3200
#define AUTODELAY 10000     //delay countdown to prepare long term exposure
#define Min_Dist 275
#define Max_Dist 825
#define ARRAYSIZE 160

AccelStepper Xaxis(1, 2, 5); // pin 2 = step, pin 5 = direction
AccelStepper Yaxis(1, 3, 6); // pin 2 = step, pin 5 = direction

const byte enablePin = 8;
int t1 =0;  int t2 =0;  int t3 =0;
int Joy_X = 0;          int Joy_Y = 0;      
int cur_x=0;            int cur_y=0;
int i=0;    int c=0;    int counterserial = 0;
int error=0;

unsigned long timeoutcounter =0;      unsigned long timeout =100000;
bool SerialNextPos = false;           bool Calibrated=false;
int Border_Left = X_Resolution/10;    int Border_Right =X_Resolution*9/10;        
int Border_Top = Y_Resolution/10;     int Border_Bottom = Y_Resolution*9/10;
int TestArray[ARRAYSIZE*2];          int currentpos = 9999;

//---Function declarations
void SetAcceleration(float acc);
bool AtPosition();
void DoMotorStuff();
//------------------------

void GetAnalogPosition(){
  Joy_X = analogRead(4); // Read voltage A4 --> VRx - X-Axis
  Joy_Y = analogRead(5); // Read voltage A5 --> VRy - Y-Axis
}
void PrintAnalogPosition(){
  GetAnalogPosition();
  Serial.println("X=" + String(Joy_X) + " Y=" + String(Joy_Y));
}
String getstatus(){
  long x = Xaxis.currentPosition();
  long y = Yaxis.currentPosition();
  if (x<XMIN||x>XMAX||y<YMIN||y>YMAX){
    Serial.println("Error Position !");
    error = 1;
  }
  return "X= "+String(x)+" Y= " + String(y);
}

// Main Motor functions
String GoTo(long x, long y, double speed){
  if (x<XMIN||x>XMAX||y<YMIN||y>YMAX) return "Position not reachable.";
  Xaxis.moveTo(x);
  Yaxis.moveTo(y);
  long xdist = Xaxis.distanceToGo();
  long ydist = Yaxis.distanceToGo();
  double G = sqrt((xdist*xdist)+(ydist*ydist));
  double xspeed = xdist*speed/G;
  double yspeed = ydist*speed/G;
  Xaxis.setMaxSpeed(xspeed);
  Yaxis.setMaxSpeed(yspeed);
  //return "currentX=" + String(Xaxis.currentPosition())+ "--> X="+String(x)+" xdis=" + xdist  ;
  return "GoTo X="+String(x)+ " Y=" + String(y) + " Speed= " + speed + " ("+ xspeed + ","+ yspeed+ ") " + " G=" +G +" xdist " + xdist + " ydist " + ydist;
}
void RunToTarget(int timedelay=0, bool SendFeedback=false, bool SendADC=false,bool SendFastADC=false){
  digitalWrite(7,LOW);
  while (!AtPosition())
  {
    DoMotorStuff();
  }
  digitalWrite(7,HIGH);
  delay(timedelay);
  if (SendFeedback)  Serial.println("$");
}
String GoToAndWait(long x, long y, double speed, int Time){
  String result = GoTo(x,y,speed);
  RunToTarget(Time);
  return result;
}
void Calibrate(){
  digitalWrite(enablePin, LOW);
  GetAnalogPosition();
  Xaxis.setMaxSpeed(300.0);
  Xaxis.setAcceleration(12000.0);

  Yaxis.setMaxSpeed(300.0);
  Yaxis.setAcceleration(12000.0);  

   Serial.println("Calibrating X.");
  if (Joy_X>=0 && Joy_X<499){ //Move left to the end
    Xaxis.moveTo(-1000);
    while (Joy_X<1022)  { GetAnalogPosition(); Xaxis.run();}
  } 
  Xaxis.moveTo(1000);         //Move right to the end
  while (Joy_X>1)       { GetAnalogPosition(); Xaxis.run();}
  Xaxis.setCurrentPosition(0);
  Xaxis.moveTo(-129);         //move to the center
  while (Xaxis.isRunning()) Xaxis.run();  
  Serial.println("Calibration X complete."); 
  
  Serial.println("Calibrating Y.");
  if (Joy_Y>=0 && Joy_Y<499){ //Move left to the end
    Yaxis.moveTo(1000);
    while (Joy_Y<1022)  { GetAnalogPosition(); Yaxis.run();}
  } 
  Yaxis.moveTo(-1000);         //Move right to the end
  while (Joy_Y>1)       { GetAnalogPosition(); Yaxis.run();}
  Yaxis.setCurrentPosition(0);
  Yaxis.moveTo(200);         //move to the center
  while (Yaxis.isRunning()) Yaxis.run();  
  Serial.println("Calibration Y complete.");
  Xaxis.setCurrentPosition(0);
  Yaxis.setCurrentPosition(0);

  delay(100); 
 // digitalWrite(enablePin, HIGH);
  Serial.println("Calibration complete.");

  Calibrated = true;
  SetAcceleration(80000);
  digitalWrite(enablePin, HIGH);

}
void DoMotorStuff(){
  if (error==1){
    Yaxis.stop();
    Xaxis.stop();
  }
  else{
    Yaxis.run();
    Xaxis.run();
  }
  if ((!Xaxis.isRunning())&&(!Yaxis.isRunning())) {
    if (timeoutcounter<timeout)  timeoutcounter++;
    if (timeoutcounter==timeout) 
    {
      Serial.println("Timeout reached, time to turn off the motors");
      //To keep the motors cool and save energy
      digitalWrite(enablePin, HIGH);
      timeoutcounter++;
    }
  }
  else  {   
    digitalWrite(enablePin, LOW);
    timeoutcounter =0;
  }


}
void SetAcceleration(float acc){
  Xaxis.setAcceleration(acc);
  Yaxis.setAcceleration(acc);
  Serial.println("Accelaration: "+ String(acc));
}
// Coord conversion
double deg2step(double deg){ return deg * 80.0 / 9.0; }
double step2deg(double step){ return step * 9.0 / 80.0; }
double rad2deg(double rad){return rad * (180.0 / PI); }
long Pixel2mmX(int x){  return x * X_SIZE / X_Resolution;}
long Pixel2mmY(int y){  return y * Y_SIZE / Y_Resolution;}
int GetXStep(int coord){
    double a_max_rad = 2.0 * atan2(X_SIZE, 2.0 * DIST);
    double a_max_deg = rad2deg(a_max_rad);
    double a_coord = a_max_rad * ((double)coord) / X_SIZE;
    double a_coord_deg = rad2deg(a_coord);
    double steps = deg2step(a_coord_deg) ;
    double maxsteps = deg2step(a_max_deg) ;
    int steps_Round = round(steps-maxsteps/2.0);
    if (DEBUG)
    {
        Serial.print(" X Coord: ");
        Serial.print(coord);
        Serial.print(" Resulting Steps: ");
        Serial.println(steps_Round);
    }
    return -steps_Round;
}
int GetYStep(int coord){
    double a_max_rad = 2.0 * atan2(Y_SIZE, 2.0 * DIST);
    double a_max_deg = rad2deg(a_max_rad);
    double a_coord = a_max_rad * ((double)coord) / Y_SIZE;
    double a_coord_deg = rad2deg(a_coord);
    double steps = deg2step(a_coord_deg) ;
    double maxsteps = deg2step(a_max_deg) ;
    int steps_Round = round(steps-maxsteps/2.0);
    if (DEBUG)
    {
        Serial.print(" Y Coord: ");
        Serial.print(coord);
        Serial.print(" Resulting Steps: ");
        Serial.println(steps_Round);
    }
    return -steps_Round;
}
void GoToCoord(int x, int y, int speed){
  if (DEBUG)
 Serial.println(GoTo(GetXStep(x),GetYStep(y),speed));
 else
 {
   GoTo(GetXStep(x),GetYStep(y),speed);
 }
}
void TrackerPattern(int startpos=-1){
  int coords[] = {960,540,960,92,960,988,115,540,1805,540,115,92,1805,92,115,988,1805,988,538,316,1382,316,538,764,1382,764};
  //coordinates calculated with SR research calibrator
  if (startpos!=-1) i=(startpos-1)*2;
  if (i>24) i=0;
  GoToCoord(Pixel2mmX(coords[i]),Pixel2mmY(coords[i+1]),500);
  RunToTarget();
  //Serial.println("i: " + String(i)+" x: " + String(coords[i]) + " y: " + String(coords[i+1]));
  Serial.println("Pos : " + String(1+(long)i/2));
  i++; i++;  
}
void Spiral(){
   i++;
    if (i%4==0) cur_x+=i;
    else if (i%2==0) cur_x-=i;
    if ((i-1)%4==0) cur_y+=i;
    else if ((i-1)%2==0) cur_y-=i;
    Serial.println(cur_x);
    GoTo(cur_x,cur_y,5000);
  if (i >80) {
     cur_x=0;
     cur_y=0;
     i=0;
  }
}
void AutoSpiral(int speed){
   for (size_t i = 0; i < 200; i++)
   {
      if (i%4==0) cur_x+=i;
      else if (i%2==0) cur_x-=i;
      if ((i-1)%4==0) cur_y+=i;
      else if ((i-1)%2==0) cur_y-=i;
      // Serial.println(cur_x);
      GoTo(cur_x,cur_y,speed);
      RunToTarget();
   }
      cur_x=0;
      cur_y=0;
}
void Square(int size){
  if (i>3) i=0;
  if (i==0) GoTo(size,size,5000);
  if (i==1) GoTo(-size,size,5000);
  if (i==2) GoTo(-size,-size,5000);
  if (i==3) GoTo(size,-size,5000);
  i++;
}
// Movement functions (loop stopping)
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))
bool AtPosition(){  return (Xaxis.distanceToGo()==0)&&(Yaxis.distanceToGo()==0);}
void TimeCritical(long positions[], unsigned int pos_count, long speed=5000, int DelayAfterEachPoint=0,bool ReturnAfterEachPoint=false){
  unsigned int i=0;
  while (i<pos_count)
  {
    //Serial.println(GoTo(positions[i],positions[i+1],speed));
    GoTo(positions[i],positions[i+1],speed);
    RunToTarget(DelayAfterEachPoint);
    i=i+2;
    if (ReturnAfterEachPoint){
      GoTo(0,0,speed);
      RunToTarget(DelayAfterEachPoint);
    }
  }
  GoTo(0,0,speed);
}
void TimeCriticalSerial(int positions[], unsigned int pos_count, long speed=5000, int DelayAfterEachPoint=0,bool ReturnAfterEachPoint=false){
  if (SerialNextPos){
    if (currentpos<(int)pos_count*2)
    {
      GoToCoord(Pixel2mmX(positions[currentpos]),Pixel2mmY(positions[currentpos+1]),speed);
      RunToTarget(DelayAfterEachPoint,true);
      currentpos=currentpos+2;
      SerialNextPos=false;
    }
  }

}
void PrintPositions(){

   /* //for manual testing
   int x[]={615,334,960,1508,1611,1478,1308,1659,1200,1522,1178,1286,689,390,1006,1587,989,572,1333,1616,1081,1261,724,448,815,1374,1482,1124,1631,1040,1596,1280,1154,762,1362,1421,1049,359,1117,497,368,1150,824,1473,975,853,879,718,1033,810,494,1088,540,550,703,248,631,1104,1221,430,972,1702,920,1273,1724,1702,1360,1637,1234,1631,1040,430,326,374,1165,1602,1112,1186,1220,651,942,665,878,1570,1314,1652,1555,1049,579,1096,1399,1005,1538,870,1520,1262,522,314,883,373,1008,406,856,551,1013,644,222,255,599,956,379,533,1096,1391,938,1221,467,1213,1186,525,863,833,1445,1635,1011,1117,446,545,1361,1660,1083,1051,1437,813,1100,513,1323,668,571,997,823,464,1153,1307,538,329,1128,958,1067,1675,1034,280,841,203,934,268,478,248,857,1452};
   int y[]={114,661,770,425,695,312,943,386,110,142,130,686,272,608,698,515,955,960,643,636,311,959,814,887,537,708,365,719,550,560,149,380,632,906,470,171,799,721,748,792,318,362,527,343,551,854,251,621,333,591,365,899,358,814,113,438,135,429,829,961,951,840,820,755,625,256,713,249,133,762,307,270,744,253,430,782,233,907,558,792,261,694,436,825,184,490,209,546,385,779,419,664,847,901,839,729,631,291,113,702,689,301,647,322,367,222,896,325,958,796,370,770,277,416,129,623,646,933,243,481,892,386,745,299,421,828,953,537,483,217,230,969,803,360,489,266,293,157,965,763,400,213,147,537,659,297,216,694,336,479,894,608,671,963,599,645,394,647,695,208};
   for (size_t i = 0; i < 160; i++)
   {
         TestArray[i*2]= x[i];
         TestArray[i*2+1]= y[i];
   }
   */

   Serial.print("X POS: ");
   for (int i = 0; i < 160; i++) 
   {
      Serial.print(String(TestArray[i*2]));
      Serial.print(',');
   }
   Serial.println(";");
   Serial.print("Y POS: ");
   for (int i = 0; i < 160; i++) 
   {
      Serial.print(String(TestArray[i*2+1]));
      Serial.print(',');
   }
   Serial.println(";");
}
long GetDistance(long P1X,long P1Y,long P2X,long P2Y){
	return sqrt(   (P2X-P1X)*(P2X-P1X)  +  (P2Y-P1Y)*(P2Y-P1Y)  );    
	// Pythagorean distance 
}
void GeneratePositions(int Total_Points){
	randomSeed(millis());
   // Left boundry   
	int Border_Left = X_Resolution/10;
   // Right boundry   
	int Border_Right = X_Resolution*9/10;   
   // Top boundry      
	int Border_Top = Y_Resolution/10;    
   // Bottom boundry 
	int Border_Bottom = Y_Resolution*9/10;
	TestArray[0]=random(Border_Right-Border_Left)+Border_Left;                        
   // First X is random
	TestArray[1]=random(Border_Bottom-Border_Top)+Border_Top;
   // First Y is random                      

	for (int i = 1; i < Total_Points; i++)                             
	{
		while (1){
         // fill the vector with random points until the distance condition is met
			long x=random(Border_Right-Border_Left)+Border_Left;
         // rand X
			long y=random(Border_Bottom-Border_Top)+Border_Top;                  
         // rand Y
			TestArray[i*2]= x;
			TestArray[i*2+1]= y;
			long e=GetDistance(x,y,TestArray[2*i-2],TestArray[2*i-1]);           
         // calculate the distance from the previous point
			if  ((e>Min_Dist)  &&  (e<Max_Dist))   break;     
         // if the dist is correct, leave the loop, else repeat with other random possition
		}
	}

	Serial.print("X POS: ");
	for (int i = 0; i < Total_Points; i++) 
	{
		Serial.print(String(TestArray[i*2]));
		Serial.print(',');
	}
  Serial.println(";");
	Serial.print("Y POS: ");
	for (int i = 0; i < Total_Points; i++) 
	{
		Serial.print(String(TestArray[i*2+1]));
		Serial.print(',');
	}
  Serial.println(";");
}
//Horizontal pattern single line
void HorizontalScan(int count, long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(i*8,0,speed,time);
    GoToAndWait(0,0,speed,time);
  }
  GoTo(0,0,speed);
}
//Vertical pattern single line
void VerticalScan(int count, long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(0,i*8,speed,time);
    GoToAndWait(0,0,speed,time);
  }
  GoTo(0,0,speed);
}
//Diagonal pattern multiple lines
void DiagonalScan(int count, long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(i*8,i*8,speed,time);
    GoToAndWait(0,0,speed,time);
  }
    for (int i = -count; i < count; i++)
  {
    GoToAndWait(-i*8,i*8,speed,time);
    GoToAndWait(0,0,speed,time);
  }
  GoTo(0,0,speed);
}
//Vertical pattern multiple lines
void VerticalPattern(int count,long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(i*8,80,speed,time);
    GoToAndWait(i*8,-80,speed,time);
  }
}
//Horizontal patter nmultiple lines
void HorizontalPattern(int count,long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(150,i*8,speed,time);
    GoToAndWait(-150,i*8,speed,time);
  }
}
//Oblique saccade top right, bottom left
void ObliqueSaccade(int count,long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(130,-60,speed,time);
    GoToAndWait(-130,80,speed,time);
  }
}
void CalibPattern(long speed=5000, int time=0){   
  int coords[] = {960,540,960,92,960,988,115,540,1805,540,115,92,1805,92,115,988,1805,988,538,316,1382,316,538,764,1382,764};
  //coordinates calculated with SR research calibrator
  for (int i = 0; i < 26; i++)
  {
  GoToAndWait(GetXStep(Pixel2mmX(coords[i])),GetYStep(Pixel2mmY(coords[i+1])),speed,time);  
  i++;  
  }
}
void ArtificialSaccadesExperiment(){
  TimeCriticalSerial(TestArray,POINTS,3200,300);
}
void ComplexPattern(int positions[], unsigned int pos_count, long speed=5000, int DelayAfterEachPoint=0){
    while (currentpos<(int)pos_count*2)
    {
      GoToCoord(Pixel2mmX(positions[currentpos]),Pixel2mmY(positions[currentpos+1]),speed);
      RunToTarget(DelayAfterEachPoint,true);
      currentpos=currentpos+2;
    }
}
void CheckSerialInput()
{
    while (Serial.available() > 0) {
        char incomingCharacter = Serial.read();
        switch (incomingCharacter) {
        case 'n':   SerialNextPos=true;                                                                 break;    
        case '-':   if (Xaxis.currentPosition()>XMIN) Xaxis.move(-1);                                   break;
        case '+':   if (Xaxis.currentPosition()<XMAX) Xaxis.move(1);                                    break;
        case '/':   if (Yaxis.currentPosition()>YMIN) Yaxis.move(-1);                                   break;
        case '*':   if (Yaxis.currentPosition()<YMAX) Yaxis.move(1);                                    break;
        case 't':   TrackerPattern();                                                                   break;
        case 'T':   TrackerPattern(1);                                                                  break;
        case 'c':   Calibrate();                                                                        break;
        case 'C':   delay(AUTODELAY); Calibrate();                                                      break;
        case '0':   delay(AUTODELAY); CalibPattern(TSPEED,200);          Serial.println("END_TRIAL");   break;
        case '1':   delay(AUTODELAY); HorizontalScan(15,TSPEED,200);     Serial.println("END_TRIAL");   break;
        case '2':   delay(AUTODELAY); VerticalScan(10,TSPEED,200);       Serial.println("END_TRIAL");   break;
        case '3':   delay(AUTODELAY); DiagonalScan(10,TSPEED,200);       Serial.println("END_TRIAL");   break;
        case '4':   delay(AUTODELAY); VerticalPattern(15,TSPEED,200);    Serial.println("END_TRIAL");   break;
        case '5':   delay(AUTODELAY); HorizontalPattern(11,TSPEED,200);  Serial.println("END_TRIAL");   break;
        case '6':   currentpos=0;     SerialNextPos=true;      ArtificialSaccadesExperiment();          break;
        case '7':   GeneratePositions(ARRAYSIZE);                                                       break;
        case '8':   delay(1000);      Calibrate();
                    delay(1000);      CalibPattern(TSPEED,200);
                    delay(1000);      HorizontalScan(15,TSPEED,200);
                    delay(1000);      VerticalScan(10,TSPEED,200);
                    delay(1000);      DiagonalScan(10,TSPEED,200);
                    delay(1000);      VerticalPattern(15,TSPEED,200);
                    delay(1000);      HorizontalPattern(11,TSPEED,200);
                    delay(1000);                                                                        break;
        case 'q':   delay(AUTODELAY); currentpos=0; ComplexPattern(TestArray,160,TSPEED,300);           break;
        case 'x':   Serial.println("END_TRIAL");                                                        break;
        case 's':   ObliqueSaccade(30,TSPEED,200); TrackerPattern(1);                                    break;
        case 'S':   ObliqueSaccade(3,TSPEED,200);  TrackerPattern(1);                                    break;
        if (SerialNextPos) ArtificialSaccadesExperiment();
        if (currentpos==ARRAYSIZE*2)   {Serial.println("END_TRIAL");currentpos++;}
        getstatus();
    }
}
}
void setup()
{  
    Serial.begin(230400);
    Serial.println("---RELAY---");
    Serial.println("Press 'c' for calibration");
    Serial.println("Press 't' for Eye tracking calibration pattern");
    Serial.println("Press 'q' for complex pattern.");
    Serial.println("Press [0-8] to start a trial.");
    Serial.println("Press 'x' to Stop a trial");
    pinMode(enablePin, OUTPUT);
    pinMode(7, OUTPUT);
    digitalWrite(enablePin, LOW);
    SetAcceleration(80000);
}
void loop()
{
  CheckSerialInput();   // wait for input
  DoMotorStuff();       // move motors if necessary
}
