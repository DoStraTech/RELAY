/*
   RELAY: Robotic EyeLink AnalYsis

   Copyright (C) 2022  Dominykas.strazdas@ovgu.de

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

//Motor Values
#define XMIN -260.0 
#define XMAX 260.0  
#define YMIN -240.0  
#define YMAX 240.0
//Screen Values
#define X_SIZE 531.36
#define Y_SIZE 298.89
#define DIST 927
#define X_Resolution 1920
#define Y_Resolution 1080

#define DEBUG false  
#define POINTS 160
#define TSPEED 3200
#define AUTODELAY 10000 
//delay countdown to prepare long term exposure
#define Min_Dist 275
#define Max_Dist 825

AccelStepper Xaxis(1, 2, 5); // pin 2 = step, pin 5 = direction
AccelStepper Yaxis(1, 3, 6); // pin 2 = step, pin 5 = direction
const byte enablePin = 8;

int t1 =0;
int t2 =0;
int t3 =0;
int Joy_X = 0;      
int Joy_Y = 0;      
int cur_x=0;
int cur_y=0;
int i=0;
int c=0;
int counterserial = 0;

unsigned long currentMillis;
unsigned long timestamp;
unsigned long timeoutcounter =0;
unsigned long timeout =100000;

bool Calibrated=false;
bool SerialNextPos = false;

int grenzel = X_Resolution/10;     
int grenzer =X_Resolution*9/10;        
int grenzeo = Y_Resolution/10;    
int grenzeu = Y_Resolution*9/10;
int currentpos = 9999;

//#define ARRAYSIZE 160
//int finalArray[ARRAYSIZE*2];
int    finalArray[] = {615,114,334,661,960,770,1508,425,1611,695,1478,312,1308,943,1659,386,1200,110,1522,142,1178,130,1286,686,689,272,390,608,1006,698,1587,515,989,955,572,960,1333,643,1616,636,1081,311,1261,959,724,814,448,887,815,537,1374,708,1482,365,1124,719,1631,550,1040,560,1596,149,1280,380,1154,632,762,906,1362,470,1421,171,1049,799,359,721,1117,748,497,792,368,318,1150,362,824,527,1473,343,975,551,853,854,879,251,718,621,1033,333,810,591,494,365,1088,899,540,358,550,814,703,113,248,438,631,135,1104,429,1221,829,430,961,972,951,1702,840,920,820,1273,755,1724,625,1702,256,1360,713,1637,249,1234,133,1631,762,1040,307,430,270,326,744,374,253,1165,430,1602,782,1112,233,1186,907,1220,558,651,792,942,261,665,694,878,436,1570,825,1314,184,1652,490,1555,209,1049,546,579,385,1096,779,1399,419,1005,664,1538,847,870,901,1520,839,1262,729,522,631,314,291,883,113,373,702,1008,689,406,301,856,647,551,322,1013,367,644,222,222,896,255,325,599,958,956,796,379,370,533,770,1096,277,1391,416,938,129,1221,623,467,646,1213,933,1186,243,525,481,863,892,833,386,1445,745,1635,299,1011,421,1117,828,446,953,545,537,1361,483,1660,217,1083,230,1051,969,1437,803,813,360,1100,489,513,266,1323,293,668,157,571,965,997,763,823,400,464,213,1153,147,1307,537,538,659,329,297,1128,216,958,694,1067,336,1675,479,1034,894,280,608,841,671,203,963,934,599,268,645,478,394,248,647,857,695,1452,208};


//Function declarations
void spiral();
void CheckLight();
void Calibrate();
void SetAcceleration(float acc);
void square(int size=60);
String GoTo(long x, long y, double speed);
void outlines();
void line();
void Mirror13Dots();
void GetTime();
void SetTime();
bool AtPosition();
void DoMotorStuff();

void GetTime(){
  Serial.println(String(millis()-timestamp));
  timestamp = millis();
}
void SetTime(){
  timestamp = millis();
}
void getanalog(){
    Joy_X = analogRead(4); // Read voltage A4 --> VRx - X-Axis
    Joy_Y = analogRead(5); // Read voltage A5 --> VRy - Y-Axis
}
void printanalog(){
  getanalog();
  Serial.println("X=" + String(Joy_X) + " Y=" + String(Joy_Y));
}


int error=0;
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
  getanalog();
  Xaxis.setMaxSpeed(300.0);
  Xaxis.setAcceleration(12000.0);

  Yaxis.setMaxSpeed(300.0);
  Yaxis.setAcceleration(12000.0);  

   Serial.println("Calibrating X.");
  if (Joy_X>=0 && Joy_X<499){ //Move left to the end
    Xaxis.moveTo(-1000);
    while (Joy_X<1022)  { getanalog(); Xaxis.run();}
  } 
  Xaxis.moveTo(1000);         //Move right to the end
  while (Joy_X>1)       { getanalog(); Xaxis.run();}
  Xaxis.setCurrentPosition(0);
  Xaxis.moveTo(-129);         //move to the center
  while (Xaxis.isRunning()) Xaxis.run();  
  Serial.println("Calibration X complete."); 
  
  Serial.println("Calibrating Y.");
  if (Joy_Y>=0 && Joy_Y<499){ //Move left to the end
    Yaxis.moveTo(1000);
    while (Joy_Y<1022)  { getanalog(); Yaxis.run();}
  } 
  Yaxis.moveTo(-1000);         //Move right to the end
  while (Joy_Y>1)       { getanalog(); Yaxis.run();}
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

// Functions using coordinates

void Calibrate_13_Target(int startpos=-1){
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
void spiral(){
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
void autospiral(int speed){
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
void square(int size){
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

void Print_Positions(){

   /* //for manual testing
   int x[]={615,334,960,1508,1611,1478,1308,1659,1200,1522,1178,1286,689,390,1006,1587,989,572,1333,1616,1081,1261,724,448,815,1374,1482,1124,1631,1040,1596,1280,1154,762,1362,1421,1049,359,1117,497,368,1150,824,1473,975,853,879,718,1033,810,494,1088,540,550,703,248,631,1104,1221,430,972,1702,920,1273,1724,1702,1360,1637,1234,1631,1040,430,326,374,1165,1602,1112,1186,1220,651,942,665,878,1570,1314,1652,1555,1049,579,1096,1399,1005,1538,870,1520,1262,522,314,883,373,1008,406,856,551,1013,644,222,255,599,956,379,533,1096,1391,938,1221,467,1213,1186,525,863,833,1445,1635,1011,1117,446,545,1361,1660,1083,1051,1437,813,1100,513,1323,668,571,997,823,464,1153,1307,538,329,1128,958,1067,1675,1034,280,841,203,934,268,478,248,857,1452};
   int y[]={114,661,770,425,695,312,943,386,110,142,130,686,272,608,698,515,955,960,643,636,311,959,814,887,537,708,365,719,550,560,149,380,632,906,470,171,799,721,748,792,318,362,527,343,551,854,251,621,333,591,365,899,358,814,113,438,135,429,829,961,951,840,820,755,625,256,713,249,133,762,307,270,744,253,430,782,233,907,558,792,261,694,436,825,184,490,209,546,385,779,419,664,847,901,839,729,631,291,113,702,689,301,647,322,367,222,896,325,958,796,370,770,277,416,129,623,646,933,243,481,892,386,745,299,421,828,953,537,483,217,230,969,803,360,489,266,293,157,965,763,400,213,147,537,659,297,216,694,336,479,894,608,671,963,599,645,394,647,695,208};
   for (size_t i = 0; i < 160; i++)
   {
         finalArray[i*2]= x[i];
         finalArray[i*2+1]= y[i];
   }
   */

   Serial.print("X POS: ");
   for (int i = 0; i < 160; i++) 
   {
      Serial.print(String(finalArray[i*2]));
      Serial.print(',');
   }
   Serial.println(";");
   Serial.print("Y POS: ");
   for (int i = 0; i < 160; i++) 
   {
      Serial.print(String(finalArray[i*2+1]));
      Serial.print(',');
   }
   Serial.println(";");
}

long get_distance(long P1X,long P1Y,long P2X,long P2Y){
	return sqrt(   (P2X-P1X)*(P2X-P1X)  +  (P2Y-P1Y)*(P2Y-P1Y)  );    
	// Pythagorean distance 
}

void GeneratePositions(int Total_Points){
	randomSeed(millis());
   // Left boundry   
	int grenzel = X_Resolution/10;
   // Right boundry   
	int grenzer = X_Resolution*9/10;   
   // Top boundry      
	int grenzeo = Y_Resolution/10;    
   // Bottom boundry 
	int grenzeu = Y_Resolution*9/10;
	finalArray[0]=random(grenzer-grenzel)+grenzel;                        
   // First X is random
	finalArray[1]=random(grenzeu-grenzeo)+grenzeo;
   // First Y is random                      

	for (int i = 1; i < Total_Points; i++)                             
	{
		while (1){
         // fill the vector with random points until the distance condition is met
			long x=random(grenzer-grenzel)+grenzel;
         // rand X
			long y=random(grenzeu-grenzeo)+grenzeo;                  
         // rand Y
			finalArray[i*2]= x;
			finalArray[i*2+1]= y;
			long e=get_distance(x,y,finalArray[2*i-2],finalArray[2*i-1]);           
         // calculate the distance from the previous point
			if  ((e>Min_Dist)  &&  (e<Max_Dist))   break;     
         // if the dist is correct, leave the loop, else repeat with other random possition
		}
	}

	Serial.print("X POS: ");
	for (int i = 0; i < Total_Points; i++) 
	{
		Serial.print(String(finalArray[i*2]));
		Serial.print(',');
	}
  Serial.println(";");
	Serial.print("Y POS: ");
	for (int i = 0; i < Total_Points; i++) 
	{
		Serial.print(String(finalArray[i*2+1]));
		Serial.print(',');
	}
  Serial.println(";");
}

//Obsolete function, goes through points p[]
void Horizontal_Trial(long speed, int delay){
  long p[]={-140,0,	-132,0,	-120,0,	-108,0,	-104,0,	-100,0,	-88,0,	-80,0,	-64,0,	-48,0,	-44,0,	-40,0,	0,0,	40,0,	44,0,	48,0,	64,0,	80,0,	88,0,	100,0,	104,0,	108,0,	120,0,	132,0,	140,0};
  TimeCritical(p,NELEMS(p),speed,delay,true);
}
//Obsolete function, goes through points p[]
void Vertical_Trial(long speed, int delay){
  long p[]={0,-108,	0,-104,	0,-100,	0,-88,	0,-80,	0,-64,	0,-48,	0,-44,	0,-40,	0,0,	0,40,	0,44,	0,48,	0,64,	0,80,	0,88,	0,100,	0,104,	0,108};
  TimeCritical(p,NELEMS(p),speed,delay,true);
}
//Obsolete function, goes through points p[]
void Circular_Trial(long speed, int delay){
  long p[]={-108,-108,	-100,-100,	-80,-80,	-64,-64,	-48,-48,	-40,-40,	0,0,	40,40,	48,48,	64,64,	80,80,	100,100,	108,108,	108,-108,	100,-100,	80,-80,	64,-64,	48,-48,	40,-40,	-40,40,	-48,48,	-64,64,	-80,80,	-100,100,	-108,108};
  TimeCritical(p,NELEMS(p),speed,delay,true);
}
//Obsolete function, goes through points p[]
void Axial_Trial(long speed, int delay){
  long p[]={-140,-108,	-108,-80,	-80,-64,	-64,-40,	64,40,	80,64,	104,80,	140,108,	140,-108,	108,-80,	80,-64,	64,-40,	-64,40,	-80,64,	-108,80,	-140,108};
  TimeCritical(p,NELEMS(p),speed,delay,true);
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
void VertiCros(int count,long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(i*8,80,speed,time);
    GoToAndWait(i*8,-80,speed,time);
  }
}
//Horizontal patter nmultiple lines
void HoriCros(int count,long speed=5000, int time=0){
  for (int i = -count; i < count; i++)
  {
    GoToAndWait(150,i*8,speed,time);
    GoToAndWait(-150,i*8,speed,time);
  }
}
//Oblique saccade top right, bottom left
void SpotCros(int count,long speed=5000, int time=0){
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

void Sweep(){
  GoTo(40,30,200);
  RunToTarget(0,false,true);
  GoTo(-40,-30,200);
  RunToTarget(0,false,true);
}

void FastSweep(){
  GoTo(30,30,5000);
  RunToTarget(0,false,true);
  GoTo(-30,-30,5000);
  RunToTarget(0,false,true);
}



void maintest(){
  TimeCriticalSerial(finalArray,POINTS,3200,300);
}

void main160test(int positions[], unsigned int pos_count, long speed=5000, int DelayAfterEachPoint=0){
    while (currentpos<(int)pos_count*2)
    {
      GoToCoord(Pixel2mmX(positions[currentpos]),Pixel2mmY(positions[currentpos+1]),speed);
      RunToTarget(DelayAfterEachPoint,true);
      currentpos=currentpos+2;
    }
}



//// Main Menu
void CheckSerial(){
  while (Serial.available() > 0) {
   char incomingCharacter = Serial.read();
   switch (incomingCharacter) {
    case 'n':
      SerialNextPos=true;
    break;    
    case '-':
     if (Xaxis.currentPosition()>XMIN)
      Xaxis.move(-1);
    break;
    case '+':
     if (Xaxis.currentPosition()<XMAX)
      Xaxis.move(1);
    break;
    case '/':
     if (Yaxis.currentPosition()>YMIN)
      Yaxis.move(-1);
    break;
    case '*':
     if (Yaxis.currentPosition()<YMAX)
      Yaxis.move(1);
    break;
    case '0':
    delay(AUTODELAY);
      CalibPattern(TSPEED,200);
      Serial.println("END_TRIAL");
    break;
    case '1':
    delay(AUTODELAY);
      HorizontalScan(15,TSPEED,200);
     Serial.println("END_TRIAL");
    break;
    case '2':
    delay(AUTODELAY);
      VerticalScan(10,TSPEED,200);
      Serial.println("END_TRIAL");
    break;
    case '3':
    delay(AUTODELAY);
      DiagonalScan(10,TSPEED,200);
      Serial.println("END_TRIAL");
    break;
    case '4':
    delay(AUTODELAY);
    VertiCros(15,TSPEED,200);
     Serial.println("END_TRIAL");
    break;
    case '5':
    delay(AUTODELAY);
    HoriCros(11,TSPEED,200);
     Serial.println("END_TRIAL");
    break;
    case '6':
      currentpos=0;
      SerialNextPos=true;
      maintest();
    break;
    case '7':
      GeneratePositions(POINTS);
    break;
    case '8':
      delay(1000);
      Calibrate();
      delay(1000);
      CalibPattern(TSPEED,200);
      delay(1000);
      HorizontalScan(15,TSPEED,200);
      delay(1000);
      VerticalScan(10,TSPEED,200);
      delay(1000);
      DiagonalScan(10,TSPEED,200);
      delay(1000);
      VertiCros(15,TSPEED,200);
      delay(1000);
      HoriCros(11,TSPEED,200);
      delay(1000);
    break;
    case 'c':
      Calibrate();
    break;
    case 'C':
      delay(AUTODELAY);
      Calibrate();
    break;
    case '9':
      delay(AUTODELAY);
      VertiCros(15,TSPEED,200);
      HoriCros(11,TSPEED,200);
    break;    
    case 'q': 
      Print_Positions();
      Calibrate();
      delay(AUTODELAY);
      currentpos=0;
      main160test(finalArray,160,3200,300);
    break;
        case 'Q': 
      Print_Positions();
      currentpos=0;
      main160test(finalArray,160,3200,300);
    break;
    case 's':
      SpotCros(30,TSPEED,200);
      Calibrate_13_Target(1);
    break;
    case 'S':
      SpotCros(3,TSPEED,200);
      Calibrate_13_Target(1);
    break;
    case 'f':
      Calibrate_13_Target();
    break;
    case 'F':
      Calibrate_13_Target(1);
    break;
    case 'x':
    Serial.println("END_TRIAL");
    break;
    case 'k':
      Sweep();
    break;
    case 'K':
      FastSweep();
    break;
    }
   if (SerialNextPos) maintest();
    if (currentpos==POINTS*2)   {Serial.println("END_TRIAL");currentpos++;}
    if (DEBUG)                  Serial.println(getstatus());
    else getstatus();
  }
}

void setup()
{  
    Serial.begin(230400);
    Serial.println("---RELAY---");
    Serial.println("Press c for callibration");
    pinMode(enablePin, OUTPUT);
    pinMode(7, OUTPUT);
    digitalWrite(enablePin, LOW);
    SetAcceleration(80000);
}

void loop()
{
  currentMillis = millis();  
  // get the current time
  CheckSerial();
  // wait for input
  DoMotorStuff();
  // move motors if necessary
}