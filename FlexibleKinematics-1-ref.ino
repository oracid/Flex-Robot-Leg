// FlexibleKinematics-1 - 21/12/2020

#include <Servo.h>
int Speed=6000;                           // determines the speed
Servo SrvL, SrvR;                         // Left Servo and Right Servo
int ErrSrvL=0, ErrSrvR=0;  

void  setup() {
  Serial.begin(9600);

  // Servos angle initialization
  int a=73;                                  // a=73, rest point(0,0) for Ay=230mm, a=c=120mm
  SrvL.attach(2); SrvL.write(    a+ErrSrvL); 
  SrvR.attach(3); SrvR.write(180-a+ErrSrvR);

  Serial.print("\n\t To start, click on the Start button");
  while( digitalRead(0) );  delay(400);       // waiting for start button pressed 
  Serial.print("\n\t Started");
}

void loop() {
  Square(-50,120,50,0);

//  Point(0,0);

//  HLine(-50,50,60);
//  HLine(50,-50,60);

//  VLine(0,100,0);
//  VLine(100,0,0);
}

void Point(int x, int y){
  InverseKinematics(x,y,SrvL,SrvR);  
}

void HLine(int Xa,int Xb,int y){
  if (Xa<=Xb) for(int i=Xa; i<=Xb; i++) InverseKinematics(i,y,SrvL,SrvR);
  if (Xa>Xb)  for(int i=Xa; i>Xb; i--)  InverseKinematics(i,y,SrvL,SrvR);
}

void VLine(int Ya,int Yb,int x){
  if (Ya<=Yb) for(int i=Ya; i<=Yb; i++) InverseKinematics(x,i,SrvL,SrvR);
  if (Ya>Yb)  for(int i=Ya; i>Yb; i--)  InverseKinematics(x,i,SrvL,SrvR);
}

void Square(int Xa, int Ya, int Xb, int Yb){
  for(int i=Xa; i<=Xb; i++)  InverseKinematics(i,Ya,SrvL,SrvR);
  for(int i=Ya; i>=Yb; i--)  InverseKinematics(Xb,i,SrvL,SrvR);
  for(int i=Xb; i>=Xa; i--)  InverseKinematics(i,Yb,SrvL,SrvR);
  for(int i=Yb; i<=Ya; i++)  InverseKinematics(Xa,i,SrvL,SrvR);
}

void InverseKinematics(int Px, int Py, Servo LS, Servo RS){
  const float A1x=0, A1y=230, A2x0, A2y=230;                   // Values of servos positions
  const float a1=120, c1=120, a2=120, c2=120;             // Values of leg sizes lengths

  float d=A1y-Py, e=Px;                                   // Calculation of inverse kinematics
  float b=sqrt((d*d)+(e*e));                              // Calculation of inverse kinematics
  float S=acos(d/b);  if(e<0)S=(-S);                      // Calculation of inverse kinematics
  float A12=acos(((b*b)+(c1*c1)-(a1*a1))/(2*b*c1));       // Calculation of inverse kinematics
  float A22=acos(((b*b)+(c2*c2)-(a2*a2))/(2*b*c2));       // Calculation of inverse kinematics
  float A11=(PI/2)-A12+S;                                 // Calculation of inverse kinematics
  float A21=(PI/2)-A22-S;                                 // Calculation of inverse kinematics
  int S1=round(A11*57.296);                               // left servo angle in degree
  int S2=round(180-(A21*57.296));                         // right servo angle in degree

// DEBUG
//  Serial.print("\n\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);
//  Serial.print("\n\t d=");Serial.print(d);Serial.print("\t\t e=");Serial.print(e);
//  Serial.print("\t\t b=");Serial.print(b);Serial.print("\t\t S=");Serial.print(S*57.296);
//  Serial.print("\n\t A11=");Serial.print(A11*57.296);Serial.print("\t\t A12=");Serial.print(A12*57.296);
//  Serial.print("\t\t A22=");Serial.print(A22*57.296);Serial.print("\t\t A21=");Serial.print(A21*57.296);
//  Serial.print("\n\t Result of calculations, angles of the servos");
//  Serial.print("\n\t S1=");Serial.print(S1);Serial.print("°\t\t\t S2=");Serial.print(S2);Serial.print("°");

  if ( b>(a1+c1) ){
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print("\t b=");Serial.print(b);Serial.print(" > ");
    Serial.print(a1+c1);Serial.print(" is too long. Target impossible to reach   !!!!!");
    return;
  }
  if (S1+ErrSrvL<0){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1<0° is not reachable   !!!!!");
    return;
  }
  if (S2+ErrSrvR>180){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2<0° is not reachable   !!!!!");
    return;
  }
  if (S1+ErrSrvL>120){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1>140° is not reachable   !!!!!");
    return;
  }
  if (S2+ErrSrvR<60){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2<40° is not reachable   !!!!!");
    return;
  }
//  Serial.print("\t executed command");
  LS.write(S1+ErrSrvL);   // set target Left servo position if servo switch is On
  RS.write(S2+ErrSrvR);   // set target Right servo position if servo switch is On
  delayMicroseconds(Speed); 
}
