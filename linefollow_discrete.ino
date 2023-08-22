int sensorPins[] = {A0, A1, A2, A3, A4,A5,A6};
int sensorsA[7]={0,0,0,0,0,0,0};
int minValues[7];
int maxValues[7];
int threshold[7];
int calibrationDone = 0;
int lfspeed=150;

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
float Kp = 0.5;
float Kd = 0;
float Ki = 0 ;

int R1 = 4;
int R2= 3;
int L1= 10;
int L2 = 9;
int enA = 5; //left
int enB = 6;  //rigt

 //digital values



void setup() {
  Serial.begin(9600);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

}


//caliberation
void autocaliberate()
{
  for (int i=0; i<=6 ; i++)
  {
    minValues[i]=analogRead(i);
    maxValues[i]=analogRead(i);

  }
  for (int i=0 ; i <3000; i++)
  {
    for ( int i=0 ; i <=6 ; i++)
    {
      if (analogRead(i)<minValues[i])
      {
        minValues[i]=analogRead[i];
      }
      if(analogRead(i)>maxValues[i])
      {
        maxValues[i]=analogRead(i);
      }
    }
  }

  for (int i=1 ; i<6 ; i++)
  {
    threshold[i]=(minValues[i] +maxValues[i])/2;
    Serial.print(threshold[i]);
    Serial.println("  ");
  }
}



///pid

void linefollow()
{
  int error = (analogRead(5) - analogRead(2));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  digitalWrite(R1,1);
  digitalWrite(R2,0);
  digitalWrite(L1,1);
  digitalWrite(L2,0);
  analogWrite(enA,lsp);
  analogWrite(enB,rsp);

}

void loop() {

  if (!calibrationDone){
    autocaliberate();
    calibrationDone = 1;
  }

if (analogRead(1) > threshold[1] && analogRead(6) < threshold[6] )
    {
      lsp = 0; rsp = lfspeed;
      digitalWrite(R1,1);
      digitalWrite(R2,0);
      digitalWrite(L1,1);
      digitalWrite(L2,0);
      analogWrite(enA,0);
      analogWrite(enB,lfspeed);
      
    }
 else if (analogRead(6) > threshold[6] && analogRead(1) < threshold[1])
    { lsp = lfspeed; rsp = 0;
      digitalWrite(R1,1);
      digitalWrite(R2,0);
      digitalWrite(L1,1);
      digitalWrite(L2,0);
      analogWrite(enA,lfspeed);
      analogWrite(enB,0);
    }
  
  else if (analogRead(3) > threshold[3] &&analogRead(4) > threshold[4]&&analogRead(0) > threshold[0])
    {
      linefollow();
    }
}


  
