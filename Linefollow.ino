float Kp = 100;
float Kd = 0;
float Ki = 0;
int p;
int d;
int i;
int error;
int lasterror;
int threshhold = 600;
int setpoint = 2500;
int basespeed = 150;



int minValues[7];
int maxValues[7];
int threshold[7];
int calibrationDone = 0;

int R1 = 4;
int R2= 3;
int L1= 10;
int L2 = 9;
int enA = 5; //left
int enB = 6;  //rigt
int sensorPins[] = {A0, A1, A2, A3, A4,A5,A6};
float weights[] = {2500, 0, 1000, 2000, 3000, 4000, 5000};
int numSensors = 7;
int sensorsA[7]={0,0,0,0,0,0,0}; //analog values off sensor
int sensorsD[7]={0,0,0,0,0,0,0}; //digital values



void setup() {
  Serial.begin(9600);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

}

void autocaliberate()
{
  for (int i=0; i<=6 ; i++)
  {
    minValues[i]=analogRead(i);
    maxValues[i]=analogRead(i);

  }
  for (int i=0 ; i <3000; i++)
  {
    for ( int i=1 ; i <=6 ; i++)
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


void loop() {
   if (!calibrationDone){
    autocaliberate();
    calibrationDone = 1;
  }
  // put your main code here, to run repeatedly:
  for (int i = 0; i < numSensors; i++) //read funtion
  {
    sensorsA[i] = analogRead(sensorPins[i]);
  }
  for(int i = 0; i<numSensors; i++)
  {
    if (sensorsA[i] < threshhold)
      sensorsD[i] = 1;
    else
      sensorsD[i] = 0;
  }

  int e = 0;  //errorcalc funtion
  int n = 0; //number of sensors high
  for (int i = 1; i<numSensors; i++)
  {
    e += weights[i]*sensorsD[i];
    n+= sensorsD[i];
  }

  

  e /= n;
  error = (e - setpoint);

  p = error;
  i = i + error;
  d = d = error - lasterror;
  lasterror = error;
  int controlSignal = (Kp*p + Ki*i + Kd*d);

  int leftspeed = basespeed -controlSignal;
  
  int rightspeed = basespeed + controlSignal;

  leftspeed = constrain(leftspeed, 0, 255);
  rightspeed = constrain(rightspeed, 0, 255);

  
  
  analogWrite(enA,rightspeed);
  analogWrite(enB, leftspeed);
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);

}
