float Kp = 1;
float Kd = 0;
float Ki = 0;
int p;
int d;
int i;
int error;
int lasterror;
int thresh = 127;
int basespeed = 60;



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
float weights[] = {0, -3, -2, -1, 1, 2, 3};
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

void autocaliberate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = 1023; // Set initial minimum values to maximum possible value
    maxValues[i] = 0;    // Set initial maximum values to minimum possible value
  }

  for (int i = 0; i < 3000; i++) {
    for (int j = 0; j < 7; j++) {
      int sensorValue = analogRead(sensorPins[j]);
      if (sensorValue < minValues[j]) {
        minValues[j] = sensorValue;
      }
      if (sensorValue > maxValues[j]) {
        maxValues[j] = sensorValue;
      }
    }
  }

  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    threshold[i] = map(threshold[i], minValues[i], maxValues[i], 0, 255);
    thresh += threshold[i];
  }
  thresh /= 7;

}
void loop() {
  /*if (!calibrationDone){
    autocaliberate();
    calibrationDone = 1;
  }*/
  // put your main code here, to run repeatedly:
  for (int i = 0; i < numSensors; i++) //read funtion
  {
    sensorsA[i] = analogRead(sensorPins[i]);
    Serial.print(sensorsA[i]);
    Serial.print("\t");
    sensorsA[i] = map(sensorsA[i], minValues[i], maxValues[i], 0,255);
    sensorsA[i] = 255 - sensorsA[i];
    Serial.println(sensorsA[i]);
  }

  int e = 0;  //errorcalc funtion
  int n = 0; //number of sensors high
  
  for(int i = 0; i<numSensors; i++)
  {
    if (sensorsA[i] > thresh)
    { e+= sensorsA[i]*weights[i];
      n++;
      Serial.println("yes");
    }
  }

  e /= n;
  error = e;

  p = error;
  i = i + error;
  d = error - lasterror;
  lasterror = error;
  int controlSignal = (Kp*p + Ki*i + Kd*d);

  int leftspeed = basespeed - controlSignal;
  
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
