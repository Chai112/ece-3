#include <ECE3.h>

uint16_t sensorValues[8];

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  int sum = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    sum = sum + (i + 1) * (sensorValues[i]);
  }
  sum = sum / 36;
  Serial.print(sum);
  Serial.print('\t');
  if (sensorValues[3] > sensorValues[0] and
      sensorValues[3] > sensorValues[1] and
      sensorValues[3] > sensorValues[2] and
      sensorValues[4] > sensorValues[5] and
      sensorValues[4] > sensorValues[6] and
      sensorValues[4] > sensorValues[7])
  {
    Serial.print(255);
    Serial.print('\t');
    Serial.print(255);
    Serial.print('\t');
  }
  else if (sum >= 800)
  {
    int r = (sum - 840) / 1.267;
    Serial.print(255);
    Serial.print('\t');
    Serial.print(r);
    Serial.print('\t');
  }
  else if (sum <= 800)
  {
    int l = (sum - 730) / 0.275;
    Serial.print(l);
    Serial.print('\t');
    Serial.print(255);
    Serial.print('\t');
  }
  
  Serial.println();

  delay(50);
}
