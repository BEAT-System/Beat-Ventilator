#include <Arduino.h>
#include "Rpuffer.h"

Rpuffer meinPuffer(3);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for(int i = 1; i <= 12; i++){
    meinPuffer.set_NewValue(i);
  }

  for(int i = 1; i <= 3; i++){
    Serial.println(meinPuffer.get_Value(i));
  }

  Serial.println(meinPuffer.get_NewValue());
  Serial.println(meinPuffer.get_Mean());

  Serial.println(meinPuffer.get_Size());
  meinPuffer.clear();

  Serial.println(meinPuffer.get_Size());

  for(int i = 20; i <= 30; i++){
    meinPuffer.set_NewValue(i);
  }


  for(int i = 1; i <= 3; i++){
    Serial.println(meinPuffer.get_Value(i));
  }

  Serial.println(meinPuffer.get_NewValue());
  Serial.println(meinPuffer.get_Mean());

  for(int i = 1; i <= 3; i++){
    Serial.println(meinPuffer.get_Value(i));
  }

  Serial.println(meinPuffer.get_Size());


}


void loop() {
  
}