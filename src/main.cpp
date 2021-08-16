#include <Arduino.h>
#include <Wire.h>
#include "Rpuffer.h"
#include <SPI.h>
#include "SerialManager.h"
#include "VentiData.h"
#include <SoftwareSerial.h>

// ANALOG PINS
#define AV1_Pin 3           // Analog Value 1 PIN (= FLOW Sensor)

// DIGITAL PINS
#define sw_end_PIN 3        // Endschalter Digital PIN
#define sw_start_PIN 2      // Startschalter Digital PIN
#define ESCON_PWM_Pin 5     // PWM out zum ESCON Modul
#define ESCON_START_Pin 6   // Start Freigabe zum ESCON MOdul
#define ESCON_CC_Pin 7      // Drehrichtung zum ESCON Modul
#define SWserial_TX_Pin 8   // Software Serial TX Pin
#define SWserial_RX_Pin 9   // Software Serial RX Pin

// HARDWARE SERIAL (SERIAL MANGER)
Data input;
Data output;
SerialManager<Data, Data> manager(Serial, input, output, FAST, FAST);

// SOFTWARE SERIAL
SoftwareSerial SWserial (SWserial_RX_Pin, SWserial_TX_Pin);
int InputBuffer[32];        // Eingangspuffer
int Values[32];             // Serielle einkommende Daten
int n = 0;                  // Zähler
int dummy = 0;              // Dummybyte

// ABTASTUNG 
unsigned long tRegler = 0;  // Zeitmarke Regler
unsigned long tMessung = 0; // Zeitmarke Messwertaufnahme
unsigned long tSerialSend = 0;   // Zeitmarke Serielle Communikation
unsigned long tZielfunktion = 0; // Zeitmarke Zielfunktion
int dtRegler = 100;         // Abtastzeit Regler in ms
int dtMessung = 100;        // Abtastzeit Messwertaufnahme in ms
int dtSerialSend = 100;     // Abtastzeit Serielle Communikation in ms
int dtZielfunktion = 0;     // Halbe Periodenlänge der Zeilfunktion in ms -> Wird nach bpm definiert ( = 60 s / (2*bpm) * 1000 )

// DRUCKMESSUNG
byte byte_msb, byte_lsb;    // 8 bit values High, Low Byte from Sensor
int16_t p_counts;           // 16 bit Value in Counters
float p_ist;                // Druck IST Wert in mBar

// FLOW + VOLUMEN MESSUNG
float flow = 0;
float volume = 0;
float volume_insp = 0;
float volume_exp = 0;

// ZIELFUNKTION
int switchZielfunktion = 0; // Status INSP = 1 / EXSP = 0
int p_ins = 0;              // Inspiratuins Druck-Sollwert in mBar
int p_exp = 0;              // Exsiprations Druck-Sollwert in mBar
int p_soll;                 // Zielfunktion Sollwert (p_ins und P_exp abwechselnd)// 
int bpm = 0;                // Beatmungen pro Minute
int state = 0;              // STATUS STEUERVARIABLE der LOOPS
int limiter = 1;

// REGLER
float p_delta;                          // Druckabweichung soll - ist
float p_deltaProz;                      // Druckabweichung soll - ist in Prozent
float p_deltaProzSLOWModeInsp = 15.0;   // Toleranzschlauch in Prozent in Welchem der SLOW Mode aktiviert ist     
float p_deltaProzSLOWModeExp = 10.0;    // Toleranzschlauch in Prozent in Welchem der SLOW Mode aktiviert ist 
int v_motor = 0;                        // Stellgröße Servomotor Geschwindigkeit
int v_pos = 50;                         // Geschwindigkeit für Anfahr und Abfahrroutine
int v_contr_inital_forward  = 10 ;      // Initial Geschwindigkeit für Inspirationsphase
int v_contr_inital_backward =-70 ;      // Initial Geschwindigkeit für Exspirationsphase
int v_contr_slowm_limit_forward  =  15; // positives Geschwidigkeitslimit im SLOW Mode
int v_contr_slowm_limit_backward = -10; // negatives Geschwindigkeitslimit im SLOW Mode
int Kp = 10;                            // P-Regler Faktor

// FUNCTIONS
void f_aktivateESCON (bool state);
void f_setESCON(int v_proz);


void setup() {
  // PINMODE DIGITAL PINS
  pinMode(sw_end_PIN,INPUT);        // Endschalter Digital PIN
  pinMode(sw_start_PIN,INPUT);      // Startschalter Digital PIN
  pinMode(ESCON_CC_Pin,OUTPUT);     // PWM out zum ESCON Modul
  pinMode(ESCON_PWM_Pin,OUTPUT);    // Start Freigabe zum ESCON MOdul
  pinMode(ESCON_START_Pin,OUTPUT);  // Drehrichtung zum ESCON Modul
  pinMode(SWserial_RX_Pin, INPUT);  // Software Serial RX Pin
  pinMode(SWserial_TX_Pin, OUTPUT); // Software Serial TX Pin

  Serial.begin(9600);               // Start Hardware Serial
  Wire.begin();                     // Start I2c
  SPI.begin();                      // Start SPI
  SWserial.begin(115200);           // Start Software Serial

  f_setESCON(0);                    // Set velocity zero
  f_aktivateESCON(0);               // Dektivate Escon



}




void loop() {
  //  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% HARDWARE SERIAL WERTE EMPFANGEN ##############################################
  manager.recieve(); // Seriellen Manager updaten
  
  if(input.contr_mode == 1) { // Paramter updaten
    // BPM und Amplitude
    bpm =                           10;//input.bpm;
    p_ins =                         20;//input.p_ins;
    p_exp =                         0;
    dtZielfunktion =                int( 60.0 / (2.0*bpm) * 1000.0 );
  }
  
  if ( (state == 0)&&(input.contr_mode == 1) ) { // Maschine wird eingeschaltet
    switchZielfunktion =  0;
    state = 1;
    f_aktivateESCON(1);
  }
  else if ( (state > 0)&&(input.contr_mode == 0) ) { // Maschine wird ausgeschaltet
    state = 0;
    f_aktivateESCON(0);
    f_setESCON(0);
  }
  // Ende Hardware Serial Werte Empfangen

  //  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SOFTWARE SERIAL WERTE EMPFANGEN ##############################################
  if (SWserial.available() > 0) { // Wenn Serial Werte vorhanden
    // Werte in InputBuffer schreiben
    n = 0;
    while((SWserial.available() > 0) && (n <= 31)) {
      InputBuffer[n] = SWserial.read();
      n = n + 1;
      delay(5);
    }
    // Sicherheitsprüfung und Werte Übernehmen
    // Stelle 31 = Zeilenumbruch = ASCII 10
    // Stelle 20 = Komma = ASCII 44
    // Stelle 28 = Komma = ASCII 44
    if ( (InputBuffer[31] == 10)&&(InputBuffer[20] == 44)&&(InputBuffer[28] == 44) ){ // Sicherheitsüberprüfung
        // Werte übernehmen und InputBuffer leeren
        for(int k = 0; k <= 31; k++) {
          Values[k] = InputBuffer[k];
          InputBuffer[k] = 0; 
        }
    }
    else {
        // InputBuffer leeren
        for(int k = 0; k <= 31; k++) {
          InputBuffer[k] = 0; 
        }
    }

    // Seriellen Buffer leeren
    while(SWserial.available() > 0) {
      dummy = SWserial.read();
      delay(5);
    }

    // BPM und Amplitude ausrechnen
    bpm  =                          (Values[0]-48)*10+(Values[1]-48);
    p_ins =                         (Values[3]-48)*10+(Values[4]-48);
    p_exp =                         (Values[6]-48)*10+(Values[7]-48);
    dtZielfunktion = int( 60.0 / (2.0*bpm) * 1000.0 );
    
    // Reglerparameter ausrechnen
    v_contr_inital_forward  =       (Values[9]-48)*10+(Values[10]-48);
    v_contr_inital_backward =       - ((Values[12]-48)*10+(Values[13]-48));
    v_contr_slowm_limit_forward  =  (Values[15]-48)*10+(Values[16]-48);
    v_contr_slowm_limit_backward =  -((Values[18]-48)*10+(Values[19]-48));
    p_deltaProzSLOWModeInsp =float( (Values[21]-48)*100+(Values[22]-48)*10+(Values[23]-48) );
    p_deltaProzSLOWModeExp  =float( (Values[25]-48)*100+(Values[26]-48)*10+(Values[27]-48) );
    Kp =                            (Values[29]-48)*10+(Values[30]-48);

    
    // // START ABFRAGE
    // if( (bpm > 0) && (p_ins > 0) ) {

    //   if(state == 0) { // Maschine wird einschalten
    //     // Anfahrroutine
    //     /* while(digitalRead(sw_start_PIN) == 0){
    //       f_setEPOS(v_pos); // Langsam vorfahren bis Startschalter überfahren wurde
    //     }
    //     f_setEPOS(0); // Halten
    //     delay(3000); */ 
    //     // Anfahrroutine Ende
    //     switchZielfunktion = 0;
    //     state = 1;
    //     f_aktivateESCON(1);
    //   } 

    // }
    // else if ((bpm == 0) && (p_ins == 0)) {
      
    //   if(state > 0) { // Maschine ausschalten
    //     // Abfahrroutine
    //     /* while(digitalRead(sw_start_PIN) == 1){
    //       f_setEPOS(-v_pos); // Langsam zurückfahren bis Startschalter erreicht wurde
    //     }
    //     f_setEPOS(0); // Halten
    //     delay(3000);
    //     // Abfahrroutine Ende */

    //     state = 0;
    //     f_setESCON(0);
    //     f_aktivateESCON(0);
    //   } 
    // }
    
  } 
  // Ende Software Serial Werte Empfangen


  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%########%%% LOOPS #########################################################
  if (state == 1) { // START LOOPS
    tRegler = millis();
    tMessung = millis();
    tSerialSend = millis();
    tZielfunktion =  millis();
    state = 2;
  }

  if (state == 2) {
    
    // %%%%%%%%%%%%% LOOP ZIELFUNKTION %%%%%%%%%%%%%%%%%%
    if (millis() >= (tZielfunktion+dtZielfunktion) ){      
      
      if(switchZielfunktion == 0) {
        switchZielfunktion = 1;
        p_soll = p_ins; // Inspiration
        limiter = 0;
        volume_exp = volume;
        volume = 0;
      }
      else if(switchZielfunktion == 1) {
        switchZielfunktion = 0;
        p_soll = p_exp; // Exspiration
        limiter = 0;
        volume_insp = volume;
        volume = 0;
      }
      
      tZielfunktion =  millis();
    }   
    // %%%%%%%%%%%%%%%%%%%%%% ENDE %%%%%%%%%%%%%%%%%%%%%%

    // %%%%%%%%%%%%%%%%%% LOOP MESSUNG %%%%%%%%%%%%%%%%%%
    if (millis() >= (tMessung+dtMessung) ){
      
      // Druck Messen
      Wire.requestFrom(0x78, 2); // request two bytes from I2C slave address 0x78
      byte_msb = Wire.read();
      byte_lsb = Wire.read();
      p_counts = ((int16_t)byte_msb << 8) | byte_lsb;
      p_ist = (p_counts - 2731) / 218.44;
      p_delta = float(p_soll) - p_ist;
      p_deltaProz = (p_delta / float(p_soll))*100;

      // Flow Messwert (Vorzeichen durch Zielfunktion ermitteln)
      if (switchZielfunktion == 1) {
        flow = ( (float(analogRead(AV1_Pin))*5/1024)  -0.36 )*(100/3)   ; // Flowsensor Analogsignal
      }
      else if (switchZielfunktion == 0) {
        flow = ( (float(analogRead(AV1_Pin))*-5/1024) + 0.36 )*(100/3)  ; // Flowsensor Analogsignal
      }

      // Volumen berechen
      volume = volume + ( flow * ((millis()-tMessung)/1000.0/60.0) );
      
      tMessung = millis();
    }
    // %%%%%%%%%%%%%%%%%%%%%% ENDE %%%%%%%%%%%%%%%%%%%%%

    // %%%%%%%%%%%%%%%%%% LOOP REGLER %%%%%%%%%%%%%%%%%%
    if (millis() >= (tRegler+dtRegler) ){

      if (switchZielfunktion == 1) {
        // INSPIRATIONSPHASE
        if(digitalRead(sw_end_PIN) == 0 ){
          v_motor = 0;
          limiter = 1;
        }
        else if (limiter == 0 && (abs(p_deltaProz) > p_deltaProzSLOWModeInsp) ) {
          // FAST CONTROL MODE
          v_motor = v_contr_inital_forward + int(Kp * p_delta); // Stellgröße normal
        }
        else if (limiter == 0 && (abs(p_deltaProz) <= p_deltaProzSLOWModeInsp) ) {
          // SLOW CONTROL MODE
          v_motor = v_contr_inital_forward + int(Kp * p_delta); // Stellgröße ... 
          v_motor = constrain(v_motor,v_contr_slowm_limit_backward,v_contr_slowm_limit_forward); // ... begrenzen
        }

        f_setESCON(v_motor);

      }
      else if (switchZielfunktion == 0) {
        // EXSPIRATIONSPHASE
        if(digitalRead(sw_start_PIN) == 0 ){
          v_motor = 0;
          limiter = 1;
        }
        else if (limiter == 0 ){//&& (abs(p_deltaProz) > p_deltaProzSLOWModeExp) ) {
          // FAST CONTROL MODE
          v_motor = v_contr_inital_backward;// + int(Kp * p_delta); // Stellgröße normal
        }
        //else if (limiter == 0 && (abs(p_deltaProz) <= p_deltaProzSLOWModeExp)) {
        //  // SLOW CONTROL MODE
        //  v_motor = v_contr_inital_backward;// + int(Kp * p_delta); // Stellgröße ...
        //  v_motor = constrain(v_motor,v_contr_slowm_limit_backward,v_contr_slowm_limit_forward); // ... begrenzen
        //}
        
        f_setESCON(v_motor);

      }

      
      tRegler = millis();
    }
    // %%%%%%%%%%%%%%%%%%%%%% ENDE %%%%%%%%%%%%%%%%%%%%%%

    // %%%%%%%%%%%%% LOOP SERIELL SEND %%%%%%%%%%%%%%%%%%
    if (millis() >= (tSerialSend+dtSerialSend) ){
      
      SWserial.print(millis()); //1
      SWserial.print(",");
      SWserial.print(bpm);//2
      SWserial.print(",");
      SWserial.print(p_soll);//3
      SWserial.print(",");
      SWserial.print(p_ist);//4
      SWserial.print(",");
      SWserial.print(v_motor);//5
      SWserial.print(",");
      SWserial.print(limiter);//6
      SWserial.print(",");
      SWserial.print(v_contr_inital_forward);//7
      SWserial.print(",");
      SWserial.print(v_contr_inital_backward);//8
      SWserial.print(",");
      SWserial.print(v_contr_slowm_limit_forward);//9
      SWserial.print(",");
      SWserial.print(v_contr_slowm_limit_backward);//10
      SWserial.print(",");
      SWserial.print(p_deltaProzSLOWModeInsp);//11
      SWserial.print(",");
      SWserial.print(p_deltaProzSLOWModeExp);//12
      SWserial.print(",");
      SWserial.print(Kp);//13
      SWserial.print(",");
      SWserial.print(flow);//14
      SWserial.print(",");
      SWserial.print(p_deltaProz);//15
      SWserial.print(",");
      SWserial.println(volume);//16
      //SWserial.print(",");
      //SWserial.println(volume_exp);//17

      tSerialSend = millis();
      
    }
    // %%%%%%%%%%%%%%%%%%%%%% ENDE %%%%%%%%%%%%%%%%%%%%%%

    
  } // %% ENDE if (state == 2) {


} // Ende Loop






// ################################### F U N C T I O N S #########################################

void f_aktivateESCON (bool state) {
  if(state) {
    digitalWrite(ESCON_START_Pin,1);
  }
  else
  {
    digitalWrite(ESCON_START_Pin,0);
  }
  
}

void f_setESCON(int v_proz) {
  v_proz = constrain(v_proz,-100,100);
  int v_proz_abs = abs(v_proz);
  int PWM_value = (255.0-24.0-26.0)/100.0*float(v_proz_abs) + 24.0;
  PWM_value = constrain(PWM_value,24,229);

  if(v_proz == 0) {
    analogWrite(ESCON_PWM_Pin,PWM_value);
    digitalWrite(ESCON_CC_Pin,0);
    digitalWrite(ESCON_START_Pin,1);
  }
  else if (v_proz > 0) {
    analogWrite(ESCON_PWM_Pin,PWM_value);
    digitalWrite(ESCON_CC_Pin,0);
    digitalWrite(ESCON_START_Pin,1);
  }
  else if (v_proz < 0) {
    analogWrite(ESCON_PWM_Pin,PWM_value);
    digitalWrite(ESCON_CC_Pin,1);
    digitalWrite(ESCON_START_Pin,1);
  }
}