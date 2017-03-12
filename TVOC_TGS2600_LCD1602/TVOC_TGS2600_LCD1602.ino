#include <Wire.h>   
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x3F,16,2);    

#define SENSOR_WARMUP_WINDOW 1200
#define MAXIMUM_UPDATE_WINDOW 240
#define SATURATION_WINDOW 900

enum tvoc_state {CLEAR, AL_1, AL_2, AL_3, WARM_UP, RE_CALIBRATION};
tvoc_state tvoc_fsm = WARM_UP;

float tvoc = 0;
int Vrl = 0;  // variable to store the value coming from the sensor
int Vk = 0;  // variable to store the value coming from the sensor
float K=0;
boolean tvoc_updisp = true;

int VrlPin = A0;
int VkPin = A1;
float rs=0;
float rs_max=0;
float rs_step=50;
float rs_max_r=0;
float rs_max_abs=0;
int tvoc_received_pkg=0;
int alert_lvl = 4;
float G[]={0, 0.08, 0.16, -0.03, 0.05, 0.13, 0.03, 0.95, 0.87};
float determ_r=0;
float satura_r=0;
int saturation_count=0;
int rs_satu =0;

void setup()  
{  
  randomSeed(analogRead(0));
  lcd.init();                  
  lcd.backlight();               
  Serial.begin(9600);
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(2, HIGH);  
  digitalWrite(3, HIGH);  
  digitalWrite(4, HIGH);  
  digitalWrite(5, HIGH);  
  lcd.setCursor(0,0);                
  lcd.print("TvocQuickFlag v1");       
  lcd.setCursor(0,1);  
  lcd.print("   by Wang Jiang");  
  delay(3000);
  lcd.clear();
  digitalWrite(2, LOW);  
  digitalWrite(3, LOW);  
  digitalWrite(4, LOW);  
  digitalWrite(5, LOW);  
}  


void loop()  
{  
  TvocSensorRead();
  
  TvocLevelDetermination();
  
  TvocLedDisplay();
  
  if (tvoc_fsm==WARM_UP){
    lcd.home();
    lcd.print ("TVOC Sensor");     
    lcd.setCursor(0, 1);
    lcd.print ("Warm Up...  ");     
    lcd.setCursor(12, 1);
    lcd.print (tvoc_received_pkg/10);     
    lcd.setCursor(15, 1);
    lcd.print ("s");     
  } else if (tvoc_fsm==RE_CALIBRATION){
    lcd.home();
    lcd.print ("Sensor Saturated");     
    lcd.setCursor(0, 1);
    lcd.print ("ReCalibrate ");     
    lcd.setCursor(12, 1);
    lcd.print (tvoc_received_pkg/10);     
    lcd.setCursor(15, 1);
    lcd.print ("s");     
  } else {
    lcd.home();
    lcd.print ("TVOC: ");
    lcd.setCursor(6, 0);
    lcd.print(tvoc);
    lcd.setCursor(10, 0);
    lcd.print (" mg/m3");
    lcd.setCursor(0, 1);
    lcd.print ("Level: ");
    lcd.setCursor(7, 1);
    switch (alert_lvl){
      case 0:
      lcd.print("CLEAN ^_^");
      break;
      
      case 1:
      lcd.print("LIGHT!   ");
      break;
            
      case 2:
      lcd.print("MEDIUM!! ");
      break;
      
      case 3:
      lcd.print("HEAVY!!! ");
      break;
          
      default:
        ;      
    }
  }

}  

void TvocSensorRead(){
  int v_rl[8], v_k[8];
  int v_rl_sum=0, v_k_sum=0;
  if(tvoc_fsm==WARM_UP){
    tvoc_received_pkg++;  
    delay(100);    
    Vrl = analogRead(VrlPin);    
    rs=((1023.0-Vrl)/Vrl)*9760.0;
  } else {
    for (int i=0; i<8;i++){
      v_rl[i] = analogRead(VrlPin);    
      v_k[i] = analogRead(VkPin);
      delay(250);    
      v_rl_sum=v_rl_sum+v_rl[i];
      v_k_sum=v_k_sum+v_k[i];  
    }
    Vrl=v_rl_sum/8.0;
    Vk=v_k_sum/8.0;
    rs_satu=rs;
    rs=((1023.0-Vrl)/Vrl)*9760.0;
    K = ((float)Vk)/1023.0;
    if (K<0.06 || K>0.96)
      K=0.84;    
    tvoc_received_pkg++;  
  }
  rs_max_abs=rs_max_abs>rs? rs_max_abs : rs;
  tvoc=((rs_max_abs-rs)/rs_step)/100.0;
  digitalWrite(13, lowByte(tvoc_received_pkg)&0x01); 
}

void TvocLevelDetermination(){
  determ_r=rs/rs_max_r;
  satura_r=rs/rs_satu;
  switch (tvoc_fsm){
  case CLEAR:
    if(tvoc_received_pkg<MAXIMUM_UPDATE_WINDOW){
      rs_max=rs>rs_max? rs : rs_max;
    }else{
       tvoc_received_pkg=0;
       rs_max_r=rs_max>rs_max_r? rs_max : rs_max_r;
       rs_max=rs;
    }
    if(determ_r<=(K-G[0])){
      alert_lvl=1;
      tvoc_fsm=AL_1;
      rs_satu=rs;
      tvoc_received_pkg=0;
    }
  break;
  
  case AL_1:
    if(tvoc_received_pkg<MAXIMUM_UPDATE_WINDOW){
      rs_max=rs>rs_max? rs : rs_max;
    }else{
       tvoc_received_pkg=0;
       rs_max_r=rs_max>rs_max_r? rs_max : rs_max_r;
       rs_max=rs;
    }
    if(determ_r<=(K-G[1])){
      alert_lvl++;
      tvoc_fsm=AL_2;
      saturation_count=0;
      tvoc_received_pkg=0;
    } else if(determ_r>=(K-G[3])){
      alert_lvl--;
      tvoc_fsm=CLEAR;
      rs_max_r=rs;
      rs_max=rs;
      saturation_count=0;
      tvoc_received_pkg=0;
    } 
    if(satura_r>=G[7]){
      saturation_count++;
    } else {
      saturation_count--;
    }
    if (saturation_count==SATURATION_WINDOW){
      tvoc_fsm=RE_CALIBRATION;
      alert_lvl=4;
      rs_max=rs;
      rs_max_r=rs;
      saturation_count=0;
      tvoc_received_pkg=0;
      rs_max_abs=rs;
    }
  break;
  
  case AL_2:
     if(tvoc_received_pkg<MAXIMUM_UPDATE_WINDOW){
       rs_max=rs>rs_max? rs : rs_max;
    }else{
       tvoc_received_pkg=0;
       rs_max_r=rs_max>rs_max_r? rs_max : rs_max_r;
       rs_max=rs;
    }
    if(determ_r<=(K-G[2])){
      alert_lvl++;
      tvoc_fsm=AL_3;
      saturation_count=0;
      tvoc_received_pkg=0;
    } else if(determ_r>=(K-G[4])){
      alert_lvl--;
      tvoc_fsm=AL_1;
      saturation_count=0;
      tvoc_received_pkg=0;
    }
    if(satura_r>=G[7]){
      saturation_count++;
    } else {
      saturation_count--;
    }
    if (saturation_count==SATURATION_WINDOW){
      tvoc_fsm=RE_CALIBRATION;
      alert_lvl=4;
      rs_max=rs;
      rs_max_r=rs;
      saturation_count=0;
      tvoc_received_pkg=0;
      rs_max_abs=rs;
    }
    break;
  
  case AL_3:
     if(tvoc_received_pkg<MAXIMUM_UPDATE_WINDOW){
      rs_max=rs>rs_max? rs : rs_max;
    }else{
       tvoc_received_pkg=0;
       rs_max_r=rs_max>rs_max_r? rs_max : rs_max_r;
       rs_max=rs;
    }
    if(determ_r>=(K-G[5])){
      alert_lvl--;
      tvoc_fsm=AL_2;
      saturation_count=0;
      tvoc_received_pkg=0;
    }
    if(satura_r>=G[7]){
      saturation_count++;
    } else {
      saturation_count--;
    }
    if (saturation_count==SATURATION_WINDOW){
      tvoc_fsm=RE_CALIBRATION;
      alert_lvl=4;
      rs_max=rs;
      rs_max_r=rs;
      saturation_count=0;
      tvoc_received_pkg=0;
      rs_max_abs=rs;
    }
    break;    
  
  case WARM_UP:
    if(tvoc_received_pkg > SENSOR_WARMUP_WINDOW){
      alert_lvl=0;
      tvoc_fsm=CLEAR;
      tvoc_received_pkg=0;
      rs_max=rs;
      rs_max_r=rs;
      rs_max_abs=rs;
    }
  break;     
  
  case RE_CALIBRATION:
    if(tvoc_received_pkg > SENSOR_WARMUP_WINDOW){
      alert_lvl=0;
      tvoc_fsm=CLEAR;
      tvoc_received_pkg=0;
      rs_max=rs;
      rs_max_r=rs;
      rs_max_abs=rs;
    }  
  break;
  
  default:
    alert_lvl=0;
    rs_max=0;
    rs_max_r=0;  
    saturation_count=0;
  }
  Serial.print(tvoc_received_pkg);
  Serial.print(":");
  Serial.print(tvoc);
  Serial.print(":");
  Serial.print(rs);
  Serial.print(":");
  Serial.print(rs_max_r);
  Serial.print(":");
  Serial.print(rs_max_abs);
  Serial.print(":");
  Serial.print(determ_r);
  Serial.print(":");
  Serial.print(satura_r);
  Serial.print(":");
  Serial.print(tvoc_fsm);
  Serial.print(":");
  Serial.print(alert_lvl);
  Serial.print(":");
  Serial.print(saturation_count);
  Serial.print("\n");
}


void TvocLedDisplay(){
  int random_seed=0;
  switch (alert_lvl){
  case 0:
  digitalWrite(2, HIGH);  
  digitalWrite(3, LOW);  
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  break;
  
  case 1:
  digitalWrite(2, LOW);  
  digitalWrite(3, HIGH);  
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  break;
  
  case 2:
  digitalWrite(2, LOW);  
  digitalWrite(3, LOW);  
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  break;
  
  case 3:
  digitalWrite(2, LOW);  
  digitalWrite(3, LOW);  
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  break;    
  
  default:
  digitalWrite(2, lowByte(tvoc_received_pkg) & 0x01);  
  digitalWrite(3, lowByte(tvoc_received_pkg) & 0x02);  
  digitalWrite(4, lowByte(tvoc_received_pkg) & 0x04);  
  digitalWrite(5, lowByte(tvoc_received_pkg) & 0x08);  

  }
}
