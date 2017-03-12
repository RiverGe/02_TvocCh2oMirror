// V0.2 Added Tmep and Humid sensor.
// V0.3 Added the Temperature Compensation

#include <Wire.h>   
#include <LiquidCrystal_I2C.h>
#include <dht11.h>

LiquidCrystal_I2C lcd(0x3F,16,2);    
dht11 DHT11;
// Pin Assignment
#define DHT11VCC 8
#define DHT11PIN 9
#define DHT11GND 10
float humidity =0;
float temperature =-20;

float rs_at_table[]={0.89, 0.9, 0.91, 0.91, 0.92, 0.92, 0.93, 0.93, 0.93, 0.94, 0.94, 0.95, 0.95, 0.96, 0.97, 0.97, 0.98, 0.98, 0.99, 0.99, 1, 1.01, 1.02, 1.03, 1.03, 1.04, 1.05, 1.06, 1.07, 1.08, 1.09, 1.1, 1.11, 1.12, 1.13, 1.14, 1.15, 1.16, 1.17, 1.18, 1.19, 1.19, 1.2, 1.21, 1.22, 1.23, 1.24, 1.25, 1.26, 1.27, 1.28};
float rs_rh_table[]={0.61, 0.61, 0.62, 0.62, 0.63, 0.63, 0.63, 0.64, 0.64, 0.65, 0.65, 0.65, 0.66, 0.66, 0.67, 0.67, 0.68, 0.68, 0.68, 0.69, 0.69, 0.70, 0.70, 0.71, 0.71, 0.72, 0.72, 0.73, 0.74, 0.74, 0.75, 0.75, 0.76, 0.76, 0.77, 0.78, 0.78, 0.79, 0.79, 0.80, 0.81, 0.81, 0.82, 0.83, 0.83, 0.84, 0.85, 0.85, 0.86, 0.87, 
0.88, 0.88, 0.89, 0.90, 0.91, 0.92, 0.93, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99, 1.00, 1.01, 1.01, 1.02, 1.03, 1.03, 1.04, 1.05, 1.06, 1.06, 1.07, 1.08, 1.09, 1.09, 1.10, 1.11, 1.12, 1.13, 1.14, 1.15, 1.15, 1.16, 1.17, 1.18, 1.19, 1.20, 1.21, 1.22, 1.23, 1.24, 1.25, 1.26, 1.27, 1.28, 1.29, 1.30};
#define SENSOR_WARMUP_WINDOW 12
#define MAXIMUM_UPDATE_WINDOW 240
#define SATURATION_WINDOW 900

enum tvoc_state {CLEAR, AL_1, AL_2, AL_3, WARM_UP, RE_CALIBRATION};
tvoc_state tvoc_fsm = WARM_UP;

float tvoc = 0;
int Vrl = 0;  // variable to store the value coming from the sensor
int Vk = 0;  // variable to store the value coming from the sensor
float K=0;

int VrlPin = A0;
int VkPin = A1;
float rs=0;
float rs_step=50;
float rs_max_abs=0;
float rs_max_rmdisturb=0;
int tvoc_received_pkg=0;
int alert_lvl = 4;
float G[]={0, 0.08, 0.16, -0.03, 0.05, 0.13, 0.03, 0.95, 0.87};
float determ_r=0;
float satura_r=0;
int saturation_count=0;
float rs_prev =0;

void setup()  
{  
  randomSeed(analogRead(0));
  lcd.init();                  
  lcd.backlight();               
  Serial.begin(9600);
  // Power Supply to DHT11 
  pinMode (DHT11VCC, OUTPUT);
  pinMode (DHT11GND, OUTPUT);
  // initialize digital pins as an output.
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
  digitalWrite (DHT11VCC, HIGH);
  digitalWrite (DHT11GND, LOW);
  lcd.blink();
}  


void loop()  
{  
  TempHumidSensorRead();

  TvocSensorRead();
    
  TvocLevelDetermination();
  
  TvocLedDisplay();
  
  TvocLcdDisplay();
  
}  

void   TvocLcdDisplay(){
  if (tvoc_fsm==WARM_UP){
    lcd.home();
    lcd.print ("TVOC Sensor Warm");     
    lcd.setCursor(0, 1);
    lcd.print ("Up...");     
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
    lcd.print ("Te:");
    lcd.setCursor(3, 1);
    lcd.print((int)temperature);
    lcd.setCursor(6, 1);
    lcd.print ("oC RH:");
    lcd.setCursor(12, 1);
    lcd.print((int)humidity);
    lcd.setCursor(14, 1);
    lcd.print ("% ");

  }
}
void TvocSensorRead(){
  int v_rl[8], v_k[8];
  int v_rl_sum=0, v_k_sum=0;
  float rs_at=1;
  if(temperature <=-20.0){
    rs_at=0.68;
  } else if(temperature <=-15.0){
    rs_at=0.73;
  } else if(temperature <=-10.0){
    rs_at=0.78;
  } else if(temperature <=-5.0){
    rs_at=0.84;
  } else if(temperature < 0 ){
    rs_at=0.89;
  } else if(temperature >= 0 && temperature <= 50){
    rs_at=rs_at_table[(int)temperature];
  } else if(temperature < 55 ){
    rs_at=1.33;
  } else {
    rs_at=1.39;  
  }
    
  if(tvoc_fsm==WARM_UP || tvoc_fsm==RE_CALIBRATION){
    tvoc_received_pkg++;  
    delay(100);    
    Vrl = analogRead(VrlPin);    
    rs=(((1023.0-Vrl)/Vrl)*9760.0)*rs_at*rs_rh_table[(int)humidity];
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
    rs_prev=rs;
    rs=(((1023.0-Vrl)/Vrl)*9760.0)*rs_at*rs_rh_table[(int)humidity];
    K = ((float)Vk)/1023.0;
    if (K<0.06 || K>0.96)
      K=0.84;    
    tvoc_received_pkg++;  
    if(tvoc_received_pkg>=MAXIMUM_UPDATE_WINDOW){
       tvoc_received_pkg=0;
    }
  }
  rs_max_abs=rs_max_abs>rs? rs_max_abs : rs;
//  rs_max_abs=(rs_max_abs<rs) && ((rs-rs_max_abs)<2000)? rs: rs_max_abs;
  rs_step = rs_max_abs/300.0; 
  if(rs_max_abs >= rs)
    tvoc=((rs_max_abs-rs)/rs_step)/100.0;
  digitalWrite(13, lowByte(tvoc_received_pkg)&0x01); 
}

void TvocLevelDetermination(){
  determ_r=rs/rs_max_abs;
  satura_r=rs/rs_prev;
  switch (tvoc_fsm){
    
  case CLEAR:
    if(determ_r<=(K-G[0])){
      alert_lvl=1;
      tvoc_fsm=AL_1;
    }
  break;
  
  case AL_1:
    if(determ_r<=(K-G[1])){
      alert_lvl++;
      tvoc_fsm=AL_2;
      saturation_count=0;
    } else if(determ_r>=(K-G[3])){
      alert_lvl--;
      tvoc_fsm=CLEAR;
      saturation_count=0;
    } 
    if(satura_r>=G[7]){
      saturation_count++;
    } else {
      saturation_count--;
    }
    if (saturation_count==SATURATION_WINDOW){
      tvoc_fsm=RE_CALIBRATION;
      alert_lvl=4;
      saturation_count=0;
      rs_max_abs=rs;
      lcd.clear();
      lcd.blink();
    }
  break;
  
  case AL_2:
    if(determ_r<=(K-G[2])){
      alert_lvl++;
      tvoc_fsm=AL_3;
      saturation_count=0;
    } else if(determ_r>=(K-G[4])){
      alert_lvl--;
      tvoc_fsm=AL_1;
      saturation_count=0;
    }
    if(satura_r>=G[7]){
      saturation_count++;
    } else {
      saturation_count--;
    }
    if (saturation_count==SATURATION_WINDOW){
      tvoc_fsm=RE_CALIBRATION;
      alert_lvl=4;
      saturation_count=0;
      rs_max_abs=rs;
      lcd.clear();
      lcd.blink();
    }
    break;
  
  case AL_3:
    if(determ_r>=(K-G[5])){
      alert_lvl--;
      tvoc_fsm=AL_2;
      saturation_count=0;
    }
    if(satura_r>=G[7]){
      saturation_count++;
    } else {
      saturation_count--;
    }
    if (saturation_count==SATURATION_WINDOW){
      tvoc_fsm=RE_CALIBRATION;
      alert_lvl=4;
      saturation_count=0;
      rs_max_abs=rs;
      lcd.clear();
      lcd.blink();
    }
    break;    
  
  case WARM_UP:
    if(tvoc_received_pkg > SENSOR_WARMUP_WINDOW){
      alert_lvl=0;
      tvoc_fsm=CLEAR;
      tvoc_received_pkg=0;
      rs_max_abs=rs;
      lcd.clear();
      lcd.noBlink();
    }
  break;     
  
  case RE_CALIBRATION:
    if(tvoc_received_pkg > SENSOR_WARMUP_WINDOW){
      alert_lvl=0;
      tvoc_fsm=CLEAR;
      tvoc_received_pkg=0;
      rs_max_abs=rs;
      lcd.clear();
      lcd.noBlink();
    }  
  break;
  
  default:{
    alert_lvl=0;
    tvoc_fsm=CLEAR;
    saturation_count=0;
    tvoc_received_pkg=0;
    rs_max_abs=rs;
  }
  }
  Serial.print(tvoc_received_pkg);
  Serial.print(":");
  Serial.print(tvoc);
  Serial.print(":");
  Serial.print(rs);
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
  Serial.print(":");
  Serial.print((int)temperature);
  Serial.print(":");
  Serial.print((int)humidity);
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
void TempHumidSensorRead(){
  int chk =0;
  chk =DHT11.read(DHT11PIN);
  if (chk == DHTLIB_OK){
    humidity = DHT11.humidity;
    temperature = DHT11.temperature;
  }
}
