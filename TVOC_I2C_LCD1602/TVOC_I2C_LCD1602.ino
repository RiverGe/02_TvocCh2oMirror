#include <Wire.h>   
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x3F,16,2);    
//SoftwareSerial Serial(4, 5); // RX, TX

unsigned int tvoc_start_sign = 0xFF;
unsigned int tvoc_id_sign = 0x01;
unsigned int tvoc_temp_sign =0x0; 
unsigned int tvoc_h = 0;
unsigned int tvoc_l = 0;
unsigned int tvoc_state = 0; 
word tvoc_check_sum = 0;
word tvoc_received_pkg = 0;
word tvoc_check_failed = 0;
float tvoc = 0;

boolean tvoc_updisp = true;


void setup()  
{  
  lcd.init();                  
  lcd.backlight();               
  Serial.begin(9600);
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  lcd.setCursor(0,0);                
  lcd.print("TVOC Test");       
  lcd.setCursor(0,1);  
  lcd.print("         by W.J.");  
  delay(2000);
  lcd.clear();

}  

void loop()  
{  
  TvocSensorRead();

  if (tvoc_updisp){
    lcd.home();
    lcd.print ("TVOC: ");
    lcd.setCursor(6, 0);
    lcd.print(tvoc);
    lcd.setCursor(11, 0);
    lcd.print ("mg/m3");
  }

}  
/*
void Ch2oSetRead () {
 ch2oSerial.write(ch2o_read_cmd,5);
 }
 
 void Ch2oSensorRead(){
 unsigned int ch2o_value[6];
 
 Ch2oSetRead();
 
 delay(100);
 while (ch2oSerial.available() > 0) {
 switch (tvoc_state){
 
 case 0:  
 ch2o_temp_sign=ch2oSerial.read();
 if (ch2o_temp_sign == ch2o_start_sign1) {
 ch2o_state++;
 }else{
 ch2o_state=0;
 }
 break;
 
 case 1:  
 ch2o_temp_sign=ch2oSerial.read();
 if (ch2o_temp_sign == ch2o_start_sign2) {
 ch2o_state++;
 }else{
 ch2o_state=0;
 }
 break;
 
 case 2: 
 for(int i=0; i<6; i++){
 if (ch2oSerial.available() > 0){
 ch2o_value[i]=ch2oSerial.read();
 }          
 delay(10);
 }  
 ch2o_state++;
 break;
 
 case 3:
 ch2o_temp_sign=ch2oSerial.read();
 if (ch2o_temp_sign == ch2o_end_sign) {
 ch2o_state++;
 }else{
 ch2o_state=0;
 }      
 break;
 
 case 4:
 ch2o=(ch2o_value[2]*256.0+ch2o_value[3])/100.0;
 ch2o_ppm=(ch2o_value[4]*256.0+ch2o_value[5])/100.0;
 ch2o_updisp=true;
 ch2o_received_pkg++;
 break;  
 
 default:
 ;  
 }
 }
 }
 */
void TvocSensorRead(){
  while (Serial.available() > 0) {
    tvoc_temp_sign=Serial.read();
    switch (tvoc_state){

    case 0:  
      if (tvoc_temp_sign == tvoc_start_sign) {
        tvoc_state++;
        tvoc_check_sum = 0 + tvoc_temp_sign;        
      }
      break;

    case 1:  
      if (tvoc_temp_sign == tvoc_start_sign) {
        tvoc_state++;
        tvoc_check_sum = tvoc_check_sum + tvoc_temp_sign;        
      }
      break;       

    case 2:   
      tvoc_state++;
      tvoc_check_sum = tvoc_check_sum + tvoc_temp_sign;        
      break;

    case 3:   
      tvoc_state++;
      tvoc_check_sum = tvoc_check_sum + tvoc_temp_sign;        
      break;        

    case 4:
      tvoc_h= tvoc_temp_sign;
      tvoc_state++;
      tvoc_check_sum = tvoc_check_sum + tvoc_temp_sign;        
      break;

    case 5:
      tvoc_l= tvoc_temp_sign;
      tvoc_state++;
      tvoc_check_sum = tvoc_check_sum + tvoc_temp_sign;           
      break;


    case 6:
      tvoc_state++;        
      tvoc_check_sum = tvoc_check_sum + tvoc_temp_sign;        
      break;

    case 7:
      tvoc_state++;        
      tvoc_check_sum = tvoc_check_sum + tvoc_temp_sign;        
      break;  

    case 8:
      //        if (lowByte(tvoc_check_sum) != tvoc_temp_sign){
      //          tvoc_state=0;  
      //          tvoc_check_failed++;          
      //        } else {
      tvoc_state++;
      //        }
      break;     

    default:
      tvoc_state = 0;  
    }
    delay (10);
  }

  if (tvoc_state == 9) {
    tvoc_state = 0;
    tvoc = (tvoc_h*256.0 + tvoc_l)/100;
    tvoc_received_pkg++;
    digitalWrite(13, lowByte(tvoc_received_pkg)& 0x01);   

    tvoc_updisp=true;
  }

}

