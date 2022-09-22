#include <Arduino.h>
#include <IO_Outseal.h>
// #include <defineandinitialitation.h>
#include <stdio.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <FlowMeter.h>
#include <ModbusRtu.h>
#include <EEPROM.h>

#define ID_SLAVE 1
/* X0 -> Sensor flow1        Y0 -> Motor 1
   X1 -> Sensor flow2        Y1 -> Selenoid 1
   X2 -> Button 1            Y2 -> Motor 2
   X3 -> Button 2            Y3 -> Selenoid 2
   X4 -> Selector right 1    Serial2 -> RS485
   X5 -> Selector right 2
   X6 -> Selector left 1
   x7 -> Selector left 2 */
enum{
  index_mode_system,
  manual,
  setting,
  otomatis
};
enum{
  index_show_lcd,
  show_initialitation,
  show_bar,
  note,
  show_value,
  show_mode
};
enum{
  index_class_clear,
  LCD_CHARACTER1,
  LCD_CHARACTER2,
  FLOWMETER1_CLEAR,
  FLOWMETER2_CLEAR
};
enum{
  REG_totalofliter1, //reg 0
  REG1, //reg 1
  REG_totalofcubic1, //reg 2
  REG3, //reg 3
  REG_literpermenit1,//reg 4
  REG5,//reg 5
  REG_setpoint_data1,//reg 6
  REG7,//reg 7
  REG_totalofliter2, //reg 8
  REG9, //reg 9
  REG_totalofcubic2, //reg 10
  REG11, //reg 11
  REG_literpermenit2,//reg 12
  REG13,//reg 13
  REG_setpoint_data2,//reg 14
  REG15,//reg 15
  REG_currentliter1,//reg 16
  REG17,//reg 17
  REG_currentliter2,//reg 18
  REG19,//reg 19
  REG_onMotor1,//reg 20
  REG_onMotor2,//reg 21
  HOLDING_REGS_SIZE
};
uint16_t holdingRegs[HOLDING_REGS_SIZE];
Modbus slave(ID_SLAVE, Serial2, 0);
LiquidCrystal_I2C lcd1(0x27,20,4);
LiquidCrystal_I2C lcd2(0x26,20,4);

byte zero[] = {B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000};
byte one[] = {B10000,B10000,B10000,B10000,B10000,B10000,B10000,B10000};
byte two[] = {B11000,B11000,B11000,B11000,B11000,B11000,B11000,B11000};
byte three[] = {B11100,B11100,B11100,B11100,B11100,B11100,B11100,B11100};
byte four[] = {B11110,B11110,B11110,B11110,B11110,B11110,B11110,B11110};
byte five[] = {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};

/* ================ Properti flow meter ========================
Example : "mySensor = {Kapasitas flow meter, K-Factor, {M-Factor 1 - 10}};"
-> Kapasitas flow meter = kemampuan flow meter mengukur debit air dalam L/menit
-> K-Factor = Karakteristik sinyal flow meter dihitung dengan : (jumlah signal/L)/60
-> M-Factor = Perbandingan jumlah nominal dibagi jumlah terukur disebut juga meter factor */
FlowSensorProperties SEA_YF_DN50_FL1 = {500.0f, 0.2f, {1,1,1,1,1,1,1,1,1,1}};
FlowSensorProperties SEA_YF_DN50_FL2 = {500.0f, 0.2f, {1,1,1,1,1,1,1,1,1,1}};
FlowMeter *FLMeter1;
FlowMeter *FLMeter2;

uint8_t addr_totalliter1 = 0;
uint8_t addr_totalliter2 = 8;
uint8_t addr_setpoint1 = 17;
uint8_t addr_setpoint2 = 22;

uint8_t mode_fl1 = 0, mode_fl2 = 0;
bool pulseDetected1 = false, pulseDetected2 = false;
bool flag_on_relay_fl1 = false, flag_on_relay_fl2 = false;
float liter_fl1 = 0, liter_fl2 = 0;
float lpm_fl1 = 0, lpm_fl2 = 0;
float setpoint_fl1 = 0.0, setpoint_fl2 = 0.0;
float setpoint_prev_fl1 = 0.0, setpoint_prev_fl2 = 0.0;
float totalofcubic1 = 0.0, totalofcubic2 = 0.0;
double totalofliter1 = 0.0, totalofliter2 = 0.0;
unsigned long timeOut1 = 3000UL, lastTime1 = 0, timeOut2 = 3000UL, lastTime2 = 0;
unsigned long flowStartTime1, flowLastTime1, flowStartTime2, flowLastTime2;
unsigned long time_com = 0;
unsigned long time_button_fl1 = 0.0, time_button_fl2 = 0.0;
unsigned long time_interval_rs485 = 0;

IRAM_ATTR void handleInterrupt1();
IRAM_ATTR void handleInterrupt2();
void calculate_flow_meter(uint8_t index_flmeter);
void floatconv(float dataf, uint8_t NumOfRegister);
float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b);
void clear_(uint8_t index_clear);
void updateProgressBar(uint8_t lcd_case, long count, unsigned long totalCount, int lineToPrintOn);
void lcd_show(uint8_t lcd_switch, uint8_t index_lcd);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  clear_(LCD_CHARACTER1);
  clear_(LCD_CHARACTER2);
  slave.start();
  init_IO();
  FLMeter1 = new FlowMeter(digitalPinToInterrupt(X0), SEA_YF_DN50_FL1, handleInterrupt1, RISING);
  FLMeter2 = new FlowMeter(digitalPinToInterrupt(X1), SEA_YF_DN50_FL2, handleInterrupt2, RISING);
  lcd_show(LCD_CHARACTER1, show_initialitation);
  lcd_show(LCD_CHARACTER2, show_initialitation);
  lcd_show(LCD_CHARACTER1, show_bar);
  lcd_show(LCD_CHARACTER1, note);
  lcd_show(LCD_CHARACTER2, show_bar);
  lcd_show(LCD_CHARACTER2, note);
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  totalofliter1 = EEPROM.readDouble(addr_totalliter1);
  setpoint_fl1 = EEPROM.readFloat(addr_setpoint1);
  setpoint_prev_fl1 = EEPROM.readFloat(addr_setpoint1);
  totalofcubic1 = totalofliter1 / 1000;
  totalofliter2 = EEPROM.readDouble(addr_totalliter2);
  setpoint_fl2 = EEPROM.readFloat(addr_setpoint2);
  setpoint_prev_fl2 = EEPROM.readFloat(addr_setpoint2);
  totalofcubic2 = totalofliter2 / 1000;
  liter_fl1 = FLMeter1->getCurrentFlowrate();
  lpm_fl1 = FLMeter1->getTotalVolume();
  liter_fl2 = FLMeter2->getCurrentFlowrate();
  lpm_fl2 = FLMeter2->getTotalVolume();
  lcd_show(LCD_CHARACTER1, show_value);
  lcd_show(LCD_CHARACTER2, show_value);
  floatconv(setpoint_fl1, REG_setpoint_data1);
  floatconv(setpoint_fl2, REG_setpoint_data2);
}

void loop() {
  // put your main code here, to run repeatedly:
  calculate_flow_meter(1);
  calculate_flow_meter(2);
  if(Serial2.available() > 0){
    lcd2.setCursor(17, 0); lcd2.print("   ");
    lcd2.setCursor(17, 0); lcd2.print("ON");
    lcd1.setCursor(17, 0); lcd1.print("   ");
    lcd1.setCursor(17, 0); lcd1.print("ON");
    time_com = millis();
  }else{
    if(millis() - time_com >= 1000){
      lcd1.setCursor(17, 0); lcd1.print("OFF");
      lcd2.setCursor(17, 0); lcd2.print("OFF");
    }
  }
  if (digitalRead(X4) == OFF && digitalRead(X6) == OFF && mode_fl1 != manual){
    mode_fl1 = manual;
    lcd1.setCursor(5, 0); lcd1.print("      ");
    lcd1.setCursor(5, 0); lcd1.print("Manual");
  }
  if(digitalRead(X4) == ON && digitalRead(X6) == OFF && mode_fl1 != otomatis){
    mode_fl1 = otomatis;
    lcd1.setCursor(5, 0); lcd1.print("      ");
    lcd1.setCursor(5, 0); lcd1.print("Auto");
  }
  if(digitalRead(X4) == OFF && digitalRead(X6) == ON && mode_fl1 != setting){
    mode_fl1 = setting;
    lcd1.setCursor(5, 0); lcd1.print("      ");
    lcd1.setCursor(5, 0); lcd1.print("Set FL");
  }
  if (digitalRead(X5) == OFF && digitalRead(X7) == OFF && mode_fl2 != manual){
    mode_fl2 = manual;
    lcd2.setCursor(5, 0); lcd2.print("      ");
    lcd2.setCursor(5, 0); lcd2.print("Manual");
  }
  if(digitalRead(X5) == ON && digitalRead(X7) == OFF && mode_fl2 != otomatis){
    mode_fl2 = otomatis;
    lcd2.setCursor(5, 0); lcd2.print("      ");
    lcd2.setCursor(5, 0); lcd2.print("Auto");
  }
  if(digitalRead(X5) == OFF && digitalRead(X7) == ON && mode_fl2 != setting){
    mode_fl2 = setting;
    lcd2.setCursor(5, 0); lcd2.print("      ");
    lcd2.setCursor(5, 0); lcd2.print("Set FL");
  }
  if(setpoint_fl1 != setpoint_prev_fl1){
    setpoint_prev_fl1 = setpoint_fl1;
    floatconv(setpoint_fl1, REG_setpoint_data1);
    EEPROM.writeFloat(addr_setpoint1, setpoint_fl1);
    EEPROM.commit();
    setpoint_prev_fl1 = EEPROM.readFloat(addr_setpoint1);
  }
  if(setpoint_fl2 != setpoint_prev_fl2){
    setpoint_prev_fl2 = setpoint_fl2;
    floatconv(setpoint_fl2, REG_setpoint_data2);
    EEPROM.writeFloat(addr_setpoint2, setpoint_fl2);
    EEPROM.commit();
    setpoint_prev_fl2 = EEPROM.readFloat(addr_setpoint2);
  }
  switch (mode_fl1)
  {
  case otomatis:
    //setpoint_fl1 = modbus_16bit_register_pair_to_float(holdingRegs[REG_setpoint_data1], holdingRegs[REG_setpoint_data1 + 1]);
    if(digitalRead(X2) == ON && millis() - time_button_fl1 >= 1000 && flag_on_relay_fl1 == false){
      time_button_fl1 = millis();
      digitalWrite(Y0, ON);
      digitalWrite(Y1, ON);
      flag_on_relay_fl1 = true;
      FLMeter1->reset();
      liter_fl1 = 0.0;
      clear_(LCD_CHARACTER1);
      lcd_show(LCD_CHARACTER1, note);
      lcd_show(LCD_CHARACTER1, show_value);
      lcd_show(LCD_CHARACTER1, show_mode);
      clear_(LCD_CHARACTER2);
      lcd_show(LCD_CHARACTER2, note);
      lcd_show(LCD_CHARACTER2, show_value);
      lcd_show(LCD_CHARACTER2, show_mode);
    }
    if(liter_fl1 >= setpoint_fl1){
      digitalWrite(Y0, OFF);
      digitalWrite(Y1, OFF);
      flag_on_relay_fl1 = false;
    }
    break;
  case manual:
    if(digitalRead(X2) == ON && millis() - time_button_fl1 >= 1000){
      time_button_fl1 = millis();
      if(flag_on_relay_fl1 == false){
        digitalWrite(Y0, ON);
        digitalWrite(Y1, ON);
        flag_on_relay_fl1 = true;
        FLMeter1->reset();
        liter_fl1 = 0.0;
        clear_(LCD_CHARACTER1);
        lcd_show(LCD_CHARACTER1, note);
        lcd_show(LCD_CHARACTER1, show_value);
        lcd_show(LCD_CHARACTER1, show_mode);
        clear_(LCD_CHARACTER2);
        lcd_show(LCD_CHARACTER2, note);
        lcd_show(LCD_CHARACTER2, show_value);
        lcd_show(LCD_CHARACTER2, show_mode);
      }else if(flag_on_relay_fl1 == true){
        digitalWrite(Y0, OFF);
        digitalWrite(Y1, OFF);
        flag_on_relay_fl1 = false;
      }
    }
    break;
  case setting:
    if(digitalRead(X2) == ON){
      setpoint_fl1 += 500.0;
      if(setpoint_fl1 > 5000){
        setpoint_fl1 = 500.0;
      }
      clear_(LCD_CHARACTER1);
      lcd_show(LCD_CHARACTER1, note);
      lcd_show(LCD_CHARACTER1, show_value);
      lcd_show(LCD_CHARACTER1, show_mode);
      clear_(LCD_CHARACTER2);
      lcd_show(LCD_CHARACTER2, note);
      lcd_show(LCD_CHARACTER2, show_value);
      lcd_show(LCD_CHARACTER2, show_mode);
    }
    break;
  }
  switch (mode_fl2)
  {
  case otomatis:
    //setpoint_fl2 = modbus_16bit_register_pair_to_float(holdingRegs[REG_setpoint_data2], holdingRegs[REG_setpoint_data2 + 1]);
    if(digitalRead(X3) == ON && millis() - time_button_fl2 >= 1000 && flag_on_relay_fl2 == false){
      time_button_fl2 = millis();
      digitalWrite(Y2, ON);
      digitalWrite(Y3, ON);
      flag_on_relay_fl2 = true;
      FLMeter2->reset();
      liter_fl2 = 0.0;
      clear_(LCD_CHARACTER1);
      lcd_show(LCD_CHARACTER1, note);
      lcd_show(LCD_CHARACTER1, show_value);
      lcd_show(LCD_CHARACTER1, show_mode);
      clear_(LCD_CHARACTER2);
      lcd_show(LCD_CHARACTER2, note);
      lcd_show(LCD_CHARACTER2, show_value);
      lcd_show(LCD_CHARACTER2, show_mode);
    }
    if(liter_fl2 >= setpoint_fl2){
      digitalWrite(Y2, OFF);
      digitalWrite(Y3, OFF);
      flag_on_relay_fl2 = false;
    }
    break;
  case manual:
    if(digitalRead(X3) == ON && millis() - time_button_fl2 >= 1000){
      time_button_fl2 = millis();
      if(flag_on_relay_fl2 == false){
        digitalWrite(Y2, ON);
        digitalWrite(Y3, ON);
        flag_on_relay_fl2 = true;
        FLMeter2->reset();
        liter_fl2 = 0.0;
        clear_(LCD_CHARACTER1);
        lcd_show(LCD_CHARACTER1, note);
        lcd_show(LCD_CHARACTER1, show_value);
        lcd_show(LCD_CHARACTER1, show_mode);
        clear_(LCD_CHARACTER2);
        lcd_show(LCD_CHARACTER2, note);
        lcd_show(LCD_CHARACTER2, show_value);
        lcd_show(LCD_CHARACTER2, show_mode);
      }else if(flag_on_relay_fl2 == true){
        digitalWrite(Y2, OFF);
        digitalWrite(Y3, OFF);
        flag_on_relay_fl2 = false;
      }
    }
    break;
  case setting:
    if(digitalRead(X3) == ON){
      setpoint_fl2 += 500.0;
      if(setpoint_fl2 > 5000){
        setpoint_fl2 = 500.0;
      }
      clear_(LCD_CHARACTER1);
      lcd_show(LCD_CHARACTER1, note);
      lcd_show(LCD_CHARACTER1, show_value);
      lcd_show(LCD_CHARACTER1, show_mode);
      clear_(LCD_CHARACTER2);
      lcd_show(LCD_CHARACTER2, note);
      lcd_show(LCD_CHARACTER2, show_value);
      lcd_show(LCD_CHARACTER2, show_mode);
    }
    break;
  }
  if(millis() - time_interval_rs485 >= 1000){
    time_interval_rs485 = millis();
    holdingRegs[REG_onMotor1] = flag_on_relay_fl1;
    holdingRegs[REG_onMotor2] = flag_on_relay_fl2;
    slave.poll(holdingRegs, HOLDING_REGS_SIZE);
  }
  yield();
}
void IRAM_ATTR handleInterrupt1() {
  FLMeter1->count();
  if (!pulseDetected1) {
    pulseDetected1 = true; // Flag untuk menandakan flow meter sedang menghitung flow
    flowStartTime1 = millis(); // Untuk mengetahui kapan pertama kali mulai menghitung flow
  }
  flowLastTime1 = millis();
}
void IRAM_ATTR handleInterrupt2() {
  FLMeter2->count();
  if (!pulseDetected2) {
    pulseDetected2 = true; // Flag untuk menandakan flow meter sedang menghitung flow
    flowStartTime2 = millis(); // Untuk mengetahui kapan pertama kali mulai menghitung flow
  }
  flowLastTime2 = millis();
}
void calculate_flow_meter(uint8_t index_flmeter){
  switch (index_flmeter)
  {
  case 1:
      if (pulseDetected1) {
        unsigned long currentTime1 = millis();
        unsigned long duration1 = currentTime1 - lastTime1; 
        if (currentTime1 - flowLastTime1 >= timeOut1) { // Dieksekusi jika selama 3 detik sudah tidak ada pulsa baru yang masuk lagi 
          pulseDetected1 = false;
          EEPROM.writeDouble(addr_totalliter1, totalofliter1);
          EEPROM.commit();
        }
        else if (duration1 >= 1000) { // Dieksekusi setiap 1 detik        
          // Fungsi untuk menghitung flow rate dan volume dalam 1 period
          FLMeter1->tick(duration1);
          Serial.print("Current flow rate: " + String(FLMeter1->getCurrentFlowrate()) + " l/min; ");
          Serial.println("Overall volume: " + String(FLMeter1->getTotalVolume()) + " l");             
          lastTime1 = currentTime1;
          totalofliter1 += FLMeter1->getCurrentVolume();
          totalofcubic1 = totalofliter1 / 1000;
          lpm_fl1 = FLMeter1->getCurrentFlowrate();
          liter_fl1 = FLMeter1->getTotalVolume();
          EEPROM.writeDouble(addr_totalliter1, totalofliter1);
          EEPROM.commit();
          floatconv(totalofliter1, REG_totalofliter1);
          floatconv(totalofcubic1, REG_totalofcubic1);
          floatconv(lpm_fl1, REG_literpermenit1);
          floatconv(liter_fl1, REG_currentliter1);
          lcd_show(LCD_CHARACTER1, show_value);
        }
    }
    break;
  case 2:
    if (pulseDetected2) {
        unsigned long currentTime2 = millis();
        unsigned long duration2 = currentTime2 - lastTime2; 
        if (currentTime2 - flowLastTime2 >= timeOut2) { // Dieksekusi jika selama 3 detik sudah tidak ada pulsa baru yang masuk lagi 
          pulseDetected2 = false;
          EEPROM.writeDouble(addr_totalliter2, totalofliter2);
          EEPROM.commit();
        }
        else if (duration2 >= 1000) { // Dieksekusi setiap 1 detik        
          // Fungsi untuk menghitung flow rate dan volume dalam 1 period
          FLMeter2->tick(duration2);
          Serial.print("Current flow rate: " + String(FLMeter2->getCurrentFlowrate()) + " l/min; ");
          Serial.println("Overall volume: " + String(FLMeter2->getTotalVolume()) + " l");             
          lastTime2 = currentTime2;
          totalofliter2 += FLMeter2->getCurrentVolume();
          totalofcubic2 = totalofliter2 / 1000;
          lpm_fl2 = FLMeter2->getCurrentFlowrate();
          liter_fl2 = FLMeter2->getTotalVolume();
          EEPROM.writeDouble(addr_totalliter2, totalofliter2);
          EEPROM.commit();
          floatconv(totalofliter2, REG_totalofliter2);
          floatconv(totalofcubic2, REG_totalofcubic2);
          floatconv(lpm_fl2, REG_literpermenit2);
          floatconv(liter_fl2, REG_currentliter2);
          lcd_show(LCD_CHARACTER2, show_value);
        }
    }
    break;
  }
}
void floatconv(float dataf, uint8_t NumOfRegister){
  union fsend_t{
  float f;
  uint16_t u[2];
  };
  union fsend_t floatConvert;
  floatConvert.f = dataf;
  holdingRegs[NumOfRegister] = floatConvert.u[1];
  holdingRegs[NumOfRegister + 1] = floatConvert.u[0];
}
float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b) {
    uint32_t combined = ((uint32_t)a << 16) | b;
    float f;
    memcpy(&f, &combined, sizeof f);
    return f;
}
void clear_(uint8_t index_clear){
  switch (index_clear)
  {
  case 1:
    lcd1.init();
    lcd1.backlight();
    lcd1.clear();
    break;
  case 2:
    lcd2.init();
    lcd2.backlight();
    lcd2.clear();
    break;
  case 3:
    //flow meter clear1
    break;
  case 4:
    //flow meter clear2
    break;
  }
}
void updateProgressBar(uint8_t lcd_case, long count, unsigned long totalCount, int lineToPrintOn){
double factor = totalCount/80.0;
int percent = (count+1)/factor;
int number = percent/5;
int remainder = percent%5;
if(lcd_case == 1){
  if(number > 0){
    lcd1.setCursor(number-1,lineToPrintOn);
    lcd1.write(5);
  }
  lcd1.setCursor(number,lineToPrintOn);
  lcd1.write(remainder);  
}
if(lcd_case == 2){
  if(number > 0){
    lcd2.setCursor(number-1,lineToPrintOn);
    lcd2.write(5);
  }
  lcd2.setCursor(number,lineToPrintOn);
  lcd2.write(remainder); 
}
}
void lcd_show(uint8_t lcd_switch, uint8_t index_lcd){
  if(lcd_switch == 1){
    switch (index_lcd)
    {
    case 1: //initialitation
      lcd1.createChar(0, zero);
      lcd1.createChar(1, one);
      lcd1.createChar(2, two);
      lcd1.createChar(3, three);
      lcd1.createChar(4, four);
      lcd1.createChar(5, five);
      lcd1.setCursor(0, 0); lcd1.print(" FLOW METER DIGITAL");
      lcd1.setCursor(0, 1); lcd1.print("    FLOW METER 1    ");
      lcd1.setCursor(0, 2); lcd1.print("WAIT INITIALITATION");
      break;
    case 2: //Prograss bar
      for(int i=0; i <= 50; i++){
        updateProgressBar(LCD_CHARACTER1, i, 100, 3);   //This line calls the subroutine that displays the progress bar.  The 3 arguments are the current count, the total count and the line you want to print on.
        delay(10);
      }
      lcd1.clear();
      break;
    case 3: // Awal note keterangan
      lcd1.setCursor(0, 0); lcd1.print("Mode:");
      lcd1.setCursor(13, 0); lcd1.print("com:");
      lcd1.setCursor(0, 1); lcd1.print("SetL:");
      lcd1.setCursor(12, 1); lcd1.print("Lpm:");
      lcd1.setCursor(0, 2); lcd1.print("Liter:");
      lcd1.setCursor(19, 2); lcd1.print("L");
      lcd1.setCursor(0, 3); lcd1.print("Total:");
      lcd1.setCursor(19, 3); lcd1.print("L");
      break;
    case 4: //value
      lcd1.setCursor(5, 1); lcd1.print("     ");
      lcd1.setCursor(5, 1); lcd1.print(setpoint_fl1, 1);
      lcd1.setCursor(16, 1); lcd1.print("    ");
      lcd1.setCursor(16, 1); lcd1.print(lpm_fl1, 1);
      lcd1.setCursor(6, 2); lcd1.print("             ");
      lcd1.setCursor(6, 2); lcd1.print(liter_fl1, 2);
      lcd1.setCursor(6, 3); lcd1.print("             ");
      lcd1.setCursor(6, 3); lcd1.print(totalofliter1, 2);
      break;
    case 5: //mode
      switch (mode_fl1)
      {
      case otomatis:
        lcd1.setCursor(5, 0); lcd1.print("      ");
        lcd1.setCursor(5, 0); lcd1.print("Auto");
        break;
      case manual:
        lcd1.setCursor(5, 0); lcd1.print("      ");
        lcd1.setCursor(5, 0); lcd1.print("Manual");
        break;
      case setting:
        lcd1.setCursor(5, 0); lcd1.print("      ");
        lcd1.setCursor(5, 0); lcd1.print("Set FL");
        break;
      }
      break;
    }
  }
  if(lcd_switch == 2){
    switch (index_lcd)
    {
    case 1: //initialitation
      lcd2.createChar(0, zero);
      lcd2.createChar(1, one);
      lcd2.createChar(2, two);
      lcd2.createChar(3, three);
      lcd2.createChar(4, four);
      lcd2.createChar(5, five);
      lcd2.setCursor(0, 0); lcd2.print(" FLOW METER DIGITAL");
      lcd2.setCursor(0, 1); lcd2.print("    FLOW METER 2    ");
      lcd2.setCursor(0, 2); lcd2.print("WAIT INITIALITATION");
      break;
    case 2:
      for(int i=0; i <= 50; i++){
        updateProgressBar(LCD_CHARACTER2, i, 100, 3);   //This line calls the subroutine that displays the progress bar.  The 3 arguments are the current count, the total count and the line you want to print on.
        delay(10);
      }
      lcd2.clear();
      break;
    case 3:
      lcd2.setCursor(0, 0); lcd2.print("Mode:");
      lcd2.setCursor(13, 0); lcd2.print("com:");
      lcd2.setCursor(0, 1); lcd2.print("SetL:");
      lcd2.setCursor(12, 1); lcd2.print("Lpm:");
      lcd2.setCursor(0, 2); lcd2.print("Liter:");
      lcd2.setCursor(19, 2); lcd2.print("L");
      lcd2.setCursor(0, 3); lcd2.print("Total:");
      lcd2.setCursor(19, 3); lcd2.print("L");
      break;
    case 4:
      lcd2.setCursor(5, 1); lcd2.print("     ");
      lcd2.setCursor(5, 1); lcd2.print(setpoint_fl2, 1);
      lcd2.setCursor(16, 1); lcd2.print("    ");
      lcd2.setCursor(16, 1); lcd2.print(lpm_fl2, 1);
      lcd2.setCursor(6, 2); lcd2.print("             ");
      lcd2.setCursor(6, 2); lcd2.print(liter_fl2, 2);
      lcd2.setCursor(6, 3); lcd2.print("             ");
      lcd2.setCursor(6, 3); lcd2.print(totalofliter2, 2);
      break;
    case 5:
      switch (mode_fl2)
      {
      case otomatis:
        lcd2.setCursor(5, 0); lcd2.print("      ");
        lcd2.setCursor(5, 0); lcd2.print("Auto");
        break;
      case manual:
        lcd2.setCursor(5, 0); lcd2.print("      ");
        lcd2.setCursor(5, 0); lcd2.print("Manual");
        break;
      case setting:
        lcd2.setCursor(5, 0); lcd2.print("      ");
        lcd2.setCursor(5, 0); lcd2.print("Set FL");
        break;
      }
      break;
    }
  }
}

#include <Arduino.h>
#include <IO_Outseal.h>
#include <stdio.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <FlowMeter.h>
#include <ModbusRtu.h>
#include <EEPROM.h>

#define ID_SLAVE 1
enum{
  index_mode_system,
  manual,
  setting,
  otomatis,
  flow_meter1,
  flow_meter2,
};
enum{
  show_init,
  show_value,
  show_mode
};
enum{
  REG_totalofliter1, //reg 0
  REG1, //reg 1
  REG_totalofcubic1, //reg 2
  REG3, //reg 3
  REG_literpermenit1,//reg 4
  REG5,//reg 5
  REG_setpoint_data1,//reg 6
  REG7,//reg 7
  REG_totalofliter2, //reg 8
  REG9, //reg 9
  REG_totalofcubic2, //reg 10
  REG11, //reg 11
  REG_literpermenit2,//reg 12
  REG13,//reg 13
  REG_setpoint_data2,//reg 14
  REG15,//reg 15
  REG_currentliter1,//reg 16
  REG17,//reg 17
  REG_currentliter2,//reg 18
  REG19,//reg 19
  REG_onMotor1,//reg 20
  REG_onMotor2,//reg 21
  HOLDING_REGS_SIZE
};
uint16_t holdingRegs[HOLDING_REGS_SIZE];
Modbus slave(ID_SLAVE, Serial2, 0);
LiquidCrystal_I2C lcd1(0x27,20,4);
LiquidCrystal_I2C lcd2(0x26,20,4);
/* ================ Properti flow meter ========================
Example : "mySensor = {Kapasitas flow meter, K-Factor, {M-Factor 1 - 10}};"
-> Kapasitas flow meter = kemampuan flow meter mengukur debit air dalam L/menit
-> K-Factor = Karakteristik sinyal flow meter dihitung dengan : (jumlah signal/L)/60
-> M-Factor = Perbandingan jumlah nominal dibagi jumlah terukur disebut juga meter factor */
FlowSensorProperties SEA_YF_DN50_FL1 = {500.0f, 0.2f, {1,1,1,1,1,1,1,1,1,1}};
FlowSensorProperties SEA_YF_DN50_FL2 = {500.0f, 0.2f, {1,1,1,1,1,1,1,1,1,1}};
FlowMeter *FLMeter1;
FlowMeter *FLMeter2;

uint8_t addr_totalliter1 = 0;
uint8_t addr_totalliter2 = 8;
uint8_t addr_setpoint1 = 17;
uint8_t addr_setpoint2 = 22;

uint8_t mode_fl1 = 0, mode_fl2 = 0;
bool pulseDetected1 = false, pulseDetected2 = false;
bool flag_on_relay_fl1 = false, flag_on_relay_fl2 = false;
float setpoint_fl1 = 0.0, setpoint_fl2 = 0.0;
float setpoint_prev_fl1 = 0.0, setpoint_prev_fl2 = 0.0;
float totalofcubic1 = 0.0, totalofcubic2 = 0.0;
double totalofliter1 = 0.0, totalofliter2 = 0.0;
unsigned long timeOut1 = 3000UL, lastTime1 = 0, timeOut2 = 3000UL, lastTime2 = 0;
unsigned long flowStartTime1, flowLastTime1, flowStartTime2, flowLastTime2;
unsigned long time_com = 0;
unsigned long time_button_fl1 = 0.0, time_button_fl2 = 0.0;
 
IRAM_ATTR void handleInterrupt1();
IRAM_ATTR void handleInterrupt2();
void calculate_flow_meter(uint8_t index_flmeter);
void readcom();
void floatconv(float dataf, uint8_t NumOfRegister);
float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b);
void lcd_show(uint8_t lcd_switch, uint8_t index_lcd);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  slave.start();
  init_IO();
  FLMeter1 = new FlowMeter(digitalPinToInterrupt(X0), SEA_YF_DN50_FL1, handleInterrupt1, RISING);
  FLMeter2 = new FlowMeter(digitalPinToInterrupt(X1), SEA_YF_DN50_FL2, handleInterrupt2, RISING);
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  totalofliter1 = EEPROM.readDouble(addr_totalliter1);
  setpoint_fl1 = EEPROM.readFloat(addr_setpoint1);
  floatconv(setpoint_fl1, REG_setpoint_data1);
  setpoint_prev_fl1 = setpoint_fl1;
  totalofcubic1 = totalofliter1 / 1000;
  totalofliter2 = EEPROM.readDouble(addr_totalliter2);
  setpoint_fl2 = EEPROM.readFloat(addr_setpoint2);
  floatconv(setpoint_fl2, REG_setpoint_data2);
  setpoint_prev_fl2 = setpoint_fl2;
  totalofcubic2 = totalofliter2 / 1000;
  lcd_show(flow_meter1, show_init);
  lcd_show(flow_meter1, show_value);
  lcd_show(flow_meter2, show_init);
  lcd_show(flow_meter2, show_value);
  Serial.println("initialitation done");
}

void loop() {
  // put your main code here, to run repeatedly:
  calculate_flow_meter(flow_meter1);
  calculate_flow_meter(flow_meter2);
  if (digitalRead(X4) == OFF && digitalRead(X6) == OFF && mode_fl1 != manual){mode_fl1 = manual; lcd_show(flow_meter1, show_mode);}
  if(digitalRead(X4) == ON && digitalRead(X6) == OFF && mode_fl1 != otomatis){mode_fl1 = otomatis; lcd_show(flow_meter1, show_mode);}
  if(digitalRead(X4) == OFF && digitalRead(X6) == ON && mode_fl1 != setting){mode_fl1 = setting; lcd_show(flow_meter1, show_mode);}
  if (digitalRead(X5) == OFF && digitalRead(X7) == OFF && mode_fl2 != manual){mode_fl2 = manual; lcd_show(flow_meter2, show_mode);}
  if(digitalRead(X5) == ON && digitalRead(X7) == OFF && mode_fl2 != otomatis){mode_fl2 = otomatis; lcd_show(flow_meter2, show_mode);}
  if(digitalRead(X5) == OFF && digitalRead(X7) == ON && mode_fl2 != setting){mode_fl2 = setting; lcd_show(flow_meter2, show_mode);}
  switch (mode_fl1)
  {
  case otomatis:
    //setpoint_fl1 = modbus_16bit_register_pair_to_float(holdingRegs[REG_setpoint_data1], holdingRegs[REG_setpoint_data1 + 1]);
    if(digitalRead(X2) == ON && millis() - time_button_fl1 >= 1000 && flag_on_relay_fl1 == false){
      time_button_fl1 = millis();
      digitalWrite(Y0, ON);
      digitalWrite(Y1, ON);
      FLMeter1->setTotalVolume(0.0);
      flag_on_relay_fl1 = true;
      lcd_show(flow_meter1, show_init);
      lcd_show(flow_meter1, show_value);
      lcd_show(flow_meter1, show_mode);
      lcd_show(flow_meter2, show_init);
      lcd_show(flow_meter2, show_value);
      lcd_show(flow_meter2, show_mode);
    }
    if(FLMeter1->getTotalVolume() >= setpoint_fl1){
      digitalWrite(Y0, OFF);
      lcd_show(flow_meter1, show_init);
      lcd_show(flow_meter1, show_value);
      lcd_show(flow_meter1, show_mode);
      lcd_show(flow_meter2, show_init);
      lcd_show(flow_meter2, show_value);
      lcd_show(flow_meter2, show_mode);
      digitalWrite(Y1, OFF);
      flag_on_relay_fl1 = false;
    }
    break;
  case manual:
    if(digitalRead(X2) == ON && millis() - time_button_fl1 >= 1000){
      time_button_fl1 = millis();
      if(flag_on_relay_fl1 == false){
        digitalWrite(Y0, ON);
        digitalWrite(Y1, ON);
        FLMeter1->setTotalVolume(0.0);
        flag_on_relay_fl1 = true;
        lcd_show(flow_meter1, show_init);
        lcd_show(flow_meter1, show_value);
        lcd_show(flow_meter1, show_mode);
        lcd_show(flow_meter2, show_init);
        lcd_show(flow_meter2, show_value);
        lcd_show(flow_meter2, show_mode);
      }else if(flag_on_relay_fl1 == true){
        digitalWrite(Y0, OFF);
        lcd_show(flow_meter1, show_init);
        lcd_show(flow_meter1, show_value);
        lcd_show(flow_meter1, show_mode);
        lcd_show(flow_meter2, show_init);
        lcd_show(flow_meter2, show_value);
        lcd_show(flow_meter2, show_mode);
        digitalWrite(Y1, OFF);
        flag_on_relay_fl1 = false;
      }
    }
    break;
  case setting:
    if(digitalRead(X2) == ON){
      setpoint_fl1 += 500.0;
      if(setpoint_fl1 > 5000){setpoint_fl1 = 500.0;}
      EEPROM.writeFloat(addr_setpoint1, setpoint_fl1);
      EEPROM.commit();
      lcd_show(flow_meter1, show_value);
      delay(250);
    }
    break;
  }
  switch (mode_fl2)
  {
  case otomatis:
    //setpoint_fl2 = modbus_16bit_register_pair_to_float(holdingRegs[REG_setpoint_data2], holdingRegs[REG_setpoint_data2 + 1]);
    if(digitalRead(X3) == ON && millis() - time_button_fl2 >= 1000 && flag_on_relay_fl2 == false){
      time_button_fl2 = millis();
      digitalWrite(Y2, ON);
      digitalWrite(Y3, ON);
      FLMeter2->setTotalVolume(0.0);
      flag_on_relay_fl2 = true;
      lcd_show(flow_meter2, show_init);
      lcd_show(flow_meter2, show_value);
      lcd_show(flow_meter2, show_mode);
      lcd_show(flow_meter1, show_init);
      lcd_show(flow_meter1, show_value);
      lcd_show(flow_meter1, show_mode);
    }
    if(FLMeter2->getTotalVolume() >= setpoint_fl2){
      digitalWrite(Y2, OFF);
      lcd_show(flow_meter2, show_init);
      lcd_show(flow_meter2, show_value);
      lcd_show(flow_meter2, show_mode);
      lcd_show(flow_meter1, show_init);
      lcd_show(flow_meter1, show_value);
      lcd_show(flow_meter1, show_mode);
      digitalWrite(Y3, OFF);
      flag_on_relay_fl2 = false;
    }
    break;
  case manual:
    if(digitalRead(X3) == ON && millis() - time_button_fl2 >= 1000){
      time_button_fl2 = millis();
      if(flag_on_relay_fl2 == false){
        digitalWrite(Y2, ON);
        digitalWrite(Y3, ON);
        FLMeter2->setTotalVolume(0.0);
        flag_on_relay_fl2 = true;
        lcd_show(flow_meter2, show_init);
        lcd_show(flow_meter2, show_value);
        lcd_show(flow_meter2, show_mode);
        lcd_show(flow_meter1, show_init);
        lcd_show(flow_meter1, show_value);
        lcd_show(flow_meter1, show_mode);
      }else if(flag_on_relay_fl2 == true){
        digitalWrite(Y2, OFF);
        lcd_show(flow_meter2, show_init);
        lcd_show(flow_meter2, show_value);
        lcd_show(flow_meter2, show_mode);
        lcd_show(flow_meter1, show_init);
        lcd_show(flow_meter1, show_value);
        lcd_show(flow_meter1, show_mode);
        digitalWrite(Y3, OFF);
        flag_on_relay_fl2 = false;
      }
    }
    break;
  case setting:
    if(digitalRead(X3) == ON){
      setpoint_fl2 += 500.0;
      if(setpoint_fl2 > 5000){setpoint_fl2 = 500.0;}
      EEPROM.writeFloat(addr_setpoint2, setpoint_fl2);
      EEPROM.commit();
      lcd_show(flow_meter2, show_value);
      delay(250);
    }
    break;
  }
  readcom();
  //yield();
}
void handleInterrupt1() {
  FLMeter1->count();
  if (!pulseDetected1) {
    pulseDetected1 = true; // Flag untuk menandakan flow meter sedang menghitung flow
    flowStartTime1 = millis(); // Untuk mengetahui kapan pertama kali mulai menghitung flow
  }
  flowLastTime1 = millis();
}
void handleInterrupt2() {
  FLMeter2->count();
  if (!pulseDetected2) {
    pulseDetected2 = true; // Flag untuk menandakan flow meter sedang menghitung flow
    flowStartTime2 = millis(); // Untuk mengetahui kapan pertama kali mulai menghitung flow
  }
  flowLastTime2 = millis();
}
void calculate_flow_meter(uint8_t index_flmeter){
  switch (index_flmeter)
  {
  case flow_meter1:
      if (pulseDetected1) {
        unsigned long currentTime1 = millis();
        unsigned long duration1 = currentTime1 - lastTime1; 
        if (currentTime1 - flowLastTime1 >= timeOut1) { // Dieksekusi jika selama 3 detik sudah tidak ada pulsa baru yang masuk lagi 
          pulseDetected1 = false;
          EEPROM.writeDouble(addr_totalliter1, totalofliter1);
          EEPROM.commit();
          Serial.println();
        }
        else if (duration1 >= 1000) { // Dieksekusi setiap 1 detik        
          // Fungsi untuk menghitung flow rate dan volume dalam 1 period
          FLMeter1->tick(duration1);
          lastTime1 = currentTime1;
          totalofliter1 += FLMeter1->getCurrentVolume();
          totalofcubic1 = totalofliter1 / 1000;
          Serial.print("1->LPM: " + String(FLMeter1->getCurrentFlowrate()) + " l/min | ");
          Serial.print("Liter: " + String(FLMeter1->getTotalVolume()) + " l | ");
          Serial.println("Total of Liter: " + String(totalofliter1) + " l");
          floatconv(totalofliter1, REG_totalofliter1);
          floatconv(totalofcubic1, REG_totalofcubic1);
          floatconv(FLMeter1->getCurrentFlowrate(), REG_literpermenit1);
          floatconv(FLMeter1->getTotalVolume(), REG_currentliter1);
          lcd_show(flow_meter1, show_value);
        }
    }
    break;
  case flow_meter2:
    if (pulseDetected2) {
        unsigned long currentTime2 = millis();
        unsigned long duration2 = currentTime2 - lastTime2; 
        if (currentTime2 - flowLastTime2 >= timeOut2) { // Dieksekusi jika selama 3 detik sudah tidak ada pulsa baru yang masuk lagi 
          pulseDetected2 = false;
          EEPROM.writeDouble(addr_totalliter2, totalofliter2);
          EEPROM.commit();
          Serial.println();
        }
        else if (duration2 >= 1000) { // Dieksekusi setiap 1 detik        
          // Fungsi untuk menghitung flow rate dan volume dalam 1 period
          FLMeter2->tick(duration2);
          lastTime2 = currentTime2;
          totalofliter2 += FLMeter2->getCurrentVolume();
          totalofcubic2 = totalofliter2 / 1000;
          Serial.print("2->LPM: " + String(FLMeter2->getCurrentFlowrate()) + " l/min | ");
          Serial.print("Liter: " + String(FLMeter2->getTotalVolume()) + " l | ");
          Serial.println("Total of Liter: " + String(totalofliter2) + " l");
          floatconv(totalofliter2, REG_totalofliter2);
          floatconv(totalofcubic2, REG_totalofcubic2);
          floatconv(FLMeter2->getCurrentFlowrate(), REG_literpermenit2);
          floatconv(FLMeter2->getTotalVolume(), REG_currentliter2);
          lcd_show(flow_meter2, show_value);
        }
    }
    break;
  }
}
void readcom(){
  // if(setpoint_fl1 != setpoint_prev_fl1){
  //   setpoint_prev_fl1 = setpoint_fl1;
  //   floatconv(setpoint_fl1, REG_setpoint_data1);
  //   EEPROM.writeFloat(addr_setpoint1, setpoint_fl1);
  //   EEPROM.commit();
  //   setpoint_prev_fl1 = EEPROM.readFloat(addr_setpoint1);
  // }
  // if(setpoint_fl2 != setpoint_prev_fl2){
  //   setpoint_prev_fl2 = setpoint_fl2;
  //   floatconv(setpoint_fl2, REG_setpoint_data2);
  //   EEPROM.writeFloat(addr_setpoint2, setpoint_fl2);
  //   EEPROM.commit();
  //   setpoint_prev_fl2 = EEPROM.readFloat(addr_setpoint2);
  // }
  if(Serial2.available() > 0){
    lcd2.setCursor(17, 0); lcd2.print("   ");
    lcd2.setCursor(17, 0); lcd2.print("ON");
    lcd1.setCursor(17, 0); lcd1.print("   ");
    lcd1.setCursor(17, 0); lcd1.print("ON");
  holdingRegs[REG_onMotor1] = flag_on_relay_fl1;
  holdingRegs[REG_onMotor2] = flag_on_relay_fl2;
  slave.poll(holdingRegs, HOLDING_REGS_SIZE);
    time_com = millis();
  }else{
    if(millis() - time_com >= 1000){
      lcd1.setCursor(17, 0); lcd1.print("OFF");
      lcd2.setCursor(17, 0); lcd2.print("OFF");
    }
  }
}
void floatconv(float dataf, uint8_t NumOfRegister){
  union fsend_t{
  float f;
  uint16_t u[2];
  };
  union fsend_t floatConvert;
  floatConvert.f = dataf;
  holdingRegs[NumOfRegister] = floatConvert.u[1];
  holdingRegs[NumOfRegister + 1] = floatConvert.u[0];
}
float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b) {
    uint32_t combined = ((uint32_t)a << 16) | b;
    float f;
    memcpy(&f, &combined, sizeof f);
    return f;
}
void lcd_show(uint8_t lcd_switch, uint8_t index_lcd){
  if(lcd_switch == flow_meter1){
    switch (index_lcd)
    {
    case show_init:
      lcd1.init();
      lcd1.backlight();
      lcd1.clear();
      lcd1.setCursor(0, 0); lcd1.print("Mode:");
      lcd1.setCursor(13, 0); lcd1.print("com:");
      lcd1.setCursor(0, 1); lcd1.print("SetL:");
      lcd1.setCursor(12, 1); lcd1.print("v:");
      lcd1.setCursor(0, 2); lcd1.print("Liter:");
      lcd1.setCursor(19, 2); lcd1.print("L");
      lcd1.setCursor(0, 3); lcd1.print("Total:");
      lcd1.setCursor(19, 3); lcd1.print("L");
      break;
    case show_value:
      lcd1.setCursor(5, 1); lcd1.print("     ");
      lcd1.setCursor(5, 1); lcd1.print(setpoint_fl1, 1);
      lcd1.setCursor(14, 1); lcd1.print("    ");
      lcd1.setCursor(14, 1); lcd1.print(FLMeter1->getCurrentFlowrate(), 0);
      lcd1.setCursor(6, 2); lcd1.print("             ");
      lcd1.setCursor(6, 2); lcd1.print(FLMeter1->getTotalVolume(), 2);
      lcd1.setCursor(6, 3); lcd1.print("             ");
      lcd1.setCursor(6, 3); lcd1.print(totalofliter1, 2);
      break;
    case show_mode:
      switch (mode_fl1)
      {
      case otomatis:
        lcd1.setCursor(5, 0); lcd1.print("      ");
        lcd1.setCursor(5, 0); lcd1.print("Auto");
        Serial.println("1->mode auto");
        break;
      case manual:
        lcd1.setCursor(5, 0); lcd1.print("      ");
        lcd1.setCursor(5, 0); lcd1.print("Manual");
        Serial.println("1->mode Manual");
        break;
      case setting:
        lcd1.setCursor(5, 0); lcd1.print("      ");
        lcd1.setCursor(5, 0); lcd1.print("Set FL");
        Serial.println("1->mode Set Flow Meter");
        break;
      }
      break;
    }
  }
  if(lcd_switch == flow_meter2){
    switch (index_lcd)
    {
    case show_init:
      lcd2.init();
      lcd2.backlight();
      lcd2.clear();
      lcd2.setCursor(0, 0); lcd2.print("Mode:");
      lcd2.setCursor(13, 0); lcd2.print("com:");
      lcd2.setCursor(0, 1); lcd2.print("SetL:");
      lcd2.setCursor(12, 1); lcd2.print("v:");
      lcd2.setCursor(0, 2); lcd2.print("Liter:");
      lcd2.setCursor(19, 2); lcd2.print("L");
      lcd2.setCursor(0, 3); lcd2.print("Total:");
      lcd2.setCursor(19, 3); lcd2.print("L");
      break;
    case show_value:
      lcd2.setCursor(5, 1); lcd2.print("     ");
      lcd2.setCursor(5, 1); lcd2.print(setpoint_fl2, 1);
      lcd2.setCursor(14, 1); lcd2.print("    ");
      lcd2.setCursor(14, 1); lcd2.print(FLMeter2->getCurrentFlowrate(), 0);
      lcd2.setCursor(6, 2); lcd2.print("             ");
      lcd2.setCursor(6, 2); lcd2.print(FLMeter2->getTotalVolume(), 2);
      lcd2.setCursor(6, 3); lcd2.print("             ");
      lcd2.setCursor(6, 3); lcd2.print(totalofliter2, 2);
      break;
    case show_mode:
      switch (mode_fl2)
      {
      case otomatis:
        lcd2.setCursor(5, 0); lcd2.print("      ");
        lcd2.setCursor(5, 0); lcd2.print("Auto");
        Serial.println("2->mode auto");
        break;
      case manual:
        lcd2.setCursor(5, 0); lcd2.print("      ");
        lcd2.setCursor(5, 0); lcd2.print("Manual");
        Serial.println("2->mode Manual");
        break;
      case setting:
        lcd2.setCursor(5, 0); lcd2.print("      ");
        lcd2.setCursor(5, 0); lcd2.print("Set FL");
        Serial.println("2->mode Set Flow Meter");
        break;
      }
      break;
    }
  }
}