#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <FlowMeter.h>
#include <ModbusRtu.h>
#include <EEPROM.h>
#include <EasyNextionLibrary.h>
#define ON LOW
#define OFF HIGH
#define ID_SLAVE 1

enum{
  index_mode_system,
  manual,
  setting,
  otomatis,
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
  REG_currentliter1,//reg 16
  REG9,//reg 17
  REG_onMotor1,//reg 20
  HOLDING_REGS_SIZE
};
uint16_t holdingRegs[HOLDING_REGS_SIZE];
FlowSensorProperties SEA_YF_DN50_FL1 = {500.0f, 1.0f, {1,1,1,1,1,1,1,1,1,1}};
FlowMeter *FLMeter1;
EasyNex nextion(Serial);
Modbus slave(ID_SLAVE, Serial2, 0);

static const uint8_t X0 = 27, X1 = 26, X2 = 33, X3 = 32, X4 = 35, X5 = 34, X6 = 39, X7 = 36;
static const uint8_t Y0 = 15, Y1 = 2, Y2 = 0, Y3 = 4, Y4 = 13, Y5 = 14;
static const uint8_t pin_input[8] = {X0, X1, X2, X3, X4, X5, X6, X7};
static const uint8_t pin_output[6] = {Y0, Y1, Y2, Y3, Y4, Y5};
bool flag_on_relay_fl1 = false, flag_on_relay_fl2 = false;
bool pulseDetected1 = false;
unsigned long time_button_fl1 = 0.0, time_button_fl2 = 0.0;
unsigned long flowStartTime1, flowLastTime1;
unsigned long timeOut1 = 3000UL, lastTime1 = 0;
double totalofliter1 = 0.0;
float totalofcubic1 = 0.0;
float setpoint_fl1 = 300.0;

uint8_t mode_fl1 = 0;
uint8_t addr_totalliter1 = 0;

IRAM_ATTR void handleInterrupt1();
int mapf(float x, float in_min, float in_max, float out_min, float out_max);
void floatconv(float dataf, uint8_t NumOfRegister);
float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b);
void setup() {
  // put your setup code here, to run once:
  nextion.begin(9600);
  Serial2.begin(9600);
  slave.start();
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(pin_input[i], INPUT);
  }
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(pin_output[i], OUTPUT);
    digitalWrite(pin_output[i], OFF);
  }
  FLMeter1 = new FlowMeter(digitalPinToInterrupt(X0), SEA_YF_DN50_FL1, handleInterrupt1, RISING);
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  totalofliter1 = EEPROM.readDouble(addr_totalliter1);
  totalofcubic1 = totalofliter1 / 1000;
  floatconv(setpoint_fl1, REG_setpoint_data1);
}

void loop() {
  if(Serial2.available() > 0){
    nextion.writeStr("t3.txt", "ON");
    nextion.writeNum("r0.val", 1);
    setpoint_fl1 = modbus_16bit_register_pair_to_float(holdingRegs[REG_setpoint_data1], holdingRegs[REG_setpoint_data1 + 1]);
    slave.poll(holdingRegs, HOLDING_REGS_SIZE);
  }else{
    nextion.writeStr("t3.txt", "OFF");
    nextion.writeNum("r0.val", 0);
  }
  if (pulseDetected1) {
    unsigned long currentTime1 = millis();
    unsigned long duration1 = currentTime1 - lastTime1; 
    if (currentTime1 - flowLastTime1 >= timeOut1) { // Dieksekusi jika selama 3 detik sudah tidak ada pulsa baru yang masuk lagi 
      pulseDetected1 = false;
    }
    else if (duration1 >= 1000) { // Dieksekusi setiap 1 detik        
      // Fungsi untuk menghitung flow rate dan volume dalam 1 period
      FLMeter1->tick(duration1);
      lastTime1 = currentTime1;
      floatconv(FLMeter1->getCurrentFlowrate(), REG_literpermenit1);
      if(flag_on_relay_fl1 == true){
        nextion.writeNum("x1.val", (FLMeter1->getTotalVolume() * 10));
        floatconv(FLMeter1->getTotalVolume(), REG_currentliter1);
      }
    }
  }
  nextion.writeNum("x0.val", (FLMeter1->getCurrentFlowrate() * 10));
  int int_lpm1 = (mapf(FLMeter1->getCurrentFlowrate(), 0.0, 1000.0, 0, 100));
  nextion.writeNum("j0.val", int_lpm1);
  nextion.writeNum("n0.val", int_lpm1);
  nextion.writeNum("x2.val", (setpoint_fl1 * 10));
  nextion.writeNum("x3.val", (totalofliter1 * 10));
  nextion.writeNum("x4.val", (totalofcubic1 * 10));

  if (digitalRead(X4) == OFF && digitalRead(X6) == OFF && mode_fl1 != manual){mode_fl1 = manual; nextion.writeStr("t31.txt", "Manual");}
  if(digitalRead(X4) == ON && digitalRead(X6) == OFF && mode_fl1 != otomatis){mode_fl1 = otomatis; nextion.writeStr("t31.txt", "Auto");}
  if(digitalRead(X4) == OFF && digitalRead(X6) == ON && mode_fl1 != setting){mode_fl1 = setting; nextion.writeStr("t31.txt", "Setting");}

  if(mode_fl1 != setting){
    if(digitalRead(X2) == ON && millis() - time_button_fl1 >= 1000){
      time_button_fl1 = millis();
      if(flag_on_relay_fl1 == false){
        digitalWrite(Y0, ON);
        digitalWrite(Y1, ON);
        FLMeter1->setTotalVolume(0.0);
        flag_on_relay_fl1 = true;
        holdingRegs[REG_onMotor1] = flag_on_relay_fl1;
      }else if(flag_on_relay_fl1 == true && mode_fl1 == manual){
        digitalWrite(Y0, OFF);
        flag_on_relay_fl1 = false;
        holdingRegs[REG_onMotor1] = flag_on_relay_fl1;
        totalofliter1 += FLMeter1->getTotalVolume();
        totalofcubic1 = totalofliter1 / 1000;
        floatconv(totalofliter1, REG_totalofliter1);
        floatconv(totalofcubic1, REG_totalofcubic1);
        if(totalofliter1 >= 9999999.9){
          totalofliter1 = 0.0;
        }
        EEPROM.writeDouble(addr_totalliter1, totalofliter1);
        EEPROM.commit();
        delay(1000);
        digitalWrite(Y1, OFF);
      }
    }
    if(FLMeter1->getTotalVolume() >= (setpoint_fl1-2) && mode_fl1 == otomatis){
      if(flag_on_relay_fl1 == true){
        digitalWrite(Y0, OFF);
        flag_on_relay_fl1 = false;
        holdingRegs[REG_onMotor1] = flag_on_relay_fl1;
        double result_liter = (setpoint_fl1 + (abs(FLMeter1->getTotalVolume() - setpoint_fl1)/10));
        nextion.writeNum("x1.val", (result_liter * 10));
        floatconv(result_liter, REG_currentliter1);
        totalofliter1 += result_liter;
        totalofcubic1 = totalofliter1 / 1000;
        floatconv(totalofliter1, REG_totalofliter1);
        floatconv(totalofcubic1, REG_totalofcubic1);
        if(totalofliter1 >= 9999999.9){
          totalofliter1 = 0.0;
        }
        EEPROM.writeDouble(addr_totalliter1, totalofliter1);
        EEPROM.commit();
        delay(1000);
        digitalWrite(Y1, OFF);
      }
    }
  }else if(mode_fl1 == setting){
    if(digitalRead(X2) == ON){
      setpoint_fl1 += 100.0;
      if(setpoint_fl1 > 2000){setpoint_fl1 = 100.0;}
      floatconv(setpoint_fl1, REG_setpoint_data1);
      delay(250);
    }
  }
  // put your main code here, to run repeatedly:
  yield();
}
IRAM_ATTR void handleInterrupt1(){
  FLMeter1->count();
  if (!pulseDetected1) {
    pulseDetected1 = true; // Flag untuk menandakan flow meter sedang menghitung flow
    flowStartTime1 = millis(); // Untuk mengetahui kapan pertama kali mulai menghitung flow
  }
  flowLastTime1 = millis();
}
int mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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