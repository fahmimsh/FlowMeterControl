#include <Arduino.h>
#include <HardwareSerial.h>
#include <FlowMeter.h>
#include <EasyNextionLibrary.h>
#define ON LOW
#define OFF HIGH

enum{
  index_mode_system,
  manual,
  setting,
  otomatis,
};

FlowSensorProperties SEA_YF_DN50_FL1 = {500.0f, 1.06f, {1,1,1,1,1,1,1,1,1,1}}; //0.1
FlowMeter *FLMeter1;
EasyNex nextion(Serial);

static const uint8_t X0 = 27, X1 = 26, X2 = 33, X3 = 32, X4 = 35, X5 = 34, X6 = 39, X7 = 36;
static const uint8_t Y0 = 15, Y1 = 2, Y2 = 0, Y3 = 4, Y4 = 13, Y5 = 14;
static const uint8_t pin_input[8] = {X0, X1, X2, X3, X4, X5, X6, X7};
static const uint8_t pin_output[6] = {Y0, Y1, Y2, Y3, Y4, Y5};
bool flag_on_relay_fl1 = false;
bool pulseDetected1 = false;
unsigned long time_btn_1 = 0;
unsigned long time_off_relay1 = 0;
unsigned long flowStartTime1, flowLastTime1;
unsigned long timeOut1 = 3000UL, lastTime1 = 0;
double volume_fl1 = 0.0;
double current_fl1_rate = 0.0;
float setpoint_fl1 = 300.0;
double result_liter = 0.0;

uint8_t mode_fl1 = 0;
uint8_t addr_totalliter1 = 0;

IRAM_ATTR void handleInterrupt1();
int mapf(float x, float in_min, float in_max, float out_min, float out_max);
void setup() {
  // put your setup code here, to run once:
  nextion.begin(9600);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(pin_input[i], INPUT);
  }
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(pin_output[i], OUTPUT);
    digitalWrite(pin_output[i], OFF);
  }
  FLMeter1 = new FlowMeter(digitalPinToInterrupt(X0), SEA_YF_DN50_FL1, handleInterrupt1, RISING);
  nextion.writeNum("x2.val", (setpoint_fl1 * 10));
}

void loop() {
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
      if(flag_on_relay_fl1 == true){
        volume_fl1 = FLMeter1->getTotalVolume();
        current_fl1_rate = FLMeter1->getCurrentFlowrate();
        nextion.writeNum("x1.val", (volume_fl1 * 10));
        nextion.writeNum("x0.val", (current_fl1_rate * 10));
        int int_lpm1 = (mapf(current_fl1_rate, 0.0, 1000.0, 0, 100));
        nextion.writeNum("j0.val", int_lpm1);
        nextion.writeNum("n0.val", int_lpm1);
      }
    }
  }
  if (digitalRead(X4) == OFF && digitalRead(X6) == OFF && mode_fl1 != manual){mode_fl1 = manual; nextion.writeStr("t31.txt", "Manual");}
  if(digitalRead(X4) == ON && digitalRead(X6) == OFF && mode_fl1 != otomatis){mode_fl1 = otomatis; nextion.writeStr("t31.txt", "Auto");}
  if(digitalRead(X4) == OFF && digitalRead(X6) == ON && mode_fl1 != setting){mode_fl1 = setting; nextion.writeStr("t31.txt", "Setting");}
  if(digitalRead(X2) == ON && (millis() - time_btn_1 >= 1000)){
    time_btn_1 = millis();
    if(mode_fl1 == manual){
      if(flag_on_relay_fl1 == false){
        digitalWrite(Y0, ON);
        digitalWrite(Y1, ON);
        FLMeter1->setTotalVolume(0.0);
        flag_on_relay_fl1 = true;
      }else if(flag_on_relay_fl1 == true){
        digitalWrite(Y0, OFF);
        time_off_relay1 = millis();
      }
    }else if(mode_fl1 == otomatis){
      if(flag_on_relay_fl1 == false){
        digitalWrite(Y0, ON);
        digitalWrite(Y1, ON);
        FLMeter1->setTotalVolume(0.0);
        flag_on_relay_fl1 = true;
      }
    }else if(mode_fl1 == setting){
      setpoint_fl1 += 100.0;
      if(setpoint_fl1 > 2000){
        setpoint_fl1 = 100.0;
      }
    }
  }
  if(volume_fl1 >= (setpoint_fl1-2) && mode_fl1 == otomatis){
    result_liter = (setpoint_fl1 + (abs(volume_fl1 - setpoint_fl1)/10));
    nextion.writeNum("x1.val", (result_liter * 10));
    digitalWrite(Y0, OFF);
    time_off_relay1 = millis();
  }
  if((millis() - time_off_relay1 > 1000) && flag_on_relay_fl1 == true){
    digitalWrite(Y1, OFF);
    flag_on_relay_fl1 = false;
    nextion.writeNum("x2.val", (setpoint_fl1 * 10));
  }
  // put your main code here, to run repeatedly:
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