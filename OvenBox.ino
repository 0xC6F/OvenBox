#include <SPI.h>
#include <SD.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define SSR_PIN 9
#define THERM1_PIN A0
#define THERM2_PIN A1

#define CONTROL "BANGBANG"
//#define CONTROL "PID"

#define kp = 2;
#define ki = 5;
#define kd = 1;

#define THERMISTOR_VOLTAGE 5
#define RESISTOR_HIGH 180.00
#define RESISTOR_MID 4700.00
#define RESISTOR_LOW 100000.00

bool start = false;

LiquidCrystal_I2C lcd(0x27,16,2);
File profilesFile;
byte index;
int targetTemp = 0;
int currentTemperature;

/*PID data variables*/
double error;
double lastError;
double setPoint;
double cumError, rateError;

String selectedProfile[] = {"","","","","",""};

void setup() {
  pinMode(SSR_PIN,OUTPUT);
  lcd.begin();
  lcd.backlight();
  getProfileFromSD('0');
  currentTemperature = (steinHartTemp(THERM1_PIN,RESISTOR_LOW)+steinHartTemp(THERM2_PIN,RESISTOR_LOW))/2;
}

void loop() {
  writeLCD(selectedProfile[0]+": "+ selectedProfile[1],"S*:"+selectedProfile[2]+" St:"+selectedProfile[3]);
  delay(4000);
  writeLCD(selectedProfile[0]+": "+ selectedProfile[1],"R*:"+selectedProfile[4]+" Rt:"+selectedProfile[5]);
  delay(4000);
  while(start == true)
  {
    if(currentTemperature <= 75){ currentTemperature = (steinHartTemp(THERM1_PIN,RESISTOR_LOW)+steinHartTemp(THERM2_PIN,RESISTOR_LOW))/2; }
    if(currentTemperature <= 180 && currentTemperature >= 76){ currentTemperature = (steinHartTemp(THERM1_PIN,RESISTOR_MID)+steinHartTemp(THERM2_PIN,RESISTOR_MID))/2; }
    if(currentTemperature <= 250 && currentTemperature >= 181){  currentTemperature = (steinHartTemp(THERM1_PIN,RESISTOR_HIGH)+steinHartTemp(THERM2_PIN,RESISTOR_HIGH))/2; }
    else { while(1){digitalWrite(SSR_PIN,LOW);} }
    analogWrite(SSR_PIN,heatCheck(currentTemperature));
    writeLCD(selectedProfile[0]+": "+ selectedProfile[1]+" T0:"+(String)currentTemperature,"S*:"+selectedProfile[2]+" St:"+selectedProfile[3]);
    delay(4000);
    writeLCD(selectedProfile[0]+": "+ selectedProfile[1]+" T1:"+(String)currentTemperature,"R*:"+selectedProfile[4]+" Rt:"+selectedProfile[5]);
    delay(4000);
  }
}

void getProfileFromSD(char desiredIndex){
  selectedProfile[0] = "";
  selectedProfile[1] = "";
  selectedProfile[2] = "";
  selectedProfile[3] = "";
  selectedProfile[4] = "";
  selectedProfile[5] = "";
  
  Serial.begin(115200);
  int i = 0;
  
  if (!SD.begin(10)) {
    Serial.println("Reading SD failed!");
  }
  else {
  profilesFile = SD.open("profiles.txt");
  if (profilesFile) {
    while (profilesFile.available()) {
      char temp = profilesFile.read();
      if(temp == ':'){
        temp = profilesFile.read();
        selectedProfile[0] = desiredIndex; 
        if(temp == desiredIndex){
          while(temp != '.'){
            temp = profilesFile.read();
            if(temp == ','){ i++; temp = profilesFile.read(); }
            if(temp == '.'){ break; }
            selectedProfile[i] += temp;
          }
          break;
          }
        }
      }
    }
  }
  profilesFile.close();
  Serial.end();
}

int heatCheck(int temperature){
  int output = 0;
  if(CONTROL == "BANGBANG"){
    if(targetTemp > temperature) {output = 255;}
    if(targetTemp <= temperature) {output = 0;}
    }
  if(CONTROL == "PID"){
    double elapsedTime;
    if(millis() >= 8100){elapsedTime = (double)8100;}
    else {elapsedTime = (double)(millis());}        //compute time elapsed from previous computation
    
    error = setPoint - temperature;                                // determine error
    cumError += error * elapsedTime;                // compute integral
    rateError = (error - lastError)/elapsedTime;   // compute derivative
    double output = kp*error + ki*cumError + kd*rateError;                //PID output               
    lastError = error;                                //remember current error
    }
    return output;
  }

int steinHartTemp(int thermPin, float resistor){
  /* https://rusefi.com/Steinhart-Hart.html */
  double a = 0.0007958948;
  double b = 0.0002135888;
  double c = 0.0000000650;

  float voltage = (analogRead(thermPin)/1023)*THERMISTOR_VOLTAGE;
  
  int temperature =(-1.0/b)*(log(((resistor*voltage)/(a*(THERMISTOR_VOLTAGE-voltage)))-(c/a)));
  //t = 273,15 ° C + [a0 + a1 · ln r]-1
  return temperature;
  }

void writeLCD(String line0, String line1){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(line0);
    lcd.setCursor(0,1);
    lcd.print(line1);
}
