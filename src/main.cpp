#include <M5EPD.h>
#include <Multichannel_Gas_GMXXX.h>
#include <SGP30.h>
static uint8_t recv_cmd[8] = {};//multi-channel gas sensor data buffer
char temStr[10],humStr[10],warn[20];
char xStr[10],yStr[10];
float tem,hum;
int ts=0,tsco=0;
float tem1,tem2,hum1,hum2,co2old,tvocold;
int point[2];
uint32_t lastTime = 0;
bool HE=false;// turn on or off H2 and Ethanol detection
#define CARDKB_ADDR 0x5F
#define MULTIG_ADDR 0x08//maybe 3c
#define SGP30G_ADDR 0x58
#include <Wire.h>
GAS_GMXXX<TwoWire> multigas;
SGP30 SGP(&Wire1);

M5EPD_Canvas canvas(&M5.EPD);

void setup(){
    //pinMode(M5EPD_MAIN_PWR_PIN, OUTPUT);
    //M5.enableEXTPower();
    //M5.enableMainPower();
    //M5.enableEPDPower();
    Serial.print("starting");
    M5.begin();//false);
    M5.SHT30.Begin();
    M5.EPD.SetRotation(90);
    M5.EPD.Clear(true);
    Wire1.begin(25, 32);
    canvas.createCanvas(540, 960);
    canvas.setTextSize(3);
    multigas.begin(Wire1, MULTIG_ADDR);
    ts=0;tsco=0;
    SGP.begin(25,32);
    SGP.GenericReset();
    if(HE) SGP.request();
}

// GM102B (NO2)
// Rs means resistance of sensor in 2ppm NO2 under different temp. and humidity.
// Rso means resistance of the sensor in 2ppm NO2 under 20°C/55%RH.

const float gm102b_rh_offset[4][7] PROGMEM = {
  { -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0 }, // °C
  { 1.71, 1.58, 1.45, 1.39, 1.12, 1.00, 0.89 }, // Rs/R0 @ 30%RH
  { 1.49, 1.32, 1.28, 1.08, 0.99, 0.88, 0.71 }, // Rs/R0 @ 60%RH
  { 1.28, 1.15, 10.9, 0.90, 0.86, 0.71, 0.68 }  // Rs/R0 @ 85%RH
};

const float gm102b_u2gas[2][12] PROGMEM = {
  { 0.0, 0.21, 0.39, 0.7, 0.95, 1.15, 1.35, 1.45, 1.6, 1.69, 1.79, 1.81 }, // V
  { 0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 }, // NO2 [ppm]
};

// GM302B (Ethanol=C2H5OH)
// Rs means resistance of sensor in 50ppm ethanol under different temp. and humidity.
// Rso means resistance of the sensor in 50ppm ethanol under 20°C/65%RH.

const float gm302b_rh_offset[4][13] PROGMEM = {
  { -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 },  // °C
  { 1.71, 1.61, 1.58, 1.50, 1.42, 1.30, 1.25, 1.18, 1.15, 1.12, 1.00, 0.92, 0.88 }, // Rs/R0 @ 30%RH
  { 1.45, 1.36, 1.33, 1.28, 1.20, 1.11, 1.08, 1.00, 0.98, 0.95, 0.85, 0.79, 0.73 }, // Rs/R0 @ 60%RH
  { 1.27, 1.20, 1.18, 1.10, 1.05, 0.95, 0.92, 0.88, 0.86, 0.81, 0.72, 0.69, 0.64 }  // Rs/R0 @ 85%RH
};

const float gm302b_u2gas[2][11] PROGMEM = {
  { 1.25, 1.5, 2.0, 2.25, 2.5, 3.1, 3.3, 3.6, 3.7, 3.8, 3.85 }, // Alcohol/Ethanol [V]
  { 0.0, 1.0, 3.5, 5.0, 10.0, 30.0, 50.0, 80.0, 100.0, 200.0, 500.0 } // VOC [ppm]
};

// GM502B (VOC)
// Rs means resistance of sensor in 150ppm CO gas under different temp. and humidity.
// Rso means resistance of the sensor in 150ppm CO gas under 20°C/55%RH.

const float gm502b_rh_offset[4][13] PROGMEM = {
  { -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 },  // °C
  { 1.71, 1.62, 1.54, 1.50, 1.42, 1.30, 1.25, 1.16, 1.14, 1.11, 1.00, 0.92, 0.88 }, // Rs/R0 @ 30%RH
  { 1.45, 1.38, 1.35, 1.28, 1.21, 1.11, 1.08, 1.00, 0.98, 0.96, 0.85, 0.79, 0.75 }, // Rs/R0 @ 60%RH
  { 1.25, 1.20, 1.18, 1.10, 1.05, 0.95, 0.92, 0.88, 0.86, 0.81, 0.73, 0.68, 0.62 }  // Rs/R0 @ 85%RH
};

const float gm502b_u2gas[2][9] PROGMEM = {//for me ,here 0.95v is 110ppb at 17C45%RH
  { 2.52, 2.90, 3.20, 3.40, 3.60, 3.90, 4.05, 4.15, 4.20 }, // Alcohol [V]
  { 0.0, 1.0, 3.5, 5.0, 10.0, 30.0, 50.0, 80.0, 100.0 }  // VOC [ppm]
};

// GM702B (CO)
// Rs means resistance of sensor in 150ppm CO gas under different temp. and humidity.
// Rso means resistance of the sensor in 150ppm CO gas under 20°C/55%RH

const float gm702b_rh_offset[4][7] PROGMEM = {
  { -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0 }, // °C
  { 1.71, 1.58, 1.45, 1.38, 1.13, 1.01, 0.88 }, // Rs/R0 @ 30%RH
  { 1.47, 1.32, 1.28, 1.08, 0.98, 0.88, 0.72 }, // Rs/R0 @ 60%RH
  { 1.28, 1.15, 1.08, 0.90, 0.87, 0.71, 0.68 }  // Rs/R0 @ 85%RH
};

const float offset = 1.28;
const float gm702b_u2gas[2][9] PROGMEM = {// for me ,here 2.22v is 2-3ppm at 17C45%RH
  { 0.25+offset, 0.65+offset, 0.98+offset, 1.35+offset, 1.8+offset, 1.98+offset, 2.1+offset, 2.38+offset, 2.42+offset }, // V
  { 0.0, 5.0, 10.0, 20.0, 50.0, 100.0, 160.0, 500.0, 1000.0 }  // CO [ppm]
};

float u_corr_rh(float u, float temp, float humidity, float* u_corr, size_t size) {
  size_t hum_idx1;
  size_t hum_idx2;
  float ref_hum1, ref_hum2;
  if (humidity <= 30.0) {
    hum_idx1 = 1;
    hum_idx2 = 1;
    ref_hum1 = 30.0;
    ref_hum1 = 60.0;
  } else if (humidity <= 60.0) {
    hum_idx1 = 1;
    hum_idx2 = 2;
    ref_hum1 = 30.0;
    ref_hum2 = 60.0;
  } else if (humidity <= 85.0) {
    hum_idx1 = 2;
    hum_idx2 = 3;
    ref_hum1 = 60.0;
    ref_hum2 = 85.0;
  } else {
    hum_idx1 = 3;
    hum_idx2 = 3;
    ref_hum1 = 60.0;
    ref_hum2 = 85.0;
  }
  size_t hum_off1 = size * hum_idx1;
  size_t hum_off2 = size * hum_idx2;
  // First get Rs/R0
  float old_rsr01 = *(u_corr + hum_off1);
  float old_rsr02 = *(u_corr + hum_off2);
  float rsr01 = old_rsr01;
  float rsr02 = old_rsr02;
  float old_temp = u_corr[0];
  if (temp >= old_temp) {
    for (size_t i = 1; i < size; i++) {
      float new_temp = *(u_corr + i);
      rsr01 = *(u_corr + hum_off1 + i);
      rsr02 = *(u_corr + hum_off2 + i);
      //Serial.println("*i=" + String(i) + "  old_temp=" + String(old_temp, 2) + "  new_temp=" + String(new_temp, 2) + "  rsr01=" + String(rsr01, 2) + "  rsr02=" + String(rsr02, 2));
      if (temp <= new_temp) {
        //Serial.println("-- " + String(temp - old_temp, 2) + "  " + String(new_temp - old_temp, 2) + "  " + String(rsr01 - old_rsr01, 2) + "  " + String(rsr02 - old_rsr02, 2));
        old_rsr01 += (temp - old_temp) / (new_temp - old_temp) * (rsr01 - old_rsr01);
        old_rsr02 += (temp - old_temp) / (new_temp - old_temp) * (rsr02 - old_rsr02);
        break;
      }
      old_temp = new_temp;
      old_rsr01 = rsr01;
      old_rsr02 = rsr02;
    }
  }
  float fact = (old_rsr01 + (humidity - ref_hum1) / (ref_hum2 - ref_hum1) * (old_rsr02 - old_rsr01));
  //Serial.println("old_rsr01=" + String(old_rsr01, 2) + "  old_rsr02=" + String(old_rsr02, 2) + "  ref_hum1=" + String(ref_hum1, 2) + "  ref_hum2=" + String(ref_hum2, 2));
  //Serial.println("fact=" + String(fact));
  return u / fact;
}

float u2ppm(float u, float* u2gas, size_t size) {
  float old_ppm = *(u2gas + size);
  float old_u = u2gas[0];
  if (u <= old_u) {
    return old_ppm;
  }
  for (size_t i = 1; i < size; i++) {
    float new_u = *(u2gas + i);
    float ppm = *(u2gas + size + i);
    //Serial.println("i=" + String(i) + "  new_u=" + String(new_u, 2) + "  ppm=" + String(ppm, 2));
    if (u <= new_u) {
      //Serial.println("++ " + String(u - old_u, 2) + "  " + String(new_u - old_u, 2) + "  " + String(ppm - old_ppm, 2));
      return old_ppm + (u - old_u) / (new_u - old_u) * (ppm - old_ppm);
    }
    old_u = new_u;
    old_ppm = ppm;
  }
  return old_ppm;
}

float getNO2ppm(uint32_t raw, float temp, float humidity) {
  float no2_u = multigas.calcVol(raw);
  float no2_corr = u_corr_rh(no2_u, temp, humidity, (float *)gm102b_rh_offset, 7);
  return u2ppm(no2_corr, (float *)gm102b_u2gas, 12);
}

float getC2H5OHppm(uint32_t raw, float temp, float humidity) {
  float c2h5oh_u = multigas.calcVol(raw);
  float c2h5oh_corr = u_corr_rh(c2h5oh_u, temp, humidity, (float *)gm302b_rh_offset, 13);
  return u2ppm(c2h5oh_corr, (float *)gm302b_u2gas, 11);
}

float getVOCppm(uint32_t raw, float temp, float humidity) {
  float voc_u = multigas.calcVol(raw);
  float voc_corr = u_corr_rh(voc_u, temp, humidity, (float *)gm502b_rh_offset, 13);
  return u2ppm(voc_corr, (float *)gm502b_u2gas, 9);
}

float getCOppm(uint32_t raw, float temp, float humidity) {
  float co_u = multigas.calcVol(raw);
  float co_corr = u_corr_rh(co_u, temp, humidity, (float *)gm702b_rh_offset, 7);
  return u2ppm(co_corr, (float *)gm702b_u2gas, 9);
}



void loop(){
    strcpy(warn,"Warning:");
    canvas.drawRoundRect(50, 0, 450, 200, 20, 15);
    canvas.drawRoundRect(50, 200, 450, 200, 20, 15);
    canvas.drawString("30" , 10, 0);   
    canvas.drawString("10" , 10, 180);
    canvas.drawString("80" , 10, 210);
    canvas.drawString("30" , 10, 380);
    M5.SHT30.UpdateData();
    //Serial.printf("%i\n",ts);
    tem = M5.SHT30.GetTemperature();
    hum = M5.SHT30.GetRelHumidity();
    //Serial.printf("Temperatura: %2.2f*C  Humedad: %0.2f%%\r\n", tem, hum);
    dtostrf(tem, 2, 2 , temStr);
    dtostrf(hum, 2, 2 , humStr);
    if(tem>35) strcat(warn,"High Tem! ");
    if(tem<10) strcat(warn,"Low Tem! ");
    if(hum>75) strcat(warn,"High Hum! ");
    if(hum<30) strcat(warn,"Low Hum! ");
    canvas.drawString("Humd:   " + String(humStr)+"%" , 100, 460);
    if(ts>=450){
      ts=0;
      canvas.fillRoundRect(50, 0, 450, 200, 20, 0);
      canvas.fillRoundRect(50, 200, 450, 200, 20, 0);
      canvas.drawRoundRect(50, 0, 450, 200, 20, 15);
      canvas.drawRoundRect(50, 200, 450, 200, 20, 15);
    }
    else if(ts==0){
      tem1=tem;
      hum1=hum;
      ts++;
    }
    else{
      tem2=tem;
      hum2=hum;
      ts++;
      canvas.drawLine(ts+49, 200-(tem1-10)/20.0*200, ts+50, 200-(tem2-10)/20.0*200, 2, 15);
      canvas.drawLine(ts+49, 400-(hum1-30)/50.0*200, ts+50, 400-(hum2-30)/50.0*200, 2, 15);
      tem1=tem2;
      hum1=hum2;
    }   
    M5.TP.update();
    if(M5.TP.getFingerNum() == 1){
      point[0] = M5.TP.readFingerX(0);
      point[1] = M5.TP.readFingerY(0);
    }
    //Serial.printf("Touch-%i,%i\n", point[0], point[1]);
    char data='C';
    char c = 0;
    Wire1.requestFrom(
        CARDKB_ADDR,
        1);  // Request 1 byte from the slave device.
    while (
        Wire1.available())  // If received data is detected.
        c = Wire1.read();  // Store the received data.
        if (c != 0) {
            Serial.println(c);
            data=c;
        }
    canvas.drawString("Temp:   " + String(temStr) + "*"+String(data), 100, 420);
    
    uint8_t len = 0;
    uint8_t addr = 0;
    uint8_t i;
    uint32_t val = 0;

    val = multigas.getGM102B(); //Serial.print("GM102B: "); Serial.print(val); Serial.print("  =  ");
    //Serial.print(multigas.calcVol(val)); Serial.println("V");
    //Serial.print("NO2: "); Serial.print(getNO2ppm(val,tem,hum));Serial.println(" ppm");
    if(val!=0) canvas.drawString("NO2:    " + String(getNO2ppm(val,tem,hum)) + "ppm     ", 100, 500);
    else canvas.drawString("NO2:    Connect Err", 100, 500);
    //Serial.println("----");
    val = multigas.getGM302B(); //Serial.print("GM302B: "); Serial.print(val); Serial.print("  =  ");
    //Serial.print(multigas.calcVol(val)); Serial.println("V");
    //Serial.print("C2H5OH: "); Serial.print(getC2H5OHppm(val,tem,hum));Serial.println(" ppm");
    if(val!=0) canvas.drawString("CH2H5OH:" + String(getC2H5OHppm(val,tem,hum)) + "ppm       ", 100, 540);
    else canvas.drawString("CH2H5OH:Connect Err", 100, 540);
    //Serial.println("----");
    val = multigas.getGM502B(); //Serial.print("GM502B: "); Serial.print(val); Serial.print("  =  ");
    //Serial.print(multigas.calcVol(val)); Serial.println("V");
    //Serial.print("VOC: "); Serial.print(getVOCppm(val,tem,hum)*1000);Serial.println(" ppb");
    if(val!=0) canvas.drawString("VOC:    " + String(getVOCppm(val,tem,hum)*1000) + "ppb       ", 100, 580);
    else canvas.drawString("VOC:    Connect Err", 100, 580);
    //Serial.println("----");
    val = multigas.getGM702B(); //Serial.print("GM702B: "); Serial.print(val); Serial.print("  =  ");
    //Serial.print(multigas.calcVol(val)); Serial.println("V");
    //Serial.print("CO: "); Serial.print(getCOppm(val,tem,hum));Serial.println(" ppm");
    if(val!=0) canvas.drawString("CO:     " + String(getCOppm(val,tem,hum)) + "ppm    ", 100, 620);
    else canvas.drawString("CO:     Connect Err", 100, 620);
    //if(getCOppm(val,tem,hum)>9) strcat(warn,"CO! ");
    //Serial.println("----");
    if(ts==1&&HE==false){
      canvas.drawString("H2:     Disabled", 100, 740);
      canvas.drawString("Ethanol:Disabled", 100, 780);
    }
    if(millis() - lastTime > 1000){
        tsco++;
        SGP.setRelHumidity(tem, hum);
        bool measure=SGP.isConnected();
        if((tsco%2==1 && tsco<1000) || (!HE && tsco<1000)) {
          SGP.read();//for co2 and tvoc
          if(measure && SGP.getTVOC()<60000){
            tvocold=SGP.getTVOC();
            canvas.drawString("tVOC:   " + String(tvocold) + "ppb    ", 100, 700);
          }
          else if(measure && SGP.getTVOC()>=60000) canvas.drawString("tVOC:   Unstable         ", 100, 700);
          else canvas.drawString("tVOC:   Connect Err", 100, 700);
          if(measure && SGP.getCO2()<57330){
            co2old=SGP.getCO2();
            canvas.drawString("CO2:    " + String(co2old) + "ppm    ", 100, 660);
          }
          else if(measure && SGP.getCO2()>=57330) canvas.drawString("CO2:    Unstable       ", 100, 660);
          else canvas.drawString("CO2:    Connect Err", 100, 660);
          SGP.request();
        }
        else if(tsco%2==0 && tsco<1000 && HE){
          SGP.readRaw();
          if(measure && SGP.getH2()<=100000) canvas.drawString("H2:     " + String(SGP.getH2()) + "ppm    ", 100, 740);
          else if(measure && SGP.getH2()>100000) canvas.drawString("H2:     Unstable                ", 100, 740);
          else canvas.drawString("H2:     Connect Err ", 100, 740);
          if(measure && SGP.getEthanol()<=100000) canvas.drawString("Ethanol:" + String(SGP.getEthanol()) + "ppm    ", 100, 780);
          else if(measure && SGP.getEthanol()>100000) canvas.drawString("Ethanol:Unstable                ", 100, 780);
          else canvas.drawString("Ethanol:Connect Err ", 100, 780);
          SGP.requestRaw();
        }
        else if(tsco>=1000) tsco=0;

        lastTime = millis();
    }
    if(co2old>1000&&co2old<2000) strcat(warn,"CO2! ");
    if(co2old>2000) strcat(warn,"CO2!! ");
    if(tvocold>250) strcat(warn,"tVOC! ");
    if(strlen(warn)<=8) strcpy(warn,"Good Air Condition!");
    canvas.fillRect(100, 820, 500, 40, 0);
    canvas.drawString(warn, 100, 820);
    canvas.pushCanvas(0,50,UPDATE_MODE_A2);
    delay(100);
}
