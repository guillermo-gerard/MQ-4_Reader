//Eng. Guillermo A. Gerard
//2018-06-09 - Entre Rios, Argentina 

#include <Arduino.h>

#include <U8g2lib.h>
#include <EEPROM.h>
#include <SerialCommand.h>

const uint8_t mq4 = A0;
const uint8_t batterySense = A1;
const uint8_t vccSense = A2;
const uint8_t calibrateR0Button = PD5;
const uint8_t Vc = 5;
const float RsRo = 4.3;

const int airMeasurements = 500; //number of R0 measures to average
const uint8_t r0Address = 0x00;
const uint8_t historyPoints = 60; //at intervals of 1 sec, this is 1 minute of data
const uint8_t historyPpmAvgPoints = 3;
const int historyUpdateIntervalMs = 1000;
const int voltageChangeIntervalMs = 3000;
const float vccDividerRatio = 0.31543;//measured. in theory should be: 10.0F / (10.0F + 22.0F);

U8G2_SH1106_128X64_NONAME_1_HW_I2C Display(U8G2_R0);
SerialCommand SerialCmd;

float r0; // should be measured by pressing the button
float history[historyPoints];
float historyPpmAvg[historyPpmAvgPoints];
unsigned long initialTimeHistory;
unsigned long initialTimeVoltageView;

void setR0FromSerial(void);
float getBatteryValue(void);
void calibrateR0(void);
float getVccValue(void);
double getGasPpm(void);
void refreshOled(float vBattery, float vcc, float ppm, float history[]);
float value2Voltage(int value);
float getPpmAvg(float history[], int q);
void updateArray(float newPoint, float array[], int lenght);

void setup() {
	Serial.begin(115200);

	pinMode(calibrateR0Button, INPUT);

	Display.begin();
	Display.setFont(u8g2_font_5x7_mf);
	Display.enableUTF8Print();

	EEPROM.get(r0Address, r0);

	initialTimeHistory = 0;
	initialTimeVoltageView = 0;
	for (int index = 0; index < historyPoints; index++) {
		history[index] = 0.0f;
	}
	for (int index = 0; index < historyPpmAvgPoints; index++) {
		historyPpmAvg[index] = 0.0f;
	}

	SerialCmd.addCommand("RZero", setR0FromSerial);
}

void loop() {
	float vBattery = getBatteryValue();
	SerialCmd.readSerial();
	float vcc = getVccValue();

	if (digitalRead(calibrateR0Button)) {
		calibrateR0();
	}

	float ppm = getGasPpm();

	refreshOled(vBattery, vcc, ppm, history);

	if (millis() - initialTimeHistory > historyUpdateIntervalMs) {
		initialTimeHistory = millis();
		updateArray(ppm, history, historyPoints);
	}
}

float getBatteryValue() {
	int batteryValue = analogRead(batterySense);
	return value2Voltage(batteryValue);
}

float getVccValue() {
	int vccValue = analogRead(vccSense);
	vccValue = analogRead(vccSense);
	vccValue = analogRead(vccSense);
	return value2Voltage(vccValue)/vccDividerRatio;
}

float value2Voltage(int value) {
	return (float)value * Vc / 1023;
}

double getGasPpm() {
	float sensorValue = analogRead(mq4);
	sensorValue = analogRead(mq4);
	sensorValue = analogRead(mq4);
	updateArray(sensorValue = analogRead(mq4), historyPpmAvg, historyPpmAvgPoints);

	sensorValue = getPpmAvg(historyPpmAvg, historyPpmAvgPoints);
	float vSensor = value2Voltage(sensorValue);

	float rs = 1000*((Vc/vSensor)-1); //Rs value for gas 

	float ratio = rs / r0;  // get ratio RS(gas)/RS(air)=R0

	double ppm_log = (-3.21422 * log10(ratio) + 3.551895); //Get ppm value in linear scale according to the the ratio value

	double ppm = pow(10, ppm_log); //Convert ppm value to log scale

	return ppm;
}

float getPpmAvg(float history[], int q) {
	float sum = 0;
	for (int i = 0; i < min(q, historyPoints) ; i++) {
		sum += history[i];
	}
	return sum / (float)q;
}

void refreshOled(float vBattery, float vcc, float ppm, float history[]) {
	static bool showVcc = true;
	if (millis() - initialTimeVoltageView > voltageChangeIntervalMs) {
		initialTimeVoltageView = millis();
		showVcc = !showVcc;
	}
	//float ppmAvg = getPpmAvg(ppm, history, 5);
	Display.firstPage();
	do {
		Display.setCursor(64, 10);
		if (showVcc) {
			Display.print(F("VBat = "));
			Display.print(vBattery, 2);
		}
		else {
			Display.print(F("Vcc  = "));
			Display.print(vcc, 2);
		}
		Display.print(F("V"));
		Display.setCursor(64, 21);
		Display.print(F("R0="));
		Display.print(r0, 2);
		Display.drawHLine(65, 31, 63);
		Display.setCursor(88, 33);
		Display.print(F("Gas"));
		Display.setCursor(66, 51);
		Display.setFont(u8g2_font_ncenB10_tr);
		Display.print(ppm);
		Display.setCursor(88, 60);
		Display.setFont(u8g2_font_5x7_mf);
		Display.print(F("ppm"));
		// upper end of the graph = 0px, the lines are 64px max
		// data in graph ranges from 200 to 10000 ppm
		const int totalRange = 10000;//9800;
		uint8_t plotHeight = 64;
		for (uint8_t x = 0; x < historyPoints; x++) {
			Display.drawVLine(x, (uint8_t)(plotHeight - (history[x] * plotHeight / totalRange)),
				(uint8_t)(history[x] * plotHeight / totalRange));
		}
		Display.setFont(u8g2_font_4x6_mf);
		Display.setFontMode(0);
		Display.setDrawColor(0);
		Display.drawStr(0,5,"10k");
		//Display.drawStr(1, 35, "5k");
		Display.setFont(u8g2_font_5x7_mf);
		Display.setDrawColor(1);
	} while (Display.nextPage());
}

//void updateHistory(float newPoint) {
//	for (int i = historyPoints - 1; i > 0; i--) {
//		history[i] = history[i - 1];
//	}
//	history[0] = newPoint;
//}

void updateArray(float newPoint, float array[], int lenght) {
	for (int i = lenght - 1; i > 0; i--) {
		array[i] = array[i - 1];
	}
	array[0] = newPoint;
}

bool showRegressiveCounter(int);

void calibrateR0() {
	if(!showRegressiveCounter(3)){
		return;
	}

	Display.firstPage();
	do {
		
		Display.setFont(u8g2_font_ncenB08_tr);
		Display.setCursor(25, 10);
		Display.print(F("Calculating..."));
		Display.setCursor(5, 45);
		Display.print(F("Release the button..."));
	} while (Display.nextPage());
	
	Serial.println(F("now reading r0"));
	float avgReading = 0.0;
	/*--- Get an average measure by reading the A0 value airMeasurements times ---*/
	for (int x = 0; x < airMeasurements; x++)
	{
		delay(10);
		avgReading = avgReading + analogRead(A0);
	}
	avgReading = avgReading / (float)airMeasurements;
	
	float vRl = avgReading * Vc / 1023 ;
	Serial.print(F("VRL reading: "));
	Serial.println(vRl);
	
	float rs = 1000*((Vc/vRl)-1);
	
	r0 = rs / RsRo; // = R0: The ratio of RS/R0 is 4.3 in a clear air from datasheet graph

	Serial.print(F("r0 = "));
	Serial.println(r0);

	Display.firstPage();
	do {
		Display.setFont(u8g2_font_ncenB08_tr);
		Display.setCursor(30, 10);
		Display.print(F("Finished"));
		Display.setCursor(5, 45);
		Display.print(F("Release the button..."));
	} while (Display.nextPage());
	Display.setFont(u8g2_font_5x7_mf);

	EEPROM.put(r0Address, r0);
	delay(2000);
}

void setR0FromSerial() {
	char* arg = SerialCmd.next();
	String value = String(arg);
	r0 = value.toFloat();
	EEPROM.put(r0Address, r0);
	Serial.print(F("New R0 value received and saved: "));
	Serial.println(r0);
}

bool showRegressiveCounter(int waitSeconds){
	
	for (int second = waitSeconds; second > 0; second--) {
		Display.firstPage();
		do {
			
			Display.setFont(u8g2_font_ncenB08_tr);
			Display.setCursor(15, 10);
			Display.print(F("Hold pressed for:"));
			Display.setFont(u8g2_font_ncenB18_tr);
			Display.setCursor(50, 45);
			Display.print(second);
			Display.setFont(u8g2_font_ncenB08_tr);
			Display.setCursor(35, 60);
			Display.print(F("seconds"));
		} while (Display.nextPage());
		for (int waitMs = 1000; waitMs > 0; waitMs = waitMs - 100) {
			delay(100);
			if (!digitalRead(calibrateR0Button)) {
				Display.setFont(u8g2_font_5x7_mf);
				return false;
			}
		}
	}

	Display.setFont(u8g2_font_5x7_mf);
	return true;
}