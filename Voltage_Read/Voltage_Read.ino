/** DIGITAL MULTIMETER (DMM) using Arduino
 * 
 *  Author			: Ayush Chinmay
 *  Date Created	: 08 August 2023
 *	Last Modified	: 08 August 2023
 *
 * 	NOTES:
 * 		- 	
 *
 *	CHANGELOG:
 **		-	08 August 2023
 *						- Initial Commit
 *
 *	TODO:
 *!		[-]		Measure Voltager higher than 5V (protect GPIO Pins)
 */

/**
 *  Circuit Connections
 *  A0 <--> 4.7kOhm resistor <--> Pout
 *  GND <--> 4.7kOhm resistor <--> Pout
 */

 unsigned long timer = 0;

/** ==========[ SETUP ]========== *
 *  Setup function runs once when the program starts
*/
void setup() {
    Serial.begin(115200);
    pinMode(A1, INPUT);
    timer = millis();
}

/** ==========[ MAIN LOOP ]========== *
 *  Main Loop of the program
*/
void loop() {
    float R1 = 4700;    /// Resistor 1 (Ohms)
    float R2 = 4700;    /// Resistor 2 (Ohms)

    float voltage_in = measVoltage();   /// Measure Voltage at A0
    float voltage_out = calcVoltageDiv(R1, R2, voltage_in);   /// Calculate Voltage at input terminals of Voltage Divider
    float current = calcCurrent(voltage_in, R1 + R2);   /// Calculate current through circuit

    // Pint timer (ms)
    Serial.print("Timestamp = ");
    Serial.print(millis() - timer);
    Serial.print("\t");
    
    // print output voltage
    Serial.print("Voltage Out = ");
    Serial.print(voltage_out);
    Serial.print(" mV\t");

    // print input voltage
    Serial.print("Voltage Read = ");
    Serial.print(voltage_in);
    Serial.print(" mV\t");

    // print total circuit current
    Serial.print("Current = ");
    Serial.print(current);
    Serial.println(" mA\t");

//   Serial.println("====================");

  delay(100);
}

/** ==========[ MEASURE VOLTAGE ]========== **
 *	Measure Voltage at A0 and convert it to millivolts
 */
float measVoltage() {
  /// Read value at A0 and convert it to millivolts
  int rawVolt = analogRead(A7);            
  float milliVolt = rawVolt * (5.0 / 1023);

  /// Print value to Serial Monitor
  Serial.print("Voltage = ");   Serial.print(milliVolt);    Serial.println(" mV");

  return milliVolt;   /// Return millivolts          
}

/** ==========[ CALCULATE VOLTAGE DIVIDER ]========== **
 *  Calculate Voltage at input terminals of Voltage Divider
*/
float calcVoltageDiv(float R1, float R2, float Vout)  {
  float Vin = Vout * (R1 + R2) / R2;
  return Vin;
}

float calcCurrent(float Vin, float Rtot) {
  float current = Vin / Rtot;
  return current;
}