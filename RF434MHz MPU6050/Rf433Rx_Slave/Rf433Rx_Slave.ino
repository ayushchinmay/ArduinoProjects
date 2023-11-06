/**
 *  Project: RF 433Mhz Tranceiver [Slave]
 *
 *  Author          : Ayush Chinmay
 *  Date Created    : 22 Sept 2023
 *  Date Modified   : 05 Oct 2023
 *
 *  NOTES:
 *      - 
 *      -
 *      - 
 *
 *  CHANGELOG:
 *      [22 Sept 2023]
 *          - Transmission and Reception functionality
 *          - Transmitter reflects the received signal
 *
 *      [24 Sept 2023]
 *          - Round - trip communication
 *  
 *      [05 Oct 2023]
 *          - Documentation and general code clean-up
 *          - Receive Accel/Gyro raw values through RF433, and reconstruct the original data
 *
 *  TODO:
 *      [-] Raise an Alert when data received
 *      [-] Calculate Heading
 *      [-] Calcualte Velocity
 **/


/** ==========[ LIBRARIES ]========== **/
#include <RH_ASK.h>
#include <SPI.h>

// 433 MHz RF object
RH_ASK driver;

// MPU6050 Variables
uint8_t accrot_bin[13];
int16_t accrot[6];

// LED Indicators
const int ledRx = 6;
const int ledTx = 2;

// RF Data Variables
long dataTx = 0;
bool flagRx = false;
bool initFl = false;
uint8_t countRx = 0;

// Timers & Delay Variables
long int timerRx = 0;
long int delayRx = 1;


/** ==========[ SETUP ]========== **
 *	Initial Setup for Arduino & MPU6050 & RF433
 */
void setup() {
    Serial.begin(115200);
    if (!driver.init())
        Serial.println("[ERROR] RF433 Initialization Failed!");
    else
        Serial.println("[INFO] RF433 Initialized!");

    pinMode(ledRx, OUTPUT);
    pinMode(ledTx, OUTPUT);
}

/** ==========[ LOOP ]========== **
 * 	Main Loop for Arduino
 */
void loop() {
    if (timerRx >= 30000) 
        initFl = false;
    getRx();
    parseMPU();
}

/** ==========[ RAISE ALERT ]========== **
 *  Process data, and then raise an alert based on the data
 */
 void alert() {
    Serial.println("");
 }


/** ==========[ GET RX ]========== **
 *	Listen for signals on the 433MHz channel receiver
 *  Print the received data, and the timestamp
 *  Transmit the reflection signal on the 433MHz transmitter
 */
void getRx() {
    // Save current timestamp into the timerRx variable    
    uint8_t buflen = sizeof(accrot_bin);
    // Receive data: [ax_h, ax_l, ay_h, ay_l, az_h, az_l, gx_h, gx_l, gy_h, gy_l, gz_h, gz_l]
    if (driver.recv((uint8_t *)accrot_bin, &buflen)) {
        if(!initFl)
            countRx = (uint8_t)accrot_bin[12];
        else
            countRx++;
            
        initFl = true;
        digitalWrite(ledRx, HIGH);
        timerRx = millis();
        
        // Print the received data
        Serial.println("==================================================");
        Serial.print("timerRx: "); Serial.println(timerRx);
        Serial.print("dataRx:  ");
        for (int i = 0; i < 6; i++) { Serial.print(accrot_bin[2 * i]); Serial.print("|"); Serial.print(accrot_bin[2 * i + 1]); Serial.print(" , "); }
        Serial.println();

        flagRx = true;
        setTx();
        digitalWrite(ledRx, LOW);
    }
}

/** ==========[ SET TX ]========== **
 *	Send data on the 433MHz channel through the transmitter
 *  Blink TxLED
 *  Print the sent data, and the timestamp
 */
void setTx() {
    digitalWrite(ledTx, HIGH);

    dataTx = countRx;
    driver.send((uint8_t *)&dataTx, sizeof(dataTx));
    driver.waitPacketSent();

    Serial.print("dataTx:  "); Serial.println(dataTx);
    Serial.println();

    digitalWrite(ledTx, LOW);
}

/** ==========[ PARSE MPU ]========== **
 *  Parse the received data
 *  Combine the HIGH and LOW uint8_t bytes into int16_t data and save it into an array using the following
 *      (int16_t) ax = (ax_h << 8) | ax_l ...
 */
void parseMPU() {
    if (flagRx) {
        for(int i=0; i<sizeof(accrot); i++) {
            accrot[i] = (int16_t)((accrot_bin[2*i] << 8) | accrot_bin[2*i+1]);
        }

        PrintData();
    }
    flagRx = false;
}

/** ==========[ PRINT DATA ]========== **
 * Prints the Data for the corresponding IMU
 * format: "acc: ax, ay, az | gx, gy, gz"
 */
void PrintData() {
    // Print Acceleration -- X, Y, Z  [m/s^2]
    Serial.print(F("acc:\t")); Serial.print(accrot[0]/16384.0);
    Serial.print(F(",\t")); Serial.print(accrot[1]/16384.0);
    Serial.print(F(",\t")); Serial.print(accrot[2]/16384.0);
    Serial.print(F("\t|\t"));

    // Print Gyro -- X, Y, Z  [rad/s]
    Serial.print(F("rot:\t")); Serial.print(accrot[3]/131.0);
    Serial.print(F(",\t")); Serial.print(accrot[4]/131.0);
    Serial.print(F(",\t")); Serial.println(accrot[5]/131.0);
    Serial.println();
}