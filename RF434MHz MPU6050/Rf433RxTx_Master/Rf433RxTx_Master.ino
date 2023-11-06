/**
 *  Project: RF 433Mhz Tranceiver [Master]
 *
 *  Author          : Ayush Chinmay
 *  Date Created    : 22 Sept 2023
 *  Date Modified   : 05 Oct 2023
 *
 *  NOTES:
 *      - ACCEL {2: 16384.0 | 4: 8192.0 | 8: 4096.0 | 16: 2048.0}
 *      - GYRO {250: 131.0 | 500: 65.5 | 1000: 32.8 | 2000: 16.4}
 *      -
 *      - 
 *
 *  CHANGELOG:
 *      [22 Sept 2023]
 *          - Transmission and Reception functionality
 *          - Transmitter sends current clock (in us) as long int
 *
 *      [24 Sept 2023]
 *          - Round - trip communication
 *
 *      [05 Oct 2023]
 *          - Documentation and general code clean-up
 *          - Added MPU6050 Setup and Accel / Gyro measurement capabilities
 *          - Save raw values as HIGH and LOW bytes, which can be sent as a uint8_t array[12]
 *          - Transmit the array that can be received by the 'slave' unit
 *  
 *  TODO:
 *      [-] Clean up code, and work on the slave side to process the raw data
 **/


/** ==========[ LIBRARIES ]========== **/
#include "RH_ASK.h"
#include "SPI.h"

#include "MPU6050_6Axis_MotionApps612.h"
#include "I2Cdev.h"

// 433 MHz RF object
RH_ASK driver;
// MPU6050 object
MPU6050 mpu;

// ------[ MPU6050 VARIABLES ]------
int16_t accrot[6];
uint8_t accrot_bin[13];

/// DMP Variables
uint16_t packet_size;     /// DMP Packet Size
uint16_t fifo_count;      /// FIFO Count
uint8_t fifo_buffer[64];  /// FIFO Buffer

/// Interrupt Variables
bool mpu_interrupt;   /// MPU Interrupt
uint16_t fifo_alive;  /// FIFO Alive

// ------[ OTHER VARIABLES ]------
// LED Indicators
const int ledRx = 6;
const int ledTx = 2;

// RF Data Variables
long dataRx = 0;
bool flagRx = false;
uint8_t countTx = 0;
uint8_t countRx = 0;

// Timers & Delay Variables
unsigned long lastTx = 0;
unsigned long timerTx = 0;
unsigned long delayTx = 750;

// Radio State Variable
enum State { TRANSMIT, LISTEN};
State currentState = TRANSMIT;

/** ==========[ SETUP ]========== **
 *	Initial Setup for Arduino & MPU6050 & RF433
 */
void setup() {
    // Initialize Serial and I2C
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MPU6050 and RF433
    initMPU();
    initRF();
}

/** ==========[ LOOP ]========== **
 * 	Main Loop for Arduino
 */
void loop() {
    // GetDMPInterrupt();
    // GetMPUData();

    // unsigned long currentTime = millis();
    // if (currentTime - lastTx >= delayTx) {
    //     setTx();
    //     lastTx = currentTime;
    // }

    // getRx();

    switch(currentState) {
        case TRANSMIT:
            if(millis() - lastTx >= delayTx) {
                GetMPUData();
                setTx();
                currentState = LISTEN;
                lastTx = millis();
            }
            break;
        case LISTEN:
            getRx();
            currentState = TRANSMIT;
            Serial.print("Tx: "); Serial.print(countTx); Serial.print(" | Rx: "); Serial.print(countRx);
            Serial.print(" | Loss: "); Serial.print(100.0*((countTx-countRx)/(float)countTx)); Serial.println("%");
            break;
    }
}


/** ==========[ INITIALIZE MPU ]========== **
 *	Initialize MPU6050 settings
 *  Set Accel/Gyro Range
 *  Calibrate Accel/Gyro
 */
void initMPU() {
    Serial.print(F("[INFO] Initializing MPU..."));
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.CalibrateGyro(3);
    mpu.CalibrateAccel(3);

    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    packet_size = mpu.dmpGetFIFOPacketSize();
    fifo_count = mpu.getFIFOCount();
    Serial.println(F("... Calibrated!"));
}

/** ==========[ INITIALIZE RF ]========== **
 *	Initialize RF433 driver settings
 */
void initRF() {
    if (!driver.init())
        Serial.println("[ERROR] RF433 Initialization Failed!");
    else
        Serial.println("[INFO] RF433 Initialized!");

    pinMode(ledRx, OUTPUT);
    pinMode(ledTx, OUTPUT);
}

/** ==========[ SET TX ]========== **
 *	Send data on the 433MHz channel through the transmitter
 *  Blink TxLED
 *  Print the sent data, and the timestamp
 */
void setTx() {
    digitalWrite(ledTx, HIGH);
    countTx++;
    // Update timerTx timestamp, and sent the MPU6050 data array
    accrot_bin[12] = countTx;
    timerTx = millis();
    driver.send((uint8_t *)&accrot_bin, sizeof(accrot_bin));
    driver.waitPacketSent();

    // Print the data that is being transmitted
    Serial.println("==================================================");
    Serial.print("timerTx: "); Serial.println(timerTx);
    Serial.print("dataTx:  ");
    for (int i = 0; i < 6; i++) { Serial.print(accrot_bin[2 * i]); Serial.print("|"); Serial.print(accrot_bin[2 * i + 1]); Serial.print(" , "); }
    Serial.println();

    digitalWrite(ledTx, LOW);
}

/** ==========[ GET RX ]========== **
 *	Listen for signals on the 433MHz channel receiver
 *  if data received, set flagRX to True
 *  Blink RxLED
 *  Print the received data, and the timestamp
 */
void getRx() {
    if (driver.recv((uint8_t *)&dataRx, sizeof(dataRx))) {
        unsigned long timerRx = millis();
        countRx++;
        digitalWrite(ledRx, HIGH);
        Serial.print("dataRx:  "); Serial.println((long)dataRx);
        Serial.print("timerRx: "); Serial.println(timerRx);
        Serial.print("Round Trip: "); Serial.print(timerRx - timerTx); Serial.println(" ms");
        Serial.println();
    }

    digitalWrite(ledRx, LOW);
}

/** ==========[ SET DMP INTERRUPT ]========== **
 *	Set DMP Interrupt for the MPU6050
 *  If interrupt is set, get DMP data
 */
void GetDMPInterrupt() {
    static unsigned long _ETimer;

    if (millis() - _ETimer >= (10)) {  /// After 10ms, enable MPU Interrupt
        _ETimer += (10);                 /// Increment Timer by 10ms
        mpu_interrupt = true;            /// Set MPU Interrupt to true
    }
    if (mpu_interrupt) {  /// If MPU Interrupt is true
        GetDMP();           /// Get DMP Data
    }
}

/** ==========[ GET DMP ]========== **
 *	Get DMP Data from MPU6050
 *	If FIFO Count is 0 or not a multiple of packet size, reset FIFO
 *	Else, Proceed with Calculations
 */
void GetDMP() {
    // Serial.print("[INFO] Fetching DMP... ");

    /// Local Variables -- FIFO Count, Packet Size, Last Good Packet Time
    static unsigned long LastGoodPacketTime;
    mpu_interrupt = false;
    fifo_alive = 1;
    fifo_count = mpu.getFIFOCount();

    /// If FIFO Count is 0 or not a multiple of packet size
    if ((!fifo_count) || (fifo_count % packet_size)) {
        // Serial.print(F("[ERROR] FIFO COUNT {")); Serial.print(fifo_count); Serial.print("} PACKET SIZE {");
        // Serial.print(packet_size); Serial.println(F("} FIFO Count is 0 or not a multiple of packet size!"));
        mpu.resetFIFO();  /// Failed to fetch DMP, reset FIFO
    } else {
        while (fifo_count >= packet_size) {            /// While FIFO Count is greater than packet size
            mpu.getFIFOBytes(fifo_buffer, packet_size);  /// Get latest packet
            fifo_count -= packet_size;                   /// Update FIFO Count
        }
        LastGoodPacketTime = millis();
        GetMPUData();  /// On Success, Calculate MPU Math
    }
}

/** ========== [ MPU MATH ] ========== **
 * 	Get Acceleration and Gyroscope raw values and save it into an array: [ax, ay, az, gx, gy, gz]
 *  Split raw data into HIGH and LOW bytes that can be transmitted over RF
 */
void GetMPUData() {
    // Get RAW Values from Acceleration and Gyroscope registers
    mpu.getMotion6(&accrot[0], &accrot[1], &accrot[2], &accrot[3], &accrot[4], &accrot[5]);

    // deconstruct raw data into HIGH & LOW bytes:
    //      (uint8_t) [ax_h, ax_l, ay_h, ay_l, az_h, az_l, gx_h, gx_l, gy_h, gy_l, gz_h, gz_l]
    for (int i = 0; i < sizeof(accrot); i++) {
        accrot_bin[i * 2] = (uint8_t)((accrot[i] >> 8) & 0xFF);  // High Byte
        accrot_bin[i * 2 + 1] = (uint8_t)(accrot[i] & 0xFF);     // Low Byte
    }

    //   PrintData();
}

/** ==========[ PRINT DATA ]========== **
 * Prints the Data for the corresponding IMU
 * format: "acc: ax, ay, az | gx, gy, gz"
 */
void PrintData() {
    // Print Acceleration -- X, Y, Z  [m/s^2]
    Serial.print(F("acc:\t"));
    Serial.print(accrot[0] / 16384.0);
    Serial.print(F(",\t"));
    Serial.print(accrot[1] / 16384.0);
    Serial.print(F(",\t"));
    Serial.print(accrot[2] / 16384.0);
    Serial.print(F("\t|\t"));

    // Print Gyro -- X, Y, Z  [rad/s]
    Serial.print(F("rot:\t"));
    Serial.print(accrot[3] / 131.0);
    Serial.print(F(",\t"));
    Serial.print(accrot[4] / 131.0);
    Serial.print(F(",\t"));
    Serial.println(accrot[5] / 131.0);
    Serial.println();
}