// IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// Kalman Filter
#include <KalmanSensorFusionSigmaBLEAz.h>
#include <math.h>
// Bluetooth
#include <SPI.h>
#include <BLEPeripheral.h>

#define N 5

////////////////////////////////////////////////////// Position
float XY_IR[2], *XY_IMU_IR;
float X = 1, Y = 1;           // initial position

////////////////////////////////////////////////////// Kalman Filtering
float r1 = 32.0, r2 = 2.0, R = 1.2;
float r_array[] = {r1, r1, r1, r1, r1};
KalmanSensorFusion myFilter(0.125,1023.0,X,Y);  //suggested initial values for high noise filtering 

////////////////////////////////////////////////////// IMU - BNO055
Adafruit_BNO055 bno = Adafruit_BNO055();
const uint8_t calibData[] = {12,  0,  229,  255,  32,  0,  117,  0,  87,  254,  143,  255,  253,  255,  255,  255,  0,  0,  232,  3,  166,  2};
float sumAccelx, sumAccely;
int countAccelx = 0, countAccely = 0;

float px = X, py = Y, px_array[5], py_array[5];         //current position
float currAccelx, currAccely, currAccelz;           //current acceleration
float prevAccx, prevAccy, prevAccz;                 //previous acceleration
float currVelx, currVely, currVelz;                 //current velocity 
float prevVelx, prevVely, prevVelz;                 //previous velocity
unsigned long prevTime = millis(), currTime, t1, t2;
int c1;

////////////////////////////////////////////////////// Bluetooth - nRF8001
//custom boards may override default pin definitions with BLEPeripheral(PIN_REQ, PIN_RDY, PIN_RST)
BLEPeripheral blePeripheral = BLEPeripheral();
// create service
BLEService testService = BLEService("fff0");
// create counter characteristic
BLEIntCharacteristic xCharacteristic  = BLEIntCharacteristic("fff1", BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify /*| BLEIndicate*/);
BLEIntCharacteristic yCharacteristic  = BLEIntCharacteristic("fff2", BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify /*| BLEIndicate*/);
//BLECharacteristic testCharacteristic  = BLECharacteristic("fff1", BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify, 2);
// create user description descriptor for characteristic
BLEDescriptor testDescriptor = BLEDescriptor("2901", "counter");
// last counter update time
unsigned long long lastSent = 0;

////////////////////////////////////////////////////// Serial - Arduino: Uno -> Zero
char buff1[10], buff2[10];


void poll_IMU() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    
    currAccelx = euler.x(); //+ 0.37; 
    currAccely = euler.y(); //- 0.17;

    currTime = millis();
    
    if((currAccelx < 0.40) && (currAccelx > -0.40)){
      currAccelx = 0.0;
    }
    if((currAccely < 0.40) && (currAccely > -0.40)){
      currAccely = 0.0;
    }

    // Keeping track of all accelerations until IR update
    sumAccelx = sumAccelx + currAccelx; sumAccely = sumAccely + currAccely;
    countAccelx++; countAccely++;    
    
    pos_calculation_prev_pos();
}

void pos_calculation_prev_pos() {
    //Calculating velocity 
    currVelx = (currAccelx)*(currTime - prevTime)/1000; //1000 added to get the same units
    currVely = (currAccely)*(currTime - prevTime)/1000;
    
    //Serial.println("px: " + String(px));
    //Calculating position 
    px = px + (currVelx)*(currTime - prevTime)/1000;
    py = py + (currVely)*(currTime - prevTime)/1000;
    //Serial.println("px: " + String(px));

    if(px < 0.0) px = 0.0;
    if(py < 0.0) py = 0.0;
    
    // Update new position
    px_array[c1] = px; py_array[c1] = py;

    //Storing previous acceleration, velocity & position
    //prevAccx = currAccelx; prevAccy = currAccely; 
    //prevVelx = currVelx; prevVely = currVely; 
    prevTime = currTime;
}

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}


void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);

  //////////////////////////////////////////////////////
  // SETUP BLUETOOTH CONNECTION
  #if defined (__AVR_ATmega32U4__)
  delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
  #endif

  blePeripheral.setLocalName("test");
  #if 1
  blePeripheral.setAdvertisedServiceUuid(testService.uuid());
  #else
  const char manufacturerData[4] = {0x12, 0x34, 0x56, 0x78};
  blePeripheral.setManufacturerData(manufacturerData, sizeof(manufacturerData));
  #endif

  // set device name and appearance
  blePeripheral.setDeviceName("Test");
  blePeripheral.setAppearance(0x0080);

  // add service, characteristic, and decriptor to peripheral
  blePeripheral.addAttribute(testService);
  blePeripheral.addAttribute(xCharacteristic);
  blePeripheral.addAttribute(yCharacteristic);
  //blePeripheral.addAttribute(testDescriptor);

  // set initial value for characteristic
  xCharacteristic.setValue(0);
  yCharacteristic.setValue(0);

  // begin initialization
  blePeripheral.begin();
  
  //////////////////////////////////////////////////////
  // SETUP IMU BNO055
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //delay(1000);

  //bno.setSensorOffsets(calibData);
  displaySensorStatus(); 
  bno.setExtCrystalUse(true);

  
}

int count1 = 0;
void loop() {
  
  BLECentral central = blePeripheral.central();

  if (central) {

    // reset counter value
    xCharacteristic.setValue(0);
    yCharacteristic.setValue(0);

    while (central.connected()) {         // TOP OF MY LOOP !

      // POLL IMU ////////////////////////////////////////////
      currTime = millis();
      
      if( (currTime - prevTime) > 10 ) {
        poll_IMU();     // poll IMU @10Hz
        c1++;
      }

      // POLL IR  ////////////////////////////////////////////
      if (Serial1.available() > 8) {
          if( (Serial1.peek() == '\0') && (count1 == 0) ) { // To Sync Properly with serial buffer
          
            Serial1.readBytes(buff1,8);
            //Serial1.flush()
          } else {
            
            Serial1.readBytes(buff1,4);
            Serial1.readBytes(buff2,4);

            //Serial1.flush() 
            
            Serial.print("IR POLL "); Serial.println(count1);
            Serial.print(buff1); Serial.print(" "); Serial.println(buff2);
            XY_IR[0] = atof(buff1)/1000;
            XY_IR[1] = atof(buff2)/1000;
            count1++;

            //sumAccelx = sumAccelx/countAccelx; sumAccely = sumAccely/countAccely;
            
            XY_IMU_IR = myFilter.getLinearFiltered_IR(XY_IR, r2, sumAccelx, sumAccely);
            px = XY_IMU_IR[0];
            py = XY_IMU_IR[1];

            //Serial.println("**SumAcc  X: " + String(sumAccelx) + " SumAcc  Y: " + String(sumAccely));
            Serial.println("**XY_IR   X: " + String(XY_IR[0]) + " XY_IR   Y: " + String(XY_IR[1]));
            Serial.println("**IR FILT X: " + String(px) + " IR DATA Y: " + String(py) + "\n\n");
            
            sumAccelx = 0.0; sumAccely = 0.0; countAccelx = 0; countAccely = 0;
            
            //r_array[0] = r1; r_array[1] = r1; r_array[2] = r1; r_array[3] = r1; r_array[4] = r1; 
          }
      }
      
      
      if(c1 == N){
        c1 = 0;

        //increase IMU error over each cycle
        //r_array[0] = r_array[0]*R; r_array[1] = r_array[1]*R; r_array[2] = r_array[2]*R; r_array[3] = r_array[3]*R; r_array[4] = r_array[4]*R;
        // SIGMA FILTER
        XY_IMU_IR = myFilter.getSigmaFiltered_IMU(px_array, py_array, r_array);
        px = XY_IMU_IR[0];
        py = XY_IMU_IR[1];
        
      }
      
      // central still connected to peripheral
      if (xCharacteristic.written()) {

        // reset counter value
        lastSent = 0;
        xCharacteristic.setValue(0);
      }

      // central still connected to peripheral
      if (yCharacteristic.written()) {

        // reset counter value
        lastSent = 0;
        yCharacteristic.setValue(0);
      }

      if (millis() > 500 && (millis() - 500) > lastSent) {
        // atleast one second has passed since last increment
        lastSent = millis();      
        
        //if(XY_IMU_IR[0] < 0) XY_IMU_IR[0] = 0.0;
        //else if(XY_IMU_IR[0] > 2) XY_IMU_IR[0] = 2.0;
        //if(XY_IMU_IR[1] < 0) XY_IMU_IR[1] = 0.0;
        //else if(XY_IMU_IR[1] > 2) XY_IMU_IR[1] = 2.0;

        Serial.println("currAccelx: " + String(currAccelx) +   "  currAccely: " + String(currAccely));
        Serial.println("px[0]: " + String(px_array[0]) + " px[1]: " + String(px_array[1]) + " px[2]: " + String(px_array[2]) + " px[3]: " + String(px_array[3]) + " px[4]: " + String(px_array[4]));
        Serial.println("py[0]: " + String(py_array[0]) + " py[1]: " + String(py_array[1]) + " py[2]: " + String(py_array[2]) + " py[3]: " + String(py_array[3]) + " py[4]: " + String(py_array[4]));
        Serial.print("FILT X:     "); Serial.print(XY_IMU_IR[0]); Serial.print("  FILT Y:     "); Serial.print(XY_IMU_IR[1]); Serial.println("\n\n");
        
        xCharacteristic.setValue((int)(XY_IMU_IR[0]*1000));
        yCharacteristic.setValue((int)(XY_IMU_IR[1]*1000));
        
      }
    }
  
  }
}

