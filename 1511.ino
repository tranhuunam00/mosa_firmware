#include <bluefruit.h>
#include <Adafruit_CircuitPlayground.h>
#include <SimpleKalmanFilter.h>

/* Accelerometer Service Definitions
 * UUID16_SVC_INDOOR_POSITIONING:  0x1821
 * UUID16_UNIT_ACCELERATION_METRES_PER_SECOND_SQUARED: 0x2713
 * UUID16_UNIT_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED:   0x2744
 */
BLEService        positionService = BLEService(UUID16_SVC_INDOOR_POSITIONING);
BLECharacteristic accelerometerCharacter = BLECharacteristic(UUID16_UNIT_ACCELERATION_METRES_PER_SECOND_SQUARED);
BLECharacteristic gyroscopeCharacter = BLECharacteristic(UUID16_UNIT_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

float X, Y, Z;
int xNew,yNew,zNew;
int absX,absY,absZ;
uint8_t  bps = 0;


SimpleKalmanFilter bo_locx(1, 1, 0.5);
SimpleKalmanFilter bo_locy(1, 1, 0.5);
SimpleKalmanFilter bo_locz(1, 1, 0.5);
SimpleKalmanFilter bo_loc(1, 1, 0.5);
void setup()
{
  Serial.begin(115200);

//  Serial.println("Bluefruit52 acce Example");
//  Serial.println("-----------------------\n");

  // Initialise the Bluefruit module
//  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
//  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
//  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

 
//  Serial.println("Configuring the Heart Rate Monitor Service");
  setupPosition();

  // Setup the advertising packet(s)
//  Serial.println("Setting up the advertising payload(s)");
  startAdv();

//  Serial.println("Ready Player One!!!");
//  Serial.println("\nAdvertising");
  CircuitPlayground.begin();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(positionService);

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void setupPosition(void)
{
 
  positionService.begin();

  accelerometerCharacter.setProperties(CHR_PROPS_NOTIFY+CHR_PROPS_READ+CHR_PROPS_WRITE );
  accelerometerCharacter.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelerometerCharacter.setFixedLen(9);
  accelerometerCharacter.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  accelerometerCharacter.begin();
  uint8_t accelerometerData[9] = { 0b00000000, 0b00000000, 0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000 }; 
  // Set the characteristic to use 8-bit values, with the sensor connected and detected
  accelerometerCharacter.write(accelerometerData, 9);


  gyroscopeCharacter.setProperties(CHR_PROPS_READ);
  gyroscopeCharacter.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  gyroscopeCharacter.setFixedLen(1);
  gyroscopeCharacter.begin();
  gyroscopeCharacter.write8(2);    // Set the characteristic to 'Wrist' (2)
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

//  Serial.print("Connected to ");
//  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
//
//  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
//  Serial.println("Advertising!");
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
//    Serial.print("CCCD Updated: ");
//    //Serial.printBuffer(request->data, request->len);
//    Serial.print(cccd_value);
//    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == accelerometerCharacter.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
//            Serial.println("Acce Measurement 'Notify' enabled");
        } else {
//            Serial.println("Acce Measurement 'Notify' disabled");
        }
    }
}

uint8_t convertData(int data){
  uint8_t res = 0;
  res = res + abs(data);
  return res;
}

uint8_t check0(int data){
  
  uint8_t res = 1;
  if(data<0) res = 0;
  
  return res;
}


void loop()
{
  digitalToggle(LED_RED);
  
  if ( Bluefruit.connected() ) {
     X = CircuitPlayground.motionX();
     Y = CircuitPlayground.motionY();
     Z = CircuitPlayground.motionZ();
     
     float X_filter,Y_filter,Z_filter;
    
     X_filter = bo_locx.updateEstimate(X);
     Y_filter = bo_locy.updateEstimate(Y);
     Z_filter = bo_locz.updateEstimate(Z);
//     Serial.print(Z);
//     Serial.print(",");
//     Serial.print(Z_filter);
//     Serial.println();
     
     xNew=(int)(X_filter*1000);
     yNew=(int)(Y_filter*1000);
     zNew=(int)(Z_filter*1000);
     absX=abs(xNew);
     absY=abs(yNew);
     absZ=abs(zNew);
     
    uint8_t  accelerometerData[9] = {
      check0(xNew),
      convertData(absX/100),
      convertData(absX - absX / 100*100),
      check0(yNew),
      convertData(absY / 100 ),
      convertData(absY-absY /100*100),
      check0(zNew),
      convertData(absZ/100),
      convertData(absZ-absZ /100*100),
    };           // Sensor connected, increment BPS value
    
    // Note: We use .notify instead of .write!
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if ( accelerometerCharacter.notify(accelerometerData, sizeof(accelerometerData)) ){
//      Serial.print("Accelerometer Data X "); Serial.println(X*1000); 
//      Serial.print("Accelerometer Data Y"); Serial.println(Y*1000); 
//      Serial.print("Accelerometer Data Z"); Serial.println(Z*1000); 
    }else{
//      Serial.println("ERROR: Notify not set in the CCCD or not connected!");
    }
  }

  // Only send update once per second
  delay(100);
}
