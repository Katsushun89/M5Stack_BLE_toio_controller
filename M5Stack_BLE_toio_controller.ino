#include <M5Stack.h>
#include <BLEDevice.h>

// toio service
static BLEUUID service_UUID("10B20100-5B3B-4571-9508-CF3EFCD7BBAE");
// read sensor characteristic
static BLEUUID read_char_UUID("10B20101-5B3B-4571-9508-CF3EFCD7BBAE");
// motor characteristic
static BLEUUID motor_char_UUID("10B20102-5B3B-4571-9508-CF3EFCD7BBAE");
// sound characteristic
static BLEUUID sound_char_UUID("10B20104-5B3B-4571-9508-CF3EFCD7BBAE");

static boolean do_connect = false;
static boolean connected = false;
static boolean do_scan = false;
static BLERemoteCharacteristic* read_characteristic;
static BLERemoteCharacteristic* motor_characteristic;
static BLERemoteCharacteristic* sound_characteristic;
static BLEAdvertisedDevice* my_device;

static bool is_req_move_forward = false;

static uint8_t motor_data_size = 8;
static uint8_t sound_data_size = 3;

static uint8_t current_sound = 0;

enum {
  sound_enter = 0,
  sound_selected,
  sound_cancel,
  sound_cursor,
  sound_mat_in,
  sound_mat_out,
  sound_get_1,
  sound_get_2,
  sound_get_3,
  sound_effect_1,
  sound_effect_2,
  sound_max, //threshold
};

typedef struct {
  uint16_t x_cube_center;
  uint16_t y_cube_center;
  uint16_t angle_cube_center;
  uint16_t x_read_sensor;
  uint16_t y_read_sensor;
  uint16_t angle_read_sensor;
} position_id_t;

typedef struct {
  uint32_t standard_id;
  uint16_t angle_cube;
} standard_id_t;

static position_id_t position_id = {0};
static standard_id_t standard_id = {0};
static bool is_missed_position_id = false;
static bool is_missed_standard_id = false;

static void readPositionID(uint8_t *data, size_t length)
{
  memcpy(&position_id, &data[1], sizeof(position_id));
}

static void readStandardID(uint8_t *data, size_t length)
{
  memcpy(&standard_id, &data[1], sizeof(standard_id));
}

static void readPositionIDMissed(uint8_t *data, size_t length)
{
  is_missed_position_id = true;
  position_id.x_cube_center = 0;
  position_id.y_cube_center = 0;
  position_id.angle_cube_center = 0;
  position_id.x_read_sensor = 0;
  position_id.y_read_sensor = 0;
  position_id.angle_read_sensor = 0;
}

static void readStandardIDMissed(uint8_t *data, size_t length)
{
  is_missed_standard_id = true;
  standard_id.standard_id = 0;
  standard_id.angle_cube = 0;
}

void (*readFunctionTable[])(uint8_t *data, size_t length) = {
    &readPositionID,
    &readStandardID,
    &readPositionIDMissed,
    &readStandardIDMissed
};

static void selectReadFunction(uint8_t *data, size_t length)
{
  uint8_t id = data[0];
  Serial.printf("read id:%d\n", id);
  //need to size check
  readFunctionTable[id - 1](data, length);
}

static void notifyReadCallback(BLERemoteCharacteristic* ble_remote_characteristic,
                            uint8_t* data,
                            size_t length,
                            bool is_notify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(ble_remote_characteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    for(uint8_t i = 0; i < length; i++){
      Serial.printf("%x ", data[i]);
    }
    Serial.println("");

    selectReadFunction(data, length);

}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
  }

  void onDisconnect(BLEClient* client) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(my_device->getAddress().toString().c_str());
    
    BLEClient*  client  = BLEDevice::createClient();
    Serial.println(" - Created client");

    client->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    client->connect(my_device);  // if you pass BLEadvertised_device instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* remote_service = client->getService(service_UUID);
    if (remote_service == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(service_UUID.toString().c_str());
      client->disconnect();
      return false;
    }
    Serial.println(" - Found toio service");

    //read
    read_characteristic = remote_service->getCharacteristic(read_char_UUID);
    if (read_characteristic == nullptr) {
      Serial.print("Failed to find read characteristic UUID: ");
      Serial.println(read_char_UUID.toString().c_str());
      client->disconnect();
      return false;
    }
    Serial.println(" - Found read characteristic");

    // Read the value of the characteristic.
    if(read_characteristic->canRead()) {
      std::string value = read_characteristic->readValue();
      Serial.print("read characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(read_characteristic->canNotify())
      read_characteristic->registerForNotify(notifyReadCallback);


    //motor
    motor_characteristic = remote_service->getCharacteristic(motor_char_UUID);
    if (motor_characteristic == nullptr) {
      Serial.print("Failed to find motor characteristic UUID: ");
      Serial.println(motor_char_UUID.toString().c_str());
      client->disconnect();
      return false;
    }
    Serial.println(" - Found motor characteristic");

    //sound
    sound_characteristic = remote_service->getCharacteristic(sound_char_UUID);
    if (sound_characteristic == nullptr) {
      Serial.print("Failed to find sound characteristic UUID: ");
      Serial.println(sound_char_UUID.toString().c_str());
      client->disconnect();
      return false;
    }
    Serial.println(" - Found sound characteristic");


    connected = true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertised_device) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertised_device.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertised_device.haveServiceUUID() && advertised_device.isAdvertisingService(service_UUID)) {

      BLEDevice::getScan()->stop();
      my_device = new BLEAdvertisedDevice(advertised_device);
      do_connect = true;
      do_scan = true;

    } // Found our server
  } // onResult
}; // Myadvertised_deviceCallbacks


void setup() {
  M5.begin();
  M5.Lcd.fillScreen(0x0000);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(0xFFFF, 0x0000);

  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* ble_scan = BLEDevice::getScan();
  ble_scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  ble_scan->setInterval(1349);
  ble_scan->setWindow(449);
  ble_scan->setActiveScan(true);
  ble_scan->start(5, false);
} // End of setup.

static String getSoundStr(void)
{
  String sound_str;
  switch(current_sound){
  case sound_enter:
    sound_str = "Enter   ";
    break;
  case sound_selected:
    sound_str = "Selected";
    break;
  case sound_cancel:
    sound_str = "Cancel  ";
    break;
  case sound_cursor:
    sound_str = "Cursor  ";
    break;
  case sound_mat_in:
    sound_str = "Mat in  ";
    break;
  case sound_mat_out:
    sound_str = "Mat out ";
    break;
  case sound_get_1:
    sound_str = "Get 1   ";
    break;
  case sound_get_2:
    sound_str = "Get 2   ";
    break;
  case sound_get_3:
    sound_str = "Get 3   ";
    break;
  case sound_effect_1:
    sound_str = "Effect 1";
    break;
  case sound_effect_2:
    sound_str = "Effect 2";
    break;
  }
  return sound_str;
}

static void checkPushButton(void)
{
  static bool is_long_pressed = false;
  is_long_pressed = M5.BtnA.pressedFor(100);
  if(is_long_pressed){
    M5.Lcd.drawString("MOVE", 0, 180);
    is_req_move_forward = true;
  }else{
    M5.Lcd.drawString("STOP", 0, 180);
    is_req_move_forward = false;
  }

  if(M5.BtnB.wasPressed()){
    sendSoundControl();
    current_sound++;
    if(current_sound >= sound_max) current_sound = sound_enter;
  }
  M5.Lcd.drawString(getSoundStr(), 160, 180);
}

static void sendMotorControl(void)
{
  uint8_t data[motor_data_size] = {0};
  data[0] = 0x02; //motor control with specifiled time
  data[1] = 0x01; //motor id 1 : left
  data[2] = 0x01; //direction
  data[3] = (is_req_move_forward ? 0x1E : 0x0); //speed
  data[4] = 0x02; //motor id 2 : right
  data[5] = 0x01; //direction
  data[6] = (is_req_move_forward ? 0x1E : 0x0); //speed
  data[7] = 0x0A; //control time
  //Serial.println("write value");

  motor_characteristic->writeValue(data, sizeof(data));
}

static void sendSoundControl(void)
{
  uint8_t data[sound_data_size] = {0};
  data[0] = 0x02; //0x02:sound effect
  data[1] = current_sound; //sound effects
  data[2] = 0x10; //volume
  //Serial.println("write value");

  sound_characteristic->writeValue(data, sizeof(data));
}

static void drawReadSensor(void)
{
  String x_cube_str = String("cube x  ") + String(position_id.x_cube_center) + String("     ");
  M5.Lcd.drawString(x_cube_str, 0, 0);
  String y_cube_str = String("cube y  ") + String(position_id.y_cube_center) + String("     ");
  M5.Lcd.drawString(y_cube_str, 0, 35);
  String angle_cube_str = String("cube angle  ") + String(position_id.angle_cube_center) + String("     ");
  M5.Lcd.drawString(angle_cube_str, 0, 70);
  String standard_id_str = String("std id  ") + String(standard_id.standard_id)+ String("              ");
  M5.Lcd.drawString(standard_id_str, 0, 105);
  String standard_id_angle_str = String("std angle  ") + String(standard_id.angle_cube) + String("    ");
  M5.Lcd.drawString(standard_id_angle_str, 0, 140);
}


// This is the Arduino main loop function.
void loop() {
  
  M5.update();

  // If the flag "do_connect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (do_connect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    do_connect = false;
  }


  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    checkPushButton();
    sendMotorControl();
    drawReadSensor();
  }else if(do_scan){
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(10); // Delay a second between loops.
} // End of loop
