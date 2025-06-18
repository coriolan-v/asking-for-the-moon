

#include <bluefruit.h>

int mode = 0;


// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;


void setup(void) 
{
  Serial.begin(115200);

 // while (!Serial)  {};

  Serial.println(F("ASKING FOR THE MOON"));
  Serial.println(F("-------------------------------------------"));

  initBLE();
}



void loop(void) 
{
  readBLE();
}



// Packet buffer
extern uint8_t packetbuffer[];

void initBLE() {
  Bluefruit.setName("AskingForTheMoon");
  Bluefruit.begin();
  Bluefruit.setName("AskingForTheMoon");
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

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
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}


void readBLE() {
  // Wait for new data to arrive
  uint8_t len = readPacket(&bleuart, 1);
  if (len == 0) return;

  // Got a packet!
  // printHex(packetbuffer, len);

  // Color
  
  
  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print("Button ");
    Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");

      if (buttnum == 1) {
        mode = 0;  // normal mode
        Serial.println("MODE 0 ACTIVATED");
       // setMaxBrightness(0);
      }
      if (buttnum == 2) {
        mode = 1;  // demo mode, no interaction
        // setMaxBrightness(255);
      }
      if (buttnum == 3) {
        mode = 3;  // fixed color, no interaction (set using color picker)
       //  setMaxBrightness(255);
      }
      if (buttnum == 4) {
        mode = 4;  // normal mode but with fixed color (set using color picker)
         ///setMaxBrightness(255);
        Serial.println("MODE 4 ACTIVATED");
       // setMaxBrightness(0);
       // setColor(red, green, blue);
      }

      if(buttnum == 5){
              Serial.println("🔄 DFU Mode triggered via Control Pad");
              delay(1000);
              enterDFUMode();
      }

      // if (buttnum == 2) ledStatus = 2; // oFF
      // if (buttnum == 3) ledStatus = 3; // CONTINUOUS
    }
  }
}

#include <string.h>
#include <Arduino.h>
#include <bluefruit.h>


#define PACKET_ACC_LEN (15)
#define PACKET_GYRO_LEN (15)
#define PACKET_MAG_LEN (15)
#define PACKET_QUAT_LEN (19)
#define PACKET_BUTTON_LEN (5)
#define PACKET_COLOR_LEN (6)
#define PACKET_LOCATION_LEN (15)

//    READ_BUFSIZE            Size of the read buffer for incoming packets
#define READ_BUFSIZE (20)


/* Buffer to hold incoming characters */
uint8_t packetbuffer[READ_BUFSIZE + 1];

/**************************************************************************/
/*!
    @brief  Casts the four bytes at the specified address to a float
*/
/**************************************************************************/
float parsefloat(uint8_t *buffer) {
  float f;
  memcpy(&f, buffer, 4);
  return f;
}

/**************************************************************************/
/*! 
    @brief  Prints a hexadecimal value in plain characters
    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void printHex(const uint8_t *data, const uint32_t numBytes) {
  uint32_t szPos;
  for (szPos = 0; szPos < numBytes; szPos++) {
    Serial.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF) {
      Serial.print(F("0"));
      Serial.print(data[szPos] & 0xf, HEX);
    } else {
      Serial.print(data[szPos] & 0xff, HEX);
    }
    // Add a trailing space if appropriate
    if ((numBytes > 1) && (szPos != numBytes - 1)) {
      Serial.print(F(" "));
    }
  }
  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Waits for incoming data and parses it
*/
/**************************************************************************/
uint8_t readPacket(BLEUart *ble_uart, uint16_t timeout) {
  uint16_t origtimeout = timeout, replyidx = 0;

  memset(packetbuffer, 0, READ_BUFSIZE);

  while (timeout--) {
    if (replyidx >= 20) break;
    if ((packetbuffer[1] == 'A') && (replyidx == PACKET_ACC_LEN))
      break;
    if ((packetbuffer[1] == 'G') && (replyidx == PACKET_GYRO_LEN))
      break;
    if ((packetbuffer[1] == 'M') && (replyidx == PACKET_MAG_LEN))
      break;
    if ((packetbuffer[1] == 'Q') && (replyidx == PACKET_QUAT_LEN))
      break;
    if ((packetbuffer[1] == 'B') && (replyidx == PACKET_BUTTON_LEN))
      break;
    if ((packetbuffer[1] == 'C') && (replyidx == PACKET_COLOR_LEN))
      break;
    if ((packetbuffer[1] == 'L') && (replyidx == PACKET_LOCATION_LEN))
      break;

    while (ble_uart->available()) {
      char c = ble_uart->read();
      if (c == '!') {
        replyidx = 0;
      }
      packetbuffer[replyidx] = c;
      replyidx++;
      timeout = origtimeout;
    }

    if (timeout == 0) break;
    delay(1);
  }

  packetbuffer[replyidx] = 0;  // null term

  if (!replyidx)  // no data or timeout
    return 0;
  if (packetbuffer[0] != '!')  // doesn't start with '!' packet beginning
    return 0;

  // check checksum!
  uint8_t xsum = 0;
  uint8_t checksum = packetbuffer[replyidx - 1];

  for (uint8_t i = 0; i < replyidx - 1; i++) {
    xsum += packetbuffer[i];
  }
  xsum = ~xsum;

  // Throw an error message if the checksum's don't match
  if (xsum != checksum) {
    Serial.print("Checksum mismatch in packet : ");
    printHex(packetbuffer, replyidx + 1);
    return 0;
  }

  // checksum passed!
  return replyidx;
}


void enterDFUMode() {
  // Tell the bootloader to enter DFU mode on next boot
  NRF_POWER->GPREGRET = 0xB1;
  NVIC_SystemReset();  // Hardware reset into bootloader
}

