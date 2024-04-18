#include <Arduino.h>
#include <chrono>
#include <esp32-hal-log.h>
#include "idd.h"
#include "spi_lib.h"
#include "i2c_lib.h"
//#include "wifi_lib.h"
#include "CommunicatorFactory.h"

//#include "argInitRoutines.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxAPI.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_terminate.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"

#define POWER_ON_PIN        4
#define LED_STATUS          27
#define PAC_POWER_PIN       25
#define RCV_BUFFER_SIZE     512

// const char* wifi_ssid = "EVONET";
// const char* wifi_password = "12345678";
// const char* wifi_hostname = "my-evo-host";
// const uint16_t wifi_port = 4556;
//
// create a instance of the server
// WiFiServer wifi_server(wifi_port);

byte rx_buffer[RCV_BUFFER_SIZE];
byte tx_buffer[RCV_BUFFER_SIZE];

SpiLib spi_lib;
I2CLib i2c_lib;
BaseCommunicator* communicator = nullptr;

// timeout in secs, used to monitor communication 'keep-a-live'
// if no communication received within that period of time, restart will occure
int timeout_restart_sec = 5 * 60; // default value, may be changed by command
std::chrono::system_clock::time_point last_communication_time;

struct0_T r;
struct30_T outStruct;

const int sizeof_struct0_T = sizeof(struct0_T);   //154488
const int sizeof_struct1_T = sizeof(struct1_T);   //24
const int sizeof_struct2_T = sizeof(struct2_T);   //92
const int sizeof_struct3_T = sizeof(struct3_T);   //36280
const int sizeof_struct4_T = sizeof(struct4_T);   //40
const int sizeof_struct5_T = sizeof(struct5_T);   //34188
const int sizeof_struct6_T = sizeof(struct6_T);   //67
const int sizeof_struct7_T = sizeof(struct7_T);   //34116
const int sizeof_struct8_T = sizeof(struct8_T);   //1600
const int sizeof_struct9_T = sizeof(struct9_T);   //768
const int sizeof_struct10_T = sizeof(struct10_T); //104
const int sizeof_struct11_T = sizeof(struct11_T); //76
const int sizeof_struct12_T = sizeof(struct12_T); //2
const int sizeof_struct13_T = sizeof(struct13_T); //40965
const int sizeof_struct14_T = sizeof(struct14_T); //5
const int sizeof_struct15_T = sizeof(struct15_T); //44
const int sizeof_struct16_T = sizeof(struct16_T); //32
const int sizeof_struct17_T = sizeof(struct17_T); //16
const int sizeof_struct18_T = sizeof(struct18_T); //1
const int sizeof_struct19_T = sizeof(struct19_T); //12
const int sizeof_struct20_T = sizeof(struct20_T); //43592
const int sizeof_struct21_T = sizeof(struct21_T); //6400
const int sizeof_struct22_T = sizeof(struct22_T); //4496
const int sizeof_struct23_T = sizeof(struct23_T); //8492
const int sizeof_struct24_T = sizeof(struct24_T); //21520
const int sizeof_struct25_T = sizeof(struct25_T); //20912
const int sizeof_struct26_T = sizeof(struct26_T); //448
const int sizeof_struct27_T = sizeof(struct27_T); //1040
const int sizeof_struct28_T = sizeof(struct28_T); //2328
const int sizeof_struct29_T = sizeof(struct29_T); //24
const int sizeof_struct30_T = sizeof(struct30_T); //24

void argInit_struct0_T(struct0_T *result);
void emxInit_struct30_T(struct30_T *pStruct);

void setup() {
//  log_d("begin setup...");
//Serial.begin(SerialBaudRate);
  communicator = CommunicatorFactory::createCommunicator(DEF_COMMUNICATOR_SERIAL);
  if (communicator == nullptr)
    while(1);

  // serial communication setup
  // Serial.begin(SerialBaudRate);
  // Serial.setTimeout(SerialTimeOutMS);

  // // wifi setup
  // WiFi.onEvent(onWiFiEvent);
  // WiFi.mode(WIFI_AP);
  // WiFi.softAP(wifi_ssid, wifi_password);
  //
  // // multicast DNS
  // MDNS.begin(wifi_hostname);
  // MDNS.addService("http", "tcp", 80);
  //
  // WiFi.setHostname(wifi_hostname);
  // wifi_server.begin(wifi_port);

  // power ON
  pinMode(POWER_ON_PIN, OUTPUT);
  digitalWrite(POWER_ON_PIN, LOW);
  delay(100);
  digitalWrite(POWER_ON_PIN, HIGH);
  delay(10);

  // PAC power ON
  pinMode(PAC_POWER_PIN, OUTPUT);
  digitalWrite(PAC_POWER_PIN, LOW);
  delay(100);
  digitalWrite(PAC_POWER_PIN, HIGH);
  delay(1);

  // SPI library init
  if (spi_lib.Init() == false)
  {
    send_error(communicator, tx_buffer, OPCODE_ERROR_SPI);
  }
  // I2C library init
  if (i2c_lib.Init() == false)
  {
    send_error(communicator, tx_buffer, OPCODE_ERROR_I2C);
  }

  // light the led
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);

  last_communication_time = std::chrono::system_clock::now();

  //  log_d("setup done...");

  emxInit_struct30_T(&outStruct);
//  argInit_struct0_T(&r);
}

void loop() {
//  log_d("start main loop...");

  if (communicator == nullptr)
    return;

// WiFiClient client = wifi_server.available();
// if (client)
// {
//     Serial.println("New client connected");
//     client.println("HTTP/1.1 200 OK");
//     client.println("Content-type:text/html");
//     client.println();
//     client.println("<html><body>");
//     client.println("<h1>Hello from ESP32 in AP mode!</h1>");
//     client.println("</body></html>");
//     delay(10);
//     client.stop();
//     Serial.println("Client disconnected");
//   return;
// }

  auto start = std::chrono::system_clock::now();

  while (communicator->available() == 0)
  {
    // check for timeout
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(start - last_communication_time).count();
    if (duration > timeout_restart_sec)
    {
      // // connection lost for too long, restart
      // esp_restart();
      
      // SPI - all ports OFF
      spi_lib.Reset();

      // power OFF/ON
      pinMode(POWER_ON_PIN, OUTPUT);
      digitalWrite(POWER_ON_PIN, LOW);
      delay(100);
      digitalWrite(POWER_ON_PIN, HIGH);
      delay(100);

      // now configure all ports
      spi_lib.ConfigurePorts();
    }
    else
      return;
  }

  // save last communication time
  last_communication_time = std::chrono::system_clock::now();

  // wait for the serial header
  while (communicator->available() < sizeof(SerialHeader)) delay(1);

  int msg_size = communicator->read(rx_buffer, RCV_BUFFER_SIZE);
  // if not enough data, return 
  if (rx_buffer == nullptr || msg_size < sizeof(SerialHeader))
  {
    return;
  }

  // search for the prefix "CAFE"
  SerialHeader *serial_header = nullptr;
  BatchHeader *batch_header = nullptr;
  DataHeader *data_header = nullptr;

  // sync on serial header
  byte error_code = idd_decode(
    IN rx_buffer, IN msg_size,
    OUT &serial_header, OUT &batch_header, OUT &data_header
  );

  if (error_code != OPCODE_ERROR_OK)
  {
    // send error
    send_error(communicator, tx_buffer, error_code);
    return;
  }

  // treat the 'timeout' update message
  if (serial_header->opCode == OPCODE_TIMEOUT_UPDATE)
  {
    // if no data, just return the current timeout value
    if (data_header->nof_elements > 0)
    {
      // get the timeout value in seconds
      int new_timeout_sec = *reinterpret_cast<int*>((byte*)(data_header) + sizeof(*data_header));
      timeout_restart_sec = new_timeout_sec;
    }
    // update some params, if only read requested 
    if (data_header->nof_elements == 0)
    {
      byte* data = ((byte*)data_header) + sizeof(*data_header);
      memcpy(data, &timeout_restart_sec, sizeof(timeout_restart_sec));
      data_header->nof_elements = sizeof(timeout_restart_sec);
      serial_header->data_size += sizeof(timeout_restart_sec);
      // update the crc
      serial_header->crc = crc16((byte*)serial_header, sizeof(*serial_header) - 
          sizeof(serial_header->crc));
    }
    // loopback the message as ACK, or add the current value of the timeout
    memcpy(tx_buffer, serial_header, sizeof(*serial_header) + serial_header->data_size);
    communicator->write(tx_buffer, sizeof(*serial_header) + serial_header->data_size);
    // Serial.write(tx_buffer, sizeof(*serial_header) + serial_header->data_size);
    // Serial.flush();
    return;
  }
  
  memset(tx_buffer, 0, sizeof(tx_buffer));
  SerialHeader* serial_header_out = (SerialHeader*)&tx_buffer[0];
  memcpy(serial_header_out, serial_header, sizeof(*serial_header_out));
  serial_header_out->data_size = sizeof(BatchHeader);
  BatchHeader* batch_header_out = (BatchHeader*)((byte*)serial_header_out + sizeof(*serial_header_out));
  memcpy(batch_header_out, batch_header, sizeof(*batch_header_out));
  batch_header_out->nof_data_elements = 0; // it will be updated lated, in accordance with message type 
  DataHeader* data_header_out = (DataHeader*)((byte*)batch_header_out + sizeof(*batch_header_out));
  memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
  byte* data_out = (byte*)data_header_out + sizeof(*data_header_out);

  bool rc;
  int out_size = 0;
  byte* data = (byte*)(data_header) + sizeof(*data_header);
  for (int k = 0; k < batch_header->nof_data_elements; k++)
  {
    switch (serial_header->opCode)
    {
    case OPCODE_I2C_WRITE:
      rc = i2c_lib.write(data[0], &data[1], data_header->nof_elements - 1);
      if (rc == false)
      {
        send_error(communicator, tx_buffer, OPCODE_ERROR_I2C);
        return;
      }
      break;
    case OPCODE_I2C_READ:
      out_size = i2c_lib.read(data[0], data_out, data_header->nof_elements - 1);
      
      // increment nof batch elements
      batch_header_out->nof_data_elements += 1;
      // update num of output elements
      data_header_out->nof_elements = out_size;
      // advance data header ptr
      data_header_out = (DataHeader *)(data_out + out_size);
      memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
      data_out += sizeof(*data_header_out) + out_size;
      // update total size
      serial_header_out->data_size += sizeof(*data_header_out) + out_size;

      break;
    case OPCODE_SPI_WRITEREAD:
      // SPI write and read
      out_size = spi_lib.writeread(data, data_out, data_header->nof_elements);

      // increment nof batch elements
      batch_header_out->nof_data_elements += 1;

      // update num of output elements
      data_header_out->nof_elements = out_size;
      // advance data header ptr
      data_header_out = (DataHeader *)(data_out + out_size);
      memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
      data_out += sizeof(*data_header_out) + out_size;
      // update total size
      serial_header_out->data_size += sizeof(*data_header_out) + out_size;
      break;
    case OPCODE_SPI_READ:
      // SPI read
      out_size = spi_lib.read(data, data_out, data_header->nof_elements);

      // increment nof batch elements
      batch_header_out->nof_data_elements += 1;

      // update num of output elements
      data_header_out->nof_elements = out_size;
      // advance data header ptr
      data_header_out = (DataHeader *)(data_out + out_size);
      memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
      data_out += sizeof(*data_header_out) + out_size;
      // update total size
      serial_header_out->data_size += sizeof(*data_header_out) + out_size;
      break;
    case OPCODE_SPI_WRITE:
      // SPI write
      spi_lib.write(data, data_header->nof_elements);
      break;
    default:
      break;
    }

    //
    data_header = (DataHeader*)(data + data_header->nof_elements);
    data = (byte*)(data_header) + sizeof(*data_header);
  }

  auto stop = std::chrono::system_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

  // write back to HOST
  // calculate and update header crc
  serial_header_out->crc = crc16((byte*)serial_header_out, sizeof(*serial_header_out) -
    sizeof(serial_header_out->crc));
  communicator->write(tx_buffer, sizeof(*serial_header_out) + serial_header_out->data_size);
  // Serial.write(tx_buffer, sizeof(*serial_header_out) + serial_header_out->data_size);
  // Serial.flush();

//  log_d("end main loop (elapsed time %f)...", difference);
  Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229(&r, &outStruct);
}
