#include <Arduino.h>
#include <chrono>
//#include <esp32-hal-log.h>
//#include "idd.h"
#include "spi_lib.h"
#include "i2c_lib.h"
//#include "wifi_lib.h"
//#include "CommunicatorFactory.h"
#include "argInitRoutines.h"

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
//#include "prm.h"

#ifndef SerialBaudRate
#define SerialBaudRate 115200
#endif
#ifndef SerialTimeOutMS
#define SerialTimeOutMS 1
#endif

#define POWER_ON_PIN        4
#define LED_STATUS          27
#define PAC_POWER_PIN       25
#define RCV_BUFFER_SIZE     512

// const char* wifi_ssid = "EVONET";
// const char* wifi_password = "12345678";
// const char* wifi_hostname = "my-evo-host";
// const uint16_t wifi_port = 4556;

// // create a instance of the server
// WiFiServer wifi_server(wifi_port);

// byte rx_buffer[RCV_BUFFER_SIZE];
// byte tx_buffer[RCV_BUFFER_SIZE];

SpiLib spi_lib;
I2CLib i2c_lib;
//BaseCommunicator* communicator = nullptr;

// timeout in secs, used to monitor communication 'keep-a-live'
// if no communication received within that period of time, restart will occure
int timeout_restart_sec = 5 * 60; // default value, may be changed by command
std::chrono::system_clock::time_point last_communication_time;

//struct0_T r{};
//struct30_T outStruct{};

const int sizeof_struct0_T = sizeof(struct0_T);   //111192
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

// void prm_init_nmax(struct0_T& r);
// void prm_init_run(struct0_T& r);
// void prm_init_seq(struct0_T& r);
// void prm_init_ser(struct0_T& r);
// void prm_init_ins(struct0_T& r);
// void prm_init_klm(struct0_T& r);
// void prm_init_bat(struct0_T& r);
// void prm_init_brd(struct0_T& r);
// void prm_init_cnfg(struct0_T& r);

void setup() {
//  log_d("begin setup...");
//Serial.begin(SerialBaudRate);
  // communicator = CommunicatorFactory::createCommunicator(DEF_COMMUNICATOR_SERIAL);
  // if (communicator == nullptr)
  //   while(1);

  // serial communication setup
  Serial.begin(SerialBaudRate);
  Serial.setTimeout(SerialTimeOutMS);

  // // wifi setup
  // WiFi.onEvent(onWiFiEvent);
  // WiFi.mode(WIFI_AP);
  // WiFi.softAP(wifi_ssid, wifi_password);
  
  // // multicast DNS
  // MDNS.begin(wifi_hostname);
  // MDNS.addService("http", "tcp", 80);
  
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
    Serial.println("failed init spi_lib...");
    while(1);
//    send_error(communicator, tx_buffer, OPCODE_ERROR_SPI);
  }
  // I2C library init
  if (i2c_lib.Init() == false)
  {
    Serial.println("failed init i2c_lib...");
    while(1);
//    send_error(communicator, tx_buffer, OPCODE_ERROR_I2C);
  }

  // light the led
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);

  last_communication_time = std::chrono::system_clock::now();

  //  log_d("setup done...");

//  emxInit_struct30_T(&outStruct);
//  argInit_struct0_T(&r);
  Serial.println("setup finished...");
}

void loop() {
//  log_d("start main loop...");

  // if (communicator == nullptr)
  //   return;

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

  // while (communicator->available() == 0)
  // {
  //   // check for timeout
  //   auto duration = std::chrono::duration_cast<std::chrono::seconds>(start - last_communication_time).count();
  //   if (duration > timeout_restart_sec)
  //   {
  //     // // connection lost for too long, restart
  //     // esp_restart();
      
  //     // SPI - all ports OFF
  //     spi_lib.Reset();

  //     // power OFF/ON
  //     pinMode(POWER_ON_PIN, OUTPUT);
  //     digitalWrite(POWER_ON_PIN, LOW);
  //     delay(100);
  //     digitalWrite(POWER_ON_PIN, HIGH);
  //     delay(100);

  //     // now configure all ports
  //     spi_lib.ConfigurePorts();
  //   }
  //   else
  //     return;
  // }

  // // save last communication time
  // last_communication_time = std::chrono::system_clock::now();

  // // wait for the serial header
  // while (communicator->available() < sizeof(SerialHeader)) delay(1);

  // int msg_size = communicator->read(rx_buffer, RCV_BUFFER_SIZE);
  // // if not enough data, return 
  // if (rx_buffer == nullptr || msg_size < sizeof(SerialHeader))
  // {
  //   return;
  // }

  // // search for the prefix "CAFE"
  // SerialHeader *serial_header = nullptr;
  // BatchHeader *batch_header = nullptr;
  // DataHeader *data_header = nullptr;

  // // sync on serial header
  // byte error_code = idd_decode(
  //   IN rx_buffer, IN msg_size,
  //   OUT &serial_header, OUT &batch_header, OUT &data_header
  // );

  // if (error_code != OPCODE_ERROR_OK)
  // {
  //   // send error
  //   send_error(communicator, tx_buffer, error_code);
  //   return;
  // }

  // // treat the 'timeout' update message
  // if (serial_header->opCode == OPCODE_TIMEOUT_UPDATE)
  // {
  //   // if no data, just return the current timeout value
  //   if (data_header->nof_elements > 0)
  //   {
  //     // get the timeout value in seconds
  //     int new_timeout_sec = *reinterpret_cast<int*>((byte*)(data_header) + sizeof(*data_header));
  //     timeout_restart_sec = new_timeout_sec;
  //   }
  //   // update some params, if only read requested 
  //   if (data_header->nof_elements == 0)
  //   {
  //     byte* data = ((byte*)data_header) + sizeof(*data_header);
  //     memcpy(data, &timeout_restart_sec, sizeof(timeout_restart_sec));
  //     data_header->nof_elements = sizeof(timeout_restart_sec);
  //     serial_header->data_size += sizeof(timeout_restart_sec);
  //     // update the crc
  //     serial_header->crc = crc16((byte*)serial_header, sizeof(*serial_header) - 
  //         sizeof(serial_header->crc));
  //   }
  //   // loopback the message as ACK, or add the current value of the timeout
  //   memcpy(tx_buffer, serial_header, sizeof(*serial_header) + serial_header->data_size);
  //   communicator->write(tx_buffer, sizeof(*serial_header) + serial_header->data_size);
  //   // Serial.write(tx_buffer, sizeof(*serial_header) + serial_header->data_size);
  //   // Serial.flush();
  //   return;
  // }
  
  // memset(tx_buffer, 0, sizeof(tx_buffer));
  // SerialHeader* serial_header_out = (SerialHeader*)&tx_buffer[0];
  // memcpy(serial_header_out, serial_header, sizeof(*serial_header_out));
  // serial_header_out->data_size = sizeof(BatchHeader);
  // BatchHeader* batch_header_out = (BatchHeader*)((byte*)serial_header_out + sizeof(*serial_header_out));
  // memcpy(batch_header_out, batch_header, sizeof(*batch_header_out));
  // batch_header_out->nof_data_elements = 0; // it will be updated lated, in accordance with message type 
  // DataHeader* data_header_out = (DataHeader*)((byte*)batch_header_out + sizeof(*batch_header_out));
  // memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
  // byte* data_out = (byte*)data_header_out + sizeof(*data_header_out);

  // bool rc;
  // int out_size = 0;
  // byte* data = (byte*)(data_header) + sizeof(*data_header);
  // for (int k = 0; k < batch_header->nof_data_elements; k++)
  // {
  //   switch (serial_header->opCode)
  //   {
  //   case OPCODE_I2C_WRITE:
  //     rc = i2c_lib.write(data[0], &data[1], data_header->nof_elements - 1);
  //     if (rc == false)
  //     {
  //       send_error(communicator, tx_buffer, OPCODE_ERROR_I2C);
  //       return;
  //     }
  //     break;
  //   case OPCODE_I2C_READ:
  //     out_size = i2c_lib.read(data[0], data_out, data_header->nof_elements - 1);
      
  //     // increment nof batch elements
  //     batch_header_out->nof_data_elements += 1;
  //     // update num of output elements
  //     data_header_out->nof_elements = out_size;
  //     // advance data header ptr
  //     data_header_out = (DataHeader *)(data_out + out_size);
  //     memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
  //     data_out += sizeof(*data_header_out) + out_size;
  //     // update total size
  //     serial_header_out->data_size += sizeof(*data_header_out) + out_size;

  //     break;
  //   case OPCODE_SPI_WRITEREAD:
  //     // SPI write and read
  //     out_size = spi_lib.writeread(data, data_out, data_header->nof_elements);

  //     // increment nof batch elements
  //     batch_header_out->nof_data_elements += 1;

  //     // update num of output elements
  //     data_header_out->nof_elements = out_size;
  //     // advance data header ptr
  //     data_header_out = (DataHeader *)(data_out + out_size);
  //     memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
  //     data_out += sizeof(*data_header_out) + out_size;
  //     // update total size
  //     serial_header_out->data_size += sizeof(*data_header_out) + out_size;
  //     break;
  //   case OPCODE_SPI_READ:
  //     // SPI read
  //     out_size = spi_lib.read(data, data_out, data_header->nof_elements);

  //     // increment nof batch elements
  //     batch_header_out->nof_data_elements += 1;

  //     // update num of output elements
  //     data_header_out->nof_elements = out_size;
  //     // advance data header ptr
  //     data_header_out = (DataHeader *)(data_out + out_size);
  //     memcpy(data_header_out->magic_word, MAGICWORD, sizeof(MAGICWORD));
  //     data_out += sizeof(*data_header_out) + out_size;
  //     // update total size
  //     serial_header_out->data_size += sizeof(*data_header_out) + out_size;
  //     break;
  //   case OPCODE_SPI_WRITE:
  //     // SPI write
  //     spi_lib.write(data, data_header->nof_elements);
  //     break;
  //   default:
  //     break;
  //   }

  //   //
  //   data_header = (DataHeader*)(data + data_header->nof_elements);
  //   data = (byte*)(data_header) + sizeof(*data_header);
  // }

  // auto stop = std::chrono::system_clock::now();
  // auto difference = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

  // // write back to HOST
  // // calculate and update header crc
  // serial_header_out->crc = crc16((byte*)serial_header_out, sizeof(*serial_header_out) -
  //   sizeof(serial_header_out->crc));
  // communicator->write(tx_buffer, sizeof(*serial_header_out) + serial_header_out->data_size);
  // // Serial.write(tx_buffer, sizeof(*serial_header_out) + serial_header_out->data_size);
  // // Serial.flush();

//  log_d("end main loop (elapsed time %f)...", difference);
  struct0_T r;
  struct30_T outStruct = { 0 };
  Serial.println("init input/output structures...");
//  argInit_struct0_T(&r);
while(1);
/*
rapidjson::Document doc;
doc.Parse(prm);
if (doc.HasParseError())
{
  Serial.println(rapidjson::GetParseError_En(doc.GetParseError()));
  delay(10);
  while(1);
}
else
{
  // Nmax
  if (false == doc.HasMember("Nmax"))
  {
    Serial.println("faile dto find 'Nmax' in prm");
    delay(10);
    while(1);
  }
  else
  {
    r.Nmax.NbatMax          = (short)doc["Nmax"]["NbatMax"].GetInt();
    r.Nmax.NbrdMax          = (short)doc["Nmax"]["NbrdMax"].GetInt();
    r.Nmax.NgrpMax          = (short)doc["Nmax"]["NgrpMax"].GetInt();
    r.Nmax.NitstMax         = (short)doc["Nmax"]["NitstMax"].GetInt();
    r.Nmax.NkalmanBatParams = (short)doc["Nmax"]["NkalmanBatParams"].GetInt();
    r.Nmax.NkalmanBatState  = (short)doc["Nmax"]["NkalmanBatState"].GetInt();
    r.Nmax.NseqMax          = (short)doc["Nmax"]["NseqMax"].GetInt();
    r.Nmax.NstateMax        = (short)doc["Nmax"]["NstateMax"].GetInt();
    r.Nmax.NstrColMax       = (short)doc["Nmax"]["NstrColMax"].GetInt();
    r.Nmax.NstrRowMax       = (short)doc["Nmax"]["NstrRowMax"].GetInt();
    r.Nmax.NswRepMax        = (short)doc["Nmax"]["NswRepMax"].GetInt();
    r.Nmax.NtimeMax         = (short)doc["Nmax"]["NtimeMax"].GetInt();
  }
  // run
  if (false == doc.HasMember("run"))
  {
    Serial.println("faile dto find 'run' in prm");
    delay(10);
    while(1);
  }
  else
  {
    r.run.Nrep			    = (short)doc["run"]["Nrep"].GetInt();
    r.run.PlotSocFlag	  = doc["run"]["PlotSocFlag"].GetBool();
    r.run.PlotItFlag	  = doc["run"]["PlotItFlag"].GetBool();
    r.run.PlotTempFlag	= doc["run"]["PlotTempFlag"].GetBool();
    r.run.PlotVFlag		  = doc["run"]["PlotVFlag"].GetBool();
    r.run.PlotIFlag		  = doc["run"]["PlotIFlag"].GetBool();
    r.run.PlotIacsFlag	= doc["run"]["PlotIacsFlag"].GetBool();
    r.run.MaxTime		    = doc["run"]["MaxTime"].GetFloat();
    r.run.dt			      = doc["run"]["dt"].GetFloat();
    r.run.T2Show		    = (short)doc["run"]["T2Show"].GetInt();
    r.run.Nt			      = (short)doc["run"]["Nt"].GetInt();
    r.run.Nt0			      = (short)doc["run"]["Nt0"].GetInt();
    r.run.testVreset	  = doc["run"]["testVreset"].GetBool();
    r.run.seq			      = (short)doc["run"]["seq"].GetInt();

    auto a = doc["run"]["SwRepId"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.run.SwRepId[k] = (short)a[k].GetInt();
    }
  }
  // seq
  if (false == doc.HasMember("seq"))
  {
    Serial.println("faile dto find 'seq' in prm");
    delay(10);
    while(1);
  }
  else
  {
    r.seq.Nst = (char)doc["seq"]["Nst"].GetInt();
    auto a = doc["seq"]["mod"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.mod[k] = (char)a[k].GetInt();
    }
    a = doc["seq"]["chr"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.chr[k] = (char)a[k].GetInt();
    }
    a = doc["seq"]["vth"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.vth[k] = a[k].GetFloat();
    }
    a = doc["seq"]["ins"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.ins[k] = (char)a[k].GetInt();
    }
    a = doc["seq"]["swm"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.swm[k] = (char)a[k].GetInt();
    }
    a = doc["seq"]["sw16to1"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.sw16to1[k] = (char)a[k].GetInt();
    }
    a = doc["seq"]["VminDisFlag"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.VminDisFlag[k] = a[k].GetFloat();
    }
    a = doc["seq"]["BrdBeforePSflag"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.BrdBeforePSflag[k] = a[k].GetBool();
    }
    auto obj = doc["seq"]["pwr"].GetObject();
    r.seq.pwr.VthDis = obj["VthDis"].GetFloat();
    r.seq.pwr.VthChr = obj["VthChr"].GetFloat();
    r.seq.pwr.VthOvDis = obj["VthOvDis"].GetFloat();
    r.seq.pwr.VthOvChr = obj["VthOvChr"].GetFloat();
    r.seq.pwr.VthUnDis = obj["VthUnDis"].GetFloat();
    r.seq.pwr.VthUnChr = obj["VthUnChr"].GetFloat();
    a = obj["VthFlag"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.pwr.VthFlag[k] = (char)a[k].GetInt();
    }
    obj = doc["seq"]["tst"].GetObject();
    r.seq.tst.savePrmFlag = obj["savePrmFlag"].GetBool();
    r.seq.tst.v.isTest = obj["v"]["isTest"].GetBool();
    r.seq.tst.v.isPrm = obj["v"]["isPrm"].GetBool();
    r.seq.tst.v.Nst = (char)obj["v"]["Nst"].GetInt();
    a = obj["v"]["ins"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.v.ins[k] = (char)a[k].GetInt();
    }
    a = obj["v"]["swm"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.v.swm[k] = (char)a[k].GetInt();
    }
    a = obj["v"]["sw16to1"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.v.sw16to1[k] = (char)a[k].GetInt();
    }
    a = obj["v"]["grp"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.v.grp[k] = (char)a[k].GetInt();
    }

    r.seq.tst.i.isTest = obj["i"]["isTest"].GetBool();
    r.seq.tst.i.NTtest = (char)obj["i"]["NTtest"].GetInt();
    r.seq.tst.i.minI = obj["i"]["minI"].GetFloat();
    r.seq.tst.i.maxI = obj["i"]["maxI"].GetFloat();
    r.seq.tst.i.Nitst = (char)obj["i"]["Nitst"].GetInt();
    r.seq.tst.i.pauseOff = obj["i"]["maxI"].GetFloat();
    r.seq.tst.i.measRintR = obj["i"]["measRintR"].GetBool();
    r.seq.tst.i.useRwireFlag = obj["i"]["useRwireFlag"].GetBool();
    r.seq.tst.i.RintBatId = (char)obj["i"]["RintBatId"].GetInt();
    r.seq.tst.i.Rload = obj["i"]["Rload"].GetFloat();
    r.seq.tst.i.Nst = (char)obj["i"]["Nst"].GetInt();

    a = obj["i"]["BrdBeforePSflag"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.BrdBeforePSflag[k] = (char)a[k].GetInt();
    }
    a = obj["i"]["NegIflag"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.NegIflag[k] = (char)a[k].GetInt();
    }
    a = obj["i"]["ins"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.ins[k] = (char)a[k].GetInt();
    }
    a = obj["i"]["meas"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.meas[k] = (char)a[k].GetInt();
    }
    a = obj["i"]["swm"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.swm[k] = (char)a[k].GetInt();
    }
    a = obj["i"]["sw16to1"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.sw16to1[k] = (char)a[k].GetInt();
    }
    a = obj["i"]["grp"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.grp[k] = (char)a[k].GetInt();
    }
    a = obj["i"]["Rin"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.Rin[k] = a[k].GetFloat();
    }
    a = obj["i"]["i_in_test"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.i_in_test[k] = a[k].GetFloat();
    }
    a = obj["i"]["ItestSwitch"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.tst.i.ItestSwitch[k] = (char)a[k].GetInt();
    }

    obj = doc["seq"]["bit"].GetObject();
    r.seq.bit.bit_flag = obj["bit_flag"].GetBool();
    r.seq.bit.Nst = (char)obj["Nst"].GetInt();
    r.seq.bit.dIthr = obj["dIthr"].GetFloat();
    r.seq.bit.dVthr = obj["dVthr"].GetFloat();
    a = obj["BrdBeforePSflag"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.BrdBeforePSflag[k] = (char)a[k].GetInt();
    }
    a = obj["chr"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.chr[k] = (char)a[k].GetInt();
    }
    a = obj["IdisChr"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.IdisChr[k] = a[k].GetFloat();
    }
    a = obj["ins"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.ins[k] = (char)a[k].GetInt();
    }
    a = obj["swm"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.swm[k] = (char)a[k].GetInt();
    }
    a = obj["sw16to1"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.sw16to1[k] = (char)a[k].GetInt();
    }
    a = obj["meas"]["V"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.meas.V[k] = (char)a[k].GetInt();
    }
    a = obj["meas"]["b_I"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.meas.b_I[k] = (char)a[k].GetInt();
    }
    a = obj["meas"]["Vd"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.seq.bit.meas.Vd[k] = a[k].GetFloat();
    }
  }
  // ser
  if (false == doc.HasMember("ser"))
  {
    Serial.println("faile dto find 'ser' in prm");
    delay(10);
    while(1);
  }
  else
  {
    r.ser.kp184_Flag	  = doc["ser"]["kp184_Flag"].GetBool();
    r.ser.ka6005p_Flag  = doc["ser"]["ka6005p_Flag"].GetBool();
    r.ser.juntek_Flag   = doc["ser"]["juntek_Flag"].GetBool();
    r.ser.swm_Flag      = doc["ser"]["swm_Flag"].GetBool();
    r.ser.sw16to1_Flag  = doc["ser"]["sw16to1_Flag"].GetBool();
    r.ser.swOut_Flag    = doc["ser"]["swOut_Flag"].GetBool();
    auto a = doc["ser"]["Esp32_v1"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.Esp32_v1[k] = a[k].GetDouble();
    }
    auto obj = doc["ser"]["com"].GetObject();
    r.ser.com.N_COM_esp32 = (short)obj["N_COM_esp32"].GetInt();
    r.ser.com.Ngrp = (short)obj["Ngrp"].GetInt();
    r.ser.com.Nbrd = (short)obj["Nbrd"].GetInt();
    r.ser.com.NuGrp = (short)obj["NuGrp"].GetInt();
    a = obj["COM_esp32"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.COM_esp32[k] = (char)a[k].GetInt();
    }
    a = obj["grp"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.grp[k] = (short)a[k].GetInt();
    }
    a = obj["uGroups"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.uGroups[k] = (short)a[k].GetInt();
    }
    a = obj["COM_kp184"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.COM_kp184[k] = (char)a[k].GetInt();
    }
    a = obj["COM_ka6005P"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.COM_ka6005P[k] = (char)a[k].GetInt();
    }
    a = obj["COM_juntek"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.COM_juntek[k] = (char)a[k].GetInt();
    }
    a = obj["COM_swm"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.COM_swm[k] = (char)a[k].GetInt();
    }
    a = obj["COM_sw16to1"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.COM_sw16to1[k] = (char)a[k].GetInt();
    }
    a = obj["COM_swOut"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k ++)
    {
      r.ser.com.COM_swOut[k] = (char)a[k].GetInt();
    }
    obj = doc["ser"]["wifi"].GetObject();
    r.ser.wifi.ip = (short)obj["ip"].GetInt();
  }
  // ins
  if (false == doc.HasMember("ins"))
  {
    Serial.println("faile dto find 'ins' in prm");
    delay(10);
    while(1);
  }
  else
  {
    r.ins.juntek	    = doc["ins"]["juntek"].GetBool();
    r.ins.ka6005p		  = doc["ins"]["ka6005p"].GetBool();
    r.ins.kp184		    = doc["ins"]["kp184"].GetBool();
    r.ins.swm		      = doc["ins"]["swm"].GetBool();
    r.ins.sw16to1		  = doc["ins"]["sw16to1"].GetBool();
    r.ins.swOut		    = doc["ins"]["swOut"].GetBool();
    r.ins.ProjectFlag = (char)doc["ins"]["ProjectFlag"].GetInt();
    auto obj = doc["ins"]["prm"].GetObject();
    auto obj1 = obj["jun"].GetObject();
    r.ins.prm.jun.ImaxAcDC = obj1["ImaxAcDC"].GetFloat();
    r.ins.prm.jun.minVjuntekInput = obj1["minVjuntekInput"].GetFloat();
    r.ins.prm.jun.juntekEfficencyFactor = obj1["juntekEfficencyFactor"].GetFloat();
    r.ins.prm.jun.baudrate = obj1["baudrate"].GetFloat();
    obj1 = obj["sw16to1"].GetObject();
    r.ins.prm.sw16to1.Ntry = (char)obj1["Ntry"].GetInt();
    obj1 = obj["swOut"].GetObject();
    r.ins.prm.swOut.IfromVdivR_flag = obj1["IfromVdivR_flag"].GetBool();
    r.ins.prm.swOut.Rshunt = obj1["Rshunt"].GetFloat();
    r.ins.prm.swOut.Vratio = obj1["Vratio"].GetFloat();;
  }
  // klm
  if (false == doc.HasMember("klm"))
  {
    Serial.println("faile dto find 'klm' in prm");
    delay(10);
    while(1);
  }
  else
  {
    r.klm.kalmanFlag	    = doc["klm"]["kalmanFlag"].GetBool();
    r.klm.Ta              = doc["klm"]["Ta"].GetFloat();
    auto a = doc["klm"]["BatParamsCell"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k++)
    {
      auto obj = a[k].GetObject();
      auto arr = obj["BatState"].GetArray();
      for (rapidjson::SizeType n = 0; n < arr.Size(); n++)
      {
        r.klm.BatParamsCell[k].BatState[n] = arr[n].GetFloat();
      }
      arr = obj["BatParams"].GetArray();
      for (rapidjson::SizeType n = 0; n < arr.Size(); n++)
      {
        r.klm.BatParamsCell[k].BatParams[n] = arr[n].GetFloat();
      }
      arr = obj["BatStateOrg"].GetArray();
      for (rapidjson::SizeType n = 0; n < arr.Size(); n++)
      {
        r.klm.BatParamsCell[k].BatStateOrg[n] = arr[n].GetFloat();
      }
    }
    a = doc["klm"]["b_struct"].GetArray();
    for (rapidjson::SizeType k = 0; k < a.Size(); k++)
    {
      auto obj = a[k].GetObject();
      r.klm.b_struct[k].eps1 = obj["eps1"].GetFloat();
      r.klm.b_struct[k].Qkalman = obj["Qkalman"].GetFloat();
      r.klm.b_struct[k].Rkalman = obj["Rkalman"].GetFloat();
      r.klm.b_struct[k].N_bat = (short)obj["Rkalman"].GetInt();
      auto arr = obj["P_k_k"].GetArray();
      for (rapidjson::SizeType n = 0; n < arr.Size(); n++)
      {
        r.klm.b_struct[k].P_k_k[n] = arr[n].GetFloat();
      }
      arr = obj["BatState_k_k"].GetArray();
      for (rapidjson::SizeType n = 0; n < arr.Size(); n++)
      {
        r.klm.b_struct[k].BatState_k_k[n] = arr[n].GetFloat();
      }
      arr = obj["BatParams"].GetArray();
      for (rapidjson::SizeType n = 0; n < arr.Size(); n++)
      {
        r.klm.b_struct[k].BatParams[n] = arr[n].GetFloat();
      }
    }
  }
}
*/
  emxInit_struct30_T(&outStruct);
  Serial.printf("enter main procedure...");
  Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229((struct0_T*)&r, &outStruct);
}

#pragma region argInit functions
/* Function Definitions */
/*
 * Arguments    : unsigned char result[256]
 * Return Type  : void
 */
static void argInit_16x16_uint8_T(unsigned char result[256])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 256; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_uint8_T();
  }
}

/*
 * Arguments    : unsigned char result[32768]
 * Return Type  : void
 */
static void argInit_16x16x8x16_uint8_T(unsigned char result[32768])
{
  int idx0;
  int idx1;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 16; idx0++) {
    for (idx1 = 0; idx1 < 16; idx1++) {
      for (idx2 = 0; idx2 < 8; idx2++) {
        for (idx3 = 0; idx3 < 16; idx3++) {
          /* Set the value of the array element.
Change this value to the value that the application requires. */
          result[((idx0 + (idx1 << 4)) + (idx2 << 8)) + (idx3 << 11)] =
              argInit_uint8_T();
        }
      }
    }
  }
}

/*
 * Arguments    : float result[496]
 * Return Type  : void
 */
static void argInit_16x31_real32_T(float result[496])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 496; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : float result[608]
 * Return Type  : void
 */
static void argInit_16x38_real32_T(float result[608])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 608; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : float result[1024]
 * Return Type  : void
 */
static void argInit_1x1024_real32_T(float result[1024])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 1024; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : boolean_T result[16]
 * Return Type  : void
 */
static void argInit_1x16_boolean_T(boolean_T result[16])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 16; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_boolean_T();
  }
}

/*
 * Arguments    : signed char result[16]
 * Return Type  : void
 */
static void argInit_1x16_int8_T(signed char result[16])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 16; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_int8_T();
  }
}

/*
 * Arguments    : float result[16]
 * Return Type  : void
 */
static void argInit_1x16_real32_T(float result[16])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 16; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : char result[256]
 * Return Type  : void
 */
static void argInit_1x256_char_T(char result[256])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 256; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_char_T();
  }
}

/*
 * Arguments    : float result[2]
 * Return Type  : void
 */
static void argInit_1x2_real32_T(float result[2])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : double result[2]
 * Return Type  : void
 */
static void argInit_1x2_real_T(double result[2])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : float result[32]
 * Return Type  : void
 */
static void argInit_1x32_real32_T(float result[32])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 32; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : char result[6]
 * Return Type  : void
 */
static void argInit_1x6_char_T(char result[6])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 6; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_char_T();
  }
}

/*
 * Arguments    : unsigned char result[12336]
 * Return Type  : void
 */
static void argInit_257x2x4x6_uint8_T(unsigned char result[12336])
{
  int idx0;
  int idx1;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 257; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      for (idx2 = 0; idx2 < 4; idx2++) {
        for (idx3 = 0; idx3 < 6; idx3++) {
          /* Set the value of the array element.
Change this value to the value that the application requires. */
          result[((idx0 + 257 * idx1) + 514 * idx2) + 2056 * idx3] =
              argInit_uint8_T();
        }
      }
    }
  }
}

/*
 * Arguments    : unsigned char result[4112]
 * Return Type  : void
 */
static void argInit_257x2x8_uint8_T(unsigned char result[4112])
{
  int idx0;
  int idx1;
  int idx2;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 257; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      for (idx2 = 0; idx2 < 8; idx2++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result[(idx0 + 257 * idx1) + 514 * idx2] = argInit_uint8_T();
      }
    }
  }
}

/*
 * Arguments    : signed char result[32]
 * Return Type  : void
 */
static void argInit_2x16_int8_T(signed char result[32])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 32; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_int8_T();
  }
}

/*
 * Arguments    : float result[64]
 * Return Type  : void
 */
static void argInit_2x16x2_real32_T(float result[64])
{
  int idx0;
  int idx1;
  int idx2;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    for (idx1 = 0; idx1 < 16; idx1++) {
      for (idx2 = 0; idx2 < 2; idx2++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result[(idx0 + (idx1 << 1)) + (idx2 << 5)] = argInit_real32_T();
      }
    }
  }
}

/*
 * Arguments    : short result[2]
 * Return Type  : void
 */
static void argInit_2x1_int16_T(short result[2])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_int16_T();
  }
}

/*
 * Arguments    : float result[4]
 * Return Type  : void
 */
static void argInit_2x1x2_real32_T(float result[4])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 4; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : char result[512]
 * Return Type  : void
 */
static void argInit_2x256_char_T(char result[512])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 512; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_char_T();
  }
}

/*
 * Arguments    : struct21_T result[4]
 * Return Type  : void
 */
static void argInit_2x2_struct21_T(struct21_T result[4])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 4; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    argInit_struct21_T(&result[i]);
  }
}

/*
 * Arguments    : struct22_T result[4]
 * Return Type  : void
 */
static void argInit_2x2_struct22_T(struct22_T result[4])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 4; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    argInit_struct22_T(&result[i]);
  }
}

/*
 * Arguments    : char result[12]
 * Return Type  : void
 */
static void argInit_2x6_char_T(char result[12])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 12; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_char_T();
  }
}

/*
 * Arguments    : short result[32]
 * Return Type  : void
 */
static void argInit_32x1_int16_T(short result[32])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 32; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_int16_T();
  }
}

/*
 * Arguments    : char result[8192]
 * Return Type  : void
 */
static void argInit_32x256_char_T(char result[8192])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 8192; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_char_T();
  }
}

/*
 * Arguments    : unsigned char result[24]
 * Return Type  : void
 */
static void argInit_6x1x2x2_uint8_T(unsigned char result[24])
{
  int idx0;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    for (idx2 = 0; idx2 < 2; idx2++) {
      for (idx3 = 0; idx3 < 2; idx3++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result[(idx0 + 6 * idx2) + 12 * idx3] = argInit_uint8_T();
      }
    }
  }
}

/*
 * Arguments    : unsigned char result[288]
 * Return Type  : void
 */
static void argInit_6x2x4x6_uint8_T(unsigned char result[288])
{
  int idx0;
  int idx1;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      for (idx2 = 0; idx2 < 4; idx2++) {
        for (idx3 = 0; idx3 < 6; idx3++) {
          /* Set the value of the array element.
Change this value to the value that the application requires. */
          result[((idx0 + 6 * idx1) + 12 * idx2) + 48 * idx3] =
              argInit_uint8_T();
        }
      }
    }
  }
}

/*
 * Arguments    : signed char result[128]
 * Return Type  : void
 */
static void argInit_8x16_int8_T(signed char result[128])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 128; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_int8_T();
  }
}

/*
 * Arguments    : float result[128]
 * Return Type  : void
 */
static void argInit_8x16_real32_T(float result[128])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 128; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : short result[8]
 * Return Type  : void
 */
static void argInit_8x1_int16_T(short result[8])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 8; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_int16_T();
  }
}

/*
 * Arguments    : double result[16]
 * Return Type  : void
 */
static void argInit_8x2_real_T(double result[16])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 16; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : boolean_T
 */
static boolean_T argInit_boolean_T(void)
{
  return false;
}

/*
 * Arguments    : void
 * Return Type  : char
 */
static char argInit_char_T(void)
{
  return '?';
}

/*
 * Arguments    : void
 * Return Type  : short
 */
static short argInit_int16_T(void)
{
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : int
 */
static int argInit_int32_T(void)
{
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : signed char
 */
static signed char argInit_int8_T(void)
{
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : float
 */
static float argInit_real32_T(void)
{
  return 0.0F;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : struct0_T *result
 * Return Type  : void
 */
static void argInit_struct0_T(struct0_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result->Nmax = argInit_struct1_T();
  argInit_struct2_T(&result->run);
  argInit_struct3_T(&result->seq);
  argInit_struct10_T(&result->ser);
//  argInit_struct13_T(&result->str);
  result->ins = argInit_struct15_T();
  argInit_struct20_T(&result->klm);
  argInit_struct23_T(&result->bat);
  argInit_struct24_T(&result->brd);
  argInit_struct27_T(&result->cnfg);
// argInit_struct28_T(&result->files);
}

/*
 * Arguments    : struct10_T *result
 * Return Type  : void
 */
static void argInit_struct10_T(struct10_T *result)
{
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_boolean_T();
  result->ka6005p_Flag = result_tmp;
  result->juntek_Flag = result_tmp;
  result->swm_Flag = result_tmp;
  result->sw16to1_Flag = result_tmp;
  result->swOut_Flag = result_tmp;
  argInit_struct11_T(&result->com);
  result->wifi = argInit_struct12_T();
  result->kp184_Flag = result_tmp;
  argInit_1x2_real_T(result->Esp32_v1);
}

/*
 * Arguments    : struct11_T *result
 * Return Type  : void
 */
static void argInit_struct11_T(struct11_T *result)
{
  int i;
  short result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result->Ngrp = result_tmp;
  result->Nbrd = result_tmp;
  result->NuGrp = result_tmp;
  argInit_1x6_char_T(result->COM_kp184);
  argInit_2x6_char_T(result->COM_esp32);
  result->N_COM_esp32 = result_tmp;
  argInit_2x1_int16_T(result->grp);
  argInit_8x1_int16_T(result->uGroups);
  for (i = 0; i < 6; i++) {
    result->COM_ka6005P[i] = result->COM_kp184[i];
    result->COM_juntek[i] = result->COM_kp184[i];
    result->COM_swm[i] = result->COM_kp184[i];
    result->COM_sw16to1[i] = result->COM_kp184[i];
    result->COM_swOut[i] = result->COM_kp184[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct12_T
 */
static struct12_T argInit_struct12_T(void)
{
  struct12_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result.ip = argInit_int16_T();
  return result;
}

/*
 * Arguments    : struct13_T *result
 * Return Type  : void
 */
static void argInit_struct13_T(struct13_T *result)
{
  int i;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_32x256_char_T(result->mod);
  result->num = argInit_struct14_T();
  for (i = 0; i < 8192; i++) {
    result->seq[i] = result->mod[i];
    result->ins[i] = result->mod[i];
    result->sw[i] = result->mod[i];
    result->sw16to1[i] = result->mod[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct14_T
 */
static struct14_T argInit_struct14_T(void)
{
  struct14_T result;
  signed char result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int8_T();
  result.seq = result_tmp;
  result.ins = result_tmp;
  result.sw = result_tmp;
  result.sw16to1 = result_tmp;
  result.mod = result_tmp;
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct15_T
 */
static struct15_T argInit_struct15_T(void)
{
  struct15_T result;
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_boolean_T();
  result.ka6005p = result_tmp;
  result.kp184 = result_tmp;
  result.swm = result_tmp;
  result.sw16to1 = result_tmp;
  result.swOut = result_tmp;
  result.juntek = result_tmp;
  result.prm = argInit_struct16_T();
  result.ProjectFlag = argInit_int8_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct16_T
 */
static struct16_T argInit_struct16_T(void)
{
  struct16_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result.jun = argInit_struct17_T();
  result.sw16to1 = argInit_struct18_T();
  result.swOut = argInit_struct19_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct17_T
 */
static struct17_T argInit_struct17_T(void)
{
  struct17_T result;
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result.minVjuntekInput = result_tmp;
  result.juntekEfficencyFactor = result_tmp;
  result.baudrate = result_tmp;
  result.ImaxAcDC = result_tmp;
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct18_T
 */
static struct18_T argInit_struct18_T(void)
{
  struct18_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result.Ntry = argInit_int8_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct19_T
 */
static struct19_T argInit_struct19_T(void)
{
  struct19_T result;
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result.Vratio = result_tmp;
  result.Rshunt = result_tmp;
  result.IfromVdivR_flag = argInit_boolean_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct1_T
 */
static struct1_T argInit_struct1_T(void)
{
  struct1_T result;
  short result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result.NbrdMax = result_tmp;
  result.NgrpMax = result_tmp;
  result.NseqMax = result_tmp;
  result.NstateMax = result_tmp;
  result.NitstMax = result_tmp;
  result.NbatMax = result_tmp;
  result.NstrRowMax = result_tmp;
  result.NstrColMax = result_tmp;
  result.NtimeMax = result_tmp;
  result.NkalmanBatState = result_tmp;
  result.NkalmanBatParams = result_tmp;
  result.NswRepMax = result_tmp;
  return result;
}

/*
 * Arguments    : struct20_T *result
 * Return Type  : void
 */
static void argInit_struct20_T(struct20_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result->kalmanFlag = argInit_boolean_T();
  result->Ta = argInit_real32_T();
  argInit_2x2_struct21_T(result->BatParamsCell);
  argInit_2x2_struct22_T(result->b_struct);
}

/*
 * Arguments    : struct21_T *result
 * Return Type  : void
 */
static void argInit_struct21_T(struct21_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_16x31_real32_T(result->BatState);
  argInit_16x38_real32_T(result->BatParams);
  memcpy(&result->BatStateOrg[0], &result->BatState[0], 496U * sizeof(float));
}

/*
 * Arguments    : struct22_T *result
 * Return Type  : void
 */
static void argInit_struct22_T(struct22_T *result)
{
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result->Qkalman = result_tmp;
  result->Rkalman = result_tmp;
  result->eps1 = result_tmp;
  argInit_1x16_real32_T(result->P_k_k);
  argInit_16x31_real32_T(result->BatState_k_k);
  argInit_16x38_real32_T(result->BatParams);
  result->N_bat = argInit_int16_T();
}

/*
 * Arguments    : struct23_T *result
 * Return Type  : void
 */
static void argInit_struct23_T(struct23_T *result)
{
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result->ImaxDis = result_tmp;
  result->Icharge = result_tmp;
  result->IchargePhase2 = result_tmp;
  result->IchargePhase3 = result_tmp;
  result->minIphase2 = result_tmp;
  result->dIphase2 = result_tmp;
  argInit_1x16_real32_T(result->CutOffDisV);
  result->Vmin = result_tmp;
  result->Vmax = result_tmp;
  result->VresetMax = result_tmp;
  argInit_1x1024_real32_T(result->t);
  result->T = result_tmp;
  result->Vd = result_tmp;
  argInit_1x32_real32_T(result->Rint);
  memcpy(&result->CutOffChrV[0], &result->CutOffDisV[0], 16U * sizeof(float));
  memcpy(&result->i_in[0], &result->t[0], 1024U * sizeof(float));
}

/*
 * Arguments    : struct24_T *result
 * Return Type  : void
 */
static void argInit_struct24_T(struct24_T *result)
{
  short result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result->N_bat2 = result_tmp;
  result->N_bat = result_tmp;
  result->Nbat = result_tmp;
  result->N_bat1 = result_tmp;
  result->Nina219 = argInit_real_T();
  argInit_struct25_T(&result->spi);
  argInit_struct26_T(&result->pac);
  argInit_1x32_real32_T(result->Rwire);
  result->useRwireFlag = argInit_boolean_T();
}

/*
 * Arguments    : struct25_T *result
 * Return Type  : void
 */
static void argInit_struct25_T(struct25_T *result)
{
  int i;
  unsigned char result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_uint8_T();
  result->bypass = result_tmp;
  argInit_257x2x8_uint8_T(result->SwitchMat_esp);
  argInit_6x1x2x2_uint8_T(result->SwitchMat_esp2);
  result->rst = argInit_real_T();
  result->disconnect = result_tmp;
  argInit_257x2x4x6_uint8_T(result->PortSpiRow_esp);
  argInit_6x2x4x6_uint8_T(result->PortSpiRow_esp2);
  memcpy(&result->Pac2Vid[0], &result->SwitchMat_esp[0],
         4112U * sizeof(unsigned char));
  for (i = 0; i < 24; i++) {
    result->Pac2Vid2[i] = result->SwitchMat_esp2[i];
  }
}

/*
 * Arguments    : struct26_T *result
 * Return Type  : void
 */
static void argInit_struct26_T(struct26_T *result)
{
  signed char result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int8_T();
  result->Iacs758Id = result_tmp;
  argInit_8x2_real_T(result->i2cPacAdd);
  argInit_2x16_int8_T(result->VIpacId);
  result->readIpacFlag = argInit_boolean_T();
  argInit_2x16x2_real32_T(result->Rval);
  argInit_1x2_real32_T(result->Rshunt);
  argInit_2x1x2_real32_T(result->pIacs758);
  result->Iacs758Flag = result_tmp;
}

/*
 * Arguments    : struct27_T *result
 * Return Type  : void
 */
static void argInit_struct27_T(struct27_T *result)
{
  int i;
  short result_tmp;
  unsigned char b_result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result->minLenIna219 = result_tmp;
  argInit_16x16_uint8_T(result->BattConfigDis1);
  b_result_tmp = argInit_uint8_T();
  result->BattConfigBypass = b_result_tmp;
  result->ToggleFlag = argInit_boolean_T();
  result->Ttoggle = argInit_real32_T();
  result->NtoggleDrop = result_tmp;
  result->BattConfigStandby = b_result_tmp;
  for (i = 0; i < 256; i++) {
    result->BattConfigChr1[i] = result->BattConfigDis1[i];
    result->BattConfigChr2[i] = result->BattConfigDis1[i];
    result->BattConfigChr3[i] = result->BattConfigDis1[i];
  }
}

/*
 * Arguments    : struct28_T *result
 * Return Type  : void
 */
static void argInit_struct28_T(struct28_T *result)
{
  int i;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_1x256_char_T(result->saveDir);
  argInit_2x256_char_T(result->savePath_pIacs);
  result->num = argInit_struct29_T();
  for (i = 0; i < 256; i++) {
    result->sufixDir[i] = result->saveDir[i];
    result->prefixFile[i] = result->saveDir[i];
  }
  for (i = 0; i < 512; i++) {
    result->savePath_Rval[i] = result->savePath_pIacs[i];
    result->savePath_Rwire[i] = result->savePath_pIacs[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct29_T
 */
static struct29_T argInit_struct29_T(void)
{
  struct29_T result;
  int result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int32_T();
  result.sufixDir = result_tmp;
  result.prefixFile = result_tmp;
  result.savePath_pIacs = result_tmp;
  result.savePath_Rval = result_tmp;
  result.savePath_Rwire = result_tmp;
  result.saveDir = result_tmp;
  return result;
}

/*
 * Arguments    : struct2_T *result
 * Return Type  : void
 */
static void argInit_struct2_T(struct2_T *result)
{
  float b_result_tmp;
  short c_result_tmp;
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_boolean_T();
  result->PlotItFlag = result_tmp;
  result->PlotTempFlag = result_tmp;
  result->PlotVFlag = result_tmp;
  result->PlotIFlag = result_tmp;
  result->PlotIacsFlag = result_tmp;
  b_result_tmp = argInit_real32_T();
  result->dt = b_result_tmp;
  c_result_tmp = argInit_int16_T();
  result->T2Show = c_result_tmp;
  result->Nt = c_result_tmp;
  result->Nt0 = c_result_tmp;
  result->testVreset = result_tmp;
  result->seq = c_result_tmp;
  argInit_32x1_int16_T(result->SwRepId);
  result->Nrep = c_result_tmp;
  result->PlotSocFlag = result_tmp;
  result->MaxTime = b_result_tmp;
}

/*
 * Arguments    : struct3_T *result
 * Return Type  : void
 */
static void argInit_struct3_T(struct3_T *result)
{
  int i;
  signed char i1;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->chr);
  argInit_1x16_int8_T(result->mod);
  argInit_1x16_real32_T(result->vth);
  result->Nst = argInit_int8_T();
  argInit_1x16_boolean_T(result->BrdBeforePSflag);
  result->pwr = argInit_struct4_T();
  argInit_struct5_T(&result->tst);
  argInit_struct8_T(&result->bit);
  memcpy(&result->ins[0], &result->chr[0], 128U * sizeof(signed char));
  for (i = 0; i < 16; i++) {
    i1 = result->mod[i];
    result->swm[i] = i1;
    result->sw16to1[i] = i1;
    result->VminDisFlag[i] = result->vth[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct4_T
 */
static struct4_T argInit_struct4_T(void)
{
  struct4_T result;
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result.VthChr = result_tmp;
  result.VthOvDis = result_tmp;
  result.VthOvChr = result_tmp;
  result.VthUnDis = result_tmp;
  result.VthUnChr = result_tmp;
  result.VthDis = result_tmp;
  argInit_1x16_int8_T(result.VthFlag);
  return result;
}

/*
 * Arguments    : struct5_T *result
 * Return Type  : void
 */
static void argInit_struct5_T(struct5_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_struct6_T(&result->v);
  argInit_struct7_T(&result->i);
  result->savePrmFlag = argInit_boolean_T();
}

/*
 * Arguments    : struct6_T *result
 * Return Type  : void
 */
static void argInit_struct6_T(struct6_T *result)
{
  int i;
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_1x16_int8_T(result->ins);
  result_tmp = argInit_boolean_T();
  result->isPrm = result_tmp;
  result->isTest = result_tmp;
  result->Nst = argInit_int8_T();
  for (i = 0; i < 16; i++) {
    result->swm[i] = result->ins[i];
    result->sw16to1[i] = result->ins[i];
    result->grp[i] = result->ins[i];
  }
}

/*
 * Arguments    : struct7_T *result
 * Return Type  : void
 */
static void argInit_struct7_T(struct7_T *result)
{
  float result_tmp;
  int i;
  signed char b_result_tmp;
  boolean_T c_result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->BrdBeforePSflag);
  argInit_1x16_int8_T(result->swm);
  result_tmp = argInit_real32_T();
  result->maxI = result_tmp;
  b_result_tmp = argInit_int8_T();
  result->Nitst = b_result_tmp;
  result->pauseOff = result_tmp;
  c_result_tmp = argInit_boolean_T();
  result->measRintR = c_result_tmp;
  result->useRwireFlag = c_result_tmp;
  result->RintBatId = b_result_tmp;
  result->Rload = result_tmp;
  result->isTest = c_result_tmp;
  argInit_8x16_real32_T(result->Rin);
  result->NTtest = b_result_tmp;
  result->minI = result_tmp;
  argInit_1x32_real32_T(result->i_in_test);
  argInit_16x16x8x16_uint8_T(result->ItestSwitch);
  result->Nst = argInit_uint8_T();
  for (i = 0; i < 128; i++) {
    result->NegIflag[i] = result->BrdBeforePSflag[i];
    result->ins[i] = result->BrdBeforePSflag[i];
    result->meas[i] = result->BrdBeforePSflag[i];
  }
  for (i = 0; i < 16; i++) {
    result->sw16to1[i] = result->swm[i];
  }
  memcpy(&result->grp[0], &result->BrdBeforePSflag[0],
         128U * sizeof(signed char));
}

/*
 * Arguments    : struct8_T *result
 * Return Type  : void
 */
static void argInit_struct8_T(struct8_T *result)
{
  float result_tmp;
  int i;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->chr);
  argInit_1x16_int8_T(result->BrdBeforePSflag);
  result_tmp = argInit_real32_T();
  result->dVthr = result_tmp;
  result->bit_flag = argInit_boolean_T();
  argInit_8x16_real32_T(result->IdisChr);
  result->Nst = argInit_int8_T();
  argInit_struct9_T(&result->meas);
  result->dIthr = result_tmp;
  memcpy(&result->ins[0], &result->chr[0], 128U * sizeof(signed char));
  for (i = 0; i < 16; i++) {
    result->swm[i] = result->BrdBeforePSflag[i];
    result->sw16to1[i] = result->BrdBeforePSflag[i];
  }
}

/*
 * Arguments    : struct9_T *result
 * Return Type  : void
 */
static void argInit_struct9_T(struct9_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->V);
  argInit_8x16_real32_T(result->Vd);
  memcpy(&result->b_I[0], &result->V[0], 128U * sizeof(signed char));
}

/*
 * Arguments    : void
 * Return Type  : unsigned char
 */
static unsigned char argInit_uint8_T(void)
{
  return 0U;
}
#pragma endregion
