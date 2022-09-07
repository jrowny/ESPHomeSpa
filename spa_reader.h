#include "esphome.h"
#include <CircularBuffer.h>

#define STRON String("ON").c_str()
#define STROFF String("OFF").c_str()

class SpaReader : public Component, public UARTDevice, public CustomAPIDevice {
 public:
  SpaReader(UARTComponent *parent) : UARTDevice(parent) {}

  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  CircularBuffer<uint8_t, 35> Q_in;
  CircularBuffer<uint8_t, 35> Q_out;
  uint8_t x, i, j;
  uint8_t last_state_crc = 0x00;
  uint8_t send = 0x00;
  uint8_t settemp = 0x00;
  uint8_t sethour = 0x00;
  uint8_t setminute = 0x00;
  uint8_t id = 0x00;
  unsigned long lastrx = 0;

  char have_config = 0; //stages: 0-> want it; 1-> requested it; 2-> got it; 3-> further processed it
  char have_faultlog = 0; //stages: 0-> want it; 1-> requested it; 2-> got it; 3-> further processed it
  char have_filtersettings = 0; //stages: 0-> want it; 1-> requested it; 2-> got it; 3-> further processed it
  char faultlog_minutes = 0; //temp logic so we only get the fault log once per 5 minutes
  char filtersettings_minutes = 0; //temp logic so we only get the filter settings once per 5 minutes

  struct {
    uint8_t jet1 :2;
    uint8_t jet2 :2;
    uint8_t blower :1;
    uint8_t light :1;
    uint8_t restmode:1;
    uint8_t highrange:1;
    uint8_t hour :5;
    uint8_t minutes :6;
  } SpaState;

  struct {
    uint8_t pump1 :2; //this could be 1=1 speed; 2=2 speeds
    uint8_t pump2 :2;
    uint8_t pump3 :2;
    uint8_t pump4 :2;
    uint8_t pump5 :2;
    uint8_t pump6 :2;
    uint8_t light1 :1;
    uint8_t light2 :1;
    uint8_t circ :1;
    uint8_t blower :1;
    uint8_t mister :1;
    uint8_t aux1 :1;
    uint8_t aux2 :1;
    uint8_t temp_scale :1; //0 -> Farenheit, 1-> Celcius
  } SpaConfig;

  struct {
    uint8_t totEntry :5;
    uint8_t currEntry :5;
    uint8_t faultCode :6;
    String faultMessage;
    uint8_t daysAgo :8;
    uint8_t hour :5;
    uint8_t minutes :6;
  } SpaFaultLog;

  struct {
    uint8_t filt1Hour :5;
    uint8_t filt1Minute :6;
    uint8_t filt1DurationHour :5;
    uint8_t filt1DurationMinute :6;
    uint8_t filt2Enable :1;
    uint8_t filt2Hour :5;
    uint8_t filt2Minute :6;
    uint8_t filt2DurationHour :5;
    uint8_t filt2DurationMinute :6;

  } SpaFilterSettings;

  Sensor *temp_sensor = new Sensor();
  Sensor *target_temp_sensor = new Sensor();
  Sensor *jet1_sensor = new Sensor();
  Sensor *jet2_sensor = new Sensor();
  Sensor *blower_sensor = new Sensor();
  Sensor *light_sensor = new Sensor();
  Sensor *restmode_sensor = new Sensor();
  Sensor *highrange_sensor = new Sensor();
  Sensor *hour_sensor = new Sensor();
  Sensor *minute_sensor = new Sensor();

  uint8_t crc8(CircularBuffer<uint8_t, 35> &data) {
    unsigned long crc;
    int i, bit;
    uint8_t length = data.size();

    crc = 0x02;
    for ( i = 0 ; i < length ; i++ ) {
      crc ^= data[i];
      for ( bit = 0 ; bit < 8 ; bit++ ) {
        if ( (crc & 0x80) != 0 ) {
          crc <<= 1;
          crc ^= 0x7;
        }
        else {
          crc <<= 1;
        }
      }
    }

    return crc ^ 0x02;
  }

  void ID_request() {
    Q_out.push(0xFE);
    Q_out.push(0xBF);
    Q_out.push(0x01);
    Q_out.push(0x02);
    Q_out.push(0xF1);
    Q_out.push(0x73);

    rs485_send();
  }

  void ID_ack() {
    Q_out.push(id);
    Q_out.push(0xBF);
    Q_out.push(0x03);

    rs485_send();
  }

  void rs485_send() {
    // Add telegram length
    Q_out.unshift(Q_out.size() + 2);

    // Add CRC
    Q_out.push(crc8(Q_out));

    // Wrap telegram in SOF/EOF
    Q_out.unshift(0x7E);
    Q_out.push(0x7E);

    for (i = 0; i < Q_out.size(); i++) {
      write(Q_out[i]);
    }

    print_msg(Q_out);

    flush();


    // DEBUG: print_msg(Q_out);
    Q_out.clear();
  }

  void print_msg(CircularBuffer<uint8_t, 35> &data) {
    String s;
    //for (i = 0; i < (Q_in[1] + 2); i++) {
    for (i = 0; i < data.size(); i++) {
      x = Q_in[i];
      if (x < 0x0A) s += "0";
      s += String(x, HEX);
      s += " ";
    }
    yield();
  }

  void decodeSettings() {
    ESP_LOGD("Spa/config/status", "Got config");
    SpaConfig.pump1 = Q_in[5] & 0x03;
    SpaConfig.pump2 = (Q_in[5] & 0x0C) >> 2;
    SpaConfig.pump3 = (Q_in[5] & 0x30) >> 4;
    SpaConfig.pump4 = (Q_in[5] & 0xC0) >> 6;
    SpaConfig.pump5 = (Q_in[6] & 0x03);
    SpaConfig.pump6 = (Q_in[6] & 0xC0) >> 6;
    SpaConfig.light1 = (Q_in[7] & 0x03);
    SpaConfig.light2 = (Q_in[7] >> 2) & 0x03;
    SpaConfig.circ = ((Q_in[8] & 0x80) != 0);
    SpaConfig.blower = ((Q_in[8] & 0x03) != 0);
    SpaConfig.mister = ((Q_in[9] & 0x30) != 0);
    SpaConfig.aux1 = ((Q_in[9] & 0x01) != 0);
    SpaConfig.aux2 = ((Q_in[9] & 0x02) != 0);
    SpaConfig.temp_scale = Q_in[3] & 0x01; //Read temperature scale - 0 -> Farenheit, 1-> Celcius
    ESP_LOGD("Spa/config/pumps1", String(SpaConfig.pump1).c_str());
    ESP_LOGD("Spa/config/pumps2", String(SpaConfig.pump2).c_str());
    ESP_LOGD("Spa/config/pumps3", String(SpaConfig.pump3).c_str());
    ESP_LOGD("Spa/config/pumps4", String(SpaConfig.pump4).c_str());
    ESP_LOGD("Spa/config/pumps5", String(SpaConfig.pump5).c_str());
    ESP_LOGD("Spa/config/pumps6", String(SpaConfig.pump6).c_str());
    ESP_LOGD("Spa/config/light1", String(SpaConfig.light1).c_str());
    ESP_LOGD("Spa/config/light2", String(SpaConfig.light2).c_str());
    ESP_LOGD("Spa/config/circ", String(SpaConfig.circ).c_str());
    ESP_LOGD("Spa/config/blower", String(SpaConfig.blower).c_str());
    ESP_LOGD("Spa/config/mister", String(SpaConfig.mister).c_str());
    ESP_LOGD("Spa/config/aux1", String(SpaConfig.aux1).c_str());
    ESP_LOGD("Spa/config/aux2", String(SpaConfig.aux2).c_str());
    ESP_LOGD("Spa/config/temp_scale", String(SpaConfig.temp_scale).c_str());
    have_config = 2;
  }

  void decodeState() {
    String s;
    double d = 0.0;
    double c = 0.0;

    // 25:Flag Byte 20 - Set Temperature
    d = Q_in[25];
    /*
    if (SpaConfig.temp_scale == 1) {
      d = Q_in[25];
    } else if (SpaConfig.temp_scale == 0){
      d = Q_in[25] / 2;
      if (Q_in[25] % 2 == 1) d += 0.5;
    }*/

    ESP_LOGD("Spa/target_temp/state", String(d, 2).c_str());
    target_temp_sensor->publish_state(d);

    // 7:Flag Byte 2 - Actual temperature
    if (Q_in[7] != 0xFF) {
      d = Q_in[7];
      /* this doesn't seem reliable
      if (SpaConfig.temp_scale == 1) {
        d = Q_in[7];
      } else if (SpaConfig.temp_scale == 0){
        d = Q_in[7] / 2;
        if (Q_in[7] % 2 == 1) d += 0.5;
      }
      */
      if (c > 0) {
        if ((d > c * 1.2) || (d < c * 0.8)) d = c; //remove spurious readings greater or less than 20% away from previous read
      }

      ESP_LOGD("Spa/temperature/state", String(d, 2).c_str());
      temp_sensor->publish_state(d);
      c = d;
    } else {
      d = 0;
    }
    // REMARK Move upper publish to HERE to get 0 for unknown temperature

    // 8:Flag Byte 3 Hour & 9:Flag Byte 4 Minute => Time
    if (Q_in[8] < 10) s = "0"; else s = "";
    SpaState.hour = Q_in[8];
    s += String(Q_in[8]) + ":";
    if (Q_in[9] < 10) s += "0";
    s += String(Q_in[9]);
    SpaState.minutes = Q_in[9];
    ESP_LOGD("Spa/time/state", s.c_str());
    sethour = SpaState.hour;
    setminute = SpaState.minutes;
    hour_sensor->publish_state(SpaState.hour);
    minute_sensor->publish_state(SpaState.minutes);

    // 10:Flag Byte 5 - Heating Mode
    switch (Q_in[10]) {
      case 0:
        ESP_LOGD("Spa/heatingmode/state", STRON); //Ready
        SpaState.restmode = 0;
        restmode_sensor->publish_state(0);
        break;
      case 3:// Ready-in-Rest
        SpaState.restmode = 0;
        restmode_sensor->publish_state(0);
        break;
      case 1:
        ESP_LOGD("Spa/heatingmode/state", STROFF); //Rest
        SpaState.restmode = 1;
        restmode_sensor->publish_state(1);
        break;
    }

    // 15:Flags Byte 10 / Heat status, Temp Range
    d = bitRead(Q_in[15], 4);
    if (d == 0) ESP_LOGD("Spa/heatstate/state", STROFF);
    else if (d == 1 || d == 2) ESP_LOGD("Spa/heatstate/state", STRON);

    d = bitRead(Q_in[15], 2);
    if (d == 0) {
      ESP_LOGD("Spa/highrange/state", STROFF); //LOW
      highrange_sensor->publish_state(0);
      SpaState.highrange = 0;
    } else if (d == 1) {
      ESP_LOGD("Spa/highrange/state", STRON); //HIGH
      highrange_sensor->publish_state(1);
      SpaState.highrange = 1;
    }

    // 16:Flags Byte 11
    if (bitRead(Q_in[16], 1) == 1) {
      ESP_LOGD("Spa/jet_1/state", STRON);
      jet1_sensor->publish_state(1);
      SpaState.jet1 = 1;
    } else {
      ESP_LOGD("Spa/jet_1/state", STROFF);
      jet1_sensor->publish_state(0);
      SpaState.jet1 = 0;
    }

    if (bitRead(Q_in[16], 3) == 1) {
      ESP_LOGD("Spa/jet_2/state", STRON);
      jet2_sensor->publish_state(1);
      SpaState.jet2 = 1;
    } else {
      ESP_LOGD("Spa/jet_2/state", STROFF);
      jet2_sensor->publish_state(0);
      SpaState.jet2 = 0;
    }

    // 18:Flags Byte 13
    if (bitRead(Q_in[18], 1) == 1)
      ESP_LOGD("Spa/circ/state", STRON);
    else
      ESP_LOGD("Spa/circ/state", STROFF);

    if (bitRead(Q_in[18], 2) == 1) {
      ESP_LOGD("Spa/blower/state", STRON);
      SpaState.blower = 1;      
      blower_sensor->publish_state(1);
    } else {
      ESP_LOGD("Spa/blower/state", STROFF);
      SpaState.blower = 0;      
      blower_sensor->publish_state(0);
    }
    // 19:Flags Byte 14
    if (Q_in[19] == 0x03) {
      ESP_LOGD("Spa/light/state", STRON);
      SpaState.light = 1;
      light_sensor->publish_state(1);
    } else {
      ESP_LOGD("Spa/light/state", STROFF);
      SpaState.light = 0;
      light_sensor->publish_state(0);
    }

    last_state_crc = Q_in[Q_in[1]];
  }


  void decodeFilterSettings() {
    String s;
    String d;
    String payld;

    SpaFilterSettings.filt1Hour = Q_in[5];
    SpaFilterSettings.filt1Minute = Q_in[6];
    SpaFilterSettings.filt1DurationHour = Q_in[7];
    SpaFilterSettings.filt1DurationMinute = Q_in[8];
    SpaFilterSettings.filt2Enable = bitRead(Q_in[9],7); // check
    SpaFilterSettings.filt2Hour = Q_in[9] ^ (SpaFilterSettings.filt2Enable << 7); // check
    SpaFilterSettings.filt2Minute = Q_in[10];
    SpaFilterSettings.filt2DurationHour = Q_in[11];
    SpaFilterSettings.filt2DurationMinute = Q_in[12];

    //Filter 1 time conversion
    if (SpaFilterSettings.filt1Hour < 10) s = "0"; else s = "";
    s = String(SpaFilterSettings.filt1Hour) + ":";
    if (SpaFilterSettings.filt1Minute < 10) s += "0";
    s += String(SpaFilterSettings.filt1Minute);

    if (SpaFilterSettings.filt1DurationHour < 10) d = "0"; else d = "";
    d = String(SpaFilterSettings.filt1DurationHour) + ":";
    if (SpaFilterSettings.filt1DurationMinute < 10) d += "0";
    d += String(SpaFilterSettings.filt1DurationMinute);

    payld = "{\"start\":\""+s+"\",\"duration\":\""+d+"\"}";
    ESP_LOGD("Spa/filter1/state", payld.c_str());

    //Filter 2 time conversion
    if (SpaFilterSettings.filt2Hour < 10) s = "0"; else s = "";
    s += String(SpaFilterSettings.filt2Hour) + ":";
    if (SpaFilterSettings.filt2Minute < 10) s += "0";
    s += String(SpaFilterSettings.filt2Minute);

    if (SpaFilterSettings.filt2DurationHour < 10) d = "0"; else d = "";
    d += String(SpaFilterSettings.filt2DurationHour) + ":";
    if (SpaFilterSettings.filt2DurationMinute < 10) d += "0";
    d += String(SpaFilterSettings.filt2DurationMinute);
    if ((int)(SpaFilterSettings.filt2Enable) == 1) ESP_LOGD("Spa/filter2_enabled/state", STRON); else ESP_LOGD("Spa/filter2_enabled/state", STROFF);


    payld = "{\"start\":\""+s+"\",\"duration\":\""+d+"\"}";
    ESP_LOGD("Spa/filter2/state", payld.c_str());

    have_filtersettings = 2;
  }


  void decodeFault() {
    SpaFaultLog.totEntry = Q_in[5];
    SpaFaultLog.currEntry = Q_in[6];
    SpaFaultLog.faultCode = Q_in[7];
    switch (SpaFaultLog.faultCode) { // this is a inelegant way to do it, a lookup table would be better
      case 15:
        SpaFaultLog.faultMessage = "Sensors are out of sync";
        break;
      case 16:
        SpaFaultLog.faultMessage = "The water flow is low";
        break;
      case 17:
        SpaFaultLog.faultMessage = "The water flow has failed";
        break;
      case 18:
        SpaFaultLog.faultMessage = "The settings have been reset";
        break;
      case 19:
        SpaFaultLog.faultMessage = "Priming Mode";
        break;
      case 20:
        SpaFaultLog.faultMessage = "The clock has failed";
        break;
      case 21:
        SpaFaultLog.faultMessage = "The settings have been reset";
        break;
      case 22:
        SpaFaultLog.faultMessage = "Program memory failure";
        break;
      case 26:
        SpaFaultLog.faultMessage = "Sensors are out of sync -- Call for service";
        break;
      case 27:
        SpaFaultLog.faultMessage = "The heater is dry";
        break;
      case 28:
        SpaFaultLog.faultMessage = "The heater may be dry";
        break;
      case 29:
        SpaFaultLog.faultMessage = "The water is too hot";
        break;
      case 30:
        SpaFaultLog.faultMessage = "The heater is too hot";
        break;
      case 31:
        SpaFaultLog.faultMessage = "Sensor A Fault";
        break;
      case 32:
        SpaFaultLog.faultMessage = "Sensor B Fault";
        break;
      case 34:
        SpaFaultLog.faultMessage = "A pump may be stuck on";
        break;
      case 35:
        SpaFaultLog.faultMessage = "Hot fault";
        break;
      case 36:
        SpaFaultLog.faultMessage = "The GFCI test failed";
        break;
      case 37:
        SpaFaultLog.faultMessage = "Standby Mode (Hold Mode)";
        break;
      default:
        SpaFaultLog.faultMessage = "Unknown error";
        break;
    }
    SpaFaultLog.daysAgo = Q_in[8];
    SpaFaultLog.hour = Q_in[9];
    SpaFaultLog.minutes = Q_in[10];
    ESP_LOGD("Spa/fault/Entries", String(SpaFaultLog.totEntry).c_str());
    ESP_LOGD("Spa/fault/Entry", String(SpaFaultLog.currEntry).c_str());
    ESP_LOGD("Spa/fault/Code", String(SpaFaultLog.faultCode).c_str());
    ESP_LOGD("Spa/fault/Message", SpaFaultLog.faultMessage.c_str());
    ESP_LOGD("Spa/fault/DaysAgo", String(SpaFaultLog.daysAgo).c_str());
    ESP_LOGD("Spa/fault/Hours", String(SpaFaultLog.hour).c_str());
    ESP_LOGD("Spa/fault/Minutes", String(SpaFaultLog.minutes).c_str());
    have_faultlog = 2;
    //ESP_LOGD("Spa/debug/have_faultlog", "have the faultlog, #2");
  }

  void on_set_temp(int temp) {
    if (temp >= 62 || temp < 105) {
      settemp = temp;
      send = 0xff;
    }
  }

  void on_set_hour(int hour) {
    if(hour >= 0 || hour <= 23) {
      sethour = hour;
      send = 0x21;
    }
  }

  void on_set_minute(int minute) {
    if(minute >= 0 || minute <= 59) {
      setminute = minute;
      send = 0x21;
    }
  }

  void on_toggle_light() {
    send = 0x11;
  }
  void on_toggle_jet1() {
    send = 0x04;
  }
  void on_toggle_jet2() {
    send = 0x05;
  }

  void setup() override {
    Q_in.clear();
    Q_out.clear();
    register_service(&SpaReader::on_set_temp, "set_target_temp", {"temp"});
    register_service(&SpaReader::on_set_hour, "set_hour", {"hour"});
    register_service(&SpaReader::on_set_minute, "set_minute", {"minute"});
    register_service(&SpaReader::on_toggle_light, "toggle_light");
    register_service(&SpaReader::on_toggle_jet1, "toggle_jet1");
    register_service(&SpaReader::on_toggle_jet2, "toggle_jet2");
  }
  
  void loop() override {
    yield();
    while (available()) {
      x = read();
      Q_in.push(x);
      
      // Drop until SOF is seen
      if (Q_in.first() != 0x7E) {
        Q_in.clear();
      } 
        
      // Double SOF-marker, drop last one
      if (Q_in[1] == 0x7E && Q_in.size() > 1) Q_in.pop();

      // Complete package
      //if (x == 0x7E && Q_in[0] == 0x7E && Q_in[1] != 0x7E) {
      if (x == 0x7E && Q_in.size() > 2) {

        // Unregistered or yet in progress
        if (id == 0) {
          ESP_LOGD("Spa/node/id", "Unregistered");
          //if (Q_in[2] == 0xFE) print_msg(Q_in);
          print_msg(Q_in);
          // FE BF 02:got new client ID
          if (Q_in[2] == 0xFE && Q_in[4] == 0x02) {
            id = Q_in[5];
            if (id > 0x2F) id = 0x2F;
            ESP_LOGD("Spa/node/id", "Got ID, acknowledging");
            ID_ack();
            ESP_LOGD("Spa/node/id", String(id).c_str());
          }

          // FE BF 00:Any new clients?
          if (Q_in[2] == 0xFE && Q_in[4] == 0x00) {
            ESP_LOGD("Spa/node/id", "Requesting ID");
            ID_request();
          }
        } else if (Q_in[2] == id && Q_in[4] == 0x06) { // we have an ID, do clever stuff
            // id BF 06:Ready to Send
            if (send == 0x21) {
              Q_out.push(id);
              Q_out.push(0xBF);
              Q_out.push(0x21);
              Q_out.push(sethour);
              Q_out.push(setminute);
            } else if (send == 0xff) {
              // 0xff marks dirty temperature for now
              Q_out.push(id);
              Q_out.push(0xBF);
              Q_out.push(0x20);
              Q_out.push(settemp);
            } else if (send == 0x00) {
              if (have_config == 0) { // Get configuration of the hot tub
                Q_out.push(id);
                Q_out.push(0xBF);
                Q_out.push(0x22);
                Q_out.push(0x00);
                Q_out.push(0x00);
                Q_out.push(0x01);
                ESP_LOGD("Spa/config/status", "Getting config");
                have_config = 1;
              } else if (have_faultlog == 0) { // Get the fault log
                Q_out.push(id);
                Q_out.push(0xBF);
                Q_out.push(0x22);
                Q_out.push(0x20);
                Q_out.push(0xFF);
                Q_out.push(0x00);
                have_faultlog = 1;
                ESP_LOGD("Spa/debug/have_faultlog", "requesting fault log, #1");
              } else if ((have_filtersettings == 0) && (have_faultlog == 2)) { // Get the filter cycles log once we have the faultlog
                Q_out.push(id);
                Q_out.push(0xBF);
                Q_out.push(0x22);
                Q_out.push(0x01);
                Q_out.push(0x00);
                Q_out.push(0x00);
                ESP_LOGD("Spa/debug/have_filtersettings", "requesting filter settings, #1");
                have_filtersettings = 1;
              } else {
                // A Nothing to Send message is sent by a client immediately after a Clear to Send message if the client has no messages to send.
                Q_out.push(id);
                Q_out.push(0xBF);
                Q_out.push(0x07);
              }
            } else {
              // Send toggle commands
              Q_out.push(id);
              Q_out.push(0xBF);
              Q_out.push(0x11);
              Q_out.push(send);
              Q_out.push(0x00);
            }

            rs485_send();
            send = 0x00;
        } else if (Q_in[2] == id && Q_in[4] == 0x2E) {
          if (last_state_crc != Q_in[Q_in[1]]) {
            decodeSettings();
          }
        } else if (Q_in[2] == id && Q_in[4] == 0x28) {
          if (last_state_crc != Q_in[Q_in[1]]) {
            decodeFault();
          }
        } else if (Q_in[2] == 0xFF && Q_in[4] == 0x13) { // FF AF 13:Status Update - Packet index offset 5
          if (last_state_crc != Q_in[Q_in[1]]) {
            decodeState();
          }
        } else if (Q_in[2] == id && Q_in[4] == 0x23) { // FF AF 23:Filter Cycle Message - Packet index offset 5
          if (last_state_crc != Q_in[Q_in[1]]) {
            ESP_LOGD("Spa/debug/have_faultlog", "decoding filter settings");
            decodeFilterSettings();
          }
        } else {
          // DEBUG for finding meaning
          //if (Q_in[2] & 0xFE || Q_in[2] == id)
          //print_msg(Q_in);
        }

        // Clean up queue
        yield();
        Q_in.clear();
      }
      lastrx = millis();
    }
  }
};