/*
 * Linky TeleInfo KNX
 * 
 * Emit
 * 
 * Receive on request changes
 * 
 * 
 * 
 */

#include <Arduino.h>
#include <knx.h>

struct TeleInfoDataType {
  const char* key;
  const enum Type { INT, STRING } type;
  const unsigned char size;
  const Dpt dpt;
  const unsigned char goSend;
//  const unsigned char paramAddr;
  bool activeSend = true;
  bool passiveSend = true;
  union {
    char str[13];
    uint32_t num;
  } value;
};

static TeleInfoDataType TeleInfoData[] = {
  {"ADCO ", TeleInfoDataType::STRING, 12, DPT_String_ASCII, 1},
  {"OPTARIF ", TeleInfoDataType::STRING, 4, DPT_String_ASCII, 2},
  {"ISOUSC ", TeleInfoDataType::INT, 2, DPT_Value_Electric_Current, 3},
  {"BASE ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 4},
  {"HCHC ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 5},
  {"HCHP ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 6},
  {"EJP HN ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 7},
  {"EJP HPM ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 8},
  {"BBR HC JB ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 9},
  {"BBR HP JB ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 10},
  {"BBR HC Jw ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 11},
  {"BBR HP JW ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 12},
  {"BBR HC JR ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 13},
  {"BBR HP JR ", TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 14},
  {"PEJP ", TeleInfoDataType::STRING, 2, DPT_String_ASCII, 15},
  {"PTEC ", TeleInfoDataType::STRING, 4, DPT_String_ASCII, 16},
  {"DEMAIN ", TeleInfoDataType::STRING, 4, DPT_String_ASCII, 17},
  {"IINST ", TeleInfoDataType::INT, 3, DPT_Value_Electric_Current, 18},
  {"ADPS ", TeleInfoDataType::INT, 3, DPT_Value_Electric_Current, 19},
  {"IMAX ", TeleInfoDataType::INT, 3, DPT_Value_Electric_Current, 20},
  {"PAPP ", TeleInfoDataType::INT, 5, DPT_Value_2_Count, 21},
  {"HHPHC ", TeleInfoDataType::INT, 1, DPT_Bool, 22}
};
static const unsigned int TeleInfoCount = sizeof(TeleInfoData)/sizeof(TeleInfoData[0]);

#define PIN_PROG_SWITCH   PB5
#define PIN_PROG_LED      PA4
#define PIN_TELE_RX       PA3
#define PIN_TELE_TX       PA2   // Unused
#define PIN_TPUART_RX     PB6   // stm32 knx uses Serial2 (pins 16,17)
#define PIN_TPUART_TX     PB7
#define PIN_TPUART_SAVE   PB3   // Unused
#define PIN_TPUART_RESET  PB4   // Unused

#define ACTIVESEND        1
#define PASSIVESEND       2

#define MIN(X,Y)    ((X)<(Y)?(X):(Y))
#define MAX(X,Y)    ((X)>(Y)?(X):(Y))

struct TeleInfo
{
    HardwareSerial serial = HardwareSerial(PIN_TELE_RX, PIN_TELE_TX);
    static KNXValue value(const TeleInfoDataType& val)
    {
        switch (val.type) {
            case TeleInfoDataType::STRING: {
                return KNXValue(val.value.str);
            }; break;
            default: {
                return KNXValue(val.value.num);
            }; break;
        }
    }

    static void value(TeleInfoDataType& val, const String& buffer)
    {
        unsigned int offset = strlen(val.key);
        if (buffer.length() < offset) return;
        unsigned int size = MIN(buffer.length() - offset, val.size);
        if (val.type == TeleInfoDataType::STRING) {
            memcpy(val.value.str, buffer.c_str(), size);
            val.value.str[size] = '\0';
        }
        else if (val.type == TeleInfoDataType::INT) {
            uint32_t value = 0;
            for (unsigned int i = 0; i < size; ++i) {
                char v = buffer[offset + i];
                if (v >= '0' && v <= '9') {
                    value = value * 10 + (v-'0');
                }
                else break;
            }
            val.value.num = value;
        }
    }

    static bool validChecksum(const String& buffer)
    {
        uint16_t sum = 0;
        bool spaceFound = false;
        unsigned int checksumOffset = 0;
        for (unsigned int i = 0; i < buffer.length(); ++i) { // from start and stop when second space for checsum is found
            char c = buffer.charAt(i);
            if (c == ' ') {
                if (spaceFound) {
                    checksumOffset = i + 1;
                    break;
                }
                spaceFound = true;
            }
            sum += c;
        }
        sum = (sum & 0x3F) + 0x20;
        return sum == buffer.charAt(checksumOffset);
    }

    void init()
    {
        // Init Serial
        serial.begin(1200, SERIAL_7E1);

        for (unsigned int index = 0; index < TeleInfoCount; ++index) {
            TeleInfoDataType & data = TeleInfoData[index];
            char mode = ACTIVESEND | PASSIVESEND; // knx.paramByte(data.paramAddr);
            data.activeSend = mode & ACTIVESEND;
            data.passiveSend = mode & PASSIVESEND;
            memset(data.value.str, 0, sizeof(data.value.str));
            knx.getGroupObject(data.goSend).dataPointType(data.dpt);
            if (data.passiveSend) {
                knx.getGroupObject(data.goSend).callback([&data](GroupObject& go) {
                    go.value(value(data));
                });
            }
        }
    }
  
    void loop()
    {
        static String buffer;

        // read buffer
        // check buffer
        if (serial.available() == 0)
            return;
        buffer += serial.readString();
        if (buffer.length() > 512)
            buffer = String();

        // extract first line if any
        String line;
        int eol = buffer.indexOf('\x0d');
        if (eol >= 0) {
            line = buffer.substring(0, eol);
            buffer = buffer.substring(eol + 1);
        }
        if (!TeleInfo::validChecksum(line))
            return;
        for (unsigned int index = 0; index < TeleInfoCount; ++index) {
            TeleInfoDataType & data = TeleInfoData[index];
            if (line.startsWith(data.key)) {
                TeleInfo::value(data, line);
                if (data.activeSend) {
                    knx.getGroupObject(data.goSend).value(TeleInfo::value(data));
                }
                break;
            }
        }
    }
} teleinfo;

void setup()
{
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    WiFi.mode(WIFI_OFF);
 #if defined(ARDUINO_ARCH_ESP32)
    btStop();
 #endif
#endif
    Serial2.setRx(PIN_TPUART_RX);
    Serial2.setTx(PIN_TPUART_TX);
    Serial2.begin(19200, SERIAL_8E1);   // Or 9N1 for register read (see datasheet p26)?
    knx.ledPin(PIN_PROG_LED);
    knx.ledPinActiveOn(HIGH);
    knx.buttonPin(PIN_PROG_SWITCH);
    knx.buttonPinInterruptOn(RISING);

    // read adress table, association table, groupobject table and parameters from eeprom
    knx.readMemory();

    if (knx.configured()) {
        teleinfo.init();
    }

    // start the framework.
    knx.start();
}

void loop() 
{
    // don't delay here to much. Otherwise you might lose packages or mess up the timing with ETS
    knx.loop();

    // only run the application code if the device was configured with ETS
    if(!knx.configured())
        return;

    static unsigned long lastTime = 0;
    unsigned long time = millis();
    if (time - lastTime < 10)
        return;
    teleinfo.loop();
    lastTime = time;
}
