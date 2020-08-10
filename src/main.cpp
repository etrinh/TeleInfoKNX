/*
 * Linky TeleInfo KNX
 */

#include <Arduino.h>
#include <knx.h>

#define MIN(X,Y)    ((X)<(Y)?(X):(Y))
#define MAX(X,Y)    ((X)>(Y)?(X):(Y))

#define PIN_PROG_SWITCH   PB5
#define PIN_PROG_LED      PA4
#define PROG_TIMEOUT      ( 15 * 60 * 1000 )    // 15 mins
#define PIN_TPUART_RX     PB6   // stm32 knx uses Serial2 (pins 16,17)
#define PIN_TPUART_TX     PB7
#define PIN_TPUART_SAVE   PB3   // Unused
#define PIN_TPUART_RESET  PB4   // Unused

#define PIN_TELE_RX       PA3
#define PIN_TELE_TX       PA2   // Unconnected

#define ADJ_UART_SPEED    1273
#define BUFFERSIZE        512U


class NullStream : public Stream
{
public:
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
    virtual size_t write(uint8_t) { return 0; }
} nullDevice;

struct TeleInfoDataType {
  const __FlashStringHelper* key;
  unsigned char keySize;
  enum Type : uint8_t { INT, STRING } type;
  unsigned char size;
  struct { short mainGroup; short subGroup; } dpt;
  unsigned char goSend;
  bool activeSend;
};
#define Dpt(M,S)    { M, S }
const TeleInfoDataType TeleInfoParam[] PROGMEM = {
  { F("ADCO "), 5, TeleInfoDataType::STRING, 12, DPT_String_ASCII, 1, false },
  { F("OPTARIF "), 8, TeleInfoDataType::STRING, 4, DPT_String_ASCII, 2, false },
  { F("ISOUSC "), 7, TeleInfoDataType::INT, 2, DPT_Value_Electric_Current, 3, false },
  { F("BASE "), 5, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 4, true },
  { F("HCHC "), 5, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 5, true },
  { F("HCHP "), 5, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 6, true },
  { F("EJP HN "), 7, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 7, true },
  { F("EJP HPM "), 8, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 8, true },
  { F("BBR HC JB "), 9, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 9, true },
  { F("BBR HP JB "), 9, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 10, true },
  { F("BBR HC JW "), 9, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 11, true },
  { F("BBR HP JW "), 9, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 12, true },
  { F("BBR HC JR "), 9, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 13, true },
  { F("BBR HP JR "), 9, TeleInfoDataType::INT, 9, DPT_ActiveEnergy, 14, true },
  { F("PEJP "), 5, TeleInfoDataType::STRING, 2, DPT_String_ASCII, 15, true },
  { F("PTEC "), 5, TeleInfoDataType::STRING, 4, DPT_String_ASCII, 16, true },
  { F("DEMAIN "), 7, TeleInfoDataType::STRING, 4, DPT_String_ASCII, 17, true },
  { F("IINST "), 6, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current, 18, true },
  { F("ADPS "), 5, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current, 19, true },
  { F("IMAX "), 5, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current, 20, true },
  { F("PAPP "), 5, TeleInfoDataType::INT, 5, DPT_Value_2_Count, 21, true },
  { F("HHPHC "), 6, TeleInfoDataType::INT, 1, DPT_Bool, 22, true }
};
#undef Dpt
const unsigned int TeleInfoCount = sizeof(TeleInfoParam)/sizeof(TeleInfoParam[0]);

class TeleInfo
{
    HardwareSerial mSerial = HardwareSerial(PIN_TELE_RX, PIN_TELE_TX);
    char mBuffer[BUFFERSIZE];    // No '\0'
    int mBufferLen = 0;
    int mFrameErrorCount = 0;

    // Hold the memory buffer for all teleinfo
    struct TeleInfoDataStruct
    {
        const TeleInfoDataType* conf; 
        union {
            char str[13] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            uint32_t num;
        } value;
    } mTeleInfoData[TeleInfoCount];

    static inline KNXValue value(const TeleInfoDataStruct& val)
    {
        if (val.conf->type == TeleInfoDataType::STRING) {
            return KNXValue(val.value.str);
        }
        else {
            return KNXValue(val.value.num);
        }
    }

    static inline bool value(TeleInfoDataStruct& val, const char* begin, const char* end)
    {
        begin += val.conf->keySize;
        if (begin < end) {
            unsigned int size = MIN(end - begin, val.conf->size);
            if (val.conf->type == TeleInfoDataType::STRING) {
                if (memcmp(val.value.str, begin, size) != 0 && val.value.str[size] == '\0') {
                    memcpy(val.value.str, begin, size);
                    val.value.str[size] = '\0';
                    return true;
                }
            }
            else {
                uint32_t value = 0;
                end = begin + size;
                for (; begin != end; ++begin) {
                    const unsigned char v = (const unsigned char)(*begin - '0');
                    if (v > 9) break;
                    value = value * 10 + v;
                }
                if (val.value.num != value) {
                    val.value.num = value;
                    return true;
                }
            }
        }
        return false;
    }

    static inline bool validChecksum(const char* begin, const char* end)
    {
        uint16_t sum = 0;
        bool spaceFound = false;
        --end;
        for (; begin != end;) {
            const char c = *begin++;
            if (c == ' ') {
                if (spaceFound) {   // checksum after second space
                    return (((char)sum & 0x3F) + 0x20) == *begin;
                }
                spaceFound = true;
            }
            sum += c;
        }
        return false;
    }

    inline void calibrateUart() {
        // Find best Serial Speed (Framing error issue with 1200baud seems working in [1221,1325] range - mid 1273)
        const int start = 1100, end = 1400;
        int minFE = 0x7fffffff, bestBaudStart = start, bestBaudEnd = end;
        for (int baud = start; baud < end; baud += 20) {
            digitalWrite(PIN_PROG_LED, HIGH);
            mSerial.begin(baud, SERIAL_7E1);
            int FE = 0;
            for (int i = 0; i < 16 && FE < minFE; ++i) {
                while (!mSerial.available()) { delay(1); }
                int c = mSerial.read();
                if (c <= 0) {
                    ++FE;
                }
            }
            if (FE < minFE) { 
                minFE = FE;
                bestBaudStart = baud;
            }
            if (FE == minFE) {
                bestBaudEnd = baud;
            }
            mSerial.end();
            digitalWrite(PIN_PROG_LED, LOW);
        }
        mSerial.begin((bestBaudEnd + bestBaudStart) / 2, SERIAL_7E1);
    }
public:
    void init()
    {
        mSerial.begin(ADJ_UART_SPEED, SERIAL_7E1);

        const TeleInfoDataType* param = TeleInfoParam;
        for (TeleInfoDataStruct * data = mTeleInfoData; data != mTeleInfoData + TeleInfoCount; ++data, ++ param) {
            data->conf = param;
            knx.getGroupObject(data->conf->goSend).dataPointType(Dpt(data->conf->dpt.mainGroup, data->conf->dpt.subGroup));
        }
    }

    void loop()
    {
        for (;;) {
            unsigned int pending = mSerial.available();
            if (pending == 0)
                break;

            while (pending > 0) {
                if (mBufferLen == BUFFERSIZE) {
                    mBufferLen = 0;  // Security - Reset buffer if full with dummies
                    // Recalibrate UART, maybe framing error
                    calibrateUart();
                    break;
                }
                unsigned int rcv = 0;
                char* ptr = mBuffer + mBufferLen;
                while (rcv < MIN(BUFFERSIZE - mBufferLen, pending)) {
                    int c = mSerial.read();
                    if (c < 0) {
                        break;
                    }
                    *ptr++ = (char)c;
                    ++rcv;
                }
                if (rcv == 0)
                    break;
                pending -= rcv;
                mBufferLen += rcv;
                const char* currentBuffer = mBuffer;
                for (;;) {
                    // extract first line if 
                    const char* eol = currentBuffer;
                    for (; eol != mBuffer + mBufferLen; ++eol) {
                        if (*eol == '\x0d') {
                            break;
                        }
                    }
                    if(eol == mBuffer + mBufferLen)
                        break;
                    // search first valid character
                    for (; currentBuffer != eol; ++currentBuffer) {
                        const char c = *currentBuffer;
                        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ' ') {
                            break;
                        }
                    }
                    const unsigned int lineLen = eol - currentBuffer;
                    if (TeleInfo::validChecksum(currentBuffer, eol)) {
                        for (TeleInfoDataStruct * data = mTeleInfoData; data != mTeleInfoData + TeleInfoCount; ++data) {
                            if (lineLen > data->conf->keySize && memcmp(currentBuffer, data->conf->key, data->conf->keySize) == 0) {
                                if (TeleInfo::value(*data, currentBuffer, eol) && data->conf->activeSend) {
                                    knx.getGroupObject(data->conf->goSend).value(TeleInfo::value(*data));
                                }
                                break;
                            }
                        }
                    }
                    currentBuffer = eol + 1;
                }
                mBufferLen -= currentBuffer - mBuffer;
                memmove(mBuffer, currentBuffer, mBufferLen);
            }
        }
    }
};
TeleInfo teleinfo;

void setup()
{
    ArduinoPlatform::SerialDebug = &nullDevice;
    static HardwareSerial serialTpuart(PIN_TPUART_TX, PIN_TPUART_RX);
    knx.platform().knxUart(&serialTpuart);
    knx.ledPin(PIN_PROG_LED);
    knx.ledPinActiveOn(HIGH);
    knx.buttonPin(PIN_PROG_SWITCH);
    knx.buttonPinInterruptOn(RISING);

    // Init device
    knx.version(1);                                         // PID_VERSION
    static const uint8_t orderNumber = 0;
    knx.orderNumber(&orderNumber);                          // PID_ORDER_INFO
    // knx.manufacturerId(0xfa);                               // PID_SERIAL_NUMBER (2 first bytes) - 0xfa for KNX Association
    knx.bauNumber(0x4c4e4b59 /* = 'LNKY'*/);     // PID_SERIAL_NUMBER (4 last bytes)
    static const uint8_t hardwareType [] = { 0, 0, 0, 0, 0, 0 };
    knx.hardwareType(hardwareType);                         // PID_HARDWARE_TYPE
    knx.bau().deviceObject().induvidualAddress(1);

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
    if(knx.configured()) {
        teleinfo.loop();
    }

    static uint32_t timerProgMode = 0;
    if (knx.progMode()) {
        if (timerProgMode == 0) {
            timerProgMode = millis();
        }
        else {
            if (millis() - timerProgMode > PROG_TIMEOUT) {
                knx.progMode(false);
                timerProgMode = 0;
            }
        }
    }
    else {
        timerProgMode = 0;
    }
}
