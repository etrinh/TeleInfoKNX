/*
 * TeleInfo KNX
 *  Can be used with French TeleInfo systems
 *  Compatible with Linky in "Historic" mode and "Blue" electric meters
 *  GPL-3.0 License
 * Copyright 2020-2021 ZapDesign Innovative - Author: Eric Trinh
 */

#include <Arduino.h>
#include <knx.h>

#define VERSION_MAJOR                       1
#define VERSION_MINOR                       0
#define VERSION_REVISION                    0

#define PIN_PROG_SWITCH                     PB5
#define PIN_PROG_LED                        PA4
#define PROG_TIMEOUT                        ( 15 * 60 * 1000 )    // 15 mins
#define PIN_TPUART_RX                       PB6     // stm32 knx uses Serial2 (pins 16,17)
#define PIN_TPUART_TX                       PB7
#define PIN_TPUART_SAVE                     PB3     // Unused
#define PIN_TPUART_RESET                    PB4     // Unused

#define PIN_TELE_RX                         PA3
#define PIN_TELE_TX                         PA2     // Unconnected

#define TELEINFO_UART_SPEED                 1200
#define TELEINFO_UART_CONFIG                SERIAL_7E1
#define TELEINFO_BUFFERSIZE                 512U

#define HISTORY_FLASH_START                 KNX_FLASH_SIZE

#define ADPS_REPEAT_PERIOD                  ( 10 * 1000 )   // Repeat ADPS > 0 every 10s 

#define HISTORY_MANUALWRITE_TEMPO           ( 60 * 60 * 1000 ) // 1 hour
#define HISTORY_RESET_PROG_SWITCH_DELAY     4000    // 4s
#define HISTORY_RESET_LED_BLINKING_PERIOD   512     // 0.512s
#define RECEPTION_LED_BLINKING_PERIOD       512     // 0.512s

#define FOURCC(a,b,c,d)                     ( ((((uint32_t)(a))<<24) | (((uint32_t)(b))<<16) | (((uint32_t)(c))<<8) | (d)) )

// Restore ram after reset (brownout)
#define INIT_MASK   0x12345678
volatile uint32_t Inited __attribute__ ((section (".noinit")));

class RTCKnx
{
    void setAndAjust() {
        if (isValid()) {
            uint32_t t = RTCKnx::millis();
            if (mLastSync != 0) {
                const int64_t num = 1000 * (secondsSinceReference(mDateTimeStamp) - secondsSinceReference(mLastDateTime)) * mCorr.denum / mCorr.num;
                const int64_t denum = ((num & ~(int64_t)UINT32_MAX)) + (t - mLastSync);
                if (num != 0 && denum != 0 && num * 10 >= denum * 9 && num * 10 <= denum * 11) {
                    mCorr.num = num; mCorr.denum = denum;
                }
            }
            bool bInit = mShift == 0;
            mLastSync = mShift = t|1;
            mLastDateTime = mDateTimeStamp;
            if (bInit && mDayCallback) {
                mDayCallback(Init);
            }
        }
    }
  public:
    RTCKnx() {}
    void init(int baseAddr, uint16_t baseGO) {
        if (mPersistentTimer != 0) new (&mDayCallback) std::function<void(DateChange)>();
        mLastSync = mLastRequested = 0;
        mTimerOffset = mPersistentTimer; // Load last timer before reset
        mParams.period = knx.paramInt(baseAddr) * 60 * 1000;    // In minutes
        knx.getGroupObject(m_GO.date = ++baseGO).dataPointType(DPT_Date);
        knx.getGroupObject(m_GO.date).callback([this](GroupObject& go) {
            const struct tm date = go.value();
            mDateTimeStamp.tm_year = date.tm_year; mDateTimeStamp.tm_mon = date.tm_mon - 1; mDateTimeStamp.tm_mday = date.tm_mday;
            setAndAjust();
        });
        knx.getGroupObject(m_GO.time = ++baseGO).dataPointType(Dpt(10, 1, 1)/*DPT_TimeOfDay*/);
        knx.getGroupObject(m_GO.time).callback([this](GroupObject& go) {
            const struct tm time = go.value();
            mDateTimeStamp.tm_hour = time.tm_hour; mDateTimeStamp.tm_min = time.tm_min; mDateTimeStamp.tm_sec = time.tm_sec;
            setAndAjust();
        });
        knx.getGroupObject(m_GO.dateTime = ++baseGO).dataPointType(DPT_DateTime);
        knx.getGroupObject(m_GO.dateTime).callback([this](GroupObject& go) {
            const struct tm time = go.value();
            mDateTimeStamp.tm_year = time.tm_year; mDateTimeStamp.tm_mon = time.tm_mon - 1; mDateTimeStamp.tm_mday = time.tm_mday; 
            mDateTimeStamp.tm_hour = time.tm_hour; mDateTimeStamp.tm_min = time.tm_min; mDateTimeStamp.tm_sec = time.tm_sec;
            setAndAjust();
        });
        knx.getGroupObject(m_GO.dateTimeStatus = ++baseGO).dataPointType(DPT_DateTime);
        if (isValid())
            updateStatus();
    }
    typedef struct { uint16_t tm_sec /*[0-59]*/, tm_min /*[0-59]*/, tm_hour /*[0-23]*/, tm_mday /*[1-31]*/, tm_mon /*[0-11]*/, tm_year /*Year*/; } DateTime;
    const DateTime& dateTime() {
        // Adjust clock deviation
        if (mShift != 0) {
            uint32_t current = RTCKnx::millis()|1;
            if ((current - mShift) < 1000) return mDateTimeStamp;
            int32_t adjmSec = (current - mShift) * mCorr.num / mCorr.denum;
            int32_t restmSec = adjmSec % 1000;
            mDateTimeStamp.tm_sec += adjmSec / 1000;
            mShift = (current - restmSec)|1;
            if (mDateTimeStamp.tm_sec >= 60) {
                mDateTimeStamp.tm_min += mDateTimeStamp.tm_sec/60;
                mDateTimeStamp.tm_sec = mDateTimeStamp.tm_sec%60;
            }
            if (mDateTimeStamp.tm_min >= 60) {
                mDateTimeStamp.tm_hour += mDateTimeStamp.tm_min/60;
                mDateTimeStamp.tm_min = mDateTimeStamp.tm_min%60;
            }
            if (mDateTimeStamp.tm_hour >= 24) {
                mDateTimeStamp.tm_mday += mDateTimeStamp.tm_hour/24;
                mDateTimeStamp.tm_hour = mDateTimeStamp.tm_hour%24;
            }
            bool bEnd = false;
            while (!bEnd) {
                switch (mDateTimeStamp.tm_mon) {
                    case 0: case 2: case 4: case 6: case 7: case 9: case 11: {
                        if (mDateTimeStamp.tm_mday > 31) {
                            ++mDateTimeStamp.tm_mon;    mDateTimeStamp.tm_mday -= 31;
                        }
                        else bEnd = true;
                    }; break;
                    case 1: {
                        int februaryDays = ((mDateTimeStamp.tm_year & 3) == 0)&&(((mDateTimeStamp.tm_year % 100) != 0)||((mDateTimeStamp.tm_year % 400)==0))?29:28;
                        if (mDateTimeStamp.tm_mday > februaryDays) {
                            ++mDateTimeStamp.tm_mon;    mDateTimeStamp.tm_mday -= februaryDays;
                        }
                        else bEnd = true;
                    }; break;
                    case 3: case 5: case 8: case 10: {
                        if (mDateTimeStamp.tm_mday > 30) {
                            ++mDateTimeStamp.tm_mon;    mDateTimeStamp.tm_mday -= 30;
                        }
                        else bEnd = true;
                    }; break;
                    default:
                        ++mDateTimeStamp.tm_year;   mDateTimeStamp.tm_mon -= 12;
                }
            }
            updateStatus();
        }
        return mDateTimeStamp;
    }
    void updateStatus() {
        knx.getGroupObject(m_GO.dateTimeStatus).valueNoSend(tm{mDateTimeStamp.tm_sec, mDateTimeStamp.tm_min, mDateTimeStamp.tm_hour, mDateTimeStamp.tm_mday, mDateTimeStamp.tm_mon + 1, mDateTimeStamp.tm_year?mDateTimeStamp.tm_year:1900, 0, 0, 0});
    }
    static int64_t secondsSinceReference(const DateTime& dt) {
        enum CumulatedDays { Jan = 31, Feb = Jan + 28, Mar = Feb + 31, Apr = Mar + 30, May = Apr + 31, Jun = May + 30, Jul = Jun + 31, Aug = Jul + 31, Sep = Aug + 30, Oct = Sep + 31, Nov = Oct + 30, Dec = Nov + 31 };
        static const uint16_t daysToMonth[] = { 0, Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov };
        uint16_t leapYears = dt.tm_year;
        if (dt.tm_mon < 2) --leapYears; // Check if the current year needs to be considered for the count of leap years or not
        leapYears = leapYears / 4 - leapYears / 100 + leapYears / 400;
        return (int64_t)dt.tm_sec + (int64_t)dt.tm_min * 60 + (int64_t)dt.tm_hour * 60 * 60 + ((int64_t)dt.tm_mday - 1 + daysToMonth[dt.tm_mon%12] + leapYears + (int64_t)(dt.tm_year + dt.tm_mon / 12 - 2020) * 365) * 60 * 60 * 24;
    }
    void loop() {
        uint32_t currentMillis = RTCKnx::millis();
        if (currentMillis - mDelay < 100) return;
        mDelay = currentMillis;
        // Ask Date/Time from the bus when required
        if (mParams.period != 0 && (mLastRequested == 0 || ((currentMillis - mLastSync) > mParams.period && (currentMillis - mLastRequested) > mParams.period))) {
            knx.getGroupObject(m_GO.date).requestObjectRead();
            knx.getGroupObject(m_GO.time).requestObjectRead();
            knx.getGroupObject(m_GO.dateTime).requestObjectRead();
            mLastRequested = currentMillis;
        }
        if (mDateTimeStamp.tm_mday == 0 || !mDayCallback) return;
        const DateTime& currentDateTime = dateTime();
        if (mLastEmittedDay.tm_mday == 0) {
            mLastEmittedDay = currentDateTime;
            return;
        }
        DateChange change = Same;
        if (currentDateTime.tm_year == mLastEmittedDay.tm_year && currentDateTime.tm_mon == mLastEmittedDay.tm_mon && currentDateTime.tm_mday > mLastEmittedDay.tm_mday) {
            change = Day;
        }
        else if (currentDateTime.tm_year == mLastEmittedDay.tm_year && currentDateTime.tm_mon > mLastEmittedDay.tm_mon) {
            change = Month;
        }
        else if (currentDateTime.tm_year > mLastEmittedDay.tm_year) {
            change = Year;
        }
        if (change != Same) {
            mDayCallback(change);
            mLastEmittedDay = currentDateTime;
        }
    }
    enum DateChange { Init = -2, Same = -1, Day = 0, Month, Year };
    void setNotifier(const std::function<void(DateChange)>& notifier) { mDayCallback = notifier; }
    uint32_t millis() { return mPersistentTimer = mTimerOffset + ::millis(); }
    bool isValid() const { return mDateTimeStamp.tm_mday != 0 && mDateTimeStamp.tm_hour != 0xffff; } // Date + Time must be both set
  private:
    uint32_t mPersistentTimer = 0;   // Should stay after reset
    uint32_t mTimerOffset = 0;
    struct { int64_t num = 1, denum = 1; } mCorr;
    DateTime mDateTimeStamp = { 0, 0, 0xffff, 0, 0, 0 };
    DateTime mLastEmittedDay = {0};
    DateTime mLastDateTime = {0};
    uint32_t mShift = 0;
    uint32_t mLastSync = 0;
    uint32_t mDelay = 0;
    uint32_t mLastRequested = 0;
    struct {
        uint32_t period;
    } mParams;
    struct {
        uint16_t date;
        uint16_t time;
        uint16_t dateTime;
        uint16_t dateTimeStatus;
    } m_GO;
    std::function<void(DateChange)> mDayCallback;
  public:
    enum { NBGO = sizeof(m_GO)/sizeof(uint16_t), SIZEPARAMS = sizeof(mParams) };
};
uint8_t rtcHolder[sizeof(RTCKnx)] __attribute__ ((section (".noinit")));
RTCKnx& rtc = *(RTCKnx*)rtcHolder;

struct TeleInfoDataType {
    const char* key;
    uint8_t keySize : 4;
    enum Type { INT = 0, STRING, OPTARIF, PTEC, DEMAIN, HHPHC } type : 4;
    uint8_t size;
    struct { short mainGroup; short subGroup; } dpt;
};
#define Dpt(M,S)    { M, S }
static const TeleInfoDataType TeleInfoParam[] PROGMEM = {
    { PSTR("ADCO "), 5, TeleInfoDataType::STRING, 12, DPT_String_ASCII },
    { PSTR("OPTARIF "), 8, TeleInfoDataType::OPTARIF, 4, DPT_Value_1_Ucount },
    { PSTR("ISOUSC "), 7, TeleInfoDataType::INT, 2, DPT_Value_Electric_Current },
    { PSTR("BASE "), 5, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("HCHC "), 5, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("HCHP "), 5, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("EJPHN "), 6, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("EJPHPM "), 7, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("BBRHCJB "), 8, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("BBRHPJB "), 8, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("BBRHCJW "), 8, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("BBRHPJW "), 8, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("BBRHCJR "), 8, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("BBRHPJR "), 8, TeleInfoDataType::INT, 9, DPT_ActiveEnergy },
    { PSTR("PEJP "), 5, TeleInfoDataType::INT, 2, DPT_TimePeriodMin },
    { PSTR("PTEC "), 5, TeleInfoDataType::PTEC, 4, DPT_Value_1_Ucount },
    { PSTR("DEMAIN "), 7, TeleInfoDataType::DEMAIN, 4, DPT_Value_1_Ucount },
    { PSTR("IINST "), 6, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("ADPS "), 5, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("IMAX "), 5, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("PAPP "), 5, TeleInfoDataType::INT, 5, DPT_Value_2_Count }, // VA
    { PSTR("HHPHC "), 6, TeleInfoDataType::HHPHC, 1, DPT_Char_ASCII },
    { PSTR("IINST1 "), 7, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("IINST2 "), 7, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("IINST3 "), 7, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("IMAX1 "), 6, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("IMAX2 "), 6, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("IMAX3 "), 6, TeleInfoDataType::INT, 3, DPT_Value_Electric_Current },
    { PSTR("PMAX "), 5, TeleInfoDataType::INT, 5, DPT_Value_Power }
};
#undef Dpt
static const unsigned int TeleInfoCount = sizeof(TeleInfoParam)/sizeof(TeleInfoParam[0]);

class TeleInfo
{
    HardwareSerial mSerial = HardwareSerial(PIN_TELE_RX, PIN_TELE_TX);
    char mBuffer[TELEINFO_BUFFERSIZE];    // No '\0'
    int mBufferLen = 0;

    struct {
        uint32_t period;
        uint32_t realTimeTimeout;
    } mParams;
    enum TarifBlock { Base = 0, HC, HP, /*BLUE, WHITE, RED,*/ TARIFCOUNT };
    struct {
        uint16_t realTimeOnOff;
        uint16_t realTimeOnOffState;
        struct {
            uint16_t today;
            uint16_t yesterday;
            uint16_t thisMonth;
            uint16_t lastMonth;
            uint16_t thisYear;
            uint16_t lastYear;
        } tariff[TARIFCOUNT];
    } mGO;

    uint32_t mRealTimeTimer = 0;
    uint32_t mHistoryLastValue[TARIFCOUNT] = {0};
    uint32_t mHistoryLastSent = 0;
    uint32_t mLastReception = 0;
    uint32_t mLastManualHistoryInit = 0;
    struct {
        RTCKnx::DateTime lastSave;
        struct {
            uint32_t index;
            uint32_t yesterday;
            uint32_t lastMonth;
            uint32_t lastYear;
            uint32_t dayM2;
            uint32_t monthM2;
            uint32_t yearM2;
        } tariff[TARIFCOUNT];
    } mHistory = {0};

    // Hold the memory buffer for all teleinfo
    struct TeleInfoDataStruct {
        uint16_t goSend;
        const TeleInfoDataType* conf; 
        union {
            char str[13];
            uint32_t num;
        } value;
        uint32_t lastSendValueCheckSum;
        uint32_t lastChange;
        uint32_t lastSend ;
    } mTeleInfoData[TeleInfoCount] = {0};

    static inline KNXValue value(const TeleInfoDataStruct& val) {
        switch (val.conf->type) {
            default: case TeleInfoDataType::INT: return KNXValue(val.value.num);
            case TeleInfoDataType::STRING: return KNXValue(val.value.str);
            case TeleInfoDataType::OPTARIF:
                switch (val.value.num&0xffffff00) {
                    case FOURCC('B','A','S',0) /*BASE*/: default: return KNXValue((uint8_t)0);
                    case FOURCC('H','C','.',0) /*HC..*/: return KNXValue((uint8_t)1);
                    case FOURCC('E','J','P',0) /*EJP.*/: return KNXValue((uint8_t)2);
                    case FOURCC('B','B','R',0) /*BBRx*/: return KNXValue((uint8_t)(val.value.num&0x3f));
//                                  - Bit 5: toujours 1
//                                  - Bit 4-3: programme circuit 1: 01-11 _ programme A-C
//                                  - Bit 2-0: programme circuit 2: 000-111 _ programme P0-P7
                }
            case TeleInfoDataType::PTEC:
                switch (val.value.num) {
                    case FOURCC('T','H','.','.') /*Toutes les Heures*/: default: return KNXValue((uint8_t)0);
                    case FOURCC('H','C','.','.') /*Heures Creuses*/: return KNXValue((uint8_t)1);
                    case FOURCC('H','P','.','.') /*Heures Pleines*/: return KNXValue((uint8_t)2);
                    case FOURCC('H','N','.','.') /*Heures Normales*/: return KNXValue((uint8_t)3);
                    case FOURCC('P','M','.','.') /*Heures de Pointe Mobile*/: return KNXValue((uint8_t)4);
                    case FOURCC('H','C','J','B') /*Heures Creuses Jours Bleus*/: return KNXValue((uint8_t)5);
                    case FOURCC('H','C','J','W') /*Heures Creuses Jours Blancs*/: return KNXValue((uint8_t)6);
                    case FOURCC('H','C','J','R') /*Heures Creuses Jours Rouges*/: return KNXValue((uint8_t)7);
                    case FOURCC('H','P','J','B') /*Heures Pleines Jours Bleus*/: return KNXValue((uint8_t)8);
                    case FOURCC('H','P','J','W') /*Heures Pleines Jours Blancs*/: return KNXValue((uint8_t)9);
                    case FOURCC('H','P','J','R') /*Heures Pleines Jours Rouges*/: return KNXValue((uint8_t)10);
                }
            case TeleInfoDataType::DEMAIN:
                switch (val.value.num) {
                    case FOURCC('-','-','-','-'): default: return KNXValue((uint8_t)0);
                    case FOURCC('B','L','E','U'): return KNXValue((uint8_t)1);
                    case FOURCC('B','L','A','N'): return KNXValue((uint8_t)2);
                    case FOURCC('R','O','U','G'): return KNXValue((uint8_t)3);
                }
            case TeleInfoDataType::HHPHC: return KNXValue((uint8_t)val.value.num);
        }
    }

    static inline bool value(TeleInfoDataStruct& val, const char* begin, const char* end) {
        begin += val.conf->keySize;
        const char* vEnd = begin + val.conf->size;
        if (vEnd < end) {
            if (val.conf->type == TeleInfoDataType::STRING) {
                const uint8_t size = val.conf->size;
                if (memcmp(val.value.str, begin, size) != 0) {
                    memcpy(val.value.str, begin, size);
                    val.value.str[size] = '\0';
                    return true;
                }
            }
            else if (val.conf->type == TeleInfoDataType::INT) {
                uint32_t _value = 0;
                for (; begin != vEnd; ++begin) {
                    const unsigned char v = (const unsigned char)(*begin - '0');
                    if (v > 9) break;
                    _value = _value * 10 + v;
                }
                if (val.value.num != _value) {
                    val.value.num = _value;
                    return true;
                }
            }
            else {
                uint32_t _value = 0;
                for (; begin != vEnd; ++begin) {
                    _value = (_value<<8) | *(uint8_t*)begin;
                }
                if (val.value.num != _value) {
                    val.value.num = _value;
                    return true;
                }
            }
        }
        return false;
    }

    static inline bool validChecksum(const char* begin, const char* end) {
        uint16_t sum = 0;
        uint8_t spaceFound = 0;
        for (; begin != end; ++begin) {
            const char c = *begin;
            if (c == ' ') {
                ++spaceFound;
            }
            else if (spaceFound == 2) {
                // checksum after second space
                return (((uint8_t)(sum - ' ') & 0x3F) + 0x20) == c;
            }
            sum += c;
        }
        return false;
    }

    static inline uint32_t simpleChecksum(const char* str) {
        uint32_t result = 0;
        while (*str) result += result + *str++;
        return result;
    }
public:
    TeleInfo() {}
    void init(int baseAddr, uint16_t baseGO) {
        mParams.period = knx.paramInt(baseAddr) * 1000; // In Seconds
        mParams.realTimeTimeout = knx.paramInt(baseAddr + 4) * 60 * 1000;   // In Minutes
        if (mLastReception == 0) {  // Cold reset
            restoreHistory();
        }
        knx.getGroupObject(mGO.realTimeOnOff = ++baseGO).dataPointType(DPT_Switch);
        knx.getGroupObject(mGO.realTimeOnOff).callback([this](GroupObject& go) { mRealTimeTimer = go.value() ? rtc.millis()|1 : 0; });
        knx.getGroupObject(mGO.realTimeOnOffState = ++baseGO).dataPointType(DPT_Switch);
        knx.getGroupObject(mGO.realTimeOnOffState).valueNoSend(mRealTimeTimer != 0);
        for (int i = 0; i < TARIFCOUNT; ++i) {
            knx.getGroupObject(mGO.tariff[i].today = ++baseGO).dataPointType(DPT_ActiveEnergy);
            knx.getGroupObject(mGO.tariff[i].yesterday = ++baseGO).dataPointType(DPT_ActiveEnergy);
            knx.getGroupObject(mGO.tariff[i].thisMonth = ++baseGO).dataPointType(DPT_ActiveEnergy);
            knx.getGroupObject(mGO.tariff[i].lastMonth = ++baseGO).dataPointType(DPT_ActiveEnergy);
            knx.getGroupObject(mGO.tariff[i].thisYear = ++baseGO).dataPointType(DPT_ActiveEnergy);
            knx.getGroupObject(mGO.tariff[i].lastYear = ++baseGO).dataPointType(DPT_ActiveEnergy);
            knx.getGroupObject(mGO.tariff[i].today).callback([this,i](GroupObject& go) {
                setHistory(mHistory.tariff[i].index, mHistory.tariff[i].yesterday, go.value(), i, RTCKnx::Day);
            });
            knx.getGroupObject(mGO.tariff[i].yesterday).callback([this,i](GroupObject& go) {
                setHistory(mHistory.tariff[i].yesterday, mHistory.tariff[i].dayM2, go.value(), i, RTCKnx::Day);
            });
            knx.getGroupObject(mGO.tariff[i].thisMonth).callback([this,i](GroupObject& go) {
                setHistory(mHistory.tariff[i].index, mHistory.tariff[i].lastMonth, go.value(), i, RTCKnx::Month);
            });
            knx.getGroupObject(mGO.tariff[i].lastMonth).callback([this,i](GroupObject& go) {
                setHistory(mHistory.tariff[i].lastMonth, mHistory.tariff[i].monthM2, go.value(), i, RTCKnx::Month);
            });
            knx.getGroupObject(mGO.tariff[i].thisYear).callback([this,i](GroupObject& go) {
                setHistory(mHistory.tariff[i].index, mHistory.tariff[i].lastYear, go.value(), i, RTCKnx::Year);
            });
            knx.getGroupObject(mGO.tariff[i].lastYear).callback([this,i](GroupObject& go) {
                setHistory(mHistory.tariff[i].lastYear, mHistory.tariff[i].yearM2, go.value(), i, RTCKnx::Year);
            });
        }
        resyncHistoryGroupObjects();
        const TeleInfoDataType* param = TeleInfoParam;
        for (TeleInfoDataStruct * data = mTeleInfoData; data != mTeleInfoData + TeleInfoCount; ++data, ++param) {
            data->conf = param; data->value.num = 0;
            knx.getGroupObject(data->goSend = ++baseGO).dataPointType(Dpt(data->conf->dpt.mainGroup, data->conf->dpt.subGroup));
            knx.getGroupObject(data->goSend).valueNoSend(value(*data));
        }
        mBufferLen = 0;
        mSerial.begin(TELEINFO_UART_SPEED, TELEINFO_UART_CONFIG);
    }
    void setHistory(uint32_t ref, uint32_t& dest, uint32_t src, int idxTariff, RTCKnx::DateChange periodToEmit) {
        if (ref - dest == src || src == dest) return;
        dest = src;
        resyncHistoryGroupObjects();
        if (periodToEmit == RTCKnx::Day) {
            if (mHistory.tariff[idxTariff].index != 0 && mHistory.tariff[idxTariff].yesterday != 0)
                knx.getGroupObject(mGO.tariff[idxTariff].today).objectWritten();
            if (mHistory.tariff[idxTariff].yesterday != 0 && mHistory.tariff[idxTariff].dayM2 != 0)
                knx.getGroupObject(mGO.tariff[idxTariff].yesterday).objectWritten();
        }
        else if (periodToEmit == RTCKnx::Month) {
            if (mHistory.tariff[idxTariff].index != 0 && mHistory.tariff[idxTariff].lastMonth != 0)
                knx.getGroupObject(mGO.tariff[idxTariff].thisMonth).objectWritten();
            if (mHistory.tariff[idxTariff].lastMonth != 0 && mHistory.tariff[idxTariff].monthM2 != 0)
                knx.getGroupObject(mGO.tariff[idxTariff].lastMonth).objectWritten();
        }
        else if (periodToEmit == RTCKnx::Year) {
            if (mHistory.tariff[idxTariff].index != 0 && mHistory.tariff[idxTariff].lastYear != 0)
                knx.getGroupObject(mGO.tariff[idxTariff].thisYear).objectWritten();
            if (mHistory.tariff[idxTariff].lastYear != 0 && mHistory.tariff[idxTariff].yearM2 != 0)
                knx.getGroupObject(mGO.tariff[idxTariff].lastYear).objectWritten();
        }
        mLastManualHistoryInit = rtc.millis();
    }

    uint32_t lastReception() const { return mLastReception; }

    void loop() {
        uint32_t current = rtc.millis()|1;
        bool isRealTime = knx.getGroupObject(mGO.realTimeOnOffState).value();
        if (mRealTimeTimer && (mParams.realTimeTimeout == 0 || current - mRealTimeTimer < mParams.realTimeTimeout)) {
            if (!isRealTime) {
                knx.getGroupObject(mGO.realTimeOnOffState).value(true);
                isRealTime = true;
            }
        }
        else {
            if (isRealTime) {
                knx.getGroupObject(mGO.realTimeOnOffState).value(false);
                isRealTime = false;
            }
            mRealTimeTimer = 0;
        }
        if (mLastManualHistoryInit && current - mLastManualHistoryInit > HISTORY_MANUALWRITE_TEMPO) {
            saveHistory();
            mLastManualHistoryInit = 0;
        }
        for (;;) {
            unsigned int pending = mSerial.available();
            if (pending == 0)
                break;

            while (pending > 0) {
                if (mBufferLen == TELEINFO_BUFFERSIZE) {
                    mBufferLen = 0;  // Security - Reset buffer if full with dummies
                    break;
                }
                unsigned int rcv = 0, ready = MIN(TELEINFO_BUFFERSIZE - mBufferLen, pending);
                char* ptr = mBuffer + mBufferLen;
                while (rcv < ready) {
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
                        mLastReception = current;
                        for (TeleInfoDataStruct * data = mTeleInfoData; data != mTeleInfoData + TeleInfoCount; ++data) {
                            if (lineLen > data->conf->keySize && memcmp(currentBuffer, data->conf->key, data->conf->keySize) == 0) {
                                if (TeleInfo::value(*data, currentBuffer, eol)) {
                                    data->lastChange = current;
                                    knx.getGroupObject(data->goSend).valueNoSend(TeleInfo::value(*data));
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

        // Update ADPS (forced) when IINST or ISOUSC changed before ADPS (ADPS = MAX(0, IINST - ISOUSC));
        const TeleInfoDataStruct &isousc = mTeleInfoData[2 /* ISOUSC*/];
        if (isousc.lastChange != 0) {
            TeleInfoDataStruct& adps = mTeleInfoData[18 /* ADPS*/];
            const TeleInfoDataStruct* iinsts[] = { &mTeleInfoData[17 /* IINST*/], &mTeleInfoData[22 /* IINST1*/], &mTeleInfoData[23 /* IINST2*/], &mTeleInfoData[24 /* IINST3*/] };
            const TeleInfoDataStruct* maxiinst = nullptr;
            for (size_t i = 0; i < sizeof(iinsts)/sizeof(iinsts[0]); ++i) {
                if (!maxiinst || maxiinst->value.num < iinsts[i]->value.num) {
                    maxiinst = iinsts[i];
                }
            }
            if (maxiinst && maxiinst->lastChange != 0 && (current == maxiinst->lastChange || current == isousc.lastChange)) {
                uint32_t adpsValue = maxiinst->value.num > isousc.value.num?maxiinst->value.num - isousc.value.num:0;
                if (adps.value.num != adpsValue) {
                    adps.value.num = adpsValue;
                    adps.lastChange = current;
                    knx.getGroupObject(adps.goSend).valueNoSend(adpsValue);
                }
            }
            if (current == adps.lastChange || (adps.value.num > 0 && current - adps.lastSend > ADPS_REPEAT_PERIOD)) {
                adps.lastSendValueCheckSum = adps.value.num;
                knx.getGroupObject(adps.goSend).objectWritten(); // Emit is forced
                adps.lastSend = current;
            }
        }

        // Send if value has changed and period is over
        for (TeleInfoDataStruct * data = mTeleInfoData; data != mTeleInfoData + TeleInfoCount; ++data) {
            if (data->lastChange != data->lastSend && (isRealTime || current - data->lastSend > mParams.period)) {
                uint32_t chksum = data->conf->type == TeleInfoDataType::STRING?simpleChecksum(data->value.str):data->value.num;
                if (chksum != data->lastSendValueCheckSum) {
                    data->lastSendValueCheckSum = chksum;
                    knx.getGroupObject(data->goSend).objectWritten();
                    data->lastSend = current;
                }
            }
        }

        // Update history
        if (mTeleInfoData[1 /* OPTARIF */].lastChange != 0) {
            uint32_t index[TARIFCOUNT] = {0};
            currentIndexes(index);
            for (int i = 0; i < TARIFCOUNT; ++i) {
                mHistory.tariff[i].index = index[i];
                if (index[i] == 0 || !rtc.isValid()) continue;
                if (index[i] >= mHistory.tariff[i].yesterday) {
                    if (mHistory.tariff[i].yesterday == 0)   mHistory.tariff[i].yesterday = index[i];
                    knx.getGroupObject(mGO.tariff[i].today).valueNoSend(index[i] - mHistory.tariff[i].yesterday);
                }
                if (index[i] >= mHistory.tariff[i].lastMonth) {
                    if (mHistory.tariff[i].lastMonth == 0)   mHistory.tariff[i].lastMonth = index[i];
                    knx.getGroupObject(mGO.tariff[i].thisMonth).valueNoSend(index[i] - mHistory.tariff[i].lastMonth);
                }
                if (index[i] >= mHistory.tariff[i].lastYear) {
                    if (mHistory.tariff[i].lastYear == 0)    mHistory.tariff[i].lastYear = index[i];
                    knx.getGroupObject(mGO.tariff[i].thisYear).valueNoSend(index[i] - mHistory.tariff[i].lastYear);
                }
            }
            if (rtc.isValid() && (isRealTime || current - mHistoryLastSent > mParams.period)) {
                for (int i = 0; i < TARIFCOUNT; ++i) {
                    if (index[i] != mHistoryLastValue[i]) {
                        knx.getGroupObject(mGO.tariff[i].today).objectWritten();
                        knx.getGroupObject(mGO.tariff[i].thisMonth).objectWritten();
                        knx.getGroupObject(mGO.tariff[i].thisYear).objectWritten();
                        mHistoryLastSent = current;
                        mHistoryLastValue[i] = index[i];
                    }
                }
            }
        }
    }
    void currentIndexes(uint32_t index[TARIFCOUNT]) const {
        // depending on OPTARIF
        uint8_t optarif = value(mTeleInfoData[1 /* OPTARIF */]);
        switch (optarif) {
            case 0: case 1 /* Base */:
                index[Base] = mTeleInfoData[3 /* BASE */].value.num;
            break;
            case 2 /* HCHP */:
                index[HC] = mTeleInfoData[4 /* HCHC */].value.num;
                index[HP] = mTeleInfoData[5 /* HCHP */].value.num;
                index[Base] = index[HC] + index[HP];
            break;
            case 3 /* EJP */:
                index[HC] = mTeleInfoData[6 /* EJPHN */].value.num;
                index[HP] = mTeleInfoData[7 /* EJPHPM */].value.num;
                index[Base] = index[HC] + index[HP];
            break;
            default/* Tempo */: {
                uint32_t blueHC = mTeleInfoData[8 /* BBRHCJB */].value.num, blueHP = mTeleInfoData[9 /* BBRHPJB */].value.num,
                         whiteHC = mTeleInfoData[10 /* BBRHCJW */].value.num, whiteHP = mTeleInfoData[11 /* BBRHPJW */].value.num,
                         redHC = mTeleInfoData[12 /* BBRHCJR */].value.num, redHP = mTeleInfoData[13 /* BBRHPJR */].value.num;
                index[HC] = blueHC + whiteHC + redHC;
                index[HP] = blueHP + whiteHP + redHP;
                //index[BLUE] = blueHC + blueHP; index[WHITE] = whiteHC + whiteHP; index[RED] = redHC + redHP;
                index[Base] = index[HC] + index[HP];
            }; break;
        }
    }
    void newDate(RTCKnx::DateChange change) {
        if (change == RTCKnx::Init) {
            validateHistory();
            return;
        }
        switch (change) {
            case RTCKnx::Year:
                for (int i = 0; i < TARIFCOUNT; ++i) {
                    mHistory.tariff[i].yearM2 = mHistory.tariff[i].lastYear;
                    mHistory.tariff[i].lastYear = mHistory.tariff[i].index;
                    if (mHistory.tariff[i].yearM2 != 0)
                        knx.getGroupObject(mGO.tariff[i].lastYear).value(mHistory.tariff[i].lastYear - mHistory.tariff[i].yearM2);
                }
            [[fallthrough]];
            case RTCKnx::Month:
                for (int i = 0; i < TARIFCOUNT; ++i) {
                    mHistory.tariff[i].monthM2 = mHistory.tariff[i].lastMonth;
                    mHistory.tariff[i].lastMonth = mHistory.tariff[i].index;
                    if (mHistory.tariff[i].monthM2 != 0)
                        knx.getGroupObject(mGO.tariff[i].lastMonth).value(mHistory.tariff[i].lastMonth - mHistory.tariff[i].monthM2);
                }
                saveHistory(); // Save only each month (due to flash write cycle limited to 10000)
            [[fallthrough]];
            case RTCKnx::Day:
                for (int i = 0; i < TARIFCOUNT; ++i) {
                    mHistory.tariff[i].dayM2 = mHistory.tariff[i].yesterday;
                    mHistory.tariff[i].yesterday = mHistory.tariff[i].index;
                    if (mHistory.tariff[i].dayM2 != 0)
                        knx.getGroupObject(mGO.tariff[i].yesterday).value(mHistory.tariff[i].yesterday - mHistory.tariff[i].dayM2);
                }
            [[fallthrough]];
            default:;
        }
    }
    void validateHistory() {
        if (mHistory.lastSave.tm_mday == 0) return; 
        const RTCKnx::DateTime& currentDateTime = rtc.dateTime();
        if (mHistory.lastSave.tm_year != currentDateTime.tm_year) {
            mHistory = {0};
        }
        else if(mHistory.lastSave.tm_mon != currentDateTime.tm_mon) {
            for (int i = 0; i < TARIFCOUNT; ++i) {
                mHistory.tariff[i].lastMonth = mHistory.tariff[i].monthM2 = mHistory.tariff[i].yesterday = mHistory.tariff[i].dayM2 = 0;
            }
        }
        else if(mHistory.lastSave.tm_mday != currentDateTime.tm_mday) {
            for (int i = 0; i < TARIFCOUNT; ++i) {
                mHistory.tariff[i].yesterday = mHistory.tariff[i].dayM2 = 0;
            }
        }
    }
    void restoreHistory() {
        uint8_t checksum = 0, mask = 0xff, mask2 = 0;
        for (size_t i = 0; i < sizeof(mHistory); ++i) {
            const uint8_t v = *((uint8_t*)&mHistory + i) = eeprom_buffered_read_byte(HISTORY_FLASH_START + i);
            mask &= v; mask2 |= v; checksum ^= v;
        }
        if (mask == 0xff || mask2 == 0 || checksum != eeprom_buffered_read_byte(HISTORY_FLASH_START + sizeof(mHistory))) {
            mHistory = {0};
        }
    }
    void saveHistory() {
        if (mHistoryLastValue[Base] == 0)   return; // Nothing sent, nothing to store...
        const RTCKnx::DateTime& dateTime = rtc.dateTime();
        mHistory.lastSave = dateTime;
        uint8_t checksum = 0;
        for (size_t i = 0; i < sizeof(mHistory); ++i) {
            eeprom_buffered_write_byte(HISTORY_FLASH_START + i, *((uint8_t*)&mHistory + i));
            checksum ^= *((uint8_t*)&mHistory + i);
        }
        if (checksum != eeprom_buffered_read_byte(HISTORY_FLASH_START + sizeof(mHistory))) {
            eeprom_buffered_write_byte(HISTORY_FLASH_START + sizeof(mHistory), checksum);
            eeprom_buffer_flush();
        } 
    }
    void resetHistory() {
        mHistory = {0};
        saveHistory();
        resyncHistoryGroupObjects();
    }
    void resyncHistoryGroupObjects() {
        for (int i = 0; i < TARIFCOUNT; ++i) {
            knx.getGroupObject(mGO.tariff[i].today).valueNoSend(mHistory.tariff[i].yesterday != 0?mHistory.tariff[i].index - mHistory.tariff[i].yesterday:(uint32_t)0);
            knx.getGroupObject(mGO.tariff[i].yesterday).valueNoSend(mHistory.tariff[i].dayM2 != 0?mHistory.tariff[i].yesterday - mHistory.tariff[i].dayM2:(uint32_t)0);
            knx.getGroupObject(mGO.tariff[i].thisMonth).valueNoSend(mHistory.tariff[i].lastMonth != 0?mHistory.tariff[i].index - mHistory.tariff[i].lastMonth:(uint32_t)0);
            knx.getGroupObject(mGO.tariff[i].lastMonth).valueNoSend(mHistory.tariff[i].monthM2 != 0?mHistory.tariff[i].lastMonth - mHistory.tariff[i].monthM2:(uint32_t)0);
            knx.getGroupObject(mGO.tariff[i].thisYear).valueNoSend(mHistory.tariff[i].lastYear != 0?mHistory.tariff[i].index - mHistory.tariff[i].lastYear:(uint32_t)0);
            knx.getGroupObject(mGO.tariff[i].lastYear).valueNoSend(mHistory.tariff[i].yearM2 != 0?mHistory.tariff[i].lastYear - mHistory.tariff[i].yearM2:(uint32_t)0);
        }
    }
  public:
    enum { NBGO = TeleInfoCount + sizeof(mGO)/sizeof(uint16_t), SIZEPARAMS = sizeof(mParams) };
};
uint8_t teleinfoHolder[sizeof(TeleInfo)] __attribute__ ((section (".noinit")));
TeleInfo& teleinfo = *(TeleInfo*)teleinfoHolder;

extern "C" void SystemClock_Config(void)
{
    // Nothing for default 4MHz MSI Clock
    // Required to downclock the 80MHz clock to be below 4096 * 16 * 1200 (TeleInfo BaudRate) due to limitation in UART_BRR 12bit register
}

static HardwareSerial serialTpuart(PIN_TPUART_TX, PIN_TPUART_RX);
void setup()
{
    if (Inited != INIT_MASK) {
        new (&rtc) RTCKnx();
        new (&teleinfo) TeleInfo();
        Inited = INIT_MASK;
    }

    knx.platform().knxUart(&serialTpuart);
    knx.ledPin(PIN_PROG_LED);
    knx.ledPinActiveOn(HIGH);
    knx.buttonPin(PIN_PROG_SWITCH);
    knx.buttonPinInterruptOn(RISING);

    // Init device
    knx.version((VERSION_MAJOR << 6) | (VERSION_MINOR & 0x3F)); // PID_VERSION
    knx.orderNumber((const uint8_t*)"ZDI-TINFO1");             // PID_ORDER_INFO
    // knx.manufacturerId(0xfa);                                  // PID_SERIAL_NUMBER (2 first bytes) - 0xfa for KNX Association
    knx.hardwareType((const uint8_t*)"M-07B0");                // PID_HARDWARE_TYPE
    knx.bau().deviceObject().individualAddress(1);

    // read adress table, association table, groupobject table and parameters from eeprom
    knx.readMemory();

    if (knx.configured()) {
        rtc.init(0, 0);
        teleinfo.init(RTCKnx::SIZEPARAMS, RTCKnx::NBGO);
        rtc.setNotifier(std::bind(&TeleInfo::newDate, &teleinfo, std::placeholders::_1));
        // attachInterrupt(PIN_TPUART_SAVE, std::bind(&TeleInfo::saveHistory, &teleinfo), LOW);    // 2ms to save history before shutdown - likely not enough
    }

    // start the framework.
    knx.start();
}

void loop() 
{
    // don't delay here too much. Otherwise you might loose packages or mess up the timing with ETS
    knx.loop();

    // only run the application code if the device was configured with ETS
    if(knx.configured()) {
        teleinfo.loop();
        rtc.loop();
    }

    uint32_t currentMillis = rtc.millis();
    // Handle Reset History by long prog button press
    static uint32_t progButtonPressedTimer = 0;
    static bool historyReset = false;
    bool progButtonState = digitalRead(PIN_PROG_SWITCH) == HIGH;
    if (!progButtonState && currentMillis - progButtonPressedTimer > 200) {
        progButtonPressedTimer = 0;
        if (historyReset) {
            digitalWrite(PIN_PROG_LED, LOW);
        }
        historyReset = false;
    }
    else {
        if (progButtonPressedTimer == 0) {
            progButtonPressedTimer = currentMillis;
        }
        else {
            uint32_t delay = currentMillis - progButtonPressedTimer;
            if (delay > HISTORY_RESET_PROG_SWITCH_DELAY) {
                if (!historyReset) {
                    knx.progMode(false);
                    teleinfo.resetHistory();
                    historyReset = true;
                }
                if (historyReset) {
                    digitalWrite(PIN_PROG_LED, (delay/HISTORY_RESET_LED_BLINKING_PERIOD)&1);
                }
            }
        }
    }

    // Handle Prog Mode timeout
    static uint32_t timerProgMode = 0;
    if (knx.progMode()) {
        if (timerProgMode == 0) {
            timerProgMode = currentMillis;
        }
        else {
            if (currentMillis - timerProgMode > PROG_TIMEOUT) {
                knx.progMode(false);
                timerProgMode = 0;
            }
        }
    }
    else {
        timerProgMode = 0;
    }

    // Handle reception blinking led (2s cycle with 0.5s On while receiving Teleinfo data)
    if (!knx.progMode() && !progButtonState) {
        if (currentMillis - teleinfo.lastReception() < RECEPTION_LED_BLINKING_PERIOD * 2) {
            digitalWrite(PIN_PROG_LED, ((currentMillis/RECEPTION_LED_BLINKING_PERIOD)&3)==0);
        }
        else {
            digitalWrite(PIN_PROG_LED, LOW);
        }
    }
}
