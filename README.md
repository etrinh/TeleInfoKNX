# **French energy provider Enedis TeleInfo (TIC) KNX interface.**

Compatible with Enedis Linky meters in "Historic" mode, and "Blue" meters.

[Buy online](https://www.tindie.com/products/zdi/knx-teleinfo/) 

# **Features:**
- Activatable RealTime mode for real-time consumption monitoring/display.
- History of total Consumption (Current Year, Current Month, Today, Last Year, Last Month, Yesterday) (an external KNX Clock participant is required to provide accurate date and time).
- ETS5 configurable.
- Bus powered (10mA).

# **Usage:**
- Connect the device to the TeleInfo terminals (I1 I2) on the energy meter (No polarity).
- Connect the device to the KNX bus.
- At first power-on, the device has address 0.0.1.
- To activate the device Programming Mode, click the "PROG / HRST".
- Configure device in ETS.
- To reset History, press the History Reset "PROG / HRST" Button more than 4 seconds.
- In case of over-current warning, the message "Depassement" (Group Object 43) is repeated every 10 seconds on the bus.

# **Led Interpretation:**
- Led Blinking (0.5s On/1.5s Off): TeleInfo data are receiving - **Device is working properly**.
- Led Continuously On: The device is in programming mode (it will automatically switch off after a delay of 15 minutes or a new press on the "PROG / HRST" button).
- Led Blinking (0.5s On/0.5s Off): The History is erased (happens when the "PROG / HRST" Button is pressed during more than 4 seconds).
- Led Off: The device is not receiving TeleInfo data, or is not configured with ETS5, or is not connected to the KNX Bus.

# **Group Objects:**
All "Consumption" Group Objects (from GO 7 to GO 24):

- Can be read to get the consumption index difference from the beginning and ending of the specified period.
- Can be written by the consumption index at the beginning of the corresponding period. It allows to specifically initialize the history from data provided by your energy provider. It is advised to set these indexes before affecting monitoring participants to these Group Objects.

# **Product Database:**
Click [here](https://github.com/etrinh/TeleInfoKNX/raw/master/ETS/teleinfo.knxprod) to download ETS5 product database (identified as KNX Association).

The Firmware can be upgraded with a ST-Link v2 interface
