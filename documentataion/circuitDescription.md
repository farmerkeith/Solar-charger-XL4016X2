# Circuit description

Reference: KiCad schematic XL4016X2Single. This is the complete schematic on a single A4 sheet.

# Solar charger circuit
Starting at the top left corner is J1 labelled Solar Panel. The positive and negative wires from the solar panel connect here.

The first component is D3 P6KE47CA which is a transient suppressor diode. Its breakdown voltage is 47 volts. Its purpose is to protect the charger from things like lightning strikes.

Next you find resistors R1, R3 and capacitor C1 (it should be a different label; I will need to change it). The resistors divide the panel voltage down to a level compatible with the Arduino nano, and connect to Arduino pin A0 which does voltage measurement. The voltage range is 5 volts (Arduino Vcc) * (100+15)/15 = 38 Volts, which is enough for a 60-cell solar panel which has an open circuit voltage of 37.8 Volts.

Next you find U2 which is an ACS712 module. It is a Hall effect current sensor. The current from the panel flows in on the P- terminals and out on the P+ terminals. The amount of current affects the voltage on the OUT pin (at the bottom of the module). With zero current the OUT pin sits at Vcc/2, nominally 2.5 Volts. The OUT pin is connected to Arduino A1, which gives a digital measurement of that voltage, and is translated in the software to a current in Amps or milliAmps. The sensitivity of the "5 Amp" module I have specified here is 185 mV per amp, so it can easily measure the maximum current from the solar panel which is 8.7 Amps.

After U2 you find four capacitors C1.1 to C1.4 in parallel. They are there to smooth the current from the panel (which needs to be steady) to the buck converters (which will be of an impulsive nature).

The buck converters U1 and U3 do a DC-DC conversion function. At their output, each converter is rated for 8 Amps, and current limited by internal circuitry to 10 Amps approximately. To achieve good utilisation of your solar panel capacity requires two of these modules added together. They work in parallel. If your panel is at its maximum power point of 31.5 Volts, and the battery is being charged at 12.0 volts with a current of 20 Amps, 10 amps for each of U1 and U3, the input current will be a total of 20*12/31.5=7.6 Amps, or 3.8 Amps per module.  Each of the converters U1 and U3 also has its own smoothing capacitors at the input. I have shown C1.1 to C1.4 because there is a lot of heat generated in these capacitors and putting in some extras is not very expensive and reduces the heating and shoule improve the life time.

The conversion ratio of the two converters U1 and U3 is controlled by PWM signals from the Arduino using pins D9 and D10.

Next is the two MOSFET diode circuits, one for each converter, which then connect the current to the battery via the second ACS712, U4. The main component in each of the two MOSFET diode circuits is the MOSFET, that is Q1 for DC module U1, and Q6 for DC module U3.

The second ACS712 (U4) measures the current both into and out of the battery. Typical situations are current in from the solar panel, and current out to the load. When the mains supply is turned on, it will either decrease the current drawn by the load, or add to the current provided by the solar panels. 

Just to the right of module U4 is the voltage divider for measuring the battery voltage, consisting of resistors R12 and R16 and capacitor C2. The voltage range is 5 volts (Arduino Vcc) * (100+47)/47 = 15.6 Volts, which is enough for a 12 volt lead acid battery. 

Further to the right again is a MP2307 DC-Dc module which draws power from the battery and delivers +5 volts to all the electronic circuits  that require it. 

The last element before the battery connector is a fuse which is there to prevent accidental short circuits causing major component failure. 

# Load control circuit

The load control circuit is in the top right corner of the schematic. It consists of an IRF4905 P-channel MOSFET controlled via a transistor from Arduino pin D4. Pin D4 should be High to turn the load ON, and Low to turn it Off. 

# Mains supply circuit

Mains power comes from a commercial mains adaptor with an output of 12 volts at up to 3 Amps. The adapter is connected via barrel jack J9 which provides a 5.5mm coaxial connection commonly used for mains adapters. These adapters can be sensitive to being fed voltage into them when there is no mains power applied, hence there is a MOSFET diode to prevent that (main component Q9). Mains can be connected or disconnected by Q10 which is controlled via a transistor from Arduino pin D3. 

# Temperature measuring

Battery temperature is measured using DS18B20 U7, connected via a cable through screw terminal J8.

Temperature of the controller is measured using DS18B20 U11 mounted on the TinyRTC real time clock module. 

# Real Time Clock 

In order to be able to accurately time stamp the logged data, a real time clock module is used with a DS1307 and crystal oscillator. The accuracy should be sufficient for this purpose. 

This particular module is also equipped with an EEPROM with 4 kbytes capacity which enables the storage of logging data while the SD card is being unloaded from time to time.

Both the clock and the EEPROM are accessed via the I2C interface from Arduino pins A4 and A5.

# Micro SD card

Data logging is done to a micro SD card which provides ample storage capability for several years of data. Data will be recorded in comma separated text files. Details of the data required and the file duration is to be decided. 

# LCD display

The LCD display will show operational parameters, such as current, voltage and power for the solar panel and the battery, and the operating mode (solar charging; load on or off; mains on or off) 

To see the charactrs in the LCD screen, its back light needs to be turned on. However the back light consumes some power, which is a waste when nobody is there to look at it. Hence we provide a button, so that when somebody wants to see the LCD, they can press the button, and the back light will come on for a set period of time, and then go off again. The time period will be in the software, it could be 15 seconds or a minute, something like that. 

