# Solar-charger-XL4016X2
MPPT Solar charger based on a design by zopinter 
This design uses two commercial DC-DC converter modules based on XL4016, and Arduino Nano for control.
It also has a supplementary input from mains supply controlled from the Nano, and a controlled Load output.
Reverse current flow to the XL4016 converter modules and to the mains supply is prevented by "MOSFET diodes" each consisting of an IRF4905 MOSFET controlled by two BJTs. 
Current flow to and from the battery is monitored, in addition to the solar panel voltage and current and battery voltage essential to the operation of the MPPT algorithm.

# Peripherals
Battery temperature is monitored with a DS18B20 sensor and used to adjust battery set points.
Operational data logging is provided by a micro SD card, with time stamps from a Tiny RTC module using a DS1307 IC. 
An LCD character display provides status information. A push button turns on the backlight of the LCD, saving power when the LCD is not required to be on. 

# Schematic
The schematic circuit diagram has been prepared using KiCad. 

# Layout
A possible layout of the components using Fritzing can be found in the Layout folder. The schematic and PCB parts of the Fritzing files have not been made useful. 

# Software
The software for the Arduino Nano (C++ code) is in the software folder. It still requires change for (at least):
 - pin allocations to correspond to hardware
 - deal with two rather than one XL4016 module
 - data logging and clock
 - testing

See the software code for a full listing of Arduino Nano pin allocations. 
