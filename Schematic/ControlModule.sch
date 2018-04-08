EESchema Schematic File Version 2
LIBS:Modules
LIBS:ESP8266
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:XL4016X2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title "XL4016X2"
Date "2018-01-11"
Rev ""
Comp "farmerkeith"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ArduinoNano U6
U 1 1 5AC8E54E
P 6600 1800
F 0 "U6" H 6725 1700 50  0000 C CNN
F 1 "ArduinoNano" V 6500 1700 50  0000 C CNN
F 2 "" H 6600 1800 50  0000 C CNN
F 3 "" H 6600 1800 50  0000 C CNN
	1    6600 1800
	1    0    0    -1  
$EndComp
Text GLabel 7850 1400 2    60   Input ~ 0
+5V
Wire Wire Line
	7850 1400 7100 1400
$Comp
L GND #PWR015
U 1 1 5AC8E582
P 8200 1200
F 0 "#PWR015" H 8200 950 50  0001 C CNN
F 1 "GND" H 8200 1050 50  0000 C CNN
F 2 "" H 8200 1200 50  0001 C CNN
F 3 "" H 8200 1200 50  0001 C CNN
	1    8200 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1200 8200 1200
$Comp
L ESP-01v090 U7
U 1 1 5AC8E755
P 3800 2000
F 0 "U7" H 3800 1900 50  0000 C CNN
F 1 "ESP-01" H 3800 2100 50  0000 C CNN
F 2 "" H 3800 2000 50  0001 C CNN
F 3 "" H 3800 2000 50  0001 C CNN
	1    3800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1850 2850 1600
Wire Wire Line
	2850 1600 5350 1600
Wire Wire Line
	5300 2150 4750 2150
$Comp
L R R16
U 1 1 5AC8E8F0
P 5950 1900
F 0 "R16" V 6030 1900 50  0000 C CNN
F 1 "1K" V 5950 1900 50  0000 C CNN
F 2 "" V 5880 1900 50  0001 C CNN
F 3 "" H 5950 1900 50  0001 C CNN
	1    5950 1900
	0    1    1    0   
$EndComp
$Comp
L R R15
U 1 1 5AC8E91B
P 5150 1850
F 0 "R15" V 5230 1850 50  0000 C CNN
F 1 "2K2" V 5150 1850 50  0000 C CNN
F 2 "" V 5080 1850 50  0001 C CNN
F 3 "" H 5150 1850 50  0001 C CNN
	1    5150 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	5800 1900 5300 1900
Wire Wire Line
	5300 1850 5300 2150
Wire Wire Line
	4750 1850 5000 1850
$Comp
L GND #PWR016
U 1 1 5AC8E9E4
P 4850 1850
F 0 "#PWR016" H 4850 1600 50  0001 C CNN
F 1 "GND" H 4850 1700 50  0000 C CNN
F 2 "" H 4850 1850 50  0001 C CNN
F 3 "" H 4850 1850 50  0001 C CNN
	1    4850 1850
	1    0    0    -1  
$EndComp
$Comp
L AP1117-33 U8
U 1 1 5AC8EA02
P 2550 2150
F 0 "U8" H 2500 2150 50  0000 C CNN
F 1 "AMS1117-33" H 2250 2300 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-223-3Lead_TabPin2" H 2550 2350 50  0001 C CNN
F 3 "" H 2650 1900 50  0001 C CNN
	1    2550 2150
	1    0    0    -1  
$EndComp
Text GLabel 2250 2150 0    60   Input ~ 0
+5V
$Comp
L GND #PWR017
U 1 1 5AC8EB1F
P 2550 2450
F 0 "#PWR017" H 2550 2200 50  0001 C CNN
F 1 "GND" H 2550 2300 50  0000 C CNN
F 2 "" H 2550 2450 50  0001 C CNN
F 3 "" H 2550 2450 50  0001 C CNN
	1    2550 2450
	1    0    0    -1  
$EndComp
Text GLabel 7100 1700 2    60   Input ~ 0
Arduino_pin_A2
Text GLabel 7100 1500 2    60   Input ~ 0
Arduino_pin_A0
Text GLabel 8000 1600 2    60   Input ~ 0
Arduino_pin_A1
Text GLabel 8000 1800 2    60   Input ~ 0
Arduino_pin_A3
Wire Wire Line
	7100 1600 8000 1600
Wire Wire Line
	7100 1800 8000 1800
Text GLabel 7100 1900 2    60   Input ~ 0
Arduino_pin_A4
Text GLabel 8000 2000 2    60   Input ~ 0
Arduino_pin_A5
Text GLabel 7100 2100 2    60   Input ~ 0
Arduino_pin_A6
Wire Wire Line
	7100 2000 8000 2000
Text GLabel 6100 2200 0    60   Input ~ 0
Arduino_pin_D9
Text GLabel 5300 2300 0    60   Input ~ 0
Arduino_pin_D10
Wire Wire Line
	5300 2300 6100 2300
Wire Wire Line
	6100 1800 5350 1800
Wire Wire Line
	5350 1800 5350 1600
Text GLabel 5550 1000 0    60   Input ~ 0
Arduino_pin_D3
Text GLabel 5550 1150 0    60   Input ~ 0
Arduino_pin_D4
Wire Wire Line
	5650 1150 5650 1700
Wire Wire Line
	5650 1700 6100 1700
Wire Wire Line
	5550 1000 5750 1000
Wire Wire Line
	5750 1000 5750 1600
Wire Wire Line
	5750 1600 6100 1600
$Comp
L CP C4
U 1 1 5AC8FDDC
P 2850 2300
F 0 "C4" H 2875 2400 50  0000 L CNN
F 1 "10uF" H 2875 2200 50  0000 L CNN
F 2 "" H 2888 2150 50  0001 C CNN
F 3 "" H 2850 2300 50  0001 C CNN
	1    2850 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2450 2550 2450
Text Notes 3700 2250 0    60   ~ 0
WiFi
Text GLabel 5550 850  0    60   Input ~ 0
Arduino_pin_D2
Wire Wire Line
	5550 1150 5650 1150
Wire Wire Line
	5550 850  5850 850 
Wire Wire Line
	5850 850  5850 1500
Wire Wire Line
	5850 1500 6100 1500
Wire Wire Line
	6100 1400 5950 1400
Wire Wire Line
	5950 1400 5950 850 
Wire Wire Line
	5950 850  8200 850 
Wire Wire Line
	8200 850  8200 1200
$Comp
L LCDSerial2004 U10
U 1 1 5AC9C05C
P 6400 3400
F 0 "U10" H 6400 3400 60  0000 C CNN
F 1 "LCDSerial2004" H 6400 3500 60  0000 C CNN
F 2 "" H 6400 3400 60  0001 C CNN
F 3 "" H 6400 3400 60  0001 C CNN
	1    6400 3400
	1    0    0    -1  
$EndComp
Text GLabel 5750 3200 0    60   Input ~ 0
+5V
$Comp
L GND #PWR018
U 1 1 5AC9C12A
P 5750 3500
F 0 "#PWR018" H 5750 3250 50  0001 C CNN
F 1 "GND" H 5750 3350 50  0000 C CNN
F 2 "" H 5750 3500 50  0001 C CNN
F 3 "" H 5750 3500 50  0001 C CNN
	1    5750 3500
	1    0    0    -1  
$EndComp
Text GLabel 5350 3300 0    60   Input ~ 0
Arduino_pin_A4
Wire Wire Line
	5350 3300 5750 3300
Text GLabel 5350 3450 0    60   Input ~ 0
Arduino_pin_A5
Wire Wire Line
	5350 3450 5500 3450
Wire Wire Line
	5500 3450 5500 3400
Wire Wire Line
	5500 3400 5750 3400
$Comp
L SW_Push SW1
U 1 1 5AC9C29E
P 7550 3400
F 0 "SW1" H 7600 3500 50  0000 L CNN
F 1 "Backlight control" H 7550 3650 50  0000 C CNN
F 2 "" H 7550 3600 50  0001 C CNN
F 3 "" H 7550 3600 50  0001 C CNN
	1    7550 3400
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR019
U 1 1 5AC9C2C9
P 7550 3600
F 0 "#PWR019" H 7550 3350 50  0001 C CNN
F 1 "GND" H 7550 3450 50  0000 C CNN
F 2 "" H 7550 3600 50  0001 C CNN
F 3 "" H 7550 3600 50  0001 C CNN
	1    7550 3600
	1    0    0    -1  
$EndComp
$Comp
L R R20
U 1 1 5AC9C2ED
P 7550 3050
F 0 "R20" V 7630 3050 50  0000 C CNN
F 1 "10K" V 7550 3050 50  0000 C CNN
F 2 "" V 7480 3050 50  0001 C CNN
F 3 "" H 7550 3050 50  0001 C CNN
	1    7550 3050
	1    0    0    -1  
$EndComp
Text GLabel 7550 2900 0    60   Input ~ 0
+5V
Text GLabel 7550 3250 2    60   Input ~ 0
Arduino_pin_A6
$EndSCHEMATC
