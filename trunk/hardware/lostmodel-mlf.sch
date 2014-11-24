EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
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
LIBS:npn-bce
LIBS:lostmodel-mlf-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Lost Model Alarm"
Date "24 nov 2014"
Rev "1.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 4800 1450
Connection ~ 4800 1750
Connection ~ 4800 2050
Connection ~ 6150 2050
Wire Wire Line
	6150 2050 5950 2050
Wire Wire Line
	4900 1450 4800 1450
Wire Wire Line
	4900 2050 4800 2050
Wire Wire Line
	5400 1750 5550 1750
Wire Wire Line
	4550 4900 4300 4900
Wire Wire Line
	3100 2650 3100 2950
Wire Wire Line
	3100 2950 2900 2950
Wire Wire Line
	7750 1350 7750 1600
Wire Wire Line
	7750 1450 7350 1450
Wire Wire Line
	7350 1450 7350 1600
Wire Wire Line
	2850 5100 4550 5100
Wire Wire Line
	4550 4700 4300 4700
Wire Wire Line
	5550 6250 5550 6100
Connection ~ 5550 3350
Wire Wire Line
	5450 3350 5650 3350
Wire Wire Line
	3100 3550 3100 3350
Wire Wire Line
	2900 3250 3250 3250
Wire Wire Line
	2900 3050 3250 3050
Wire Wire Line
	2900 2850 3250 2850
Wire Wire Line
	2900 3150 3250 3150
Wire Wire Line
	3100 3350 2900 3350
Wire Wire Line
	5550 3200 5550 3350
Wire Wire Line
	5450 6100 6500 6100
Connection ~ 5550 6100
Wire Wire Line
	4550 4800 4300 4800
Wire Wire Line
	3100 5000 3100 5100
Connection ~ 3100 5100
Connection ~ 7750 1450
Wire Wire Line
	5400 1450 5550 1450
Wire Wire Line
	5400 2050 5550 2050
Wire Wire Line
	4900 1750 4800 1750
Wire Wire Line
	5950 1750 6150 1750
Connection ~ 6150 1750
Text Notes 2400 3550 0    60   ~ 0
3x2 pogo\npin pads
$Comp
L R R6
U 1 1 51A7498E
P 7550 5100
F 0 "R6" V 7630 5100 50  0000 C CNN
F 1 "1K" V 7550 5100 50  0000 C CNN
F 2 "" H 7550 5100 60  0001 C CNN
F 3 "" H 7550 5100 60  0001 C CNN
	1    7550 5100
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR01
U 1 1 51A74974
P 8700 3800
F 0 "#PWR01" H 8700 3890 20  0001 C CNN
F 1 "+5V" H 8700 3890 30  0000 C CNN
F 2 "" H 8700 3800 60  0001 C CNN
F 3 "" H 8700 3800 60  0001 C CNN
	1    8700 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 51A74963
P 8700 5450
F 0 "#PWR02" H 8700 5450 30  0001 C CNN
F 1 "GND" H 8700 5380 30  0001 C CNN
F 2 "" H 8700 5450 60  0001 C CNN
F 3 "" H 8700 5450 60  0001 C CNN
	1    8700 5450
	1    0    0    -1  
$EndComp
$Comp
L BEEPER PZ1
U 1 1 51A7493C
P 9150 4300
F 0 "PZ1" H 9250 4000 60  0000 C CNN
F 1 "PIEZO" H 9250 4600 60  0000 C CNN
F 2 "" H 9150 4300 60  0001 C CNN
F 3 "" H 9150 4300 60  0001 C CNN
	1    9150 4300
	1    0    0    1   
$EndComp
$Comp
L GND #PWR03
U 1 1 51A744A2
P 6150 2350
F 0 "#PWR03" H 6150 2350 30  0001 C CNN
F 1 "GND" H 6150 2280 30  0001 C CNN
F 2 "" H 6150 2350 60  0001 C CNN
F 3 "" H 6150 2350 60  0001 C CNN
	1    6150 2350
	1    0    0    -1  
$EndComp
Text Label 4800 2050 2    60   ~ 0
LED03
Text Label 4800 1750 2    60   ~ 0
LED02
Text Label 4800 1450 2    60   ~ 0
LED01
$Comp
L R R5
U 1 1 51A743F1
P 5150 2050
F 0 "R5" V 5230 2050 50  0000 C CNN
F 1 "1K5" V 5150 2050 50  0000 C CNN
F 2 "" H 5150 2050 60  0001 C CNN
F 3 "" H 5150 2050 60  0001 C CNN
	1    5150 2050
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 51A743EE
P 5150 1750
F 0 "R4" V 5230 1750 50  0000 C CNN
F 1 "3K9" V 5150 1750 50  0000 C CNN
F 2 "" H 5150 1750 60  0001 C CNN
F 3 "" H 5150 1750 60  0001 C CNN
	1    5150 1750
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 51A743EB
P 5150 1450
F 0 "R3" V 5230 1450 50  0000 C CNN
F 1 "1K" V 5150 1450 50  0000 C CNN
F 2 "" H 5150 1450 60  0001 C CNN
F 3 "" H 5150 1450 60  0001 C CNN
	1    5150 1450
	0    1    1    0   
$EndComp
$Comp
L LED D3
U 1 1 51A743A6
P 5750 2050
F 0 "D3" H 5750 2150 50  0000 C CNN
F 1 "GREEN" H 5750 1950 50  0000 C CNN
F 2 "" H 5750 2050 60  0001 C CNN
F 3 "" H 5750 2050 60  0001 C CNN
	1    5750 2050
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 51A743A4
P 5750 1750
F 0 "D2" H 5750 1850 50  0000 C CNN
F 1 "YELLOW" H 5750 1650 50  0000 C CNN
F 2 "" H 5750 1750 60  0001 C CNN
F 3 "" H 5750 1750 60  0001 C CNN
	1    5750 1750
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 51A743A2
P 5750 1450
F 0 "D1" H 5750 1550 50  0000 C CNN
F 1 "RED" H 5750 1350 50  0000 C CNN
F 2 "" H 5750 1450 60  0001 C CNN
F 3 "" H 5750 1450 60  0001 C CNN
	1    5750 1450
	1    0    0    -1  
$EndComp
Text Label 6500 4700 0    60   ~ 0
TXD
NoConn ~ 4550 4000
NoConn ~ 4550 3900
NoConn ~ 4550 3800
Text Label 6500 4200 0    60   ~ 0
LED03
Text Label 6500 4100 0    60   ~ 0
LED02
Text Label 6500 4000 0    60   ~ 0
LED01
Text Label 6500 4600 0    60   ~ 0
RXD
$Comp
L +5V #PWR04
U 1 1 504AC376
P 3100 2650
F 0 "#PWR04" H 3100 2740 20  0001 C CNN
F 1 "+5V" H 3100 2740 30  0000 C CNN
F 2 "" H 3100 2650 60  0001 C CNN
F 3 "" H 3100 2650 60  0001 C CNN
	1    3100 2650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR05
U 1 1 4ED08824
P 7750 1350
F 0 "#PWR05" H 7750 1440 20  0001 C CNN
F 1 "+5V" H 7750 1440 30  0000 C CNN
F 2 "" H 7750 1350 60  0001 C CNN
F 3 "" H 7750 1350 60  0001 C CNN
	1    7750 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 4ED08810
P 7750 2250
F 0 "#PWR06" H 7750 2250 30  0001 C CNN
F 1 "GND" H 7750 2180 30  0001 C CNN
F 2 "" H 7750 2250 60  0001 C CNN
F 3 "" H 7750 2250 60  0001 C CNN
	1    7750 2250
	1    0    0    -1  
$EndComp
$Comp
L CAPAPOL C2
U 1 1 4ED087CB
P 7750 1800
F 0 "C2" H 7800 1900 50  0000 L CNN
F 1 "22u" H 7800 1700 50  0000 L CNN
F 2 "" H 7750 1800 60  0001 C CNN
F 3 "" H 7750 1800 60  0001 C CNN
	1    7750 1800
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 4ED087A6
P 7350 1800
F 0 "C1" H 7400 1900 50  0000 L CNN
F 1 "0u1" H 7400 1700 50  0000 L CNN
F 2 "" H 7350 1800 60  0001 C CNN
F 3 "" H 7350 1800 60  0001 C CNN
	1    7350 1800
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR07
U 1 1 4ECC4A1D
P 3100 4400
F 0 "#PWR07" H 3100 4490 20  0001 C CNN
F 1 "+5V" H 3100 4490 30  0000 C CNN
F 2 "" H 3100 4400 60  0001 C CNN
F 3 "" H 3100 4400 60  0001 C CNN
	1    3100 4400
	1    0    0    -1  
$EndComp
Text Label 2850 5100 2    60   ~ 0
RESET
$Comp
L R R1
U 1 1 4ECC49EE
P 3100 4750
F 0 "R1" V 3180 4750 50  0000 C CNN
F 1 "5K6" V 3100 4750 50  0000 C CNN
F 2 "" H 3100 4750 60  0001 C CNN
F 3 "" H 3100 4750 60  0001 C CNN
	1    3100 4750
	1    0    0    -1  
$EndComp
Text Label 4300 4900 2    60   ~ 0
SCK
Text Label 4300 4800 2    60   ~ 0
MISO
Text Label 4300 4700 2    60   ~ 0
MOSI
$Comp
L GND #PWR08
U 1 1 4ECC4963
P 5550 6250
F 0 "#PWR08" H 5550 6250 30  0001 C CNN
F 1 "GND" H 5550 6180 30  0001 C CNN
F 2 "" H 5550 6250 60  0001 C CNN
F 3 "" H 5550 6250 60  0001 C CNN
	1    5550 6250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR09
U 1 1 4ECC494B
P 5550 3200
F 0 "#PWR09" H 5550 3290 20  0001 C CNN
F 1 "+5V" H 5550 3290 30  0000 C CNN
F 2 "" H 5550 3200 60  0001 C CNN
F 3 "" H 5550 3200 60  0001 C CNN
	1    5550 3200
	1    0    0    -1  
$EndComp
$Comp
L ATMEGA88P IC1
U 1 1 4ECC4939
P 5550 4600
F 0 "IC1" H 5550 5050 60  0000 C CNN
F 1 "ATMEGA88P" H 5550 5200 60  0000 C CNN
F 2 "" H 5550 4600 60  0001 C CNN
F 3 "" H 5550 4600 60  0001 C CNN
	1    5550 4600
	1    0    0    -1  
$EndComp
Text Label 3250 3250 0    60   ~ 0
RESET
Text Label 3250 3150 0    60   ~ 0
MOSI
Text Label 3250 3050 0    60   ~ 0
SCK
Text Label 3250 2850 0    60   ~ 0
MISO
$Comp
L GND #PWR010
U 1 1 4ECB2F15
P 3100 3550
F 0 "#PWR010" H 3100 3550 30  0001 C CNN
F 1 "GND" H 3100 3480 30  0001 C CNN
F 2 "" H 3100 3550 60  0001 C CNN
F 3 "" H 3100 3550 60  0001 C CNN
	1    3100 3550
	-1   0    0    -1  
$EndComp
$Comp
L CONN_6 P1
U 1 1 4ECB2EE0
P 2550 3100
F 0 "P1" V 2500 3100 60  0000 C CNN
F 1 "ISP" V 2600 3100 60  0000 C CNN
F 2 "" H 2550 3100 60  0001 C CNN
F 3 "" H 2550 3100 60  0001 C CNN
	1    2550 3100
	-1   0    0    -1  
$EndComp
NoConn ~ 6500 5200
NoConn ~ 4550 4600
NoConn ~ 4550 4500
Wire Wire Line
	6500 5100 7300 5100
Wire Wire Line
	7800 5100 8400 5100
Wire Wire Line
	8700 5300 8700 5450
$Comp
L NPN-BCE Q1
U 1 1 51B6AE78
P 8600 5100
F 0 "Q1" H 8600 4950 50  0000 R CNN
F 1 "BC847C" H 8600 5250 50  0000 R CNN
F 2 "" H 8600 5100 60  0000 C CNN
F 3 "" H 8600 5100 60  0000 C CNN
	1    8600 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 6100 6500 5550
Connection ~ 5650 6100
Wire Wire Line
	5950 1450 6150 1450
Wire Wire Line
	6150 1450 6150 2350
NoConn ~ 6500 4300
NoConn ~ 6500 3900
NoConn ~ 6500 3800
NoConn ~ 6500 5300
NoConn ~ 4550 5600
NoConn ~ 4550 5400
$Comp
L AUTOTRANS L1
U 1 1 5470FC2F
P 8350 4350
F 0 "L1" H 8500 4300 40  0000 C CNN
F 1 "AUTOTRANS" H 8600 4200 40  0000 C CNN
F 2 "~" H 8350 4300 60  0000 C CNN
F 3 "~" H 8350 4300 60  0000 C CNN
	1    8350 4350
	-1   0    0    -1  
$EndComp
NoConn ~ 6500 5000
NoConn ~ 6500 4900
NoConn ~ 6500 4800
$Comp
L CONN_3 K1
U 1 1 5470FFF1
P 2550 1850
F 0 "K1" V 2500 1850 50  0000 C CNN
F 1 "CONN_3" V 2600 1850 40  0000 C CNN
F 2 "" H 2550 1850 60  0000 C CNN
F 3 "" H 2550 1850 60  0000 C CNN
	1    2550 1850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR012
U 1 1 5470FFF7
P 3250 2150
F 0 "#PWR012" H 3250 2150 30  0001 C CNN
F 1 "GND" H 3250 2080 30  0001 C CNN
F 2 "" H 3250 2150 60  0001 C CNN
F 3 "" H 3250 2150 60  0001 C CNN
	1    3250 2150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2900 1950 3250 1950
$Comp
L +5V #PWR013
U 1 1 54710003
P 3250 1350
F 0 "#PWR013" H 3250 1440 20  0001 C CNN
F 1 "+5V" H 3250 1440 30  0000 C CNN
F 2 "" H 3250 1350 60  0001 C CNN
F 3 "" H 3250 1350 60  0001 C CNN
	1    3250 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1850 3250 1850
Wire Wire Line
	3250 1850 3250 1350
Wire Wire Line
	3250 1950 3250 2150
Wire Wire Line
	2900 1750 3500 1750
Text Label 3500 1750 0    60   ~ 0
SIG
Text Label 3600 4400 2    60   ~ 0
SIG
$Comp
L R R2
U 1 1 547100F8
P 4100 4400
F 0 "R2" V 4180 4400 50  0000 C CNN
F 1 "200" V 4100 4400 50  0000 C CNN
F 2 "" H 4100 4400 60  0001 C CNN
F 3 "" H 4100 4400 60  0001 C CNN
	1    4100 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 4400 4350 4400
Wire Wire Line
	3850 4400 3600 4400
Wire Wire Line
	7750 2000 7750 2250
Wire Wire Line
	7350 2000 7350 2100
Wire Wire Line
	7350 2100 7750 2100
Connection ~ 7750 2100
Wire Wire Line
	8950 4200 8950 4100
Wire Wire Line
	8950 4100 8450 4100
Wire Wire Line
	8450 4600 8700 4600
Wire Wire Line
	8700 4600 8700 4900
Wire Wire Line
	8450 4400 8950 4400
Wire Wire Line
	8700 3800 8700 4400
Connection ~ 8700 4400
Wire Wire Line
	3100 4500 3100 4400
$EndSCHEMATC
