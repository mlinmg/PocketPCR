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
LIBS:GaudiLabsPartsLibrary
LIBS:OpenAPD-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MAX1932 U1
U 1 1 593706D3
P 2700 2400
F 0 "U1" H 2350 3250 60  0000 C CNN
F 1 "MAX1932" H 2450 3150 60  0000 C CNN
F 2 "GaudiLabsFootPrints:QFN-12_4x4" H 2700 2400 60  0001 C CNN
F 3 "" H 2700 2400 60  0001 C CNN
	1    2700 2400
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR04
U 1 1 5937072F
P 2700 1350
F 0 "#PWR04" H 2700 1200 50  0001 C CNN
F 1 "VCC" H 2717 1523 50  0000 C CNN
F 2 "" H 2700 1350 50  0001 C CNN
F 3 "" H 2700 1350 50  0001 C CNN
	1    2700 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 59370769
P 2150 3550
F 0 "#PWR03" H 2150 3300 50  0001 C CNN
F 1 "GND" H 2155 3377 50  0000 C CNN
F 2 "" H 2150 3550 50  0001 C CNN
F 3 "" H 2150 3550 50  0001 C CNN
	1    2150 3550
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5937081A
P 1950 1700
F 0 "C1" H 2065 1746 50  0000 L CNN
F 1 "1uF" H 2065 1655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1988 1550 50  0001 C CNN
F 3 "" H 1950 1700 50  0001 C CNN
	1    1950 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 593708B8
P 1950 1950
F 0 "#PWR02" H 1950 1700 50  0001 C CNN
F 1 "GND" H 1955 1777 50  0000 C CNN
F 2 "" H 1950 1950 50  0001 C CNN
F 3 "" H 1950 1950 50  0001 C CNN
	1    1950 1950
	1    0    0    -1  
$EndComp
$Comp
L L L1
U 1 1 59370915
P 3500 1800
F 0 "L1" H 3553 1846 50  0000 L CNN
F 1 "250uH / 0.1A" H 3600 1950 50  0000 L CNN
F 2 "Choke_SMD:Choke_SMD_7.3x7.3_H3.5" H 3500 1800 50  0001 C CNN
F 3 "" H 3500 1800 50  0001 C CNN
	1    3500 1800
	1    0    0    -1  
$EndComp
$Comp
L BSS138 N1
U 1 1 59370B8C
P 3400 2250
F 0 "N1" H 3605 2296 50  0000 L CNN
F 1 "BSS131" H 3605 2205 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 3600 2175 50  0001 L CIN
F 3 "" H 3400 2250 50  0001 L CNN
	1    3400 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 59370C8A
P 3500 2900
F 0 "#PWR05" H 3500 2650 50  0001 C CNN
F 1 "GND" H 3505 2727 50  0000 C CNN
F 2 "" H 3500 2900 50  0001 C CNN
F 3 "" H 3500 2900 50  0001 C CNN
	1    3500 2900
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 59370CE2
P 4250 2000
F 0 "D1" H 4250 1784 50  0000 C CNN
F 1 "220V" H 4250 1875 50  0000 C CNN
F 2 "Diodes_SMD:SMB_Handsoldering" H 4250 2000 50  0001 C CNN
F 3 "" H 4250 2000 50  0001 C CNN
	1    4250 2000
	-1   0    0    1   
$EndComp
$Comp
L C C2
U 1 1 59370DAC
P 4750 2350
F 0 "C2" H 4865 2396 50  0000 L CNN
F 1 "0.047uF/200 V(smaller)" H 4600 2150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4788 2200 50  0001 C CNN
F 3 "" H 4750 2350 50  0001 C CNN
	1    4750 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 59370E8C
P 4750 2900
F 0 "#PWR06" H 4750 2650 50  0001 C CNN
F 1 "GND" H 4755 2727 50  0000 C CNN
F 2 "" H 4750 2900 50  0001 C CNN
F 3 "" H 4750 2900 50  0001 C CNN
	1    4750 2900
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 59371005
P 5150 2000
F 0 "R1" V 4943 2000 50  0000 C CNN
F 1 "4k" V 5034 2000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5080 2000 50  0001 C CNN
F 3 "" H 5150 2000 50  0001 C CNN
	1    5150 2000
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 593710A8
P 5900 2800
F 0 "C3" H 6015 2846 50  0000 L CNN
F 1 "0.1uF / 200V" H 6015 2755 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5938 2650 50  0001 C CNN
F 3 "" H 5900 2800 50  0001 C CNN
	1    5900 2800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 59371124
P 5900 3850
F 0 "#PWR08" H 5900 3600 50  0001 C CNN
F 1 "GND" H 5905 3677 50  0000 C CNN
F 2 "" H 5900 3850 50  0001 C CNN
F 3 "" H 5900 3850 50  0001 C CNN
	1    5900 3850
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 59371162
P 5450 2900
F 0 "R5" H 5700 2850 50  0000 R CNN
F 1 "2.2M (1M)" H 5850 2950 50  0000 R CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5380 2900 50  0001 C CNN
F 3 "" H 5450 2900 50  0001 C CNN
	1    5450 2900
	-1   0    0    1   
$EndComp
$Comp
L R R8
U 1 1 5937123B
P 5450 3550
F 0 "R8" H 5380 3504 50  0000 R CNN
F 1 "32.4k" H 5380 3595 50  0000 R CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5380 3550 50  0001 C CNN
F 3 "" H 5450 3550 50  0001 C CNN
	1    5450 3550
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR07
U 1 1 593712AE
P 5450 3850
F 0 "#PWR07" H 5450 3600 50  0001 C CNN
F 1 "GND" H 5455 3677 50  0000 C CNN
F 2 "" H 5450 3850 50  0001 C CNN
F 3 "" H 5450 3850 50  0001 C CNN
	1    5450 3850
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 593712F8
P 3250 3700
F 0 "R6" V 3457 3700 50  0000 C CNN
F 1 "24.9k" V 3366 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3180 3700 50  0001 C CNN
F 3 "" H 3250 3700 50  0001 C CNN
	1    3250 3700
	0    -1   -1   0   
$EndComp
$Comp
L R R7
U 1 1 59371833
P 1750 2550
F 0 "R7" H 1680 2504 50  0000 R CNN
F 1 "20k (higher?)" H 1680 2595 50  0000 R CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1680 2550 50  0001 C CNN
F 3 "" H 1750 2550 50  0001 C CNN
	1    1750 2550
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR01
U 1 1 59371909
P 1750 3200
F 0 "#PWR01" H 1750 2950 50  0001 C CNN
F 1 "GND" H 1755 3027 50  0000 C CNN
F 2 "" H 1750 3200 50  0001 C CNN
F 3 "" H 1750 3200 50  0001 C CNN
	1    1750 3200
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5937192C
P 1750 2950
F 0 "C4" H 1865 2996 50  0000 L CNN
F 1 "0.22uF" H 1865 2905 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1788 2800 50  0001 C CNN
F 3 "" H 1750 2950 50  0001 C CNN
	1    1750 2950
	1    0    0    -1  
$EndComp
$Comp
L APD D2
U 1 1 593B112F
P 6450 2450
F 0 "D2" H 6578 2503 60  0000 L CNN
F 1 "APD" H 6578 2397 60  0000 L CNN
F 2 "GaudiLabsFootPrints:LCC3_SMD" H 6450 2450 60  0001 C CNN
F 3 "" H 6450 2450 60  0001 C CNN
	1    6450 2450
	1    0    0    -1  
$EndComp
$Comp
L PAD X9
U 1 1 593B2ECF
P 6750 2000
F 0 "X9" H 6869 2068 60  0000 L CNN
F 1 "VBiasM" H 6869 1962 60  0000 L CNN
F 2 "GaudiLabsFootPrints:PAD_SMD_3mm" H 6750 2000 60  0001 C CNN
F 3 "" H 6750 2000 60  0001 C CNN
	1    6750 2000
	1    0    0    -1  
$EndComp
$Comp
L PAD X14
U 1 1 593FF858
P 6150 3650
F 0 "X14" H 6269 3718 60  0000 L CNN
F 1 "GND" H 6269 3612 60  0000 L CNN
F 2 "GaudiLabsFootPrints:PAD_SMD_3mm" H 6150 3650 60  0001 C CNN
F 3 "" H 6150 3650 60  0001 C CNN
	1    6150 3650
	1    0    0    -1  
$EndComp
$Comp
L Jumper_NC_Dual JP2
U 1 1 59406EB5
P 3750 6500
F 0 "JP2" V 3704 6601 50  0000 L CNN
F 1 "Jumper_NC_Dual" V 3795 6601 50  0000 L CNN
F 2 "Connect:GS3" H 3750 6500 50  0001 C CNN
F 3 "" H 3750 6500 50  0001 C CNN
	1    3750 6500
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5988C4BD
P 4250 2500
F 0 "R2" V 4200 2750 50  0000 C CNN
F 1 "1.2M (0)" V 4134 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4180 2500 50  0001 C CNN
F 3 "" H 4250 2500 50  0001 C CNN
	1    4250 2500
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 5988C555
P 4250 2650
F 0 "R3" V 4500 2650 50  0000 C CNN
F 1 "1.2M (0)" V 4400 2650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4180 2650 50  0001 C CNN
F 3 "" H 4250 2650 50  0001 C CNN
	1    4250 2650
	0    1    1    0   
$EndComp
$Comp
L ARDUINO_MICRO U2
U 1 1 5988D005
P 2050 5050
F 0 "U2" V 1853 5525 60  0000 C CNN
F 1 "ARDUINO_MICRO" V 1959 5525 60  0000 C CNN
F 2 "GaudiLabsFootPrints:ARDUINO_MICRO_SMD" H 2450 5000 60  0001 C CNN
F 3 "" H 2450 5000 60  0000 C CNN
	1    2050 5050
	0    1    1    0   
$EndComp
$Comp
L DC_JACK SW2
U 1 1 5988DB91
P 7850 5200
F 0 "SW2" H 7819 4822 50  0000 C CNN
F 1 "DC_JACK" H 7819 4913 50  0000 C CNN
F 2 "GaudiLabsFootPrints:DC_JACK_SMD" H 7850 5200 60  0001 C CNN
F 3 "" H 7850 5200 60  0000 C CNN
	1    7850 5200
	-1   0    0    1   
$EndComp
Text GLabel 1650 2200 0    60   Input ~ 0
CL_Flag
$Comp
L LED D3
U 1 1 59891FB0
P 5300 6000
F 0 "D3" V 5338 5883 50  0000 R CNN
F 1 "LED" V 5247 5883 50  0000 R CNN
F 2 "GaudiLabsFootPrints:TOPLED" H 5300 6000 50  0001 C CNN
F 3 "" H 5300 6000 50  0001 C CNN
	1    5300 6000
	0    -1   -1   0   
$EndComp
Text GLabel 5100 6450 0    60   Input ~ 0
CL_Flag
$Comp
L R R4
U 1 1 598922A8
P 5300 5450
F 0 "R4" V 5093 5450 50  0000 C CNN
F 1 "3.3k" V 5184 5450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5230 5450 50  0001 C CNN
F 3 "" H 5300 5450 50  0001 C CNN
	1    5300 5450
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR09
U 1 1 59892C0A
P 5300 5100
F 0 "#PWR09" H 5300 4950 50  0001 C CNN
F 1 "VCC" H 5317 5273 50  0000 C CNN
F 2 "" H 5300 5100 50  0001 C CNN
F 3 "" H 5300 5100 50  0001 C CNN
	1    5300 5100
	1    0    0    -1  
$EndComp
Text GLabel 1500 2500 0    60   Input ~ 0
CS
Text GLabel 1500 2650 0    60   Input ~ 0
SCKL
Text GLabel 1500 2800 0    60   Input ~ 0
DIN
Text GLabel 1650 5100 0    60   Input ~ 0
DIN
Text GLabel 3500 5100 2    60   Input ~ 0
SCKL
Text GLabel 1650 6500 0    60   Input ~ 0
CS
Text GLabel 1650 6200 0    60   Input ~ 0
CL_Flag
Text Notes 4600 650  0    60   ~ 0
ADA4817 (1x)\n
Text Notes 4650 900  0    60   ~ 0
OPA656 (1x)\n
$Comp
L OPA656 U3
U 1 1 59898E1D
P 8200 3100
F 0 "U3" H 8541 3146 50  0000 L CNN
F 1 "OPA656" H 8541 3055 50  0000 L CNN
F 2 "GaudiLabsFootPrints:SOIC-08_N" H 8200 2700 50  0001 C CNN
F 3 "" H 8200 3300 50  0000 C CNN
	1    8200 3100
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 59899D50
P 6000 5450
F 0 "R9" H 6250 5400 50  0000 R CNN
F 1 "100k" H 6400 5500 50  0000 R CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5930 5450 50  0001 C CNN
F 3 "" H 6000 5450 50  0001 C CNN
	1    6000 5450
	-1   0    0    1   
$EndComp
$Comp
L R R10
U 1 1 59899DCC
P 6000 5900
F 0 "R10" H 6250 5850 50  0000 R CNN
F 1 "100k" H 6400 5950 50  0000 R CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5930 5900 50  0001 C CNN
F 3 "" H 6000 5900 50  0001 C CNN
	1    6000 5900
	-1   0    0    1   
$EndComp
Text GLabel 3700 6150 2    60   Input ~ 0
AREF
$Comp
L GND #PWR010
U 1 1 5989AEB1
P 6000 6150
F 0 "#PWR010" H 6000 5900 50  0001 C CNN
F 1 "GND" H 6005 5977 50  0000 C CNN
F 2 "" H 6000 6150 50  0001 C CNN
F 3 "" H 6000 6150 50  0001 C CNN
	1    6000 6150
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR011
U 1 1 5989AEF2
P 6000 5150
F 0 "#PWR011" H 6000 5000 50  0001 C CNN
F 1 "VCC" H 6017 5323 50  0000 C CNN
F 2 "" H 6000 5150 50  0001 C CNN
F 3 "" H 6000 5150 50  0001 C CNN
	1    6000 5150
	1    0    0    -1  
$EndComp
Text GLabel 7700 3200 0    60   Input ~ 0
AREF
Text GLabel 4100 6250 2    60   Input ~ 0
2_5V
Text GLabel 6250 5650 2    60   Input ~ 0
2_5V
Text GLabel 9350 3100 2    60   Input ~ 0
SIG
Text GLabel 3450 6000 2    60   Input ~ 0
SIG
$Comp
L VCC #PWR012
U 1 1 5989E844
P 8100 2750
F 0 "#PWR012" H 8100 2600 50  0001 C CNN
F 1 "VCC" H 7950 2800 50  0000 C CNN
F 2 "" H 8100 2750 50  0001 C CNN
F 3 "" H 8100 2750 50  0001 C CNN
	1    8100 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5989F001
P 8100 3800
F 0 "#PWR013" H 8100 3550 50  0001 C CNN
F 1 "GND" H 8105 3627 50  0000 C CNN
F 2 "" H 8100 3800 50  0001 C CNN
F 3 "" H 8100 3800 50  0001 C CNN
	1    8100 3800
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 5989F619
P 8550 2550
F 0 "R11" V 8343 2550 50  0000 C CNN
F 1 "<1G Ohm" V 8434 2550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8480 2550 50  0001 C CNN
F 3 "" H 8550 2550 50  0001 C CNN
	1    8550 2550
	0    -1   -1   0   
$EndComp
$Comp
L C C6
U 1 1 5989F744
P 8550 2300
F 0 "C6" V 8400 2450 50  0000 L CNN
F 1 "0.5-1pF" V 8400 1950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8588 2150 50  0001 C CNN
F 3 "" H 8550 2300 50  0001 C CNN
	1    8550 2300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR014
U 1 1 598A30F6
P 6900 2650
F 0 "#PWR014" H 6900 2400 50  0001 C CNN
F 1 "GND" H 6905 2477 50  0000 C CNN
F 2 "" H 6900 2650 50  0001 C CNN
F 3 "" H 6900 2650 50  0001 C CNN
	1    6900 2650
	1    0    0    -1  
$EndComp
$Comp
L Peltier_Element PE1
U 1 1 598A78F0
P 9050 5200
F 0 "PE1" H 9050 5442 50  0000 C CNN
F 1 "Peltier_Element" H 9050 5351 50  0000 C CNN
F 2 "" H 9050 5130 50  0001 C CNN
F 3 "" V 9050 5225 50  0001 C CNN
	1    9050 5200
	1    0    0    -1  
$EndComp
$Comp
L PAD X2
U 1 1 598A8A84
P 8650 5400
F 0 "X2" H 8769 5468 60  0000 L CNN
F 1 "P+" H 8769 5362 60  0000 L CNN
F 2 "GaudiLabsFootPrints:PAD_SMD_4x2mm" H 8650 5400 60  0001 C CNN
F 3 "" H 8650 5400 60  0001 C CNN
	1    8650 5400
	0    1    1    0   
$EndComp
$Comp
L PAD X3
U 1 1 598A8DC8
P 9450 5400
F 0 "X3" H 9569 5468 60  0000 L CNN
F 1 "P-" H 9569 5362 60  0000 L CNN
F 2 "GaudiLabsFootPrints:PAD_SMD_4x2mm" H 9450 5400 60  0001 C CNN
F 3 "" H 9450 5400 60  0001 C CNN
	1    9450 5400
	0    1    1    0   
$EndComp
$Comp
L PAD X1
U 1 1 598AA55A
P 7800 3450
F 0 "X1" H 7919 3518 60  0000 L CNN
F 1 "AREF" H 7919 3412 60  0000 L CNN
F 2 "GaudiLabsFootPrints:PAD_SMD_3mm" H 7800 3450 60  0001 C CNN
F 3 "" H 7800 3450 60  0001 C CNN
	1    7800 3450
	0    1    1    0   
$EndComp
$Comp
L BUT SW1
U 1 1 598ABAED
P 4650 7300
F 0 "SW1" H 4650 7737 60  0000 C CNN
F 1 "BUT" H 4650 7631 60  0000 C CNN
F 2 "GaudiLabsFootPrints:PUSH_BUTTON_SMD" H 4650 7300 60  0001 C CNN
F 3 "" H 4650 7300 60  0001 C CNN
	1    4650 7300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 598ABE49
P 5000 7400
F 0 "#PWR015" H 5000 7150 50  0001 C CNN
F 1 "GND" H 5005 7227 50  0000 C CNN
F 2 "" H 5000 7400 50  0001 C CNN
F 3 "" H 5000 7400 50  0001 C CNN
	1    5000 7400
	1    0    0    -1  
$EndComp
Text GLabel 4150 7300 0    60   Input ~ 0
BUT
Text GLabel 1650 6100 0    60   Input ~ 0
BUT
$Comp
L GND #PWR016
U 1 1 598ACFBF
P 3650 5450
F 0 "#PWR016" H 3650 5200 50  0001 C CNN
F 1 "GND" H 3655 5277 50  0000 C CNN
F 2 "" H 3650 5450 50  0001 C CNN
F 3 "" H 3650 5450 50  0001 C CNN
	1    3650 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 598AD09B
P 1350 5650
F 0 "#PWR017" H 1350 5400 50  0001 C CNN
F 1 "GND" H 1355 5477 50  0000 C CNN
F 2 "" H 1350 5650 50  0001 C CNN
F 3 "" H 1350 5650 50  0001 C CNN
	1    1350 5650
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR018
U 1 1 598AD2E4
P 4050 5250
F 0 "#PWR018" H 4050 5100 50  0001 C CNN
F 1 "VCC" H 4067 5423 50  0000 C CNN
F 2 "" H 4050 5250 50  0001 C CNN
F 3 "" H 4050 5250 50  0001 C CNN
	1    4050 5250
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 5988A67E
P 3450 6900
F 0 "C5" H 3565 6946 50  0000 L CNN
F 1 "0.1uF" H 3565 6855 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3488 6750 50  0001 C CNN
F 3 "" H 3450 6900 50  0001 C CNN
	1    3450 6900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 5988B0F6
P 3450 7150
F 0 "#PWR019" H 3450 6900 50  0001 C CNN
F 1 "GND" H 3455 6977 50  0000 C CNN
F 2 "" H 3450 7150 50  0001 C CNN
F 3 "" H 3450 7150 50  0001 C CNN
	1    3450 7150
	1    0    0    -1  
$EndComp
Text Notes 5700 3000 0    60   ~ 0
closest to APD\n
Text Notes 5050 1750 0    60   ~ 0
500uA\n(250uA-1mA)\n\n
Text Notes 6550 1500 0    60   ~ 0
0.1-0.5nA dark current
$Comp
L BSS138 N2
U 1 1 5989AF8B
P 3850 2250
F 0 "N2" H 4055 2296 50  0000 L CNN
F 1 "BSS131" H 4055 2205 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 4050 2175 50  0001 L CIN
F 3 "" H 3850 2250 50  0001 L CNN
	1    3850 2250
	1    0    0    -1  
$EndComp
$Comp
L Thermistor_NTC TH1
U 1 1 598A1818
P 7000 5900
F 0 "TH1" H 7098 5946 50  0000 L CNN
F 1 "Thermistor_NTC 10k" H 7098 5855 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 7000 5950 50  0001 C CNN
F 3 "" H 7000 5950 50  0001 C CNN
	1    7000 5900
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 598A1D9C
P 7000 5450
F 0 "R12" H 7250 5400 50  0000 R CNN
F 1 "10k" H 7400 5500 50  0000 R CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6930 5450 50  0001 C CNN
F 3 "" H 7000 5450 50  0001 C CNN
	1    7000 5450
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR020
U 1 1 598A1E24
P 7000 5150
F 0 "#PWR020" H 7000 5000 50  0001 C CNN
F 1 "VCC" H 7017 5323 50  0000 C CNN
F 2 "" H 7000 5150 50  0001 C CNN
F 3 "" H 7000 5150 50  0001 C CNN
	1    7000 5150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 598A1E79
P 7000 6200
F 0 "#PWR021" H 7000 5950 50  0001 C CNN
F 1 "GND" H 7005 6027 50  0000 C CNN
F 2 "" H 7000 6200 50  0001 C CNN
F 3 "" H 7000 6200 50  0001 C CNN
	1    7000 6200
	1    0    0    -1  
$EndComp
Text GLabel 7250 5650 2    60   Input ~ 0
TEMP
Text GLabel 1700 5900 0    60   Input ~ 0
TEMP
$Comp
L Jumper_NC_Dual JP1
U 1 1 598C3916
P 5700 2000
F 0 "JP1" V 5654 2101 50  0000 L CNN
F 1 "Jumper_NC_Dual" V 5745 2101 50  0000 L CNN
F 2 "Connect:GS3" H 5700 2000 50  0001 C CNN
F 3 "" H 5700 2000 50  0001 C CNN
	1    5700 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2300 2950 2150 2950
Wire Wire Line
	2150 2950 2150 3550
Wire Wire Line
	1950 1950 1950 1850
Wire Wire Line
	1950 1550 1950 1500
Connection ~ 2700 1500
Wire Wire Line
	3500 1500 3500 1650
Wire Wire Line
	3500 1950 3500 2050
Wire Wire Line
	3500 2450 3500 2900
Connection ~ 3500 2000
Wire Wire Line
	4400 2000 5000 2000
Wire Wire Line
	4750 2000 4750 2200
Wire Wire Line
	4750 2500 4750 2900
Connection ~ 4750 2000
Wire Wire Line
	4950 2000 4950 2500
Connection ~ 4950 2000
Wire Wire Line
	5450 2000 5450 2750
Wire Wire Line
	5900 2000 5900 2650
Connection ~ 5450 2650
Wire Wire Line
	3100 2850 3100 3250
Wire Wire Line
	3100 3250 5450 3250
Wire Wire Line
	5450 3050 5450 3400
Connection ~ 5450 3250
Wire Wire Line
	5450 3700 5450 3850
Wire Wire Line
	3450 3250 3450 3700
Wire Wire Line
	3450 3700 3400 3700
Connection ~ 3450 3250
Wire Wire Line
	2700 3700 3100 3700
Wire Wire Line
	2700 3150 2700 3700
Wire Wire Line
	2300 2350 1750 2350
Wire Wire Line
	1750 2350 1750 2400
Wire Wire Line
	1750 2700 1750 2800
Wire Wire Line
	1750 3100 1750 3200
Connection ~ 5900 2000
Wire Wire Line
	1650 2200 2300 2200
Wire Wire Line
	2700 1350 2700 1950
Wire Wire Line
	2300 2500 1500 2500
Wire Wire Line
	2300 2650 1500 2650
Wire Wire Line
	2300 2800 1500 2800
Wire Wire Line
	5300 5100 5300 5300
Wire Wire Line
	5300 5600 5300 5850
Wire Wire Line
	5300 6150 5300 6450
Wire Wire Line
	5300 6450 5100 6450
Wire Wire Line
	1850 5100 1650 5100
Wire Wire Line
	3500 5100 3200 5100
Wire Wire Line
	1850 6500 1650 6500
Wire Wire Line
	1850 6200 1650 6200
Wire Wire Line
	4950 2500 4400 2500
Wire Wire Line
	4400 2650 5450 2650
Wire Wire Line
	6000 5150 6000 5300
Wire Wire Line
	6000 5600 6000 5750
Wire Wire Line
	6000 6050 6000 6150
Wire Wire Line
	6000 5650 6250 5650
Connection ~ 6000 5650
Wire Wire Line
	7700 3200 7900 3200
Wire Wire Line
	3200 6500 3650 6500
Wire Wire Line
	3700 6150 3450 6150
Wire Wire Line
	3450 6150 3450 6750
Connection ~ 3450 6500
Wire Wire Line
	3200 6600 3600 6600
Wire Wire Line
	3600 6600 3600 6750
Wire Wire Line
	3600 6750 3750 6750
Wire Wire Line
	3750 6250 4100 6250
Wire Wire Line
	3450 6000 3300 6000
Wire Wire Line
	3300 6000 3300 6400
Wire Wire Line
	3300 6400 3200 6400
Wire Wire Line
	8500 3100 9350 3100
Wire Wire Line
	8100 3400 8100 3800
Wire Wire Line
	6450 3000 7900 3000
Wire Wire Line
	8700 2300 9050 2300
Wire Wire Line
	9050 2300 9050 3100
Connection ~ 9050 3100
Wire Wire Line
	8400 2300 7550 2300
Wire Wire Line
	7550 2300 7550 3000
Connection ~ 7550 3000
Wire Wire Line
	8100 2750 8100 2800
Wire Wire Line
	8400 2550 7550 2550
Connection ~ 7550 2550
Wire Wire Line
	8700 2550 9050 2550
Connection ~ 9050 2550
Wire Wire Line
	6900 2450 6900 2650
Wire Wire Line
	4750 2000 4950 2000
Wire Wire Line
	5800 2000 6650 2000
Wire Wire Line
	6050 3650 5900 3650
Connection ~ 5900 3650
Wire Wire Line
	5900 2950 5900 3850
Connection ~ 6450 2000
Wire Wire Line
	6450 2000 6450 2150
Wire Wire Line
	6450 2750 6450 3000
Wire Wire Line
	8150 5200 8850 5200
Wire Wire Line
	8150 4800 9700 4800
Wire Wire Line
	8150 4800 8150 5000
Wire Wire Line
	9700 4800 9700 5200
Wire Wire Line
	9700 5200 9250 5200
Wire Wire Line
	9450 5300 9450 5200
Connection ~ 9450 5200
Wire Wire Line
	8650 5300 8650 5200
Connection ~ 8650 5200
Wire Wire Line
	7800 3200 7800 3350
Connection ~ 7800 3200
Wire Wire Line
	4950 7300 5000 7300
Wire Wire Line
	5000 7300 5000 7400
Wire Wire Line
	4150 7300 4350 7300
Wire Wire Line
	1650 6100 1850 6100
Wire Wire Line
	3200 5400 3650 5400
Wire Wire Line
	3650 5400 3650 5450
Wire Wire Line
	1850 5600 1350 5600
Wire Wire Line
	1350 5600 1350 5650
Wire Wire Line
	3200 5600 4050 5600
Wire Wire Line
	4050 5600 4050 5250
Wire Wire Line
	3450 7050 3450 7150
Wire Wire Line
	3100 2250 3200 2250
Wire Wire Line
	1950 1500 3500 1500
Wire Wire Line
	3500 2550 3950 2550
Connection ~ 3500 2550
Wire Wire Line
	3950 2550 3950 2450
Wire Wire Line
	3950 2050 3950 2000
Connection ~ 3950 2000
Wire Wire Line
	3500 2000 4100 2000
Wire Wire Line
	4100 2500 3100 2500
Wire Wire Line
	3100 2650 4100 2650
Wire Wire Line
	3650 2250 3650 2400
Wire Wire Line
	3650 2400 3150 2400
Wire Wire Line
	3150 2400 3150 2250
Connection ~ 3150 2250
Wire Wire Line
	7000 6050 7000 6200
Wire Wire Line
	7000 5600 7000 5750
Wire Wire Line
	7000 5300 7000 5150
Wire Wire Line
	7000 5650 7250 5650
Connection ~ 7000 5650
Wire Wire Line
	1850 5900 1700 5900
$Comp
L PAD X4
U 1 1 598C54FE
P 9250 6000
F 0 "X4" H 9369 6068 60  0000 L CNN
F 1 "BLOWER+" H 9369 5962 60  0000 L CNN
F 2 "GaudiLabsFootPrints:PAD_SMD_3mm" H 9250 6000 60  0001 C CNN
F 3 "" H 9250 6000 60  0001 C CNN
	1    9250 6000
	1    0    0    -1  
$EndComp
$Comp
L PAD X5
U 1 1 598C576B
P 9250 6250
F 0 "X5" H 9369 6318 60  0000 L CNN
F 1 "BLOWER-" H 9369 6212 60  0000 L CNN
F 2 "GaudiLabsFootPrints:PAD_SMD_3mm" H 9250 6250 60  0001 C CNN
F 3 "" H 9250 6250 60  0001 C CNN
	1    9250 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 6250 9150 6250
Wire Wire Line
	8350 6000 9150 6000
Wire Wire Line
	5700 2250 5450 2250
Connection ~ 5450 2250
Wire Wire Line
	5300 2000 5450 2000
Connection ~ 8250 4800
Wire Wire Line
	8250 4800 8250 6250
Wire Wire Line
	8350 5200 8350 6000
Connection ~ 8350 5200
$EndSCHEMATC