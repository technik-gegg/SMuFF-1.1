EESchema Schematic File Version 4
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "SMuFF ESP32 Controller Board"
Date "2020-05-01"
Rev "1"
Comp "Technik Gegg"
Comment1 "SMuFF Firmware Version 2.xx needed"
Comment2 "Based on NodeMCU ESP32 devkit-1"
Comment3 ""
Comment4 "Proof Of Concept for SMuFF-ESP32 Controller"
$EndDescr
$Comp
L Driver_Motor:Pololu_Breakout_A4988 A1
U 1 1 5EB01BE9
P 8350 5550
F 0 "A1" H 8850 6100 50  0000 C CNN
F 1 "Feeder driver" V 8450 5500 50  0000 C CNN
F 2 "Module:Pololu_Breakout-16_15.2x20.3mm" H 8625 4800 50  0001 L CNN
F 3 "https://www.pololu.com/product/2980/pictures" H 8450 5250 50  0001 C CNN
	1    8350 5550
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:Pololu_Breakout_A4988 A2
U 1 1 5EB043F6
P 5950 5550
F 0 "A2" H 6450 6100 50  0000 C CNN
F 1 "Selector driver" V 6050 5450 50  0000 C CNN
F 2 "Module:Pololu_Breakout-16_15.2x20.3mm" H 6225 4800 50  0001 L CNN
F 3 "https://www.pololu.com/product/2980/pictures" H 6050 5250 50  0001 C CNN
	1    5950 5550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J5
U 1 1 5EB0D0E3
P 7550 5950
F 0 "J5" H 7600 6150 50  0000 C CNN
F 1 "Step Mode" H 7600 5750 50  0000 C CNN
F 2 "" H 7550 5950 50  0001 C CNN
F 3 "~" H 7550 5950 50  0001 C CNN
	1    7550 5950
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J6
U 1 1 5EB11471
P 5150 5950
F 0 "J6" H 5200 6150 50  0000 C CNN
F 1 "Step Mode" H 5200 5750 50  0000 C CNN
F 2 "" H 5150 5950 50  0001 C CNN
F 3 "~" H 5150 5950 50  0001 C CNN
	1    5150 5950
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0101
U 1 1 5EB12A34
P 7250 5800
F 0 "#PWR0101" H 7250 5650 50  0001 C CNN
F 1 "VDD" H 7250 5950 50  0000 C CNN
F 2 "" H 7250 5800 50  0001 C CNN
F 3 "" H 7250 5800 50  0001 C CNN
	1    7250 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5EB16214
P 5950 6350
F 0 "#PWR0103" H 5950 6100 50  0001 C CNN
F 1 "GND" H 5800 6300 50  0000 C CNN
F 2 "" H 5950 6350 50  0001 C CNN
F 3 "" H 5950 6350 50  0001 C CNN
	1    5950 6350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5EB16B66
P 3350 5350
F 0 "#PWR0105" H 3350 5100 50  0001 C CNN
F 1 "GND" H 3355 5177 50  0000 C CNN
F 2 "" H 3350 5350 50  0001 C CNN
F 3 "" H 3350 5350 50  0001 C CNN
	1    3350 5350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5EB1570B
P 8350 6350
F 0 "#PWR0107" H 8350 6100 50  0001 C CNN
F 1 "GND" H 8200 6300 50  0000 C CNN
F 2 "" H 8350 6350 50  0001 C CNN
F 3 "" H 8350 6350 50  0001 C CNN
	1    8350 6350
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0108
U 1 1 5EB1CFC4
P 7900 4850
F 0 "#PWR0108" H 7900 4700 50  0001 C CNN
F 1 "VDD" H 7900 5000 50  0000 C CNN
F 2 "" H 7900 4850 50  0001 C CNN
F 3 "" H 7900 4850 50  0001 C CNN
	1    7900 4850
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0109
U 1 1 5EB1D7E3
P 5500 4850
F 0 "#PWR0109" H 5500 4700 50  0001 C CNN
F 1 "VDD" H 5500 5000 50  0000 C CNN
F 2 "" H 5500 4850 50  0001 C CNN
F 3 "" H 5500 4850 50  0001 C CNN
	1    5500 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 5250 7900 5250
Wire Wire Line
	8350 4850 7900 4850
Wire Wire Line
	7900 4850 7900 5150
Wire Wire Line
	7950 5150 7900 5150
Connection ~ 7900 5150
Wire Wire Line
	7900 5150 7900 5250
Wire Wire Line
	5550 5250 5500 5250
Wire Wire Line
	5500 5250 5500 5150
Wire Wire Line
	5550 5150 5500 5150
Connection ~ 5500 5150
Wire Wire Line
	5500 5150 5500 4850
Wire Wire Line
	5950 4850 5500 4850
Connection ~ 5500 4850
$Comp
L Connector:Conn_01x03_Male J9
U 1 1 5EB2F21A
P 1000 6650
F 0 "J9" H 900 6650 50  0000 C CNN
F 1 "Selector Endstop" H 1050 6850 50  0000 C CNN
F 2 "" H 1000 6650 50  0001 C CNN
F 3 "~" H 1000 6650 50  0001 C CNN
	1    1000 6650
	1    0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male J8
U 1 1 5EB306BC
P 1000 7350
F 0 "J8" H 950 7350 50  0000 R CNN
F 1 "Feeder Endstop" H 1400 7100 50  0000 R CNN
F 2 "" H 1000 7350 50  0001 C CNN
F 3 "~" H 1000 7350 50  0001 C CNN
	1    1000 7350
	1    0    0    1   
$EndComp
$Comp
L power:VDD #PWR0110
U 1 1 5EB3C46D
P 1300 7250
F 0 "#PWR0110" H 1300 7100 50  0001 C CNN
F 1 "VDD" V 1318 7378 50  0000 L CNN
F 2 "" H 1300 7250 50  0001 C CNN
F 3 "" H 1300 7250 50  0001 C CNN
	1    1300 7250
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5EB3CF1F
P 1300 6750
F 0 "#PWR0111" H 1300 6500 50  0001 C CNN
F 1 "GND" V 1305 6622 50  0000 R CNN
F 2 "" H 1300 6750 50  0001 C CNN
F 3 "" H 1300 6750 50  0001 C CNN
	1    1300 6750
	0    -1   1    0   
$EndComp
Wire Wire Line
	1200 7450 1300 7450
Wire Wire Line
	1200 6750 1300 6750
$Comp
L Connector:Micro_SD_Card J7
U 1 1 5EB57AE5
P 6650 3650
F 0 "J7" H 5800 4150 50  0000 C CNN
F 1 "(Micro) SD Card Reader" H 6400 3100 50  0000 C CNN
F 2 "" H 7800 3950 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/693072010801.pdf" H 6650 3650 50  0001 C CNN
	1    6650 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5EB7E21C
P 5650 3850
F 0 "#PWR0112" H 5650 3600 50  0001 C CNN
F 1 "GND" V 5650 3700 50  0000 R CNN
F 2 "" H 5650 3850 50  0001 C CNN
F 3 "" H 5650 3850 50  0001 C CNN
	1    5650 3850
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR0113
U 1 1 5EB7F0BE
P 5650 3650
F 0 "#PWR0113" H 5650 3500 50  0001 C CNN
F 1 "VDD" V 5650 3800 50  0000 L CNN
F 2 "" H 5650 3650 50  0001 C CNN
F 3 "" H 5650 3650 50  0001 C CNN
	1    5650 3650
	0    -1   -1   0   
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW1
U 1 1 5EB9B1E9
P 3500 1300
F 0 "SW1" H 3500 1667 50  0000 C CNN
F 1 "Rotary Encoder" H 3500 1576 50  0000 C CNN
F 2 "" H 3350 1460 50  0001 C CNN
F 3 "~" H 3500 1560 50  0001 C CNN
	1    3500 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1400 2750 1400
Wire Wire Line
	2700 1200 3200 1200
$Comp
L power:GND #PWR0114
U 1 1 5EBA7E7B
P 3850 1400
F 0 "#PWR0114" H 3850 1150 50  0001 C CNN
F 1 "GND" V 3855 1272 50  0000 R CNN
F 2 "" H 3850 1400 50  0001 C CNN
F 3 "" H 3850 1400 50  0001 C CNN
	1    3850 1400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5EBA86EB
P 3150 1300
F 0 "#PWR0115" H 3150 1050 50  0001 C CNN
F 1 "GND" V 3155 1172 50  0000 R CNN
F 2 "" H 3150 1300 50  0001 C CNN
F 3 "" H 3150 1300 50  0001 C CNN
	1    3150 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	3800 1400 3850 1400
Wire Wire Line
	3200 1300 3150 1300
$Comp
L Device:R R1
U 1 1 5EBAFD39
P 2500 1200
F 0 "R1" V 2400 1200 50  0000 C CNN
F 1 "10K" V 2500 1200 50  0000 C CNN
F 2 "" V 2430 1200 50  0001 C CNN
F 3 "~" H 2500 1200 50  0001 C CNN
	1    2500 1200
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5EBB0953
P 2500 1400
F 0 "R2" V 2400 1400 50  0000 C CNN
F 1 "10K" V 2500 1400 50  0000 C CNN
F 2 "" V 2430 1400 50  0001 C CNN
F 3 "~" H 2500 1400 50  0001 C CNN
	1    2500 1400
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR0116
U 1 1 5EBB0FBA
P 2200 1100
F 0 "#PWR0116" H 2200 950 50  0001 C CNN
F 1 "VDD" H 2217 1273 50  0000 C CNN
F 2 "" H 2200 1100 50  0001 C CNN
F 3 "" H 2200 1100 50  0001 C CNN
	1    2200 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1200 2200 1200
Wire Wire Line
	2200 1200 2200 1100
Wire Wire Line
	2350 1400 2200 1400
Wire Wire Line
	2200 1400 2200 1200
Connection ~ 2200 1200
Wire Wire Line
	5350 1350 4850 1350
Wire Wire Line
	4900 1450 5350 1450
$Comp
L power:VDD #PWR0117
U 1 1 5EBE08AB
P 4950 1150
F 0 "#PWR0117" H 4950 1000 50  0001 C CNN
F 1 "VDD" V 5000 1100 50  0000 C CNN
F 2 "" H 4950 1150 50  0001 C CNN
F 3 "" H 4950 1150 50  0001 C CNN
	1    4950 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5EBEA456
P 4950 1250
F 0 "#PWR0118" H 4950 1000 50  0001 C CNN
F 1 "GND" V 4900 1300 50  0000 C CNN
F 2 "" H 4950 1250 50  0001 C CNN
F 3 "" H 4950 1250 50  0001 C CNN
	1    4950 1250
	0    1    1    0   
$EndComp
$Comp
L Device:CP C1
U 1 1 5EBFD83C
P 8850 4700
F 0 "C1" V 8900 4600 50  0000 C CNN
F 1 "100uF/35V" V 8700 4700 50  0000 C CNN
F 2 "" H 8888 4550 50  0001 C CNN
F 3 "~" H 8850 4700 50  0001 C CNN
	1    8850 4700
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP C2
U 1 1 5EBFE5F0
P 6500 4700
F 0 "C2" V 6550 4600 50  0000 C CNN
F 1 "100uF/35V" V 6350 4700 50  0000 C CNN
F 2 "" H 6538 4550 50  0001 C CNN
F 3 "~" H 6500 4700 50  0001 C CNN
	1    6500 4700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5EC09BD6
P 9050 4700
F 0 "#PWR0119" H 9050 4450 50  0001 C CNN
F 1 "GND" V 9055 4572 50  0000 R CNN
F 2 "" H 9050 4700 50  0001 C CNN
F 3 "" H 9050 4700 50  0001 C CNN
	1    9050 4700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5EC0ECD8
P 6700 4700
F 0 "#PWR0120" H 6700 4450 50  0001 C CNN
F 1 "GND" V 6705 4572 50  0000 R CNN
F 2 "" H 6700 4700 50  0001 C CNN
F 3 "" H 6700 4700 50  0001 C CNN
	1    6700 4700
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x03_Male J10
U 1 1 5EC1DDFC
P 950 5950
F 0 "J10" H 850 5950 50  0000 C CNN
F 1 "Wiper Servo (1)" H 1100 6200 50  0000 C CNN
F 2 "" H 950 5950 50  0001 C CNN
F 3 "~" H 950 5950 50  0001 C CNN
	1    950  5950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J11
U 1 1 5EC1F255
P 1000 5400
F 0 "J11" H 900 5400 50  0000 C CNN
F 1 "Revolver Servo (2)" H 1100 5650 50  0000 C CNN
F 2 "" H 1000 5400 50  0001 C CNN
F 3 "~" H 1000 5400 50  0001 C CNN
	1    1000 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5EC2C644
P 1400 5500
F 0 "#PWR0121" H 1400 5250 50  0001 C CNN
F 1 "GND" V 1405 5372 50  0000 R CNN
F 2 "" H 1400 5500 50  0001 C CNN
F 3 "" H 1400 5500 50  0001 C CNN
	1    1400 5500
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J14
U 1 1 5EC4D5B9
P 7050 850
F 0 "J14" V 7050 1100 50  0000 R CNN
F 1 "12V/24V" V 7050 650 50  0000 R CNN
F 2 "" H 7050 850 50  0001 C CNN
F 3 "~" H 7050 850 50  0001 C CNN
	1    7050 850 
	0    -1   -1   0   
$EndComp
$Comp
L Converter_DCDC:NCS1S1205SC U2
U 1 1 5EC64527
P 9800 1300
F 0 "U2" H 9800 1767 50  0000 C CNN
F 1 "Step-down converter" H 9800 1676 50  0000 C CNN
F 2 "Converter_DCDC:Converter_DCDC_Murata_NCS1SxxxxSC_THT" H 9800 900 50  0001 C CNN
F 3 "https://power.murata.com/data/power/ncl/kdc_ncs1.pdf" H 9775 1300 50  0001 C CNN
	1    9800 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 5EC7E711
P 10550 1500
F 0 "#PWR0124" H 10550 1250 50  0001 C CNN
F 1 "GND" V 10555 1372 50  0000 R CNN
F 2 "" H 10550 1500 50  0001 C CNN
F 3 "" H 10550 1500 50  0001 C CNN
	1    10550 1500
	0    -1   -1   0   
$EndComp
Text Label 5000 1350 0    50   ~ 0
SCL
Text Label 5000 1450 0    50   ~ 0
SDA
$Comp
L SSD1306-128x64_OLED:SSD1306 Brd1
U 1 1 5ECB7467
P 5700 1300
F 0 "Brd1" V 5400 1400 50  0000 R CNN
F 1 "SSD1306 OLED 0.96\"" H 6050 950 50  0000 R CNN
F 2 "" H 5700 1550 50  0001 C CNN
F 3 "" H 5700 1550 50  0001 C CNN
	1    5700 1300
	0    -1   1    0   
$EndComp
Wire Wire Line
	10300 1100 10550 1100
Wire Wire Line
	10300 1500 10550 1500
$Comp
L power:VDD #PWR0102
U 1 1 5EB13DD4
P 4900 5800
F 0 "#PWR0102" H 4900 5650 50  0001 C CNN
F 1 "VDD" H 4900 5950 50  0000 C CNN
F 2 "" H 4900 5800 50  0001 C CNN
F 3 "" H 4900 5800 50  0001 C CNN
	1    4900 5800
	1    0    0    -1  
$EndComp
Text Notes 700  7700 0    98   ~ 20
Endstops\n
Text Notes 4850 4400 0    98   ~ 20
SD-Card
Text Notes 4850 800  0    98   ~ 20
Display
Text Notes 750  4950 0    98   ~ 20
Servos
Text Notes 8900 750  0    98   ~ 20
Primary Buck Converter
$Comp
L Connector:Conn_01x03_Male J13
U 1 1 5EED2946
P 1000 4150
F 0 "J13" H 950 4150 50  0000 R CNN
F 1 "To Printer Controller" H 1600 3900 50  0000 R CNN
F 2 "" H 1000 4150 50  0001 C CNN
F 3 "~" H 1000 4150 50  0001 C CNN
	1    1000 4150
	1    0    0    1   
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5EEF49DE
P 5850 2100
F 0 "J1" H 5822 2074 50  0000 R CNN
F 1 "Serial" H 5822 1983 50  0000 R CNN
F 2 "" H 5850 2100 50  0001 C CNN
F 3 "~" H 5850 2100 50  0001 C CNN
	1    5850 2100
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5EEF5559
P 7350 2100
F 0 "J2" H 7322 2074 50  0000 R CNN
F 1 "Serial" H 7322 1983 50  0000 R CNN
F 2 "" H 7350 2100 50  0001 C CNN
F 3 "~" H 7350 2100 50  0001 C CNN
	1    7350 2100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 1250 5350 1250
Wire Wire Line
	4950 1150 5350 1150
$Comp
L power:GND #PWR03
U 1 1 5EF51A80
P 5550 2200
F 0 "#PWR03" H 5550 1950 50  0001 C CNN
F 1 "GND" V 5550 1950 50  0000 C CNN
F 2 "" H 5550 2200 50  0001 C CNN
F 3 "" H 5550 2200 50  0001 C CNN
	1    5550 2200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5EF5A82D
P 7050 2200
F 0 "#PWR04" H 7050 1950 50  0001 C CNN
F 1 "GND" V 7050 2000 50  0000 C CNN
F 2 "" H 7050 2200 50  0001 C CNN
F 3 "" H 7050 2200 50  0001 C CNN
	1    7050 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	5550 2200 5650 2200
Wire Wire Line
	7050 2200 7150 2200
Text Notes 4850 2550 0    98   ~ 20
Primary Serial
Text Notes 6350 2550 0    98   ~ 20
Secondary Serial\n
Text Notes 9000 1850 0    47   Italic 0
VDD = 3.3V Buck Converter on NodeMCU (3V3 pin)
Text Label 1550 5850 0    47   ~ 0
PWM
Text Label 1600 5300 0    47   ~ 0
PWM
$Comp
L Transistor_FET:BS170 Q1
U 1 1 5F01573A
P 1500 2550
F 0 "Q1" H 1706 2596 50  0000 L CNN
F 1 "BS170" H 1706 2505 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 1700 2475 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BS/BS170.pdf" H 1500 2550 50  0001 L CNN
	1    1500 2550
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5F017297
P 1400 2850
F 0 "#PWR01" H 1400 2600 50  0001 C CNN
F 1 "GND" H 1405 2677 50  0000 C CNN
F 2 "" H 1400 2850 50  0001 C CNN
F 3 "" H 1400 2850 50  0001 C CNN
	1    1400 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2750 1400 2800
$Comp
L Device:R R3
U 1 1 5F02132F
P 1950 2550
F 0 "R3" V 1850 2550 50  0000 C CNN
F 1 "10R" V 1950 2550 50  0000 C CNN
F 2 "" V 1880 2550 50  0001 C CNN
F 3 "~" H 1950 2550 50  0001 C CNN
	1    1950 2550
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F022690
P 1950 2800
F 0 "R4" V 1850 2800 50  0000 C CNN
F 1 "100K" V 1950 2800 50  0000 C CNN
F 2 "" V 1880 2800 50  0001 C CNN
F 3 "~" H 1950 2800 50  0001 C CNN
	1    1950 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 2550 1700 2550
Wire Wire Line
	1800 2800 1400 2800
Connection ~ 1400 2800
Wire Wire Line
	1400 2800 1400 2850
Wire Wire Line
	2100 2800 2150 2800
Wire Wire Line
	2150 2800 2150 2550
$Comp
L Connector:Conn_01x02_Male J12
U 1 1 5F041AA4
P 1000 2100
F 0 "J12" H 900 2050 50  0000 C CNN
F 1 "FAN" H 1108 2190 50  0000 C CNN
F 2 "" H 1000 2100 50  0001 C CNN
F 3 "~" H 1000 2100 50  0001 C CNN
	1    1000 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2200 1400 2200
Wire Wire Line
	1400 2200 1400 2350
Text GLabel 1500 2100 2    47   Input ~ 0
VIN
Wire Wire Line
	1200 2100 1500 2100
Text GLabel 6750 1100 0    47   Input ~ 0
VIN
Text GLabel 10550 1100 2    47   Input ~ 0
VCC-5V
Text GLabel 2900 3600 0    47   Input ~ 0
VCC-5V
Text GLabel 1450 5950 2    47   Input ~ 0
VCC-5V
Text GLabel 6800 2300 0    47   Input ~ 0
VCC-5V
$Comp
L Device:Buzzer BZ1
U 1 1 5F0AC937
P 1250 1200
F 0 "BZ1" H 1500 1200 50  0000 C CNN
F 1 "Piezzo" H 1250 1400 50  0000 C CNN
F 2 "" V 1225 1300 50  0001 C CNN
F 3 "~" V 1225 1300 50  0001 C CNN
	1    1250 1200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5F0BAD47
P 1500 1100
F 0 "#PWR02" H 1500 850 50  0001 C CNN
F 1 "GND" H 1505 927 50  0000 C CNN
F 2 "" H 1500 1100 50  0001 C CNN
F 3 "" H 1500 1100 50  0001 C CNN
	1    1500 1100
	0    -1   -1   0   
$EndComp
Text Notes 750  3300 0    98   ~ 20
Stepper Fan
Text Notes 750  850  0    98   ~ 20
Beeper
Text Notes 2150 750  0    98   ~ 20
Rotary Encoder
Text GLabel 6000 4700 0    47   Input ~ 0
VIN
Wire Wire Line
	1350 1100 1500 1100
$Comp
L Connector:Conn_01x05_Male J15
U 1 1 5F2203E1
P 10500 6150
F 0 "J15" H 10472 6124 50  0000 R CNN
F 1 "To I2C Device" V 10200 6400 50  0000 R CNN
F 2 "" H 10500 6150 50  0001 C CNN
F 3 "~" H 10500 6150 50  0001 C CNN
	1    10500 6150
	-1   0    0    -1  
$EndComp
$Comp
L power:VDD #PWR09
U 1 1 5F23AAAF
P 10200 6050
F 0 "#PWR09" H 10200 5900 50  0001 C CNN
F 1 "VDD" V 10200 6250 50  0000 C CNN
F 2 "" H 10200 6050 50  0001 C CNN
F 3 "" H 10200 6050 50  0001 C CNN
	1    10200 6050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5F23AAB5
P 10200 6150
F 0 "#PWR010" H 10200 5900 50  0001 C CNN
F 1 "GND" V 10200 5950 50  0000 C CNN
F 2 "" H 10200 6150 50  0001 C CNN
F 3 "" H 10200 6150 50  0001 C CNN
	1    10200 6150
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x04_Male J4
U 1 1 5EB254BA
P 6900 5550
F 0 "J4" H 7050 5800 50  0000 R CNN
F 1 "To Stepper Motor" V 6800 5800 50  0000 R CNN
F 2 "" H 6900 5550 50  0001 C CNN
F 3 "~" H 6900 5550 50  0001 C CNN
	1    6900 5550
	-1   0    0    -1  
$EndComp
$Comp
L Interface_Expansion:PCF8574 U3
U 1 1 5F260BF0
P 9800 3000
F 0 "U3" H 9500 3600 50  0000 C CNN
F 1 "PCF8574" H 9750 2750 50  0000 C CNN
F 2 "" H 9800 3000 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCF8574_PCF8574A.pdf" H 9800 3000 50  0001 C CNN
	1    9800 3000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 5EB23C13
P 9350 5550
F 0 "J3" H 9500 5800 50  0000 R CNN
F 1 "To Stepper Motor" V 9250 5800 50  0000 R CNN
F 2 "" H 9350 5550 50  0001 C CNN
F 3 "~" H 9350 5550 50  0001 C CNN
	1    9350 5550
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J16
U 1 1 5F291474
P 8300 3000
F 0 "J16" H 8350 3250 50  0000 C CNN
F 1 "I2C Address" H 8350 2800 50  0000 C CNN
F 2 "" H 8300 3000 50  0001 C CNN
F 3 "~" H 8300 3000 50  0001 C CNN
	1    8300 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 3100 8100 3000
Wire Wire Line
	8100 3000 8100 2900
Connection ~ 8100 3000
$Comp
L power:GND #PWR08
U 1 1 5F2DDAC5
P 9300 3700
F 0 "#PWR08" H 9300 3450 50  0001 C CNN
F 1 "GND" H 9400 3600 50  0000 C CNN
F 2 "" H 9300 3700 50  0001 C CNN
F 3 "" H 9300 3700 50  0001 C CNN
	1    9300 3700
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR07
U 1 1 5F2EA7C7
P 9800 2250
F 0 "#PWR07" H 9800 2100 50  0001 C CNN
F 1 "VDD" H 9700 2300 50  0000 C CNN
F 2 "" H 9800 2250 50  0001 C CNN
F 3 "" H 9800 2250 50  0001 C CNN
	1    9800 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 2250 9800 2300
Text GLabel 9200 2600 0    47   Input ~ 0
SCL
Text GLabel 9200 2700 0    47   Input ~ 0
SDA
Wire Wire Line
	9200 2600 9300 2600
Wire Wire Line
	9200 2700 9300 2700
Text GLabel 10200 6250 0    47   Input ~ 0
SCL
Text GLabel 10200 6350 0    47   Input ~ 0
SDA
Wire Wire Line
	10200 6250 10300 6250
Wire Wire Line
	10200 6350 10300 6350
$Comp
L Connector_Generic:Conn_01x12 J17
U 1 1 5F39CC30
P 10700 2800
F 0 "J17" H 10600 2050 50  0000 L CNN
F 1 "General Purpose I/O" V 10850 2400 50  0000 L CNN
F 2 "" H 10700 2800 50  0001 C CNN
F 3 "~" H 10700 2800 50  0001 C CNN
	1    10700 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4700 6700 4700
$Comp
L power:VDD #PWR06
U 1 1 5F4788AE
P 8100 2850
F 0 "#PWR06" H 8100 2700 50  0001 C CNN
F 1 "VDD" H 8200 2950 50  0000 C CNN
F 2 "" H 8100 2850 50  0001 C CNN
F 3 "" H 8100 2850 50  0001 C CNN
	1    8100 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 2850 8100 2900
Connection ~ 8100 2900
Text Notes 8050 2100 0    98   ~ 20
Spare GPI/Os
Wire Notes Line
	11000 6500 9750 6500
Text Notes 9800 5800 0    98   ~ 20
Spare I2C
Wire Notes Line
	11000 5600 9750 5600
Wire Notes Line
	9750 5600 9750 6500
Wire Notes Line
	11000 5600 11000 6500
$Comp
L power:GND #PWR05
U 1 1 5F555159
P 1350 6050
F 0 "#PWR05" H 1350 5800 50  0001 C CNN
F 1 "GND" V 1355 5922 50  0000 R CNN
F 2 "" H 1350 6050 50  0001 C CNN
F 3 "" H 1350 6050 50  0001 C CNN
	1    1350 6050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1150 6050 1350 6050
Wire Wire Line
	1200 5500 1400 5500
Text GLabel 1500 5400 2    47   Input ~ 0
VCC-5V
Text Notes 6600 750  0    98   ~ 20
VIN Terminal
Text Notes 9150 2100 0    47   Italic 9
(Optional)
Wire Wire Line
	1150 5950 1450 5950
Wire Wire Line
	1500 5400 1200 5400
$Comp
L power:GND #PWR011
U 1 1 5F7AD83D
P 1300 7450
F 0 "#PWR011" H 1300 7200 50  0001 C CNN
F 1 "GND" V 1305 7322 50  0000 R CNN
F 2 "" H 1300 7450 50  0001 C CNN
F 3 "" H 1300 7450 50  0001 C CNN
	1    1300 7450
	0    -1   1    0   
$EndComp
$Comp
L power:VDD #PWR012
U 1 1 5F7C9AD3
P 1300 6550
F 0 "#PWR012" H 1300 6400 50  0001 C CNN
F 1 "VDD" V 1318 6678 50  0000 L CNN
F 2 "" H 1300 6550 50  0001 C CNN
F 3 "" H 1300 6550 50  0001 C CNN
	1    1300 6550
	0    1    -1   0   
$EndComp
Wire Wire Line
	1200 7250 1300 7250
Wire Wire Line
	1200 6550 1300 6550
Wire Notes Line
	750  4750 2450 4750
Wire Notes Line
	2450 4750 2450 6200
Wire Notes Line
	2450 6200 750  6200
Wire Notes Line
	1850 1500 750  1500
Wire Notes Line
	4800 1850 6200 1850
Wire Notes Line
	4250 600  4250 1600
Wire Notes Line
	7750 1900 7750 2600
Wire Wire Line
	7850 5850 7950 5850
Wire Wire Line
	7950 5950 7850 5950
Wire Wire Line
	7850 6050 7950 6050
Wire Wire Line
	5550 5850 5450 5850
Wire Wire Line
	5450 5950 5550 5950
Wire Wire Line
	5550 6050 5450 6050
$Comp
L Switch:SW_Push SW2
U 1 1 5FA4F2EE
P 2650 4300
F 0 "SW2" H 2650 4500 50  0000 C CNN
F 1 "RESET" H 2650 4200 50  0000 C CNN
F 2 "" H 2650 4500 50  0001 C CNN
F 3 "~" H 2650 4500 50  0001 C CNN
	1    2650 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5FA6E79D
P 2400 4300
F 0 "#PWR013" H 2400 4050 50  0001 C CNN
F 1 "GND" V 2500 4300 50  0000 R CNN
F 2 "" H 2400 4300 50  0001 C CNN
F 3 "" H 2400 4300 50  0001 C CNN
	1    2400 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	2650 1400 2750 1400
Connection ~ 2750 1400
Wire Wire Line
	2650 1200 2700 1200
Connection ~ 2700 1200
Wire Notes Line
	2100 1600 2100 600 
Wire Notes Line
	4250 1600 2100 1600
Text Notes 4850 3000 0    47   Italic 0
Note: This pinout may not reflect your SD-Card Reader pinout. \nMake sure that all signals (MISO/MOSI/SCK/CS/VDD/GND) are\nconnected to the according pins labeled on your SD-Card Reader PCB!
Text Label 6550 5450 0    47   Italic 0
1B
Text Label 6550 5550 0    47   Italic 0
1A
Text Label 6550 5650 0    47   Italic 0
2A
Text Label 6550 5750 0    47   Italic 0
2B
Text Label 8950 5450 0    47   Italic 0
1B
Text Label 8950 5550 0    47   Italic 0
1A
Text Label 8950 5650 0    47   Italic 0
2A
Text Label 8950 5750 0    47   Italic 0
2B
Wire Notes Line
	750  650  1850 650 
Wire Notes Line
	1850 650  1850 1500
Wire Notes Line
	2250 3350 750  3350
Wire Notes Line
	750  1900 2250 1900
Wire Notes Line
	2250 1900 2250 3350
Connection ~ 7900 4850
Wire Notes Line
	6200 2600 6200 1900
Text GLabel 4250 4250 2    47   Input ~ 0
SCL
Text GLabel 4250 4150 2    47   Input ~ 0
SDA
Wire Wire Line
	4150 4150 4250 4150
Wire Wire Line
	4250 4250 4150 4250
Text GLabel 4250 4350 2    47   Input Italic 0
MOSI
Wire Wire Line
	4150 4350 4250 4350
Text GLabel 4250 4050 2    47   Input Italic 0
MISO
Wire Wire Line
	4150 4050 4250 4050
Text GLabel 4250 3950 2    47   Input Italic 0
SCK
Wire Wire Line
	4150 3950 4250 3950
Text GLabel 5550 3550 0    47   Input Italic 0
MISO
Text GLabel 5550 3950 0    47   Input Italic 0
MOSI
Text GLabel 5550 3750 0    47   Input Italic 0
SCK
Wire Wire Line
	5550 3550 5750 3550
Wire Wire Line
	5550 3750 5750 3750
Wire Wire Line
	5550 3950 5750 3950
Text GLabel 10200 5150 0    47   Input Italic 0
MISO
Text GLabel 10200 5250 0    47   Input Italic 0
MOSI
Text GLabel 10200 5350 0    47   Input Italic 0
SCK
$Comp
L Connector:Conn_01x07_Male J18
U 1 1 5FF2BBD8
P 10500 5150
F 0 "J18" H 10450 5100 50  0000 R CNN
F 1 "To SPI Device" V 10200 5400 50  0000 R CNN
F 2 "" H 10500 5150 50  0001 C CNN
F 3 "~" H 10500 5150 50  0001 C CNN
	1    10500 5150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10200 4950 10300 4950
Wire Wire Line
	10200 5050 10300 5050
Wire Wire Line
	10200 5150 10300 5150
$Comp
L power:VDD #PWR015
U 1 1 5FF5C808
P 10200 4950
F 0 "#PWR015" H 10200 4800 50  0001 C CNN
F 1 "VDD" V 10200 5150 50  0000 C CNN
F 2 "" H 10200 4950 50  0001 C CNN
F 3 "" H 10200 4950 50  0001 C CNN
	1    10200 4950
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5FF5C80E
P 10200 5050
F 0 "#PWR016" H 10200 4800 50  0001 C CNN
F 1 "GND" V 10200 4850 50  0000 C CNN
F 2 "" H 10200 5050 50  0001 C CNN
F 3 "" H 10200 5050 50  0001 C CNN
	1    10200 5050
	0    1    1    0   
$EndComp
Text GLabel 10200 5450 0    47   Input Italic 0
CS-EX
Wire Wire Line
	10200 5250 10300 5250
Text Notes 9800 4700 0    98   ~ 20
Spare SPI
Text GLabel 5050 1650 2    47   Input ~ 0
SDA
Text GLabel 5050 1750 2    47   Input ~ 0
SCL
Wire Wire Line
	4900 1450 4900 1650
Wire Wire Line
	4900 1650 5050 1650
Wire Wire Line
	4850 1750 5050 1750
Wire Wire Line
	4850 1350 4850 1750
Text GLabel 6700 2100 0    47   Input Italic 0
RXD2
Text GLabel 6700 2000 0    47   Input Italic 0
TXD2
Text GLabel 4250 3750 2    47   Input Italic 0
RXD2
Text GLabel 4250 3850 2    47   Input Italic 0
TXD2
Text GLabel 4250 2850 2    47   Input Italic 0
TDX0
Text GLabel 4250 3050 2    47   Input Italic 0
RXD0
Text GLabel 5150 2000 0    47   Input Italic 0
TXD0
Text GLabel 5150 2100 0    47   Input Italic 0
RXD0
Wire Wire Line
	6700 2000 7150 2000
Wire Wire Line
	6700 2100 7150 2100
Wire Wire Line
	4150 3750 4250 3750
Wire Wire Line
	4250 3850 4150 3850
Wire Wire Line
	5150 2000 5650 2000
Wire Wire Line
	5650 2100 5150 2100
Wire Wire Line
	6800 2300 7150 2300
Wire Wire Line
	3800 1200 4400 1200
Wire Wire Line
	4150 3050 4250 3050
Wire Wire Line
	4150 2850 4250 2850
Wire Wire Line
	4650 1800 4650 2950
Wire Wire Line
	1350 1300 1950 1300
Wire Wire Line
	3700 5350 3350 5350
Wire Wire Line
	2900 3600 2950 3600
Wire Wire Line
	2750 3200 2950 3200
Wire Wire Line
	2700 3300 2950 3300
Wire Wire Line
	2750 1400 2750 3200
Wire Wire Line
	2700 1200 2700 3300
$Comp
L power:VDD #PWR014
U 1 1 602CD788
P 3550 2500
F 0 "#PWR014" H 3550 2350 50  0001 C CNN
F 1 "VDD" H 3650 2600 50  0000 C CNN
F 2 "" H 3550 2500 50  0001 C CNN
F 3 "" H 3550 2500 50  0001 C CNN
	1    3550 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 2500 3550 2550
Wire Wire Line
	2950 4750 2700 4750
Wire Wire Line
	2950 4650 2600 4650
Wire Wire Line
	1200 7350 2700 7350
Wire Wire Line
	1200 6650 2600 6650
$Comp
L esp32-wroom-32:NodeMCU-ESP32-devkit U1
U 1 1 60395DE8
P 3550 3950
F 0 "U1" H 3100 5350 50  0000 C CNN
F 1 "NodeMCU-ESP32-devkit" V 3450 3700 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 3550 2450 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 3250 4000 50  0001 C CNN
	1    3550 3950
	1    0    0    -1  
$EndComp
Connection ~ 3350 5350
Wire Wire Line
	4150 2750 4400 2750
Wire Wire Line
	4400 2750 4400 1200
Wire Notes Line
	2450 6300 2450 7750
Wire Wire Line
	5650 3650 5750 3650
Wire Wire Line
	5650 3850 5750 3850
Text GLabel 5550 3450 0    47   Input Italic 0
CS-SD
Wire Wire Line
	5550 3450 5750 3450
Text GLabel 4250 3250 2    47   Input Italic 0
CS-SD
Wire Wire Line
	4150 3250 4250 3250
Wire Notes Line
	4800 1900 4800 2600
Wire Notes Line
	4800 1900 6200 1900
Wire Notes Line
	4800 2600 6200 2600
Wire Notes Line
	6300 1900 6300 2600
Wire Notes Line
	6300 1900 7750 1900
Wire Notes Line
	6300 2600 7750 2600
Text GLabel 8400 4700 0    47   Input ~ 0
VIN
Wire Wire Line
	8400 4700 8550 4700
Connection ~ 8550 4700
Wire Wire Line
	8550 4700 8700 4700
Wire Wire Line
	8550 4700 8550 4850
Wire Wire Line
	9000 4700 9050 4700
Wire Wire Line
	10300 3100 10500 3100
Wire Wire Line
	10300 3000 10500 3000
Wire Wire Line
	10300 2900 10500 2900
Wire Wire Line
	10300 2800 10500 2800
Wire Wire Line
	10300 2700 10500 2700
Wire Wire Line
	10300 2600 10500 2600
$Comp
L power:VDD #PWR018
U 1 1 60817023
P 10450 2400
F 0 "#PWR018" H 10450 2250 50  0001 C CNN
F 1 "VDD" V 10450 2600 50  0000 C CNN
F 2 "" H 10450 2400 50  0001 C CNN
F 3 "" H 10450 2400 50  0001 C CNN
	1    10450 2400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 6082613D
P 10450 2500
F 0 "#PWR019" H 10450 2250 50  0001 C CNN
F 1 "GND" V 10450 2300 50  0000 C CNN
F 2 "" H 10450 2500 50  0001 C CNN
F 3 "" H 10450 2500 50  0001 C CNN
	1    10450 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	10500 2500 10450 2500
Wire Wire Line
	10450 2400 10500 2400
Text GLabel 10450 2300 0    47   Input Italic 0
VCC-5V
Wire Wire Line
	10450 2300 10500 2300
Text GLabel 1300 4050 2    47   Input Italic 0
FILAMENT-RUNOUT
Wire Wire Line
	1300 4050 1200 4050
Text GLabel 3550 7350 0    47   Input Italic 0
FILAMENT-RUNOUT
$Comp
L Connector:Conn_01x05_Male J19
U 1 1 60925480
P 6500 7350
F 0 "J19" H 6650 7650 50  0000 R CNN
F 1 "To SFB" V 6400 7500 50  0000 R CNN
F 2 "" H 6500 7350 50  0001 C CNN
F 3 "~" H 6500 7350 50  0001 C CNN
	1    6500 7350
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 609484DC
P 6250 7450
F 0 "#PWR020" H 6250 7200 50  0001 C CNN
F 1 "GND" V 6250 7250 50  0000 C CNN
F 2 "" H 6250 7450 50  0001 C CNN
F 3 "" H 6250 7450 50  0001 C CNN
	1    6250 7450
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 7450 6250 7450
Text GLabel 6250 7550 0    47   Input Italic 0
VCC-5V
Text Label 6150 7150 0    47   Italic 0
TXD
Wire Wire Line
	6250 7550 6300 7550
Wire Notes Line
	8850 1700 11000 1700
Wire Notes Line
	8850 600  8850 1700
Wire Notes Line
	11000 600  11000 1700
Wire Notes Line
	9750 4500 11000 4500
Wire Notes Line
	9750 5550 11000 5550
Text Label 6150 7250 0    47   Italic 0
RXD
Wire Notes Line
	8000 1900 11000 1900
Wire Wire Line
	6750 1100 7150 1100
Wire Wire Line
	7050 1050 7050 1500
Wire Wire Line
	7150 1050 7150 1100
Connection ~ 7150 1100
Wire Notes Line
	6200 600  6200 1850
Wire Notes Line
	4800 600  4800 1850
Wire Wire Line
	1950 1800 1950 1300
Connection ~ 2150 2550
Wire Wire Line
	2150 2550 2100 2550
Wire Notes Line
	7750 1600 7750 600 
$Comp
L Jumper:Jumper_3_Bridged12 JP1
U 1 1 60DFF85C
P 3350 2150
F 0 "JP1" H 3250 2050 50  0000 C CNN
F 1 "Fan/CS-EX switch" H 3350 1950 50  0000 C CNN
F 2 "" H 3350 2150 50  0001 C CNN
F 3 "~" H 3350 2150 50  0001 C CNN
	1    3350 2150
	1    0    0    1   
$EndComp
Wire Wire Line
	3100 2150 2150 2150
Text GLabel 3750 2150 2    47   Input Italic 0
CS-EX
Wire Wire Line
	3600 2150 3750 2150
Text Label 5500 2350 0    47   Italic 0
NC
Text GLabel 4250 3350 2    47   Input Italic 0
DIR-F
Wire Wire Line
	4150 3350 4250 3350
Text GLabel 7900 5650 0    47   Input Italic 0
DIR-F
Wire Wire Line
	7900 5650 7950 5650
Text GLabel 4250 3550 2    47   Input Italic 0
STEP-F
Wire Wire Line
	4150 3550 4250 3550
Text GLabel 7900 5550 0    47   Input Italic 0
STEP-F
Text GLabel 7900 5450 0    47   Input Italic 0
ENA-F
Wire Wire Line
	7900 5450 7950 5450
Wire Wire Line
	7900 5550 7950 5550
Text GLabel 4250 4850 2    47   Input Italic 0
ENA-F
Wire Wire Line
	4150 4850 4250 4850
Text GLabel 4250 4450 2    47   Input Italic 0
ENA-S
Text GLabel 4250 4550 2    47   Input Italic 0
DIR-S
Text GLabel 4250 4650 2    47   Input Italic 0
STEP-S
Text GLabel 5450 5450 0    47   Input Italic 0
ENA-S
Text GLabel 5450 5550 0    47   Input Italic 0
STEP-S
Text GLabel 5450 5650 0    47   Input Italic 0
DIR-S
Wire Wire Line
	5450 5450 5550 5450
Wire Wire Line
	5550 5550 5450 5550
Wire Wire Line
	5450 5650 5550 5650
Wire Wire Line
	4150 4450 4250 4450
Wire Wire Line
	4250 4550 4150 4550
Wire Wire Line
	4150 4650 4250 4650
Wire Wire Line
	7250 5800 7250 5850
Wire Wire Line
	7250 6050 7350 6050
Wire Wire Line
	7350 5950 7250 5950
Connection ~ 7250 5950
Wire Wire Line
	7250 5950 7250 6050
Wire Wire Line
	7250 5850 7350 5850
Connection ~ 7250 5850
Wire Wire Line
	7250 5850 7250 5950
Wire Wire Line
	4900 5800 4900 5850
Wire Wire Line
	4950 6050 4900 6050
Wire Wire Line
	4900 5950 4950 5950
Connection ~ 4900 5950
Wire Wire Line
	4900 5950 4900 6050
Wire Wire Line
	4950 5850 4900 5850
Connection ~ 4900 5850
Wire Wire Line
	4900 5850 4900 5950
Text GLabel 4250 3150 2    47   Input Italic 0
SERVO1
Text GLabel 4250 3650 2    47   Input Italic 0
SERVO2
Text GLabel 2000 5300 2    47   Input Italic 0
SERVO2
Text GLabel 2000 5850 2    47   Input Italic 0
SERVO1
Wire Wire Line
	1200 5300 2000 5300
Wire Wire Line
	1150 5850 2000 5850
Wire Wire Line
	4250 3650 4150 3650
Wire Wire Line
	4150 3150 4250 3150
Wire Wire Line
	3350 2000 4700 2000
Wire Wire Line
	4150 3450 4700 3450
Wire Wire Line
	4150 2950 4650 2950
Wire Wire Line
	1950 1800 4650 1800
Wire Notes Line
	7750 2700 7750 4450
Wire Notes Line
	7750 4450 4800 4450
Wire Notes Line
	4800 2700 7750 2700
Wire Wire Line
	6450 5450 6700 5450
Wire Wire Line
	6450 5550 6700 5550
Wire Wire Line
	6700 5650 6450 5650
Wire Wire Line
	6450 5750 6700 5750
Wire Wire Line
	8850 5450 9150 5450
Wire Wire Line
	9150 5550 8850 5550
Wire Wire Line
	8850 5650 9150 5650
Wire Wire Line
	9150 5750 8850 5750
Wire Notes Line
	4800 2700 4800 4450
Wire Notes Line
	9600 4500 9600 6500
Wire Notes Line
	6850 6500 9600 6500
Wire Wire Line
	6000 4700 6150 4700
Wire Wire Line
	6150 4700 6150 4850
Connection ~ 6150 4700
Wire Wire Line
	6150 4700 6350 4700
Wire Notes Line
	4800 6700 6850 6700
Wire Notes Line
	4800 4500 9600 4500
Wire Wire Line
	7150 1100 9300 1100
Wire Wire Line
	7050 1500 9300 1500
Wire Wire Line
	8550 6350 8350 6350
Connection ~ 8350 6350
Wire Wire Line
	5950 6350 6150 6350
Connection ~ 5950 6350
Wire Wire Line
	2150 2150 2150 2550
Wire Wire Line
	4700 2000 4700 3450
Wire Wire Line
	2400 4300 2450 4300
Wire Wire Line
	2700 4750 2700 7350
Wire Wire Line
	2600 4650 2600 6650
Wire Notes Line
	2450 6300 750  6300
Wire Notes Line
	2450 7750 700  7750
Wire Wire Line
	2700 4750 2700 4550
Connection ~ 2700 4750
Wire Wire Line
	1350 4650 1350 4250
Connection ~ 2600 4650
Wire Wire Line
	2850 4300 2950 4300
Text Label 1350 4650 0    47   Italic 0
Selector-Sig.
Text Notes 750  4650 0    98   ~ 20
Duet3D\nonly
Wire Notes Line
	750  4700 2150 4700
Wire Notes Line
	2150 4700 2150 3800
Wire Notes Line
	2150 3800 750  3800
Wire Wire Line
	1350 4250 1200 4250
Wire Wire Line
	1350 4650 2600 4650
Wire Notes Line
	11000 4500 11000 5550
Wire Notes Line
	9750 4500 9750 5550
Text GLabel 10200 4850 0    47   Input Italic 0
VCC-5V
Text GLabel 10200 5950 0    47   Input Italic 0
VCC-5V
Wire Wire Line
	10200 5950 10300 5950
Wire Wire Line
	10200 4850 10300 4850
Wire Wire Line
	10200 5350 10300 5350
Wire Wire Line
	10200 5450 10300 5450
Wire Wire Line
	10200 6050 10300 6050
Wire Wire Line
	10300 6150 10200 6150
Wire Wire Line
	10300 3200 10500 3200
Wire Wire Line
	10300 3300 10500 3300
Wire Notes Line
	11000 1900 11000 3850
Wire Notes Line
	8000 1900 8000 3850
Wire Notes Line
	8000 3850 11000 3850
Wire Wire Line
	9800 3700 9300 3700
Connection ~ 9300 3700
Wire Wire Line
	9300 3700 9000 3700
Wire Wire Line
	9000 3600 9000 3700
$Comp
L Device:R_Network03 RN1
U 1 1 608C3648
P 8900 3400
F 0 "RN1" H 8900 3700 50  0000 L CNN
F 1 "10K" H 8900 3600 50  0000 L CNN
F 2 "Resistor_THT:R_Array_SIP4" V 9175 3400 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 8900 3400 50  0001 C CNN
	1    8900 3400
	-1   0    0    1   
$EndComp
Wire Wire Line
	8600 2900 8800 2900
Wire Wire Line
	8600 3000 8900 3000
Wire Wire Line
	8600 3100 9000 3100
Wire Wire Line
	9000 3200 9000 3100
Connection ~ 9000 3100
Wire Wire Line
	9000 3100 9300 3100
Wire Wire Line
	8900 3200 8900 3000
Connection ~ 8900 3000
Wire Wire Line
	8900 3000 9300 3000
Wire Wire Line
	8800 3200 8800 2900
Connection ~ 8800 2900
Wire Wire Line
	8800 2900 9300 2900
$Comp
L Logic_LevelTranslator:SN74AUP1T34DCK U5
U 1 1 61B1AAA5
P 4300 7350
F 0 "U5" H 4000 7600 50  0000 L CNN
F 1 "SN74AUP1T34DCK" H 3600 7100 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 4300 6650 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74aup1t34.pdf" H 4300 6750 50  0001 C CNN
	1    4300 7350
	-1   0    0    -1  
$EndComp
$Comp
L Logic_LevelTranslator:SN74AUP1T34DCK U4
U 1 1 61B31365
P 3650 6900
F 0 "U4" H 3350 7150 50  0000 L CNN
F 1 "SN74AUP1T34DCK" H 3300 7350 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 3650 6200 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74aup1t34.pdf" H 3650 6300 50  0001 C CNN
	1    3650 6900
	1    0    0    -1  
$EndComp
Text GLabel 4400 6650 1    47   Input Italic 0
VCC-5V
Wire Wire Line
	3900 7350 3550 7350
Wire Wire Line
	4400 6650 4400 6750
Wire Wire Line
	4100 6600 4100 6750
Wire Wire Line
	4100 6750 4400 6750
Connection ~ 4400 6750
Wire Wire Line
	4400 6750 4400 7050
$Comp
L power:VDD #PWR017
U 1 1 61B9A1E5
P 4200 6450
F 0 "#PWR017" H 4200 6300 50  0001 C CNN
F 1 "VDD" H 4200 6600 50  0000 C CNN
F 2 "" H 4200 6450 50  0001 C CNN
F 3 "" H 4200 6450 50  0001 C CNN
	1    4200 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 6450 4200 6500
Connection ~ 4200 6500
Wire Wire Line
	4200 6500 4200 7050
Wire Wire Line
	3750 6600 4100 6600
Wire Wire Line
	3550 6600 3550 6500
Wire Wire Line
	3550 6500 4200 6500
Text GLabel 3100 6900 0    47   Input Italic 0
RXD0
Wire Wire Line
	3100 6900 3250 6900
$Comp
L power:GND #PWR?
U 1 1 61C0BB59
P 3650 7650
F 0 "#PWR?" H 3650 7400 50  0001 C CNN
F 1 "GND" H 3500 7600 50  0000 C CNN
F 2 "" H 3650 7650 50  0001 C CNN
F 3 "" H 3650 7650 50  0001 C CNN
	1    3650 7650
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 6900 4800 6900
Wire Wire Line
	4700 7350 6300 7350
Text Notes 2850 6400 0    98   ~ 20
SFB
Wire Notes Line
	6850 6500 6850 6700
Wire Notes Line
	4800 6700 4800 4500
Wire Notes Line
	2800 6200 2800 7750
Wire Wire Line
	4800 7150 4800 6900
Wire Wire Line
	4800 7150 6300 7150
Text Notes 4900 6950 0    47   Italic 0
Note: Level shifters used to convert signals\nfrom 3.3V to 5V and vice versa
Wire Notes Line
	6850 7750 2800 7750
Text Label 6050 7350 0    47   Italic 0
Signal
Wire Notes Line
	2800 6200 4750 6200
Text Notes 3150 6400 0    47   Italic 9
(Optional)
Wire Wire Line
	10500 3400 10350 3400
Wire Wire Line
	10350 3400 10350 3700
Wire Wire Line
	10350 3700 9800 3700
Connection ~ 9800 3700
Text Notes 2900 5700 0    98   ~ 20
NodeMCU Module
Text Label 1450 4550 0    47   Italic 0
Feeder-Sig.
Wire Wire Line
	1450 4150 1200 4150
Wire Wire Line
	1450 4150 1450 4550
Wire Wire Line
	1450 4550 2700 4550
Text Notes 5050 7700 0    31   Italic 0
This connection is used optionally\nto synchronize SFB to the current\ntool. Every GCode sent to the \nSMuFF gets passed through to \nthe SFB. Baudrate on SFB must \nmatch the Serial1 baudrate.
Wire Wire Line
	3650 7200 3650 7650
Connection ~ 3650 7650
Wire Wire Line
	3650 7650 4300 7650
Wire Notes Line
	6300 600  6300 1600
Wire Notes Line
	6300 1600 7750 1600
Wire Notes Line
	6850 6750 4750 6750
Wire Notes Line
	6850 6750 6850 7750
Wire Notes Line
	4750 6200 4750 6750
$EndSCHEMATC
