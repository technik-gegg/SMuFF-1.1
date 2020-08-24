EESchema Schematic File Version 4
LIBS:SKR mini LCD board-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "SKR Mini V1.1 LCD board"
Date "2020-08-15"
Rev "2"
Comp "Technik Gegg"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L SKR-mini-LCD-board-rescue:SSD1306-SSD1306_OLED-0.91-128x32 U1
U 1 1 5D888A8E
P 3150 3550
F 0 "U1" V 1563 3867 60  0000 C CNN
F 1 "SSD1306 0.96\" OLED" V 1669 3867 60  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3150 3550 60  0001 C CNN
F 3 "" H 3150 3550 60  0001 C CNN
	1    3150 3550
	0    -1   1    0   
$EndComp
$Comp
L Device:Rotary_Encoder_Switch ENC1
U 1 1 5D88B3FF
P 5000 5500
F 0 "ENC1" H 5000 5867 50  0000 C CNN
F 1 "Rotary Encoder" H 5000 5776 50  0000 C CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm_CircularMountingHoles" H 4850 5660 50  0001 C CNN
F 3 "~" H 5000 5760 50  0001 C CNN
	1    5000 5500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J1
U 1 1 5D88D2F1
P 5850 4400
F 0 "J1" H 5900 4817 50  0000 C CNN
F 1 "Connector 5x2" H 5900 4726 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x05_P2.54mm_Vertical" H 5850 4400 50  0001 C CNN
F 3 "~" H 5850 4400 50  0001 C CNN
	1    5850 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5D8937B1
P 3850 5400
F 0 "R1" V 4057 5400 50  0000 C CNN
F 1 "10K" V 3966 5400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3780 5400 50  0001 C CNN
F 3 "~" H 3850 5400 50  0001 C CNN
	1    3850 5400
	0    -1   -1   0   
$EndComp
$Comp
L Device:Buzzer BZ1
U 1 1 5D893D6F
P 7200 4550
F 0 "BZ1" H 7400 4600 50  0000 L CNN
F 1 "Buzzer" H 7400 4500 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_12x9.5RM7.6" V 7175 4650 50  0001 C CNN
F 3 "~" V 7175 4650 50  0001 C CNN
	1    7200 4550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DIP_x01 SW1
U 1 1 5D894E2A
P 5050 1600
F 0 "SW1" H 5050 1867 50  0000 C CNN
F 1 "Reset" H 5050 1776 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H7.3mm" H 5050 1600 50  0001 C CNN
F 3 "~" H 5050 1600 50  0001 C CNN
	1    5050 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 5D89576B
P 3400 5400
F 0 "#PWR0101" H 3400 5250 50  0001 C CNN
F 1 "+5V" V 3415 5528 50  0000 L CNN
F 2 "" H 3400 5400 50  0001 C CNN
F 3 "" H 3400 5400 50  0001 C CNN
	1    3400 5400
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0102
U 1 1 5D896CC4
P 5450 3800
F 0 "#PWR0102" H 5450 3650 50  0001 C CNN
F 1 "+5V" H 5465 3973 50  0000 C CNN
F 2 "" H 5450 3800 50  0001 C CNN
F 3 "" H 5450 3800 50  0001 C CNN
	1    5450 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5D89755D
P 3500 2100
F 0 "#PWR0103" H 3500 1950 50  0001 C CNN
F 1 "+5V" H 3515 2273 50  0000 C CNN
F 2 "" H 3500 2100 50  0001 C CNN
F 3 "" H 3500 2100 50  0001 C CNN
	1    3500 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5D897F1D
P 4600 1850
F 0 "#PWR0104" H 4600 1600 50  0001 C CNN
F 1 "GND" H 4605 1677 50  0000 C CNN
F 2 "" H 4600 1850 50  0001 C CNN
F 3 "" H 4600 1850 50  0001 C CNN
	1    4600 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5D898A40
P 3550 2500
F 0 "#PWR0105" H 3550 2250 50  0001 C CNN
F 1 "GND" H 3555 2327 50  0000 C CNN
F 2 "" H 3550 2500 50  0001 C CNN
F 3 "" H 3550 2500 50  0001 C CNN
	1    3550 2500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5D899BAF
P 5650 5700
F 0 "#PWR0106" H 5650 5450 50  0001 C CNN
F 1 "GND" H 5655 5527 50  0000 C CNN
F 2 "" H 5650 5700 50  0001 C CNN
F 3 "" H 5650 5700 50  0001 C CNN
	1    5650 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5D89A417
P 7100 5350
F 0 "#PWR0107" H 7100 5100 50  0001 C CNN
F 1 "GND" H 7105 5177 50  0000 C CNN
F 2 "" H 7100 5350 50  0001 C CNN
F 3 "" H 7100 5350 50  0001 C CNN
	1    7100 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4200 5450 4200
Wire Wire Line
	5450 4200 5450 3800
Wire Wire Line
	3150 2300 3500 2300
Wire Wire Line
	3500 2300 3500 2100
Wire Wire Line
	3150 2500 3550 2500
Wire Wire Line
	4750 1600 4600 1600
Wire Wire Line
	4600 1600 4600 1850
Wire Wire Line
	7100 5250 7100 5350
$Comp
L power:GND #PWR0108
U 1 1 5D89B98C
P 6250 3800
F 0 "#PWR0108" H 6250 3550 50  0001 C CNN
F 1 "GND" H 6255 3627 50  0000 C CNN
F 2 "" H 6250 3800 50  0001 C CNN
F 3 "" H 6250 3800 50  0001 C CNN
	1    6250 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	6150 4200 6250 4200
Wire Wire Line
	4700 5500 4400 5500
Wire Wire Line
	4700 5400 4300 5400
Wire Wire Line
	5300 5600 5650 5600
Wire Wire Line
	5650 5600 5650 5700
Wire Wire Line
	5650 4600 5600 4600
Wire Wire Line
	5600 4600 5600 5400
Wire Wire Line
	5600 5400 5300 5400
Wire Wire Line
	5650 4300 4300 4300
Wire Wire Line
	4300 4300 4300 5400
Connection ~ 4300 5400
Wire Wire Line
	4300 5400 4000 5400
Wire Wire Line
	5650 4400 4500 4400
Wire Wire Line
	6400 4500 6150 4500
$Comp
L Device:R R2
U 1 1 5D892B67
P 3850 5950
F 0 "R2" V 4057 5950 50  0000 C CNN
F 1 "10K" V 3966 5950 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3780 5950 50  0001 C CNN
F 3 "~" H 3850 5950 50  0001 C CNN
	1    3850 5950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 5950 4300 5950
Wire Wire Line
	4300 5600 4300 5950
$Comp
L power:GND #PWR0109
U 1 1 5D8A9A64
P 4400 5500
F 0 "#PWR0109" H 4400 5250 50  0001 C CNN
F 1 "GND" V 4405 5372 50  0000 R CNN
F 2 "" H 4400 5500 50  0001 C CNN
F 3 "" H 4400 5500 50  0001 C CNN
	1    4400 5500
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Odd_Even #EXT2
U 1 1 5D8AB540
P 6250 1700
F 0 "#EXT2" H 6300 2117 50  0000 C CNN
F 1 "Connector on Controller" H 6300 2026 50  0000 C CNN
F 2 "" H 6250 1700 50  0001 C CNN
F 3 "~" H 6250 1700 50  0001 C CNN
	1    6250 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1600 6050 1600
Text Notes 5250 2200 0    50   ~ 0
Optional\n
Wire Notes Line width 20
	6950 2350 6950 1000
Wire Notes Line width 20
	6950 1000 4450 1000
Wire Notes Line width 20
	4450 1000 4450 2350
Wire Notes Line width 20
	6950 2350 4450 2350
Wire Wire Line
	6400 2700 6400 4500
Wire Wire Line
	3150 2700 6400 2700
Wire Wire Line
	4500 4400 4500 2900
Wire Wire Line
	4500 2900 3150 2900
Wire Wire Line
	3550 5400 3700 5400
Wire Wire Line
	3550 5950 3700 5950
Wire Wire Line
	4300 5600 4700 5600
Connection ~ 4300 5950
Wire Wire Line
	4300 5950 6250 5950
Wire Wire Line
	6250 3800 6250 4200
Wire Wire Line
	3400 5400 3550 5400
Connection ~ 3550 5400
Wire Wire Line
	3550 5400 3550 5950
$Comp
L power:+5V #PWR?
U 1 1 5F377DF5
P 7100 4300
F 0 "#PWR?" H 7100 4150 50  0001 C CNN
F 1 "+5V" H 7115 4473 50  0000 C CNN
F 2 "" H 7100 4300 50  0001 C CNN
F 3 "" H 7100 4300 50  0001 C CNN
	1    7100 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4300 7100 4450
$Comp
L Device:R R3
U 1 1 5F3793FA
P 6550 5050
F 0 "R3" V 6550 5050 50  0000 C CNN
F 1 "1K" V 6450 5050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6480 5050 50  0001 C CNN
F 3 "~" H 6550 5050 50  0001 C CNN
	1    6550 5050
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:BC547 Q1
U 1 1 5F37DDBD
P 7000 5050
F 0 "Q1" H 7191 5096 50  0000 L CNN
F 1 "BC547" H 7191 5005 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7200 4975 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 7000 5050 50  0001 L CNN
	1    7000 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4650 7100 4850
Wire Wire Line
	6700 5050 6800 5050
Wire Wire Line
	6150 4300 6250 4300
Wire Wire Line
	6250 4300 6250 5950
Wire Wire Line
	6150 4600 6400 4600
Wire Wire Line
	6400 4600 6400 5050
$EndSCHEMATC
