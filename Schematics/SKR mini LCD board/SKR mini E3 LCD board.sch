EESchema Schematic File Version 4
LIBS:SKR mini LCD board-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "SKR Mini E3 (DIP) LCD board"
Date "2020-08-15"
Rev "1"
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
P 7400 4100
F 0 "BZ1" H 7600 4150 50  0000 L CNN
F 1 "Buzzer" H 7600 4050 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_12x9.5RM7.6" V 7375 4200 50  0001 C CNN
F 3 "~" V 7375 4200 50  0001 C CNN
	1    7400 4100
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DIP_x01 SW1
U 1 1 5D894E2A
P 3750 4500
F 0 "SW1" H 3750 4767 50  0000 C CNN
F 1 "Reset" H 3750 4676 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H7.3mm" H 3750 4500 50  0001 C CNN
F 3 "~" H 3750 4500 50  0001 C CNN
	1    3750 4500
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
P 3300 4750
F 0 "#PWR0104" H 3300 4500 50  0001 C CNN
F 1 "GND" H 3305 4577 50  0000 C CNN
F 2 "" H 3300 4750 50  0001 C CNN
F 3 "" H 3300 4750 50  0001 C CNN
	1    3300 4750
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
P 7300 4900
F 0 "#PWR0107" H 7300 4650 50  0001 C CNN
F 1 "GND" H 7305 4727 50  0000 C CNN
F 2 "" H 7300 4900 50  0001 C CNN
F 3 "" H 7300 4900 50  0001 C CNN
	1    7300 4900
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
	3450 4500 3300 4500
Wire Wire Line
	3300 4500 3300 4750
Wire Wire Line
	7300 4800 7300 4900
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
	5300 5600 5650 5600
Wire Wire Line
	5650 5600 5650 5700
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
Wire Wire Line
	3150 2700 4700 2700
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
	6250 3800 6250 4200
Wire Wire Line
	3400 5400 3550 5400
Connection ~ 3550 5400
Wire Wire Line
	3550 5400 3550 5950
$Comp
L power:+5V #PWR?
U 1 1 5F377DF5
P 7300 3850
F 0 "#PWR?" H 7300 3700 50  0001 C CNN
F 1 "+5V" H 7315 4023 50  0000 C CNN
F 2 "" H 7300 3850 50  0001 C CNN
F 3 "" H 7300 3850 50  0001 C CNN
	1    7300 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 3850 7300 4000
$Comp
L Device:R R3
U 1 1 5F3793FA
P 6750 4600
F 0 "R3" V 6750 4600 50  0000 C CNN
F 1 "1K" V 6650 4600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6680 4600 50  0001 C CNN
F 3 "~" H 6750 4600 50  0001 C CNN
	1    6750 4600
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:BC547 Q1
U 1 1 5F37DDBD
P 7200 4600
F 0 "Q1" H 7391 4646 50  0000 L CNN
F 1 "BC547" H 7391 4555 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7400 4525 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 7200 4600 50  0001 L CNN
	1    7200 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 4200 7300 4400
Wire Wire Line
	6900 4600 7000 4600
Wire Wire Line
	6150 4300 6250 4300
Wire Wire Line
	6250 4300 6250 5400
Wire Wire Line
	5300 5400 6250 5400
Wire Wire Line
	4000 5400 4300 5400
Wire Wire Line
	5650 4300 4500 4300
Wire Wire Line
	4500 4300 4500 2900
Wire Wire Line
	4700 4600 4700 2700
Wire Wire Line
	4700 4600 5650 4600
Wire Wire Line
	5650 4500 4050 4500
Wire Wire Line
	6150 4400 6200 4400
Wire Wire Line
	6200 4400 6200 5000
Wire Wire Line
	6200 5000 4300 5000
Wire Wire Line
	4300 5000 4300 5400
Connection ~ 4300 5400
Wire Wire Line
	4300 5400 4700 5400
Wire Wire Line
	6300 4500 6300 5950
Wire Wire Line
	4300 5950 6300 5950
Wire Wire Line
	6300 4500 6150 4500
Wire Wire Line
	6150 4600 6600 4600
$EndSCHEMATC
