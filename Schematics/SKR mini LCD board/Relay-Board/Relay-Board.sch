EESchema Schematic File Version 4
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "SMuFF Mini Relay Board"
Date "2020-12-06"
Rev "1"
Comp "Technik Gegg"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Isolator:TLP785 U1
U 1 1 5FCC8653
P 3300 3650
F 0 "U1" H 3300 3975 50  0000 C CNN
F 1 "TLP521" H 3300 3884 50  0000 C CNN
F 2 "Package_DIP:SMDIP-4_W7.62mm" H 3100 3450 50  0001 L CIN
F 3 "https://toshiba.semicon-storage.com/info/docget.jsp?did=10569&prodName=TLP785" H 3300 3650 50  0001 L CNN
	1    3300 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5FCC9115
P 2900 3100
F 0 "R1" H 2970 3146 50  0000 L CNN
F 1 "390R" H 2970 3055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2830 3100 50  0001 C CNN
F 3 "~" H 2900 3100 50  0001 C CNN
	1    2900 3100
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC547 Q1
U 1 1 5FCCA429
P 4650 3750
F 0 "Q1" H 4841 3796 50  0000 L CNN
F 1 "BC547" H 4841 3705 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4850 3675 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 4650 3750 50  0001 L CNN
	1    4650 3750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC547 Q2
U 1 1 5FCCADE4
P 5150 4400
F 0 "Q2" H 5341 4446 50  0000 L CNN
F 1 "BC547" H 5341 4355 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5350 4325 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 5150 4400 50  0001 L CNN
	1    5150 4400
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D1
U 1 1 5FCCC4A8
P 4750 3000
F 0 "D1" V 4704 3079 50  0000 L CNN
F 1 "1N4148" V 4795 3079 50  0000 L CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 4750 2825 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 4750 3000 50  0001 C CNN
	1    4750 3000
	0    1    1    0   
$EndComp
$Comp
L Diode:1N4148 D2
U 1 1 5FCCDE35
P 6050 4200
F 0 "D2" H 6000 4300 50  0000 L CNN
F 1 "1N4148" H 5900 4100 50  0000 L CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 6050 4025 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 6050 4200 50  0001 C CNN
	1    6050 4200
	0    1    -1   0   
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5FCD4C35
P 1650 2950
F 0 "J1" H 1758 3231 50  0000 C CNN
F 1 "JST XH" H 1758 3140 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 1650 2950 50  0001 C CNN
F 3 "~" H 1650 2950 50  0001 C CNN
	1    1650 2950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J4
U 1 1 5FCD6014
P 9400 2550
F 0 "J4" H 9372 2432 50  0000 R CNN
F 1 "JST XH" H 9372 2523 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 9400 2550 50  0001 C CNN
F 3 "~" H 9400 2550 50  0001 C CNN
	1    9400 2550
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5FCD70A2
P 9350 3750
F 0 "J2" H 9322 3632 50  0000 R CNN
F 1 "JST XH" H 9322 3723 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 9350 3750 50  0001 C CNN
F 3 "~" H 9350 3750 50  0001 C CNN
	1    9350 3750
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 5FCD7D7F
P 9350 5000
F 0 "J3" H 9322 4882 50  0000 R CNN
F 1 "JST XH" H 9322 4973 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 9350 5000 50  0001 C CNN
F 3 "~" H 9350 5000 50  0001 C CNN
	1    9350 5000
	-1   0    0    1   
$EndComp
Wire Wire Line
	2900 3250 2900 3550
Wire Wire Line
	2900 3550 3000 3550
Wire Wire Line
	1850 2850 3750 2850
Wire Wire Line
	4750 3550 4750 3400
Wire Wire Line
	4750 3400 4750 3150
Wire Wire Line
	4750 3400 6900 3400
Wire Wire Line
	6900 3400 6900 3300
Connection ~ 4750 3400
Wire Wire Line
	4750 2700 4750 2850
Connection ~ 4750 2850
Wire Wire Line
	4750 3950 4750 4050
Wire Wire Line
	5250 4600 5250 4750
Wire Wire Line
	4300 3750 4450 3750
Wire Wire Line
	4450 4400 4450 3750
Connection ~ 4450 3750
Wire Wire Line
	3600 3550 3750 3550
Wire Wire Line
	3750 3550 3750 2850
Connection ~ 3750 2850
Wire Wire Line
	3750 2850 4750 2850
Wire Wire Line
	1850 3150 2250 3150
Wire Wire Line
	2250 3150 2250 4050
Wire Wire Line
	1850 3050 2600 3050
Wire Wire Line
	2600 3050 2600 3750
Wire Wire Line
	2600 3750 3000 3750
Wire Wire Line
	6050 4050 6050 3900
Wire Wire Line
	7700 3300 7700 3550
Wire Wire Line
	7700 3550 9150 3550
Wire Wire Line
	9150 3650 7300 3650
Wire Wire Line
	7300 3650 7300 3300
Wire Wire Line
	9150 3750 7250 3750
Wire Wire Line
	7250 3750 7250 4000
Wire Wire Line
	9150 3850 7650 3850
Wire Wire Line
	7650 3850 7650 4000
Wire Wire Line
	8550 2550 9200 2550
Wire Wire Line
	9200 2650 8650 2650
Wire Wire Line
	6850 4000 6850 3900
Wire Wire Line
	6850 4600 6850 4850
Wire Wire Line
	1850 2950 2900 2950
Connection ~ 6050 3900
Wire Wire Line
	6050 3900 6850 3900
Wire Wire Line
	6050 4350 6050 4850
Wire Wire Line
	4750 2700 5800 2700
Wire Wire Line
	6050 4850 6850 4850
Wire Wire Line
	5250 3900 5250 4200
Wire Wire Line
	5250 3900 6050 3900
Wire Wire Line
	4450 4400 4950 4400
Wire Wire Line
	3600 3750 4000 3750
$Comp
L Device:R R2
U 1 1 5FCC9B36
P 4150 3750
F 0 "R2" V 3943 3750 50  0000 C CNN
F 1 "1K" V 4034 3750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4080 3750 50  0001 C CNN
F 3 "~" H 4150 3750 50  0001 C CNN
	1    4150 3750
	0    1    1    0   
$EndComp
Text Notes 2000 2850 0    50   ~ 0
+5V (Relay)\n
Text Notes 2000 2950 0    50   ~ 0
+3.3V (Signal)\n
Text Notes 2000 3050 0    50   ~ 0
Signal\n
Wire Wire Line
	4750 4050 3900 4050
Wire Wire Line
	5250 4750 3900 4750
Wire Wire Line
	3900 4750 3900 4050
Connection ~ 3900 4050
Wire Wire Line
	3900 4050 2250 4050
Wire Wire Line
	6050 4850 5800 4850
Wire Wire Line
	5800 4850 5800 2700
Connection ~ 6050 4850
Connection ~ 5800 2700
Wire Wire Line
	5800 2700 6900 2700
Text Notes 2000 3150 0    50   ~ 0
GND\n
$Comp
L Relay:G6S-2 K1
U 1 1 5FD63F36
P 7300 3000
F 0 "K1" H 6550 3000 50  0000 L CNN
F 1 "G6-AK-274P" H 6150 2900 50  0000 L CNN
F 2 "Relay_THT:Relay_DPDT_Omron_G5V-2" H 7950 2950 50  0001 L CNN
F 3 "http://omronfs.omron.com/en_US/ecb/products/pdf/en-g6s.pdf" H 7100 3000 50  0001 C CNN
	1    7300 3000
	1    0    0    -1  
$EndComp
$Comp
L Relay:G6S-2 K2
U 1 1 5FD6D57A
P 7250 4300
F 0 "K2" H 6600 4350 50  0000 R CNN
F 1 "G6-AK-274P" H 6600 4450 50  0000 R CNN
F 2 "Relay_THT:Relay_DPDT_Omron_G5V-2" H 7900 4250 50  0001 L CNN
F 3 "http://omronfs.omron.com/en_US/ecb/products/pdf/en-g6s.pdf" H 7050 4300 50  0001 C CNN
	1    7250 4300
	1    0    0    1   
$EndComp
Text Notes 9850 3750 0    50   ~ 0
To Stepper Motor\n
Text Notes 9850 2550 0    50   ~ 0
From Controller Board
Text Notes 9850 5000 0    50   ~ 0
From SMuFF
Wire Wire Line
	8550 2550 8550 4950
Wire Wire Line
	8550 4950 7150 4950
Wire Wire Line
	7150 4950 7150 4600
Wire Wire Line
	7550 4600 7550 4850
Wire Wire Line
	7550 4850 8650 4850
Wire Wire Line
	8650 4850 8650 2650
Wire Wire Line
	9200 2450 7200 2450
Wire Wire Line
	7200 2450 7200 2700
Wire Wire Line
	7600 2700 7600 2350
Wire Wire Line
	7600 2350 9200 2350
Wire Wire Line
	7750 4600 7750 5100
Wire Wire Line
	7750 5100 9150 5100
Wire Wire Line
	9150 5000 7350 5000
Wire Wire Line
	7350 5000 7350 4600
Wire Wire Line
	9150 4900 8750 4900
Wire Wire Line
	8750 4900 8750 3100
Wire Wire Line
	8750 3100 8050 3100
Wire Wire Line
	8050 3100 8050 2600
Wire Wire Line
	8050 2600 7400 2600
Wire Wire Line
	7400 2600 7400 2700
Wire Wire Line
	7800 2700 7800 2550
Wire Wire Line
	7800 2550 8250 2550
Wire Wire Line
	8250 2550 8250 4800
Wire Wire Line
	8250 4800 9150 4800
$EndSCHEMATC
