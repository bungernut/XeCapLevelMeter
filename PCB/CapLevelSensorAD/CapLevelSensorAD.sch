EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "nEXOCapLevelMeter Board"
Date "2021-08-28"
Rev "246-112-06-C01"
Comp "SLAC"
Comment1 "Brian Mong"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CapLevelSensorLib:AD7151 U1
U 1 1 6129718A
P 6400 2800
F 0 "U1" H 6400 3265 50  0000 C CNN
F 1 "AD7151" H 6400 3174 50  0000 C CNN
F 2 "Package_SO:MSOP-10_3x3mm_P0.5mm" H 6300 2600 50  0001 C CNN
F 3 "https://www.digikey.com/en/products/detail/analog-devices-inc/AD7151BRMZ/1766856" H 6300 2600 50  0001 C CNN
F 4 "AD7151BRMZ-ND" H 6400 2800 50  0001 C CNN "DigikeyPN"
	1    6400 2800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 6129822C
P 8150 2800
F 0 "J1" H 7950 2650 50  0000 R CNN
F 1 "Conn_01x04_Male" H 7950 2750 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8150 2800 50  0001 C CNN
F 3 "~" H 8150 2800 50  0001 C CNN
	1    8150 2800
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 612997D8
P 8150 3150
F 0 "J2" H 8122 3032 50  0000 R CNN
F 1 "Conn_01x02_Male" H 8122 3123 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8150 3150 50  0001 C CNN
F 3 "~" H 8150 3150 50  0001 C CNN
	1    8150 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	6750 2700 7400 2700
Wire Wire Line
	7950 2800 7850 2800
Wire Wire Line
	5950 2600 6050 2600
Wire Wire Line
	6050 2700 5900 2700
Wire Wire Line
	5900 2700 5900 2200
Wire Wire Line
	5900 2200 7100 2200
Wire Wire Line
	7800 2900 7950 2900
Wire Wire Line
	7950 3050 7050 3050
Wire Wire Line
	7050 3050 7050 2900
Wire Wire Line
	7050 2900 6750 2900
Wire Wire Line
	7850 2800 7850 3150
Wire Wire Line
	7850 3150 7950 3150
Connection ~ 7850 2800
$Comp
L Device:C_Small C2
U 1 1 6129ED65
P 5750 2600
F 0 "C2" H 5700 2850 50  0000 L CNN
F 1 "0.1uF" H 5650 2750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5750 2600 50  0001 C CNN
F 3 "~" H 5750 2600 50  0001 C CNN
	1    5750 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 6129F8B2
P 5550 2600
F 0 "C1" H 5500 2750 50  0000 L CNN
F 1 "1uF" H 5350 2650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5550 2600 50  0001 C CNN
F 3 "~" H 5550 2600 50  0001 C CNN
	1    5550 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2700 5750 2700
Wire Wire Line
	5750 2700 5900 2700
Connection ~ 5750 2700
Connection ~ 5900 2700
Wire Wire Line
	5750 2500 5950 2500
Wire Wire Line
	5950 2500 5950 2600
Wire Wire Line
	5550 2500 5750 2500
Connection ~ 5750 2500
Wire Wire Line
	7700 2800 7850 2800
$Comp
L Device:C_Small C4
U 1 1 612A34A0
P 5900 3000
F 0 "C4" H 5850 3150 50  0000 L CNN
F 1 "22pF" H 5900 2900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5900 3000 50  0001 C CNN
F 3 "~" H 5900 3000 50  0001 C CNN
	1    5900 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 612A3E70
P 6850 3100
F 0 "C5" H 6800 3250 50  0000 L CNN
F 1 "47pF" H 6850 2950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3100 50  0001 C CNN
F 3 "~" H 6850 3100 50  0001 C CNN
	1    6850 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R3
U 1 1 612A48FB
P 7150 3250
F 0 "R3" H 7218 3296 50  0000 L CNN
F 1 "10k" H 7218 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7150 3250 50  0001 C CNN
F 3 "~" H 7150 3250 50  0001 C CNN
	1    7150 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 612A5258
P 5300 2900
F 0 "R1" V 5200 2900 50  0000 C CNN
F 1 "39k" V 5400 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5300 2900 50  0001 C CNN
F 3 "~" H 5300 2900 50  0001 C CNN
	1    5300 2900
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R2
U 1 1 612A5751
P 5700 2900
F 0 "R2" V 5600 2900 50  0000 C CNN
F 1 "82k" V 5800 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5700 2900 50  0001 C CNN
F 3 "~" H 5700 2900 50  0001 C CNN
	1    5700 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 2900 6050 2900
Wire Wire Line
	6750 3000 6850 3000
Wire Wire Line
	5900 3100 5900 3250
Wire Wire Line
	6850 3250 6850 3200
$Comp
L Device:C_Small C3
U 1 1 612AB2CE
P 5500 3000
F 0 "C3" H 5450 3150 50  0000 L CNN
F 1 "68pF" H 5300 2900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5500 3000 50  0001 C CNN
F 3 "~" H 5500 3000 50  0001 C CNN
	1    5500 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2900 5900 2900
Connection ~ 5900 2900
Wire Wire Line
	5600 2900 5500 2900
Wire Wire Line
	5400 2900 5500 2900
Connection ~ 5500 2900
Wire Wire Line
	6850 3000 7000 3000
Wire Wire Line
	7000 3000 7000 3150
Wire Wire Line
	7000 3150 7150 3150
Connection ~ 6850 3000
Wire Wire Line
	7150 3350 7150 3550
Wire Wire Line
	5200 2900 5200 3550
Wire Wire Line
	5500 3100 5500 3250
Wire Wire Line
	5500 3250 5900 3250
Connection ~ 5900 3250
Wire Wire Line
	7800 2900 7800 2200
Connection ~ 5550 2500
$Comp
L Device:R_Small_US R4
U 1 1 612B6792
P 7100 2350
F 0 "R4" H 7168 2396 50  0000 L CNN
F 1 "10k" H 7168 2305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7100 2350 50  0001 C CNN
F 3 "~" H 7100 2350 50  0001 C CNN
	1    7100 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R5
U 1 1 612B70BC
P 7400 2350
F 0 "R5" H 7468 2396 50  0000 L CNN
F 1 "10k" H 7468 2305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7400 2350 50  0001 C CNN
F 3 "~" H 7400 2350 50  0001 C CNN
	1    7400 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 2250 7100 2200
Connection ~ 7100 2200
Wire Wire Line
	7100 2200 7400 2200
Wire Wire Line
	7400 2250 7400 2200
Connection ~ 7400 2200
Wire Wire Line
	7400 2200 7800 2200
Wire Wire Line
	7400 2450 7400 2700
Wire Wire Line
	7100 2450 7100 2600
Wire Wire Line
	7100 2600 6750 2600
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 612C1DCE
P 6100 3750
F 0 "J3" V 6050 3850 50  0000 R CNN
F 1 "Conn_01x04_Male" V 5950 3900 50  0000 R CNN
F 2 "CapLevelSensorAD:Amphenol Anytech YO 3.81mm 4p" H 6100 3750 50  0001 C CNN
F 3 "https://www.digikey.com/en/products/detail/amphenol-anytek/YO0421500000G/2261308" H 6100 3750 50  0001 C CNN
F 4 "609-3920-ND" V 6100 3750 50  0001 C CNN "DigikeyPN"
	1    6100 3750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6300 3550 7150 3550
Wire Wire Line
	5200 3550 6000 3550
Wire Wire Line
	5900 3250 6100 3250
Connection ~ 6100 3250
Wire Wire Line
	6100 3250 6200 3250
Connection ~ 6200 3250
Wire Wire Line
	6100 3250 6100 3550
Wire Wire Line
	6200 3250 6200 3550
Wire Wire Line
	5550 2500 5550 2150
Wire Wire Line
	5550 2150 7700 2150
Wire Wire Line
	7700 2150 7700 2800
Text Notes 8150 2700 0    50   ~ 0
SDA
Text Notes 8150 2600 0    50   ~ 0
SCL\n
Text Notes 8150 2800 0    50   ~ 0
GND
Text Notes 8150 2900 0    50   ~ 0
VDD
Wire Wire Line
	7400 2700 7550 2700
Wire Wire Line
	7550 2700 7550 2600
Wire Wire Line
	7550 2600 7950 2600
Connection ~ 7400 2700
Wire Wire Line
	7300 2600 7300 2750
Wire Wire Line
	7300 2750 7650 2750
Wire Wire Line
	7650 2750 7650 2700
Wire Wire Line
	7650 2700 7950 2700
Wire Wire Line
	7100 2600 7300 2600
Connection ~ 7100 2600
Connection ~ 7700 2800
Wire Wire Line
	7700 3450 7700 2800
Wire Wire Line
	6200 3250 6600 3250
Wire Wire Line
	6600 3250 6850 3250
Connection ~ 6600 3250
Wire Wire Line
	6600 3250 6600 3450
Wire Wire Line
	6600 3450 7700 3450
$EndSCHEMATC
