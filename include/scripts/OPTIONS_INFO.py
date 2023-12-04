# -----------------------------------------------------------
# Script for printing build options
# -----------------------------------------------------------
import os,time
import sys
Import("env")

#print(sys.version)

pioenv = env["PIOENV"]
envbase = pioenv[:pioenv.find("___")].replace("_12"," V1.2").replace("_20"," V2.0").replace("_30"," V3.0").replace("_E3DIP"," E3-DIP V1.1").replace("_", " ")
if pioenv.find("___DDE") > 0:
    extruder = "\x1b[4musing Direct Drive Extruder\x1b[0m"
else:
    extruder = "\x1b[4musing Bowden Extruder\x1b[0m"
isE3_20 = pioenv.find("SKR_E3_20")
display = ""
useTerminal = ""
neopixel = ""
backlight = ""
displaySPI = ""
debug = ""
hwdebug = ""
disableSWI = "Enabled"
v4 = "Yes"
v5 = ""
v6s = ""
variant = ""
splitterEnds = ""
marlin = ""
softreset = ""
swapY = ""
swapX = ""
multiservo = ""
hsServo = ""
swapServos = ""
zServo = "Arduino"
relayOnProbe = ""
warning = ""
usbId = "STMicroelectronics (VID/PID 0483:5740)"
flashOfs = ""
swTwi = False
isTwiDsp = False
fastSPI = False
hasFastSPI = False
hwSPI = False

build_flags = env.ParseFlags(env['BUILD_FLAGS'])
build_unflags = env.ParseFlags(env['BUILD_UNFLAGS'])
#print(build_flags.get("CPPDEFINES"))
#print(build_unflags.get("CCFLAGS"))

# Lookup for user defined options
for define in build_flags.get("CPPDEFINES"):
    if define == "USE_TWI_DISPLAY":
        display = "\x1b[35mDIY TWI/I2C display"
        isTwiDsp = True
    if define == "USE_LEONERD_DISPLAY":
        display = "\x1b[35mLeoNerd's OLED Module"
        isTwiDsp = True
    if define == "USE_MINI12864_PANEL_V21":
        display = "FYSETC/BTT/MKS \x1b[35mMini12864 Panel V2.1"
    if define == "USE_MINI12864_PANEL_V20":
        display = "FYSETC/BTT/MKS \x1b[35mMini12864 Panel V2.0"
    if define == "USE_CREALITY_DISPLAY":
        display = "Creality \x1b[35mEnder-3/CR10 Stock Display"
        hasFastSPI = True
    if define == "USE_DEFAULT_DISPLAY":
        display = "Standard \x1b[35mRepRap Full-Graphic-Display"
    if define == "USE_SERIAL_DISPLAY":
        display = "\x1b[35mSerial Display (SMuFF-TFT)"

    if define == "USE_FASTLED_BACKLIGHT":
        backlight = "w. NeoPixels backlight"
        neopixel = "Enabled for Backlight only"
    if define == "USE_RGB_BACKLIGHT":
        backlight = "w. RGB backlight"
    if define == "CREALITY_HW_SPI" or define == "USE_HW_SPI":
        displaySPI = "(using hardware SPI)"
        hwSpi = True
    if define == "USE_FASTLED_TOOLS":
        if backlight != "":
            neopixel = "Enabled for Backlight and Tools"
        else:
            neopixel = "Enabled for Tools only"
    if define == "DISABLE_DEBUG_PORT":
        disableSWI = "DISABLED"
    if define == "DEBUG":
        debug = "Enabled"
    if define == "__HW_DEBUG__":
        hwdebug = "Enabled"
    if define == "SMUFF_V5":
        v5 = "Yes"
    if define == "SMUFF_V6S":
        v6s = "Yes"

    if define == "USE_SPLITTER_ENDSTOPS":
        splitterEnds = "Enabled"
    if define == "MARLIN2_ONLY":
        marlin = "Yes"
    if define == "SOFTRESET":
        softreset = "Enabled"
    if define == "SWAP_Y_STEPPER":
        swapY = "Yes"
    if define == "SWAP_X_STEPPER":
        swapX = "Yes"
    if define == "MULTISERVO":
        multiservo = "Enabled"
    if define == "USE_HIGHSPEED_SERVO":
        hsServo = "Enabled"
    if define =="SWAP_SERVOS":
        swapServos = "Yes"
    if define =="USE_ZSERVO":
        zServo = "ZServo"
    if define =="USE_MULTISERVO":
        zServo = "Adafruit Multiservo"
        relayOnProbe = "On TH0"
    if define =="USE_MULTISERVO_RELAY":
        relayOnProbe = "On Adadfruit Multiservo"
    if define =="RELAY_ON_PROBE" and zServo != "Adafruit Multiservo":
        relayOnProbe = "On PROBE"
    if define == "WARNING_NOT_RELEASED":
        warning  = "*** THIS BOARD HAS NOT BEEN RELEASED YET ***"
    if define == "MIMIC_LIBMAPLE_USB_ID":
        usbId = "LeafLabs Maple (VID/PID 1EAF:0004)"
    if define[0] == "FLASH_OFFSET":
        flashOfs = define[1]
    if define == "USE_SW_TWI":
        swTwi = True
    if define == "USE_FAST_SW_SPI":
        fastSPI = True
        displaySPI = "(using fast software SPI)"

if v5 == "" and v6s == "":
    variant = "V4 or older"
if v5 != "":
    variant = "V5/V6"
if v6s != "":
    variant = "V6S"
if v5 != "" and v6s != "":
    print("\x1b[31mInvalid build configuration. Use either SMUFF_V5 or SMUFF_V6S but not both!", "\x1b[0m\n\n")
    variant = "(Invalid)"
    env.Exit(-1)

#
# Print out the major settings for this build
#
print("\n\n\x1b[36m============================================================================================================================================")
print("\x1b[36mBuilding SMuFF firmware", variant, "for:", "\x1b[1;35m", envbase, extruder)

if warning != "":
    print("\n\x1b[31m", warning)

print("\x1b[36m--------------------------------------------------------------------------------------------------------------------------------------------")

if flashOfs != "":
    print("\x1b[34mFlash Offset:\t\t\t\x1b[1;35m", flashOfs)
else:
    print("\x1b[31mFlash Offset was not defined! Please check and set before compiling!")
    env.Exit(-1)

if disableSWI == "Enabled":
    print("\x1b[34mSWI Debug Port:\t\t\t\x1b[35m", disableSWI)
else:
    print("\x1b[34mSWI DEBUG PORT:\t\t\t\x1b[31m", disableSWI)

print("\x1b[34mUSB Device-ID:\t\t\t\x1b[35m", usbId)
print("\x1b[36m============================================================================================================================================\x1b[0m")

if display == "":
    print("\n\x1b[31m*** Unknown Display Option ***")
    print("\x1b[31m*** Please select a valid display option in section [display] of platformio.ini ***\n")
    env.Exit(-1)

print("DISPLAY OPTION:\t\t\t", display, backlight, displaySPI)

if isTwiDsp == True and isE3_20 == -1 and swTwi == True:
    print("\n\x1b[31m*** Software I2C/TWI is not available for this board ***")
    print("\x1b[31m*** Please disable '-D USE_SW_TWI' flag from platformio.ini ***\n")
    env.Exit(-1)

if isTwiDsp == True and isE3_20 != -1 and swTwi == False:
    print("\n\x1b[31m*** Hardware I2C/TWI is not available for this board ***")
    print("\x1b[31m*** Please enable '-D USE_SW_TWI' flag in platformio.ini ***\n")
    env.Exit(-1)

if fastSPI == True and hasFastSPI == False:
    print("\n\x1b[31m*** Fast SW SPI is not available on this display ***")
    print("\x1b[31m*** Please disable '-D USE_FAST_SW_SPI' flag in platformio.ini ***\n")
    env.Exit(-1)

if fastSPI == True and hwSPI == True:
    print("\n\x1b[31m*** Fast SW SPI can't be combined with HW SPI ***")

if useTerminal != "":
    print("TERMINAL MENUS OPTION:\t", useTerminal)
if neopixel != "":
    print("NEOPIXEL OPTION:\t\t", neopixel)
if debug != "":
    print("DEBUG OPTION:\t\t\t", debug)
if hwdebug != "":
    print("HW DEBUG OPTION:\t\t", hwdebug)
if splitterEnds != "":
    print("\x1b[34mSPLITTER ENDSTOPS:\t\t", splitterEnds)
if marlin != "":
    print("\x1b[34mMARLIN ONLY:\t\t\t", marlin)
if softreset != "":
    print("\x1b[34mSOFTRESET:\t\t\t", softreset)
if swapY != "":
    print("\x1b[34mSWAP Y WITH E DRIVER:\t\t\x1b[31m", swapY)
if swapX != "":
    print("\x1b[34mSWAP X WITH E DRIVER:\t\t\x1b[31m", swapX)
if swapServos != "":
    print("\x1b[35mSWAP WIPER/CUTTER SERVOS:\t", swapServos)
print("\x1b[34mSERVO LIBRARY:\t\t\t", zServo)
if multiservo != "":
    print("\x1b[34mMULTISERVO:\t\t\t", multiservo)
if multiservo == "" and hsServo != "":
    print("\x1b[34mHIGHSPEED SERVO:\t\t", hsServo)
if relayOnProbe != "":
    print("\x1b[35mRELAY:\t\t\t\t", relayOnProbe)

print("\x1b[36m============================================================================================================================================\x1b[0m\n\n")
#time.sleep(2)
