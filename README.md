# SMuFF-1.1
Latest version of the SMuFF firmware to be compiled in PlatformIO

This is the software package for the Smart Multi Filament Feeder (SMuFF) project as published on Thingiverse (https://www.thingiverse.com/thing:3431438).
![The SMuFF](https://github.com/technik-gegg/SMuFF-1.1/blob/master/images/SMuFF%20render-2.png)

You have to compile and install this firmware on the Wahnhao i3 duplicator mini board (i.e. [AliExpress Wanhao i3-Mini](https://www.aliexpress.com/item/motherboard-i3mini-0ne-motherboard-New-2017-Wanhao-printer-i3-Mini/32849200836.html?spm=a2g0x.10010108.1000001.12.20c22a870NKth9&pvid=f20ef7d9-21cb-4600-b3eb-75382e0c6661&gps-id=pcDetailBottomMoreOtherSeller&scm=1007.13338.122670.0&scm-url=1007.13338.122670.0&scm_id=1007.13338.122670.0])).

This motherboard is a very small but powerful controller, usually used to drive a 3D printer. Since it can handle up to 4 stepper motors, has it's own LC display and SD-Card / rotary encoder and runs on 12V as well as on 24V, it's the ideal tool for this project.  

This firmware might also run on other boards equipped with an AT-Mega 2560, a LC display, an SD-Card and rotary encoder but you'd have to adopt it to the hardware used (see config.h).

The basic configuration (SMuFF.cfg) has to be located on the SD-Card. Hence, changing parameters doesn't require recompiling the firmware. Just edit the configuration JSON file and reboot.

For further information head over to the [Wiki pages](https://github.com/technik-gegg/SMuFF-1.1/wiki).

## Recent changes
**1.55** - Optimization
+ reworked Revolver movement - much smoother now
+ optimized stepper motor speeds for SKR in configuration file
+ Tools swapping is now being stored in the EEPROM.DAT file so that it'll survive a reset
+ added Cancel / Retry option when loading fails
+ sound (beeper) now working flawlessly on SKR (STM32)
+ SKR still has issues with the fan (it's either full speed or no speed)
**1.54** - Not been published
**1.53** - This version has got some major changes:
+ Full integration of the SKR mini V1.1 controller board (STM32) completed
+ Prusa MMU2 Emulation mode improved even more
+ Heavy refactoring to make the code better readable
+ Optimized memory usage (all strings are now located in PROGMEM)
+ replaced the Rotary Encoder library
+ Header files are now located in the include folder
+ Pins header file separated into subfolders for different devices (uses the  -I compiler directive)
+ Added Configs folder containing different configuration samples for the modes/controllers
+ Platformio.ini modified to allow different build environments
+ Moved datastore from EEPROM to SD-Card (mainly because of the STM32)
+ Indexer for Materials in SMUFF.CFG renamed from Tool0..x to T0..x (because of memory issues)
+ Added *FeedChunks* and *EnableChunks* settings to SMUFF.CFG. Those are needed since the communcation on the 2nd serial port tends to hand in long operations (such as feeding the filament to nozzle) and Prusa won't be able to abort the feed.
+ Added *StepDelay* setting to SMUFF.CFG for the SKR Mini. This is needed because of the speed of an 32 bit board to keep the steppers from stalling.
+ Added schematics of the SKR Mini LCD board

**1.52** - Not been published

**1.51** - Not been published

**1.50**  - Not been published

**1.4x** - Not been published

**1.4**  - Major changes to gain better compatibility for Prusa Emulation Mode and setup for a different platform (STM32). Latter is unfinished yet. 
The configuration file (SMUFF.CFG) has got a new setting which defines the distance from the filament guide to the Selector (see *SelectorDist* setting).

**1.3**  - Some modifications to gain Prusa MMU2 compatibility

**1.2**  - Initial published version
