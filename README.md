# SMuFF-1.1
Latest version of the SMuFF firmware to be compiled in PlatformIO

This is the software package for the Smart Multi Filament Feeder (SMuFF) project as published on Thingiverse (https://www.thingiverse.com/thing:3431438).

You have to compile and install this firmware on the Wahnhao i3 duplicator mini board (i.e. https://www.aliexpress.com/item/motherboard-i3mini-0ne-motherboard-New-2017-Wanhao-printer-i3-Mini/32849200836.html?spm=a2g0x.10010108.1000001.12.20c22a870NKth9&pvid=f20ef7d9-21cb-4600-b3eb-75382e0c6661&gps-id=pcDetailBottomMoreOtherSeller&scm=1007.13338.122670.0&scm-url=1007.13338.122670.0&scm_id=1007.13338.122670.0).
This motherboard is a very small but powerful controller, usually used to drive a 3D printer. Since it can handle up to 4 stepper motors, has it's own LC display and SD-Card / rotary encoder and runs on 12V as well on 24V, it's the ideal tool for this project.  

This firmware might also run on other boards equipped with an AT-Mega 2560, a LC display, an SD-Card and rotary encoder but you'd have to adopt it to the hardware used (see config.h).

The basic configuration (SMuFF.cfg) has to be located on the SD-Card. Hence, changing parameters dosn't require recompiling the firmware. Just edit the configuration JSON file and reboot.

For further information head over to the Wiki pages:
https://github.com/technik-gegg/SMuFF/wiki
