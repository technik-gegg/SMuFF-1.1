# Configuration Files

Copy these files onto the SMuFFs SD-Card **root folder** (/) before you flash the firmware.

*Please keep in mind that these files are not needed for flashing the firmware but for the runtime of the SMuFF! Hence, this SD-Card has to remain in its slot all the time.*

The filenames are supposed to be self explanatory.

**Please notice:** *You don't have to modify the contents of the JSON files manually, since they're covered by the menus and GCode interface.
Although, you can if you feel the urge to do so. Just use a JSON capable editor (i.e. VSCode) to ensure the format doesn't get messed up, which will eventually render the files useless.*

---

The **debug.txt** file contains the debug output flags, which is being read before the SMuFF starts up in order not to miss any information.
*If you want to change the default debug output behaviour of the firmware, simply edit this file in a text editor.*

The (binary encoded) value of *15* is the default setting, whereas *255* means "show all". For a more specific defintion see *debug.h*:

```C++
#define D       1
#define W       2
#define I       4
...
```

If the value is followed by a semicolon ( **;** ) and a number between **0** and **3**, the firmware will route debug messages to the according serial port:

+ 0 = USB
+ 1 = Serial1
+ 2 = Serial2
+ 3 = Serial3

*Serial2* is by definition the default port for debug messages, which usualy is located at the **TFT** header.
*Please keep in mind: Not every serial port is available on all of the supported boards. The file "Pins.h" contains a couple of CAN_USE_SERIALx definitions that tells which serial port is available in general.*

If you set the debug port to **USB (0)**, make sure you **undo** this change before you attach the SMuFF to **OctoPrint** for example, because otherwise this will mess up the communication with the [OctoPrint SMuFF plugin](https://github.com/technik-gegg/OctoPrint-Smuff), which usually runs via the USB.
