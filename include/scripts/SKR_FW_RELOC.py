# -----------------------------------------------------------
# Script for relocating firmware behind (SKR) bootloader 
# -----------------------------------------------------------
Import("env")
from pprint import pprint

flash_ofs = "0x7000"    # default location is on top of bootloader 

# Lookup for user defined "flash offset" and use this instead of the predefined value
envdefs = env['CPPDEFINES'].copy()
for define in envdefs:
    if define[0] == "FLASH_OFFSET":
        flash_ofs = define[1]
    if define[0] == "VECT_TAB_OFFSET":
        env['CPPDEFINES'].remove(define)
env['CPPDEFINES'].append(("VECT_TAB_OFFSET", flash_ofs))

envlnkf = env['LINKFLAGS'].copy()
for lflag in envlnkf:
    if type(lflag) != tuple:
        if lflag.startswith("-Wl") and lflag.find("LD_FLASH_OFFSET") > -1:
            #pprint("Removing: " + lflag)
            env['LINKFLAGS'].remove(lflag)
    #else:
    #    pprint(lflag)

# Relocate firmware from 0x08000000 to 0x08000000+LD_FLASH_OFFSET
env['LINKFLAGS'].append("-Wl,--defsym=LD_FLASH_OFFSET="+flash_ofs)
