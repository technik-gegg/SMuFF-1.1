# -----------------------------------------------------------
# Script for relocating firmware behind (SKR) bootloader 
# -----------------------------------------------------------
import os
Import("env")

flash_ofs = "0x7000"    # default location is on top of bootloader 

# Lookup for user defined "flash offset" and use this instead of the predefined value
for define in env['CPPDEFINES']:
    if define[0] == "FLASH_OFFSET":
        flash_ofs = define[1]
    if define[0] == "VECT_TAB_OFFSET":
        env['CPPDEFINES'].remove(define)
env['CPPDEFINES'].append(("VECT_TAB_OFFSET", flash_ofs))
        
for lflag in env['LINKFLAGS']:
    if lflag.startswith("-Wl,--defsym=LD_FLASH_OFFSET"):
        env['LINKFLAGS'].remove(lflag)

# Relocate firmware from 0x08000000 to 0x08000000+LD_FLASH_OFFSET
env['LINKFLAGS'].append("-Wl,--defsym=LD_FLASH_OFFSET="+flash_ofs)
