# -----------------------------------------------------------
# Script for Firmware Build Configurator
# -----------------------------------------------------------
Import("env")

if env.IsIntegrationDump() or env.IsCleanTarget():
    # stop the current script execution
    Return()

import os, traceback
from pprint import pprint
try:
    # dummy import, just checking the Python environment
    import tkinter
except ImportError:
    print("\n\n{0}Tkinter package not found. Please follow the instructions here: {1}".format("\x1b[31m", "\x1b[0m"))
    print("{0}https://sites.google.com/view/the-smuff/how-to/compile-the-firmware#h.f9tt9ww98lzj{1}".format("\x1b[34m", "\x1b[0m"))
    print("\n{0}Build was cancelled due to an insufficient Python environment!{1}\n".format("\x1b[31m", "\x1b[0m"))
    env.Exit(-1)

try:
    try:
        import customtkinter
    except ImportError:
        print("Installing additional Python package CustomTkinter...")
        env.Execute("$PYTHONEXE -m pip install customtkinter")
    try:
        from CTkMessagebox import CTkMessagebox
    except ImportError:
        print("Installing additional Python package CTkMessagebox...")
        env.Execute("$PYTHONEXE -m pip install CTkMessagebox")
    try:
        from CTkToolTip import *
    except ImportError:
        print("Installing additional Python package CustomTkToolTip...")
        env.Execute("$PYTHONEXE -m pip install CTkToolTip")
except:
    traceback.print_exc()
    print("{0}Can't install additional Python packages. Please check your setup/environment.{1}".format("\x1b[31m", "\x1b[0m"))
    env.Exit(-1)

build_flags     = env.ParseFlags(env["BUILD_FLAGS"])
project_dir     = env["PROJECT_DIR"]
build_dir       = env["PROJECT_BUILD_DIR"]
pioenv          = env["PIOENV"]
env_defines     = build_flags.get("CPPDEFINES")
env_exit        = env.Exit
env_replace     = env.Replace
env_append      = env.Append
env_dump        = env.Dump
config          = env.GetProjectConfig()
cfg_defaults    = config.get("config_defaults", pioenv)

#print(env_dump(), "\n===============")

envbase     = pioenv[:pioenv.find("___")].replace("_12"," V1.2").replace("_20"," V2.0").replace("_30"," V3.0").replace("_E3DIP"," E3-DIP V1.1").replace("_", " ")
isE3_DIP    = pioenv.find("SKR_E3DIP") >= 0
isE3_12     = pioenv.find("SKR_E3_12") >= 0
isE3_20     = pioenv.find("SKR_E3_20") >= 0
isE3_30     = pioenv.find("SKR_E3_30") >= 0
isMINI      = pioenv.find("SKR_MINI") >= 0
isDDE       = False

cfg_names   = []
defines     = []
variant     = ""
display     = ""
usbId       = "STMicroelectronics (VID/PID 0483:5740)"
usbProduct  = ""
relayInfo   = "Onboard (PS-ON)"
show_HWDBG  = False

col_default = "\x1b[0m"
col_black   = "\x1b[30m"
col_red     = "\x1b[31m"
col_green   = "\x1b[32m"
col_yellow  = "\x1b[33m"
col_blue    = "\x1b[34m"
col_pink    = "\x1b[35m"
col_cyan    = "\x1b[36m"
col_white   = "\x1b[37m"
col_mode    = "system"      # CTk Modes: system, light, dark
col_theme   = "dark-blue"   # CTk Themes: blue, dark-blue, green
col_light   = "#d50000"     # "#2CC985"
col_dark    = "#3cbb66"
col_tooltip_bg = "#3a7dbf"
col_tooltip = "#ffffff"

dlg_font    = "{0}/assets/OpenSans-Regular.ttf"
dlg_ffamily = "Open Sans"
dlg_width   = 685
dlg_height  = 605 if show_HWDBG else 570

txt_TITLE   = "SMuFF Firmware-Build Configurator"
txt_ICON    = "{0}/assets/fwbc.ico"
txt_SWTWI   = "Use SW I2C for display (E3 2.0)"
txt_DDE     = "Use Direct Drive Extruder"
txt_NPX     = "Use NeoPixels for Tools"
txt_MS      = "Use Multiservo Board"
txt_MSRLY   = "Relay-Signal on Multiservo Board"
txt_MSTWI   = "Use SW I2C for Multiservo"
txt_SPMTWI  = "Use SW I2C for Spool-Rewinder"
txt_SPMFW   = "Use FeatherWing DC-Motor controller"
txt_SPM     = "Use Spool-Rewinder"
txt_SWD     = "Disable Debug Port (E3 DIP/2.0)"
txt_DBG     = "Enable Debug Messages"
txt_HWDBG   = "Enable HW Debugging"
txt_SWP_X   = "Swap X with E Stepper (E3 3.0)"
txt_SWP_Y   = "Swap Y with E Stepper (E3 2.0/3.0)"
txt_SWP_SE  = "Swap Selector Endstop X to Y (E3 3.0)"
txt_CLK     = "Use Std. Clock Settings (E3 3.0)"
txt_MARL2   = "Marlin 2 MMU Only"
txt_RST     = "Use Soft-Reset"
txt_RLY_Y   = "Relay-Signal on Y-STOP (E3 DIP)"
txt_RLY_P   = "Relay-Signal on PROBE"
txt_ZS      = "Use Onboard Servos"
txt_ONMS    = "On Multiservo Board"
txt_ONTH0   = "Onboard (TH0)"
txt_ONPS    = "Onboard (PS-ON)"
txt_ZPROBE  = " (Z-PROBE)"
txt_ZAXIS   = " (Z-Axis)"
txt_YAXIS   = " (Y-Axis)"

txt_TTSWD   = "Check this option only if SMuFF is using pins SWDIO/SWCLK on the SWD header."
txt_TTDBG   = "Uncheck this option ONLY if you're running out of memory on your board."
txt_TTHWDBG = "Check this option only if you're Technik Gegg ;o)"
txt_TTSWP_X = "Check this option only in case your X stepper driver is broken."
txt_TTSWP_Y = "Check this option only in case your Y stepper driver is broken."
txt_TTSWP_SE= "Check this option only in case you don't like X very much (or it's broken)."
txt_TTCLK   = "Check this option only if Windows/Linux won't recognize your board over USB."
txt_TTMARL2 = "Check this option only if Marlin 2 doesn't like OK responses being sent back."
txt_TTRST   = "Check this option only if the bootloader on your board sends messages while booting."
txt_TTRLY_P = "Check this option only if you're using an old configuration."
txt_TTRLY_Y = "Check this option only if you can't use the MS3 signal on the Z-Axis driver socket."
txt_TTSWTWI = "You must check this option if you're running an I2C display on the E3 2.0 board."
txt_TTSPMTWI= "Multiservo and Spool-Rewinder must share the same I2C bus."
txt_TTMSRLY = "Check this option only if you're using a SMuFF-Backbone board with the according jumper set."
txt_TTDDE   = "Check this option only if you're not using the SMuFF as your primary extruder."
txt_TTNPX   = "Check this option if you'd like some fancy bling-bling on your SMuFF."
txt_TTMS    = "Check this option if you're controlling the servos using an Adafruit Multiservo board."
txt_TTSPM   = "Check this option if you're using the Motorized Spool-Rewinder."
txt_TTSPMFW = "Check this option if you're using the FeatherWing instead of the Waveshare DC-Motor controller."
txt_TTUSBID = "This is what Windows/Linux will show when connected to the SMuFF over USB."

dsp_last_ndx = -1
dsp_option_names= [ "FYSETC / BTT / MKS Mini 12864 (HW SPI)", "Creality / BTT-TFT (Fast SW SPI)", "Creality / BTT-TFT (HW SPI)", "DIY OLED Module (I2C)", "LeoNerd's OLED Module (I2C)", "Full Graphics Display (HW SPI)", "Serial Only (No Display)" ]
dsp_options     = [ ["USE_MINI12864_PANEL_V21", "USE_FASTLED_BACKLIGHT"], ["USE_CREALITY_DISPLAY","USE_FAST_SW_SPI"], ["USE_CREALITY_DISPLAY","CREALITY_HW_SPI"], "USE_TWI_DISPLAY", "USE_LEONERD_DISPLAY", "USE_DEFAULT_DISPLAY", "USE_SERIAL_DISPLAY" ]
swtwi_option    = "USE_SW_TWI"
dde_option      = "USE_DDE"
npx_option      = "USE_FASTLED_TOOLS"
zs_option       = "USE_ZSERVO"
ms_option       = "USE_MULTISERVO"
msrly_option    = "USE_MULTISERVO_RELAY"
mstwi_option    = "USE_PCA9685_SW_I2C"
spm_option      = "USE_SPOOLMOTOR"
spmfw_option    = "USE_SPOOLMOTOR_FEATHERWING"
swd_option      = "DISABLE_DEBUG_PORT"
dbg_option      = "DEBUG"
hwdbg_option    = "__HW_DEBUG__"
swp_x_option    = "SWAP_X_STEPPER"
swp_y_option    = "SWAP_Y_STEPPER"
swp_se_option   = "SWAP_SELECTOR_ENDSTOP"
clk_option      = "USE_OLD_CLOCK_SETTINGS"
marl2_option    = "MARLIN2_ONLY"
rst_option      = "SOFTRESET"
rly_y_option    = "RELAY_ON_YSTOP"
rly_p_option    = "RELAY_ON_PROBE"
maple_option    = "MIMIC_LIBMAPLE_USB_ID"
usb_option      = "USB_PRODUCT_STRING"
flash_option    = "FLASH_OFFSET"
v5_option       = "SMUFF_V5"
v6s_option      = "SMUFF_V6S"

#===================================================
# Custom Tkinter Dialog Section
#===================================================

#
# Control event handlers
#
def change_define(flag, option, optstr=None):
    try:
        if type(option) == list:
            for opt in option:
                if opt.startswith("-"):
                    defines.remove(opt[1:])
                else:
                    defines.append(opt) if flag else defines.remove(opt)
        else:
            defines.append(option) if flag else defines.remove(option)
        if optstr != None:
            if flag:
                cfg_names.append(optstr)
            else:
                cfg_names.remove(optstr)
    except ValueError:
        #print("{0}'Value error' in change_define: \"{1}\" not found.{2}".format(col_red, option, col_black))
        pass
    #pprint(defines)

def run_build():
    global display
    display = cbo_DSP.get()

    for option in defines:
        if option == ms_option:
            try:
                env_defines.remove(zs_option)
            except ValueError:
                pass
        env_append(CPPDEFINES=option)
    #print(env_dump())
    #print("************")

    print("{0}Starting build...{1}".format(col_green, col_black))
    dlg.destroy()

def cancel_build(event = None):
    print("{0}Build-Configurator has been cancelled!{1}".format(col_red, col_black))
    env_exit(-1)

def set_display(choice):
    global dsp_last_ndx

    ndx = 0
    for display in dsp_option_names:
        if choice == display:
            break
        else:
            ndx += 1

    if isE3_20 and (ndx == 3 or ndx == 4):
        chk_SWTWI.configure(state='normal')
    else:
        chk_SWTWI.configure(state='disabled')
    
    if dsp_last_ndx > -1:
        dspdef = dsp_options[dsp_last_ndx]
        change_define(False, dspdef)

    dspdef = dsp_options[ndx]
    change_define(True, dspdef)
    dsp_last_ndx = ndx

def set_SWTWI():
    change_define(chk_SWTWI.get(), swtwi_option, txt_SWTWI)

def set_DDE():
    global isDDE
    isDDE = chk_DDE.get()
    change_define(chk_DDE.get(), dde_option, txt_DDE)
    set_MSTWI_text()

def set_NPX():
    change_define(chk_NPX.get(), npx_option, txt_NPX)

def set_MS():
    change_define(chk_MS.get(), ms_option, txt_MS)
    if not chk_MS.get():
        change_define(True, zs_option, txt_ZS)
        if chk_MSRLY.get():
            change_define(False, msrly_option, txt_MSRLY)
            chk_MSRLY.deselect()
        if not chk_SPM.get():
            chk_MSTWI.deselect()
            chk_SPMTWI.deselect()
    else:
        change_define(False, zs_option, txt_ZS)
    chk_MSRLY.configure(state='normal' if chk_MS.get() else 'disabled')
    if not (isE3_12 or isMINI):
        chk_MSTWI.configure(state='normal' if chk_MS.get() else 'disabled')

def set_MSRLY():
    change_define(chk_MSRLY.get(), msrly_option, txt_MSRLY)
    if chk_MSRLY.get():
        chk_RLY_Y.deselect()
        chk_RLY_P.deselect()
        change_define(False, rly_y_option, txt_RLY_Y)
        change_define(False, rly_p_option, txt_RLY_P)

def set_MSTWI():
    change_define(chk_MSTWI.get(), mstwi_option, txt_MSTWI)
    if chk_MSTWI.get():
        chk_SPMTWI.select()
    else:
        chk_SPMTWI.deselect()

def set_MSTWI_text():
    if isE3_30 or isE3_20:
        swtwi = txt_ZPROBE
    elif isE3_DIP:
        swtwi = txt_ZAXIS if isDDE else txt_YAXIS
    chk_MSTWI.configure(text = txt_MSTWI + swtwi)
    chk_SPMTWI.configure(text = txt_SPMTWI + swtwi)

def set_SPM():
    change_define(chk_SPM.get(), spm_option, txt_SPM)
    if chk_SPM.get():
        change_define(chk_SPMFW.get(), spmfw_option)
    else:
        change_define(False, spmfw_option)
    if not (isE3_12 or isMINI):
        chk_SPMTWI.configure(state='normal' if chk_SPM.get() else 'disabled')
        chk_SPMFW.configure(state='normal' if chk_SPM.get() else 'disabled')

def set_SPMFW():
    change_define(chk_SPMFW.get(), spmfw_option, txt_SPMFW)

def set_SPMTWI():
    change_define(chk_SPMTWI.get(), mstwi_option, txt_SPMTWI)
    if chk_SPMTWI.get():
        chk_MSTWI.select()
    else:
        chk_MSTWI.deselect()

def set_SWD():
    change_define(chk_SWD.get(), swd_option, txt_SWD)

def set_DBG():
    if not chk_DBG.get():
        answer = CTkMessagebox(title="Warning", message="Disabling debug messages will make troubleshooting much harder.\n\nAre you sure you want to disable them?", icon="warning", option_1="No", option_2="Yes")
        if answer.get() == "No":
            chk_DBG.select()
            return
    change_define(chk_DBG.get(), dbg_option, txt_DBG)

def set_HWDBG():
    change_define(chk_HWDBG.get(), hwdbg_option, txt_HWDBG)

def set_SWP_X():
    change_define(chk_SWP_X.get(), swp_x_option, txt_SWP_X)
    if chk_SWP_X.get():
        chk_SWP_Y.deselect()
        change_define(False, swp_y_option, txt_SWP_Y)

def set_SWP_Y():
    change_define(chk_SWP_Y.get(), swp_y_option, txt_SWP_Y)
    if chk_SWP_Y.get():
        chk_SWP_X.deselect()
        change_define(False, swp_x_option, txt_SWP_X)

def set_SWP_SE():
    change_define(chk_SWP_SE.get(), swp_se_option, txt_SWP_SE)
    if chk_SWP_SE.get():
        chk_SWP_Y.deselect()
        change_define(False, swp_y_option, txt_SWP_Y)

def set_CLK():
    change_define(chk_CLK.get(), clk_option, txt_CLK)

def set_MARL2():
    change_define(chk_MARL2.get(), marl2_option, txt_MARL2)

def set_RST():
    change_define(chk_RST.get(), rst_option, txt_RST)

def set_RLY_Y():
    change_define(chk_RLY_Y.get(), rly_y_option, txt_RLY_Y)
    if chk_RLY_Y.get():
        chk_RLY_P.deselect()
        chk_MSRLY.deselect()
        change_define(False, rly_p_option, txt_RLY_P)
        change_define(False, msrly_option, txt_MSRLY)

def set_RLY_P():
    change_define(chk_RLY_P.get(), rly_p_option, txt_RLY_P)
    if chk_RLY_P.get():
        chk_RLY_Y.deselect()
        chk_MSRLY.deselect()
        change_define(False, rly_y_option, txt_RLY_Y)
        change_define(False, msrly_option, txt_MSRLY)

#
# Custom Tkinter init
#
customtkinter.set_appearance_mode(col_mode)
customtkinter.set_default_color_theme(col_theme)
customtkinter.FontManager.load_font(dlg_font.format(project_dir))


dlg = customtkinter.CTk()
dlg.title(txt_TITLE)
dlg.iconbitmap(txt_ICON.format(project_dir))

fnt_title       = customtkinter.CTkFont(family=dlg_ffamily, size=22, weight="normal")
fnt_category    = customtkinter.CTkFont(family=dlg_ffamily, size=16, weight="bold")
fnt_text        = customtkinter.CTkFont(family=dlg_ffamily, size=14, weight="normal")
fnt_text_bold   = customtkinter.CTkFont(family=dlg_ffamily, size=14, weight="bold")
fnt_button      = customtkinter.CTkFont(family=dlg_ffamily, size=16, weight="normal")
fnt_tooltip     = customtkinter.CTkFont(family=dlg_ffamily, size=12, weight="normal")

center_x = int((dlg.winfo_screenwidth() - dlg_width)/2)
center_y = int((dlg.winfo_screenheight() - dlg_height)/2)

dlg.geometry(f"{dlg_width}x{dlg_height}+{center_x}+{center_y}")
dlg.resizable(False, False)
dlg.protocol("WM_DELETE_WINDOW", cancel_build)
dlg.bind('<Escape>', cancel_build)
dlg.lift()

#
# Controls creation
#
lbl_CFG     = customtkinter.CTkLabel(master=dlg,  text="", font=fnt_title, justify="center")
frame1      = customtkinter.CTkFrame(master=dlg)
frame2      = customtkinter.CTkFrame(master=dlg)
lbl_USBID   = customtkinter.CTkLabel(master=dlg,  text="", font=fnt_text, text_color=(col_light, col_dark))
btn_OK      = customtkinter.CTkButton(master=dlg, text="Start Build",   command=run_build,    height=36, font=fnt_button)
btn_CANCEL  = customtkinter.CTkButton(master=dlg, text="Cancel",        command=cancel_build, height=36, font=fnt_button)

lbl_DSP     = customtkinter.CTkLabel(master=frame1, width=300, text="Display Options",  font=fnt_category, text_color=(col_light, col_dark))
lbl_OPT     = customtkinter.CTkLabel(master=frame1, width=300, text="Main Options",     font=fnt_category, text_color=(col_light, col_dark))
lbl_OTH     = customtkinter.CTkLabel(master=frame2, width=300, text="Misc. Options",    font=fnt_category, text_color=(col_light, col_dark))

cbo_DSP     = customtkinter.CTkComboBox(master=frame1, width=220,       command=set_display, font=fnt_text, dropdown_font=fnt_text, values=dsp_option_names, state="readonly")
chk_SWTWI   = customtkinter.CTkCheckBox(master=frame1, text=txt_SWTWI,  command=set_SWTWI,   font=fnt_text, state='disabled')
chk_DDE     = customtkinter.CTkCheckBox(master=frame1, text=txt_DDE,    command=set_DDE,     font=fnt_text)
chk_NPX     = customtkinter.CTkCheckBox(master=frame1, text=txt_NPX,    command=set_NPX,     font=fnt_text)
chk_MS      = customtkinter.CTkCheckBox(master=frame1, text=txt_MS,     command=set_MS,      font=fnt_text)
chk_MSRLY   = customtkinter.CTkCheckBox(master=frame1, text=txt_MSRLY,  command=set_MSRLY,   font=fnt_text, state="disabled")
chk_MSTWI   = customtkinter.CTkCheckBox(master=frame1, text=txt_MSTWI,  command=set_MSTWI,   font=fnt_text, state="disabled")
chk_SPM     = customtkinter.CTkCheckBox(master=frame1, text=txt_SPM,    command=set_SPM,     font=fnt_text)
chk_SPMTWI  = customtkinter.CTkCheckBox(master=frame1, text=txt_SPMTWI, command=set_SPMTWI,  font=fnt_text, state="disabled")
chk_SPMFW   = customtkinter.CTkCheckBox(master=frame1, text=txt_SPMFW,  command=set_SPMFW,   font=fnt_text, state="disabled")

chk_SWD     = customtkinter.CTkCheckBox(master=frame2, text=txt_SWD,    command = set_SWD,   font=fnt_text)
chk_DBG     = customtkinter.CTkCheckBox(master=frame2, text=txt_DBG,    command = set_DBG,   font=fnt_text)
chk_HWDBG   = customtkinter.CTkCheckBox(master=frame2, text=txt_HWDBG,  command = set_HWDBG, font=fnt_text)
chk_SWP_X   = customtkinter.CTkCheckBox(master=frame2, text=txt_SWP_X,  command = set_SWP_X, font=fnt_text, state='normal' if isE3_30 else 'disabled')
chk_SWP_Y   = customtkinter.CTkCheckBox(master=frame2, text=txt_SWP_Y,  command = set_SWP_Y, font=fnt_text, state='normal' if isE3_30 or isE3_20 else 'disabled')
chk_SWP_SE  = customtkinter.CTkCheckBox(master=frame2, text=txt_SWP_SE, command = set_SWP_SE,font=fnt_text, state='normal' if isE3_30 else 'disabled')
chk_CLK     = customtkinter.CTkCheckBox(master=frame2, text=txt_CLK,    command = set_CLK,   font=fnt_text,  state='normal' if isE3_30 else 'disabled')
chk_MARL2   = customtkinter.CTkCheckBox(master=frame2, text=txt_MARL2,  command = set_MARL2, font=fnt_text)
chk_RST     = customtkinter.CTkCheckBox(master=frame2, text=txt_RST,    command = set_RST,   font=fnt_text)
chk_RLY_Y   = customtkinter.CTkCheckBox(master=frame2, text=txt_RLY_Y,  command = set_RLY_Y, font=fnt_text, state='normal' if isE3_DIP else 'disabled')
chk_RLY_P   = customtkinter.CTkCheckBox(master=frame2, text=txt_RLY_P,  command = set_RLY_P, font=fnt_text)

tt_SWD      = CTkToolTip(chk_SWD,   message=txt_TTSWD,      font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_DBG      = CTkToolTip(chk_DBG,   message=txt_TTDBG,      font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_HWDBG    = CTkToolTip(chk_HWDBG, message=txt_TTHWDBG,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_SWP_X    = CTkToolTip(chk_SWP_X, message=txt_TTSWP_X,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_SWP_Y    = CTkToolTip(chk_SWP_Y, message=txt_TTSWP_Y,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_CLK      = CTkToolTip(chk_CLK,   message=txt_TTCLK,      font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_MARL2    = CTkToolTip(chk_MARL2, message=txt_TTMARL2,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_RST      = CTkToolTip(chk_RST,   message=txt_TTRST,      font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_RLY_P    = CTkToolTip(chk_RLY_P, message=txt_TTRLY_P,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_RLY_Y    = CTkToolTip(chk_RLY_Y, message=txt_TTRLY_Y,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_SWP_SE   = CTkToolTip(chk_SWP_SE,message=txt_TTSWP_SE,   font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_SPMTWI   = CTkToolTip(chk_SPMTWI,message=txt_TTSPMTWI,   font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_SPMFW    = CTkToolTip(chk_SPMFW, message=txt_TTSPMFW,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_MSTWI    = CTkToolTip(chk_MSTWI, message=txt_TTSPMTWI,   font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_SWTWI    = CTkToolTip(chk_SWTWI, message=txt_TTSWTWI,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_MSRLY    = CTkToolTip(chk_MSRLY, message=txt_TTMSRLY,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_DDE      = CTkToolTip(chk_DDE,   message=txt_TTDDE,      font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_NPX      = CTkToolTip(chk_NPX,   message=txt_TTNPX,      font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_MS       = CTkToolTip(chk_MS,    message=txt_TTMS,       font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_SPM      = CTkToolTip(chk_SPM,   message=txt_TTSPM,      font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)
tt_USBID    = CTkToolTip(lbl_USBID, message=txt_TTUSBID,    font=fnt_tooltip, bg_color=col_tooltip_bg, text_color=col_tooltip, follow=False, alpha=0.9, corner_radius=4, border_width=1)

#
# Controls layout
#
lbl_CFG.grid    (row=1, column=0, padx=(10,10),  pady=(5, 10),  sticky="we", columnspan=2)
frame1.grid     (row=3, column=0, padx=(10, 5),  pady=(10, 10), sticky="nsw")
frame2.grid     (row=3, column=1, padx=(0, 10),  pady=(10, 10), sticky="nsw")
lbl_USBID.grid  (row=4, column=0, padx=(20, 6),  pady=(0, 10),  sticky="w", columnspan=2)
btn_OK.grid     (row=5, column=0, padx=(15, 15), pady=(0, 10),  sticky="se", columnspan=2)
btn_CANCEL.grid (row=5, column=0, padx=(15, 15), pady=(0, 10),  sticky="sw", columnspan=2)

# Controls in Frame 1
lbl_DSP.grid    (row=0, column=0, padx=(10, 10), pady=(3, 6),  sticky="we")
cbo_DSP.grid    (row=1, column=0, padx=(10, 10), pady=(0, 6),  sticky="we")
chk_SWTWI.grid  (row=2, column=0, padx=(10, 10), pady=(0, 6),  sticky="we")
lbl_OPT.grid    (row=4, column=0, padx=(10, 10), pady=(6, 3),  sticky="we")
chk_DDE.grid    (row=5, column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_NPX.grid    (row=6, column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_MS.grid     (row=7, column=0, padx=(10, 10), pady=(0, 8),  sticky="we")
chk_MSTWI.grid  (row=8, column=0, padx=(40, 10), pady=(0, 8),  sticky="we")
chk_MSRLY.grid  (row=9, column=0, padx=(40, 10), pady=(0, 8),  sticky="we")
chk_SPM.grid    (row=10,column=0, padx=(10, 10), pady=(0, 8),  sticky="we")
chk_SPMTWI.grid (row=11,column=0, padx=(40, 10), pady=(0, 8),  sticky="we")
chk_SPMFW.grid  (row=12,column=0, padx=(40, 10), pady=(0, 8),  sticky="we")

# Controls in Frame 2
lbl_OTH.grid    (row=0,  column=0, padx=(0, 0),   pady=(3, 6),  sticky="we")
chk_SWD.grid    (row=1,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_DBG.grid    (row=2,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
if show_HWDBG:
    chk_HWDBG.grid  (row=3,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_SWP_X.grid  (row=4,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_SWP_Y.grid  (row=5,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_CLK.grid    (row=6,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_MARL2 .grid (row=7,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_RST.grid    (row=8,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_RLY_Y.grid  (row=9,  column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_RLY_P.grid  (row=10, column=0, padx=(10, 10), pady=(0, 10), sticky="we")
chk_SWP_SE.grid (row=11, column=0, padx=(10, 10), pady=(0, 10), sticky="we")
#===================================================

#
# Parse the settings preset in platformio.ini
#
def parse_build():

    global variant
    global usbId, usbProduct
    global relayInfo
    global dsp_last_ndx
    global isDDE
    flashOfs = ""

    cppdefines = build_flags.get("CPPDEFINES")
    cppdefines.append(dbg_option)
    cppdefines.append(zs_option)
    if isE3_30:
        cppdefines.append(mstwi_option)
    # add the defaults
    for option in cfg_defaults.split(","):
        cppdefines.append(option.strip())

    #pprint(build_flags)

    # Iterate through user defined options
    for define in cppdefines:
        if define == dsp_options[0][0]: # "USE_MINI12864_PANEL_V21"
            dsp_last_ndx = 0

        if define == dsp_options[1][0]: # "USE_CREALITY_DISPLAY"
            dsp_last_ndx = 1

        if define == dsp_options[2][1]: # "CREALITY_HW_SPI"
            dsp_last_ndx = 2

        if define == dsp_options[3]: # "USE_TWI_DISPLAY"
            dsp_last_ndx = 3

        if define == dsp_options[4]: # "USE_LEONERD_DISPLAY"
            dsp_last_ndx = 4

        if define == dsp_options[5]: # "USE_DEFAULT_DISPLAY"
            dsp_last_ndx = 5

        if define == dsp_option_names[6]: # "USE_SERIAL_DISPLAY"
            dsp_last_ndx = 6

        if define == swtwi_option:
            chk_SWTWI.select()
            chk_SWTWI.configure(state='normal')
            cfg_names.append(txt_SWTWI)
            defines.append(swtwi_option)

        if define == dde_option:
            chk_DDE.select()
            isDDE = True
            change_define(True, dde_option, txt_DDE)

        if define == npx_option:
            chk_NPX.select()
            change_define(True, npx_option, txt_NPX)

        if define == spm_option:
            chk_SPM.select()
            change_define(True, spm_option, txt_SPM)
            if not (isE3_12 or isMINI):
                chk_SPMTWI.configure(state='normal' if chk_SPM.get() else 'disabled')
                chk_SPMFW.configure(state='normal' if chk_SPM.get() else 'disabled')

        if define == spmfw_option:
            chk_SPMFW.select()

        if define == zs_option:
            chk_MS.deselect()
            chk_MSRLY.deselect()
            change_define(True, zs_option, txt_ZS)

        if define == msrly_option:
            chk_MSRLY.select()
            chk_RLY_Y.deselect()
            chk_RLY_P.deselect()
            relayInfo = txt_ONMS
            change_define(True, msrly_option, txt_MSRLY)

        if define == mstwi_option:
            chk_MSTWI.select()
            chk_SPMTWI.select()
            change_define(True, mstwi_option, txt_MSTWI)

        if define == ms_option:
            chk_MS.select()
            change_define(True, ms_option, txt_MS)
            change_define(False, zs_option, txt_ZS)
            if relayInfo != txt_ONMS:
                if isE3_DIP:
                    relayInfo = txt_ONTH0
                else:
                    relayInfo = txt_ONPS
            chk_MSRLY.configure(state='normal' if chk_MS.get() else 'disabled')
            if not (isE3_12 or isMINI):
                chk_MSTWI.configure(state='normal' if chk_MS.get() else 'disabled')

        if define == swd_option:
            chk_SWD.select()
            change_define(True, swd_option, txt_SWD)

        if define == dbg_option:
            chk_DBG.select()
            change_define(True, dbg_option, txt_DBG)

        if define == hwdbg_option:
            chk_HWDBG.select()
            change_define(True, hwdbg_option, txt_HWDBG)

        if define == marl2_option:
            chk_MARL2.select()
            change_define(True, marl2_option, txt_MARL2)

        if define == rst_option:
            chk_RST.select()
            change_define(True, rst_option, txt_RST)

        if define == swp_x_option:
            chk_SWP_X.select()
            chk_SWP_Y.deselect()
            change_define(True, swp_x_option, txt_SWP_X)
            change_define(False, swp_y_option, txt_SWP_Y)

        if define == swp_y_option:
            chk_SWP_Y.select()
            chk_SWP_X.deselect()
            change_define(True, swp_y_option, txt_SWP_Y)
            change_define(False, swp_x_option, txt_SWP_X)

        if define == swp_se_option:
            chk_SWP_SE.select()
            change_define(True, swp_se_option, txt_SWP_SE)

        if define == rly_y_option and relayInfo != txt_ONMS:
            chk_RLY_Y.select()
            chk_RLY_P.deselect()
            change_define(True, rly_y_option, txt_RLY_Y)
            change_define(False, rly_p_option, txt_RLY_P)

        if define == rly_p_option and relayInfo != txt_ONMS:
            chk_RLY_Y.deselect()
            chk_RLY_P.select()
            change_define(False, rly_y_option, txt_RLY_Y)
            change_define(True, rly_p_option, txt_RLY_P)

        if define == clk_option:
            chk_CLK.select()
            change_define(True, clk_option, txt_CLK)

        if define == maple_option:
            usbId = "LeafLabs Maple (VID/PID 1EAF:0004)"

        if define[0] == usb_option:
            usbProduct = define[1]

        if define[0] == flash_option:
            flashOfs = define[1]

        if define == v5_option:
            variant = "V5/V6"

        if define == v6s_option:
            variant = "V6S"

    # some sanity checks after parsing defines
    if dsp_last_ndx == -1:
        dsp_last_ndx = 0
    cbo_DSP.set(dsp_option_names[dsp_last_ndx])
    if type(dsp_options[dsp_last_ndx]) == list:
        for option in dsp_options[dsp_last_ndx]:
            defines.append(option)
    else:
        defines.append(dsp_options[dsp_last_ndx])
    #pprint(defines)

    if variant == "":
        print("\n{0}Invalid build configuration. SMuFF V4 or older is no longer supported!{1}\n\n".format(col_red, col_black))
        env_exit(-1)
    elif variant == "V6S":
        print("\n{0}Invalid build configuration. SMuFF V6S is no longer supported!{1}\n\n".format(col_red, col_black))
        env_exit(-1)

    if flashOfs == "":
        print("\n{0}Flash-Offset was not defined! Please check and set before compiling!{1}".format(col_red, col_black))
        env_exit(-1)

    lbl_CFG.configure(text="Build firmware for '{0}' controller".format(envbase))
    lbl_USBID.configure(text="USB-ID: {0}; {1}".format(usbId, usbProduct))
    set_MSTWI_text()

#
# Print out the major settings for this build (for debugging only)
#
def print_build():
    for define in build_flags.get("CPPDEFINES"):
        if type(define) == list:
            print("{0}: {1}".format(define[0], define[1]))
        else:
            print("{0}".format(define))

#
# Write build info to output folder
#
def write_build_info():
    #print_build()
    try:
        options = "\n  - ".join(cfg_names)
        filename = os.path.join(build_dir, pioenv, "readme-build.txt")
        with open(filename, 'w') as info:
            info.write("Options set for this build:\n")
            info.write("===========================\n\n")
            info.write("  - Display: {0}\n  - {1}\n\n".format(display, options))
            info.write("Relay-Signal:\t{0}\n".format(relayInfo))
            info.write("USB-ID:\t\t{0}  \"{1}\"".format(usbId, usbProduct))
        print("{0}Build-Info file written...{1}".format(col_cyan, col_black))
    except:
        print("{0}Failed to write Build-Info file!{1}".format(col_red, col_black))
#
# This will run as the script is called
#
parse_build()
dlg.mainloop()
write_build_info()
