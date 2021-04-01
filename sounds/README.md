# Tunes

This folder contains tunes (sounds), which are used by the SMuFF firmware.
Copy those tunes into the SD-Cards *sounds* folder in order to be able to modify those sounds.

The content of such tune file looks like this:

```text

F440 D120 P150. F523 D120 P80. F196 D220 P80. F196 D120 P80. F587 D400 P120.F349 D80 P240.
```

It's a plain text file which consists of a list of tones (up to 150 characters), represented by **F** as the tone frequency (in Hz), **D** as the tone duration (in mS), **P** as the pause (in mS) after the tone has played and the period ( **.** ) as a delimiter between tones.
The **P**(ause) parameter can be left out but the period ( . ) is **mandantory** for the tone to be played!
Spaces in between can be placed for better readability and may be omitted.

If you'd like to compose your own melodie/sound, use the **M300** GCode command to test it out.

The naming scheme used for these files is hard coded in the firmware (extension has to be *.DAT*):

+ STARTUP - is the startup melody
+ BEEP - is the default single beep tone
+ LBEEP - is a longer beep tone, used to signal errors
+ UBEEP - is a short sequence signaling user attention is required
+ EBEEP - is the encoder tick tone

**STARTUP_NEW** and **STARTUP_SW** are variations of the startup tune - later is for StarWars fans only! (copy over to STARTUP.DAT to use one of these instead).

**EBEEP_LEONERD** is a modified encoder tick tone for LeoNerd's OLED Module (copy over to EBEEP.DAT to use this one instead).
