# Tunes

This folder contains tunes (sounds), which are used by the SMuFF firmware.
Copy those tunes onto the SD-Card in order to be able to modify those sounds.

The content of such a tune file is for expample:

```

F440 D120 P150.F523 D120 P80.F196 D220 P80.F196 D120 P80.F587 D400 P120.F349 D80 P240.
```

It consists of a list of tones (up to 500 characters) represented by **F** as the tone frequency, **D** as the tone duration **P** the pause after the tone has played and the **.** (dot) as a delimiter between tones.
The **P** parameter may be left out but the dot (.) is mandantory for the tone to be played!

The naming used for the files is:

+ TUNE    - is the startup melody
+ BEEP    - is the normal, single beep tone
+ LBEEP   - is a longer beep tone, used to signal errors
+ UBEEP   - is a short sequence to signal user attention is required
+ EBEEP   - is the encoder tick tone

TUNE_OLD and TUNE_SW are variations of the startup tune. Later is for StarWars fans only!
