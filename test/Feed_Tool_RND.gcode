; --------------------------------------------
; SMuFF test script for random tools selects
; The {RNDT} parameter gets replaced dynamically
; --------------------------------------------
M205 P"EmulatePrusa" S1
G28
G4 S1
T{RNDT}
C0
G4 S1
T{RNDT}
C0
G4 S1
U0
