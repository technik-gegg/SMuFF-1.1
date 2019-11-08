; --------------------------------------------
; SMuFF test script for random Revolver positioning
; The {RNDT} parameter gets replaced dynamically
; --------------------------------------------
M205 P"ResetBeforeFeed" S0
M205 P"HomeAfterFeed" S0
M205 P"EmulatePrusa" S1
G28 Y
G4 S1
G0 Y0
G4 S1
G0 Y1
G4 S1
G0 Y2
G4 S1
G0 Y3
G4 S1
G0 Y4
G4 S1
G28 Y
G4 S1
G0 Y{RNDT}
G4 S1
G0 Y{RNDT}
G4 S1
G0 Y{RNDT}
G4 S1
G0 Y{RNDT}
G4 S1
G0 Y{RNDT}
G4 S1
