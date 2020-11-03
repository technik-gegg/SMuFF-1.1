# SMuFF Test GCode Scripts

These test scripts may be used for initial testing of the SMuFF after you built it, to ensure the build has no mechanical or electrical issues.
To use these scripts, simply copy them to the root folder of your SD-Card for the SMuFF.
In the Main Menu on the SMuFF, you'll find a menu item called **Testrun**. Select this and you'll get a list of the script files located in the test folder of your SD-Card.
Pick one to start the test.

Feel free to modify the test scripts to adopt them to your needs. Test scripts are GCode scripts, which will get parsed and interpreted by the SMuFF at runtime.
Each GCode command has to be put in a separate line. Comments start with a semicolon ( ; ) and will be ignored by the interpreter.

For randomizing tools, you may place a *{RNDT}* macro somewhere in the GCode (i.e. T{RNDT}). The test run function will then generate a random number between 0 and the number of tools defined in the configuration file, trying to avoid using the same tool twice.

The **report.txt** file contains the text that will be sent to the log serial (usually USB-Serial), in order to display the results while the test is running.
This file is set up to display a formatted output on a **VT100** compatible terminal (program). To achive this, it utilizes VT100 ESC sequences for coloring and cursor positioning. If needed, you can use any of the available VT100 ESC sequences. Look them up [here](http://ascii-table.com/ansi-escape-sequences-vt-100.php).
Feel free to modify the reports.txt file in order to get the output you want.

In this file you'll also find a couple of macros, defining which information to print where. Those macros will get replaced by the real values at runtime:

- {TIME}    - prints the elapsed testing time
- {GCO}     - prints the current GCode executed
- {ERR}     - prints the error count (failed feeds)
- {LOOP}    - prints the loop count
- {CMDS}    - prints the overall commands processed count
- {TC}      - prints the tool changes count
- {STALL}   - prints the overall stall count (only on TMC 2209 drivers)
- {HIT}     - prints the hit count of feeder endstop 2
- {MISS}    - prints the missed count of feeder endstop 2