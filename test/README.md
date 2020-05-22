# SMuFF Test GCode Scripts

These test scripts may be used for initial testing of the SMuFF after you built it, to ensure the build has no mechanical or electrical issues.
To use these scripts, simply copy them to the root folder of your SD-Card for the SMuFF.
In the Main Menu on the SMuFF, you'll find a menu item called "Testrun". Select this and you'll get a list of the script files located in the root folder.
Pick one to start the test.

Feel free to modify the test scripts and adopt them to your needs. Test scripts are GCode scripts, which will get parsed and interpreted by the SMuFF at run-time.
Each GCode command has to be put in a separate line. Comments start with a semicoln (;) and will be ignored by the interpreter.
For randomizing tools, you may place a *{RNDT}* macro somewhere in the GCode (i.e. T{RNDT}). The test run function will then generate a random number between 0 and the tools defined in the configuration file.
