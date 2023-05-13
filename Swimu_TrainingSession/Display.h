#ifndef _Display_h_
#define _Display_h_

#include <Arduino.h>
#include <Print.h>

#define POSITION_LINE_1 0
#define POSITION_LINE_2 8
#define POSITION_LINE_3 16
#define POSITION_LINE_4 24

boolean initializeDisplay();

Print* getDisplay();

void updateDisplay(int timeout=0);

void clearDisplay(boolean update=false);

void showMessage(boolean update=false, const char* sFmt="", ...);

void showMessageAtLine(int lineNumber=POSITION_LINE_1, boolean update=false, const char* sFmt="", ...);

#endif	// _Display_h_
