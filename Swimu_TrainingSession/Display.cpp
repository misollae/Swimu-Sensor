#include "Display.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdarg.h>
#include <Wire.h>

#define ADDRESS_DISPLAY 0x3c

// Screen size in pixels
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   32

#define OLED_RESET      -1

#define FontSize 1

static Adafruit_SSD1306 display( SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET );

static boolean displayReady;

boolean initializeDisplay() {
	if ( !display.begin(SSD1306_SWITCHCAPVCC, ADDRESS_DISPLAY) ) {
		displayReady = false;
	}
	else {
		display.display();
		
		display.setTextSize( FontSize );
		display.setTextColor( WHITE );
		
		displayReady = true;
	}
	
	return displayReady;
}

void clearDisplay(boolean update) {
    if ( displayReady ) {
		display.clearDisplay();
		display.setCursor( 0, POSITION_LINE_1 );
		
		if ( update==true ) {
			display.display();
		}
    }
}

void updateDisplay(int timeout) {
    if ( displayReady ) {
        display.display();

        if ( timeout>0 ) {
            delay( timeout );
        }
    }
}

void showMessage(boolean update, const char* sFmt, ...) {
	if ( displayReady ) {
		char msgBuf[256];
		
		va_list args;
		
		va_start( args, sFmt );
		vsprintf( msgBuf, sFmt, args );
		va_end( args );
		
		((Print*)( &display ))->print( msgBuf );

        if ( update ) {
            display.display();
        }
	}
}

void showMessageAtLine(int lineNumber, boolean update, const char* sFmt, ...) {
    if ( displayReady ) {
        char msgBuf[256];
        
        va_list args;
        
        va_start( args, sFmt );
        vsprintf( msgBuf, sFmt, args );
        va_end( args );

        display.setCursor( 0, lineNumber );
        display.setTextColor( WHITE, BLACK );
        ((Print*)( &display ))->print( "                     " );

        display.setCursor( 0, lineNumber );
        display.setTextColor( WHITE );
        ((Print*)( &display ))->print( msgBuf );

        if ( update ) {
            display.display();
        }
    }
}

Print* getDisplay() {
    if ( displayReady ) {
        return (Print*)( &display );
    }

    return NULL;
}
