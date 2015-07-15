#include "ofMain.h"
#include "testApp.h"

#define DISPLAY_WIDTH 1680
#define DISPLAY_HEIGHT 1050

#define PROJECTOR_WIDTH 1024
#define PRFOJECTOR_HEIGHT 768

//========================================================================
int main( ){
	ofSetupOpenGL(DISPLAY_WIDTH+PROJECTOR_WIDTH,DISPLAY_HEIGHT,OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new testApp());

}
