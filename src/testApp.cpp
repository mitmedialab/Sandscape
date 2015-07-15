#include "testApp.h"
#include "ofAppRunner.h"

#define DISPLAY_WIDTH 1680
#define DISPLAY_HEIGHT 1050

//--------------------------------------------------------------
void testApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
    
    // allocate memory in ofImages
    colorImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    grayImage.allocate(kinect.width, kinect.height);
    
    grayOverall.allocate(kinect.width, kinect.height);
    grayOverall.set(1);
    
    grayImage_avg.allocate(kinect.width, kinect.height);
    grayImage_avg.set(1);
    
    frame.allocate(1000, 1000, OF_IMAGE_COLOR);
    
    // get current time
    startTimeout = ofGetElapsedTimef();
    
    // load 20x1 pixel image for color gradients
    if(!gradient.loadImage("gradient.png")) {
        ofLog(OF_LOG_ERROR, "Error while loading image");
    }
    
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    
    
    //store the width and height for convenience
	int width = colorImg.getWidth();
	int height = colorImg.getHeight();
    
    int TransX = -100;
    int TransY = 0;
	
	//add one vertex to the mesh for each pixel
	for (int y = height-1; y > -1; y--){
		for (int x = 0+TransX; x<width+TransX; x++){
			mainMesh.addVertex(ofPoint(x,y,0));	// mesh index = x + y*width
            // this replicates the pixel array within the camera bitmap...
			mainMesh.addColor(ofFloatColor(0,0,0,0));  // placeholder for colour data, we'll get this from the camera
		}
	}
	
    ofEnableAlphaBlending();
    
	for (int y = 0; y<height-1; y++){
		for (int x=0; x<width-1; x++){
			mainMesh.addIndex(x+y*width);				// 0
			mainMesh.addIndex((x+1)+y*width);			// 1
			mainMesh.addIndex(x+(y+1)*width);			// 10
			
			mainMesh.addIndex((x+1)+y*width);			// 1
			mainMesh.addIndex((x+1)+(y+1)*width);		// 11
			mainMesh.addIndex(x+(y+1)*width);			// 10
		}
	}
    //easycam.setScale(1,-1,1);
	
	
	//this determines how much we push the meshes out
	extrusionAmount = 50.0;
    
    
    
    ofVec3f lookAtTarget;
    lookAtTarget.x= grayImage.width*0.39;
    lookAtTarget.y=  grayImage.height*0.45;
    lookAtTarget.z= 0;
    
    
    //easycam.setGlobalPosition(100, 100, 0);
    //easycam.setDistance(100);
    easycam.setTarget(lookAtTarget);
    
    easycam.rotate(90+30,easycam.getUpDir());
    easycam.rotate(-60,easycam.getXAxis());
    easycam.rotate(0,easycam.getYAxis());
    easycam.rotate(139,easycam.getZAxis());
    
    easycam.setDistance(400);
    
    
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    //cout << easycam.getUpDir() << ", x = " << easycam.getXAxis() << ", y = " << easycam.getYAxis() << ", z = " << easycam.getZAxis() <<endl;
    ofSetWindowPosition(0, 0);
    ofBackground(0, 0, 0);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        // add effects to smooth signal and reduce noise
        grayImage.blur(7);
        grayImage.dilate();

        /*
        //for (int i=0; i < nearThreshold- farThreshold; i+=1) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(farThreshold + 1 + 1, true);
            grayThreshFar.threshold(farThreshold + 1);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        
            cvAddWeighted(grayImage_avg.getCvImage(), .51, grayImage.getCvImage(), .49, 0.0, grayImage_avg.getCvImage());
            cvSmooth(grayImage_avg.getCvImage(), grayImage_avg.getCvImage(), CV_GAUSSIAN);
        
            contourFinder.findContours(grayImage_avg, 200, (340*240)/3, 10, true);
        //}
        
        
        //cvAnd(grayImage.getCvImage(), grayOverall.getCvImage(), grayOverall.getCvImage(), NULL);
        
        */
        
        /*
         * Edit data through pixel manipulation
         */
        
        
        unsigned char * pix = grayImage.getPixels();

        // draw image
        int diffThreshold = nearThreshold - farThreshold;
        
        // iterate through pixel data
        for(int w = 0; w < grayImage.getWidth(); w++) {
            for(int h=0; h < grayImage.getHeight(); h++) {
                
                // average previous pixels with current reading
                int index = h * int(grayImage.getWidth()) + w;
                double currentDepth = pix[index];
                double prevDepth = prevPix[index];
                prevPix[index] = prevDepth* .90 + currentDepth * .1;
                double  depth = prevPix[index];
                
                // boolean operations if inside sandbox boundaries and depth boundaries
                bool isInBoundary = w<xRightBoundary && w>xLeftBoundary && h<yBottomBoundary && h>yTopBoundary;
                bool isInDepthBoundary = currentDepth < nearThreshold && currentDepth > farThreshold;
                
                // set Timeout zone
                int lowerBoundary = 210;
                int upperBoundary = 400;
                bool isInActiveZone = currentDepth<upperBoundary && currentDepth>lowerBoundary;
                
                if (isInBoundary && isInActiveZone) {
                    isTimeout = false;
                    startTimeout = ofGetElapsedTimef();
                }
                if (ofGetElapsedTimef() == startTimeout + timeout) {
                    isTimeout = true;
                }
                
                // set pixels of colorImg based on depth ratio
                if( isInDepthBoundary && isInBoundary && !isTimeout) {
                    double diffToThreshold = depth - farThreshold;
                    double ratioInDepth = diffToThreshold/diffThreshold;
                    
                    
                    
                    if(ratioInDepth>0.95){
                        ratioInDepth = 0.95;
                    } else if (ratioInDepth<0){
                        ratioInDepth = 0;
                    }
                    ofColor color = gradient.getColor(floor(ratioInDepth * 20), 0);
                    
                    
                    colorImg.setColor(w,h, color);
                    
                    ofVec3f tmpVec = mainMesh.getVertex(h*grayImage.width+w);
                    tmpVec.z = ratioInDepth * extrusionAmount;
                    mainMesh.setVertex(h*grayImage.width+w, tmpVec);
                    
                    
                }

            }
        }
        
        colorImg.update();
        
        
		// update the cv images
		grayImage.flagImageChanged();
    }
    
    for (int i=0; i<colorImg.getWidth()*colorImg.getHeight(); i++){
    ofFloatColor sampleColor(colorImg.getPixels()[i*3]/255.f,				// r
                             colorImg.getPixels()[i*3+1]/255.f,			// g
                             colorImg.getPixels()[i*3+2]/255.f);			// b
    
    //now we get the vertex aat this position
    //we extrude the mesh based on it's brightness
    
    
    mainMesh.setColor(i, sampleColor);
    }
    
    //let's move the camera when you move the mouse
	float rotateAmount = ofMap(ofGetMouseY(), 0, ofGetHeight(), 0, 360);
    
	
	//move the camera around the mesh
	ofVec3f camDirection(0,0,1);
	ofVec3f centre(grayImage.width*0.39,grayImage.height*0.45, 255/2.f);
	ofVec3f camDirectionRotated = camDirection.rotated(rotateAmount, ofVec3f(1,0,0));
	ofVec3f camPosition = centre + camDirectionRotated * extrusionAmount;
	
    
	cam.setPosition(camPosition);
	cam.lookAt(centre);
    
    //cam.setGlobalPosition(ofGetWidth()/2, ofGetWidth()/2, 0);
    
  //  cam.
         
    

}

//--------------------------------------------------------------
void testApp::draw() {
    
    
    
    /*
    ofSetColor(255, 0, 0);
    grayImage_avg.draw(10,10);


    
    for (int i = 0; i < contourFinder.nBlobs; i++){
        contourFinder.blobs[i].draw(10,10);
    }
     */
    

    
    //but we want to enable it to show the mesh
	ofEnableDepthTest();
    
    
    //ofTranslate(-1000,-500);
	//cam.begin();
    
    //
    easycam.begin();
    
    /*
    ofPushMatrix();
    ofTranslate(grayImage.width*0.39, grayImage.height*0.45);
    
    ofSetColor(255,0,0);
	ofFill();
	ofDrawBox(30);
	ofNoFill();
	ofSetColor(0);
	ofDrawBox(30);
	
	ofPushMatrix();
	ofTranslate(0,0,20);
	ofSetColor(0,0,255);
	ofFill();
	ofDrawBox(5);
	ofNoFill();
	ofSetColor(0);
	ofDrawBox(5);
	ofPopMatrix();
    
    ofPopMatrix();
     */
    
	//You can either draw the mesh or the wireframe
	// mainMesh.drawWireframe();
	mainMesh.drawFaces();
    
    
    
    
    
    easycam.end();
	//cam.end();
    
    
    frame.grabScreen(ofGetWidth()/2-500,ofGetHeight()/2-500,1000,1000);
    frame.update();
    
    
    ofFill();
    ofSetColor(0);
    //ofRect(DISPLAY_WIDTH/2+750, 0, 200, DISPLAY_HEIGHT);
    
    
    ofNoFill();
    ofSetColor(255);
    frame.draw(DISPLAY_WIDTH/2-850,DISPLAY_HEIGHT/2-850-100,1700,1700);
    
    
    // enlarge image by multiplier
    double multiplier = 2.0;
    
    double width = colorImg.width * multiplier;
    double height = colorImg.height * multiplier;
    
    ofDisableDepthTest();
    ofPushMatrix();
    ofTranslate(width/2, height/2);
    
    // 2 degrees to match the projector angle
    ofRotate(180 + 2);
    ofPushMatrix();
    int offsetX = -1575-63;
    int offsetY = 290-60;
    ofTranslate(-width/2 + offsetX, -height/2 + offsetY);
        
    colorImg.draw(0, 0, width, height);
    ofPopMatrix();
    ofPopMatrix();
}

//--------------------------------------------------------------
    void testApp::exit() {
        kinect.setCameraTiltAngle(0); // zero the tilt on exit
        kinect.close();
    }
//--------------------------------------------------------------
void testApp::keyPressed(int key){
    if (key == 'f'){
        ofToggleFullscreen();
    } else if (key == 's'){
        
        
        ofVec3f lookAtTarget;
        lookAtTarget.x= grayImage.width/2;
        lookAtTarget.y= grayImage.height/2;
        lookAtTarget.z= 0;
        
        
        //easycam.setGlobalPosition(100, 100, 0);
        //easycam.setDistance(100);
        easycam.setTarget(lookAtTarget);
    } else if (key == 'p'){
        ofLogNotice() << "test1: "<<easycam.getTarget().getX()<<","<<easycam.getTarget().getY()<<","<<easycam.getTarget().getZ();
        ofLogNotice() << "test2: "<<easycam.getDistance();
        
    }
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
