#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetFrameRate(30);
	ofBackground(0);
	
	kinectManager.setup();
	kinectManager.setFarThreshold(128);
	kinectManager.setNearThreshold(255);
}
//--------------------------------------------------------------
void ofApp::update()
{
	kinectManager.update();
}
//--------------------------------------------------------------
void ofApp::draw()
{
	kinectManager.draw();
}
//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
	switch(key) {
		case '<':
			kinectManager.decreaseFarThreshold();
			break;
		case ',':
			kinectManager.decreaseNearThreshold();
			break;
		case '>':
			kinectManager.increaseFarThreshold();
			break;
		case '.':
			kinectManager.increaseNearThreshold();
			break;
		case 'r':
			kinectManager.resetModel();
			break;
		default:
			break;
	}
}
//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
	
}
//--------------------------------------------------------------
void ofApp::exit()
{
	kinectManager.exit();
}
