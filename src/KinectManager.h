//
//  KinectManager.h
//  KinectDepthImageSmoother
//
//  Created by David Haylock on 17/01/2017.
//
//

#ifndef KinectManager_h
#define KinectManager_h
#include "ofxCv.h"
#include "ofxKinect.h"

using namespace cv;
using namespace ofxCv;
class KinectManager
{
public:
	
	//----------------------------------------------------------
	void setup()
	{
		kinect.setRegistration(false);
		kinect.init(true,true,true);
		kinect.setDepthClipping(50,5000);
		kinect.open();
		
		if(kinect.isConnected())
		{
			ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
			ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
			ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
			ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
		}
		
		kinect.setCameraTiltAngle(5);
		framecount = 0;
	}
	
	//----------------------------------------------------------
	void update()
	{
		kinect.update();
		if (kinect.isConnected())
		{
			if (kinect.isFrameNewDepth())
			{
				kinectImage = Mat(kinect.height,kinect.width,CV_8UC1,kinect.getDepthPixels(),0);
				
				if (framecount < 300)
				{
					if (framecount == 0)
					{
						unsigned char zero = 0;
						backgroundModel = Mat(kinectImage.rows, kinectImage.cols, CV_8UC1);
						for (int y = 0; y < kinectImage.rows; y++)
						{
							for (int x = 0; x < kinectImage.cols; x++)
							{
								backgroundModel.at<uchar>(y,x) = zero;
							}
						}
					}
					for (int y = 0; y < kinectImage.rows; y++)
					{
						for (int x = 0; x < kinectImage.cols; x++)
						{
							unsigned char backPixel = backgroundModel.at<uchar>(y, x);
							unsigned char currentPixel = kinectImage.at<uchar>(y, x);
							
							if (currentPixel == 0)
							{
								
							}
							else if (backPixel == 0)
							{
								backPixel = currentPixel;
							}
							else if (currentPixel < backPixel)
							{
								backPixel = currentPixel;
							}
							
							backgroundModel.at<uchar>(y, x) = backPixel;
						}
					}
					
					if (framecount == 299)
					{
						backgroundModel = backgroundModel + Scalar(6);
					}
					framecount++;
				}
				else {
					for (int y = 0; y < kinectImage.rows; y++)
					{
						for (int x = 0; x < kinectImage.cols; x++)
						{
							unsigned char backPixel = backgroundModel.at<uchar>(y, x);
							unsigned char currentPixel = kinectImage.at<uchar>(y, x);
							if (currentPixel > _nearThreshold)
							{
								currentPixel = 0;
							}
							else if (currentPixel < _farThreshold)
							{
								currentPixel = 0;
							}
							else if (currentPixel < backPixel)
							{
								currentPixel = 0;
							}
							
							kinectImage.at<uchar>(y, x) = currentPixel;
						}
						
					}
					
					foreground = kinectImage - backgroundModel;
					cv::blur(foreground,foreground,cv::Size(13,13));
					threshold(foreground,100);
					cv::blur(foreground,foreground,cv::Size(5,5));
					contourFinder.findContours(foreground);
				}
			}
		}
	}

	//----------------------------------------------------------
	void draw()
	{
		ofPushStyle();
		ofSetColor(255);
		kinect.drawDepth(0, 0, getKinectWidth()/2, getKinectHeight()/2);
		ofxCv::drawMat(backgroundModel, getKinectWidth()/2, 0, getKinectWidth()/2, getKinectHeight()/2);
		ofxCv::drawMat(kinectImage, 0, getKinectHeight()/2, kinect.getWidth()/2, getKinectHeight()/2);
		ofxCv::drawMat(foreground, getKinectWidth()/2, getKinectHeight()/2, getKinectWidth()/2, getKinectHeight()/2);
		
		ofPushMatrix();
		ofScale(0.5,0.5);
		ofSetColor(ofColor::red);
		contourFinder.draw();
		
		ofPopMatrix();
		
		ofPushMatrix();
		ofTranslate(getKinectWidth(),-getKinectHeight()/2);
		ofSetColor(_farThreshold);
		ofDrawRectangle(0, getKinectHeight()/2, 25, 25);
		ofDrawBitmapStringHighlight("Far Threshold "+ ofToString(_farThreshold), 35, getKinectHeight()/2+15);
		ofSetColor(_nearThreshold);
		ofDrawRectangle(0, kinect.getHeight()/2+25, 25, 25);
		ofDrawBitmapStringHighlight("Near Threshold "+ ofToString(_nearThreshold), 35, getKinectHeight()/2+15+30);
		ofDrawBitmapStringHighlight("Frames Captured " + ofToString(framecount), 35, getKinectHeight()/2+15+60);
		ofPopMatrix();
		
		
		ofDrawBitmapStringHighlight("Raw Depth", 10, 10);
		ofDrawBitmapStringHighlight("Kinect Image", 10, 10+getKinectHeight()/2);
		ofDrawBitmapStringHighlight("Background Model", 10+getKinectWidth()/2, 10);
		ofDrawBitmapStringHighlight("Foreground", 10+getKinectWidth()/2, 10+getKinectHeight()/2);
		ofPopStyle();
	}
	
	//----------------------------------------------------------
	void exit()
	{
		kinect.close();
	}
	
	//----------------------------------------------------------
	void setNearThreshold(int nearThreshold)
	{
		_nearThreshold = nearThreshold;
	}
	
	//----------------------------------------------------------
	void setFarThreshold(int farThreshold)
	{
		_farThreshold = farThreshold;
	}
	
	//----------------------------------------------------------
	void setKinectAngle(int angle)
	{
		kinect.setCameraTiltAngle(angle);
	}
	
	//----------------------------------------------------------
	int getKinectWidth()
	{
		return kinect.getWidth();
	}
	
	//----------------------------------------------------------
	int getKinectHeight()
	{
		return kinect.getHeight();
	}
	
	//----------------------------------------------------------
	void decreaseFarThreshold()
	{
		_farThreshold--;
	}
	
	//----------------------------------------------------------
	void increaseFarThreshold()
	{
		_farThreshold++;
	}
	
	//----------------------------------------------------------
	void increaseNearThreshold()
	{
		_nearThreshold++;
	}
	
	//----------------------------------------------------------
	void decreaseNearThreshold()
	{
		_nearThreshold--;
	}
	//----------------------------------------------------------
	void resetModel()
	{
		framecount = 0;
	}
	
private:
	ofxKinect kinect;
	cv::Mat kinectImage,backgroundModel,foreground;
	ofxCv::ContourFinder contourFinder;
	
	int _nearThreshold;
	int _farThreshold;
	int framecount;
	
	
protected:
	
};


#endif /* KinectManager_h */
