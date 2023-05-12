/*****************************************************************************************
Locates ArUco computer vision markers in a webcam image. 

    - Grabs frames using OpenCV webcam capture
	- Finds markers in the webcam image frames
	- Reconstructs the camera's location relative to the marker
	- Writes the camera location and orientation for use by navigation
	- Periodically saves an image to vidcap.jpg and vidcaps/<date>.jpg


With Logitech C920, at 640x480, a 305mm marker at 7m distance detects reliably, but does flip inside and out.
    ./camera --gui --cam 0 --res 640x480
Time detection=11.5047 milliseconds
Marker 2: Camera -2.554 5.488 -0.271 meters, heading 149.7 degrees
Time detection=11.5054 milliseconds
Marker 2: Camera 4.570 5.474 0.255 meters, heading -144.2 degrees



With Genius 120 FOV wide angle, at 720p, a 305mm marker at 5m distance is not readable (blurred).
    ./camera --gui --cam 0 --res 1280x720


Originally based on:
ArUco example Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "aruco.h"
#include "cvdrawingutils.h"
#include "errno.h"

#include "aurora/lunatic.h"

using namespace cv;
using namespace aruco;
using namespace std;


// Store info about how this marker is scaled, positioned and oriented
struct marker_info_t {
	int id; // marker's ID, from 0-1023 
	float true_size; // side length, in meters, of black part of pattern
	
	float x_shift; // translation from origin, in meters, of center of pattern
	float y_shift; 
	float z_shift; 
	
	float angle; // orientation, in degrees, relative to field up
};

const static marker_info_t marker_info[]={
	{-1,0.305}, // fallback default case
	
/* 2023-05 Full scale vinyl sticker on corrugated plastic */
	{2, 0.305, 1.0,0.0,0.0,   0.0 },
	{17,0.305, 4.0,0.0,0.0,   0.0 },

};

// Look up the calibration parameters for this marker
const marker_info_t &get_marker_info(int id) {
	for (int i=1;i<sizeof(marker_info)/sizeof(marker_info_t);i++) {
		if (marker_info[i].id==id) 
			return marker_info[i];
	}
	return marker_info[0];
}





bool showGUI=false;
Mat TheInputImage,TheInputImageCopy;




/* Keep the webcam from locking up when you interrupt a frame capture.
http://lawlorcode.wordpress.com/2014/04/08/opencv-fix-for-v4l-vidioc_s_crop-error/ */
volatile int quit_signal=0;
#ifdef __unix__
#include <signal.h>
extern "C" void quit_signal_handler(int signum)
{
	if (quit_signal!=0) exit(0); // just exit already
	quit_signal=1;
	printf("Will quit at next camera frame (repeat to kill now)\n");
}
#endif


/**
  Silly hack to detect camera disconnections.
  When you unplug a video camera, it's actually detected right in:
    opencv/modules/highgui/src/cap_libv4l.cpp
  when the VIDIOC_DQBUF ioctl fails.  However, this function 
  somehow fails to correctly report the problem via the error reporting
  return code.  It does log it using perror, so I'm hooking the global perror
  to figure out if this is what went wrong, and exit appropriately.
*/
extern "C" void perror(const char *str) {
	int e=errno;
	std::cout<<"perror errno="<<e<<": "<<str<<"\n";
	if (e==ENODEV) {
		std::cout<<"ERROR!  Camera no longer connected!\n";
		std::cerr<<"ERROR!  Camera no longer connected!\n";
		exit(1);
	}
}

#include "location_binary.h"

/**
  Convert 3D position to top-down 2D onscreen location
*/
cv::Point2f to_2D(const Marker &m,float x=0.0,int xAxis=2,int yAxis=0)
{
	// Extract 3x3 rotation matrix
	Mat Rot(3,3,CV_32FC1);
	Rodrigues(m.Rvec, Rot); // euler angles to rotation matrix

	cv::Point2f ret;
	const marker_info_t &mi=get_marker_info(m.id);
	float scale=mi.true_size*70; // world meters to screen pixels
	ret.x=scale*(m.Tvec.at<float>(xAxis,0)+x*Rot.at<float>(xAxis,xAxis));
	ret.y=scale*(m.Tvec.at<float>(yAxis,0)+x*Rot.at<float>(yAxis,xAxis));
	
	printf("Screen point: %.2f, %.2f cm\n",ret.x,ret.y);
	ret.y+=240; // approximately centered in Y
	return ret;
}

/**
 Draw top-down image of reconstructed location of marker.
*/
void draw_marker_gui_2D(Mat &img,Scalar color,const Marker &m)
{
	int lineWidth=2;
	
	cv::line(img,
		to_2D(m,-0.5),to_2D(m,0.0),
		color,lineWidth,cv::LINE_AA);
	cv::line(img,
		to_2D(m,0.0),to_2D(m,+0.5),
		Scalar(255,255,255)-color,lineWidth,cv::LINE_AA);
}


// Extract a row of this OpenCV 4x4 matrix, as a 3D vector
vec3 extract_row(const cv::Mat &matrix4x4,int row) {
    return vec3(
        matrix4x4.at<float>(0,row),
        matrix4x4.at<float>(1,row),
        matrix4x4.at<float>(2,row)
    );
}

/* Extract location data from this valid, detected marker. 
   Does not modify the location for an invalid marker.
*/
void extract_location(location_binary &bin,const Marker &marker)
{
	const marker_info_t &mi=get_marker_info(marker.id);

	// Extract 3x3 rotation matrix
	Mat Rot(3,3,CV_32FC1);
	Rodrigues(marker.Rvec, Rot); // euler angles to rotation matrix

	// Full 4x4 output matrix:
	Mat full(4,4,CV_32FC1);

	// Copy rotation 3x3
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			full.at<float>(i,j)=Rot.at<float>(i,j);
	
	// Copy translation vector
	full.at<float>(0,3)=marker.Tvec.at<float>(0,0);
	full.at<float>(1,3)=marker.Tvec.at<float>(1,0);
	full.at<float>(2,3)=marker.Tvec.at<float>(2,0);

	// Final row is identity (nothing happening on W axis)
	full.at<float>(3,0)=0.0;
	full.at<float>(3,1)=0.0;
	full.at<float>(3,2)=0.0;
	full.at<float>(3,3)=1.0;


	// Invert, to convert marker-from-camera into camera-from-marker
	Mat back=full.inv();

if (false) {
	// Splat to screen, for debugging
	for (int i=0; i<4; i++) {
		for (int j=0; j<4; j++)
			printf("%.2f	",back.at<float>(i,j));
		printf("\n");
	}
}

/*
  FIXME: sanity checking, single unified header with main vision/ code.
*/

	double scale=mi.true_size;
	
    vec3 O = extract_row(full,3)*scale;
    vec3 X = extract_row(full,0);
    vec3 Y = extract_row(full,1);
    vec3 Z = extract_row(full,2);
    if (dot(cross(X,Y),Z)<0.0) {
        // Y axis is facing away from camera--flip it forward again
        printf("--FLIP DETECTED---------\n\n");
    }
	
	bin.valid=1;
	bin.x=back.at<float>(0,3)*scale+mi.x_shift;
	bin.y=back.at<float>(1,3)*scale+mi.y_shift;
	bin.z=back.at<float>(2,3)*scale+mi.z_shift;
	float ang_rad=atan2(back.at<float>(1,0),-back.at<float>(0,0));
	float ang_deg=180.0/M_PI*ang_rad;
	bin.angle=ang_deg + mi.angle;
	bin.marker_ID=marker.id;

	// Print grep-friendly output
	printf("Marker %d: Camera %.3f %.3f %.3f meters, heading %.1f degrees\n",
	       marker.id, bin.x,bin.y,bin.z,bin.angle
	      );
}



int main(int argc,char **argv)
{
	try {
	string TheInputVideo;
	string TheIntrinsicFile;
	int camNo=1;
	float TheMarkerSize=-1;
	int ThePyrDownLevel=0;
	VideoCapture vidcap;
	vector<Marker> TheMarkers;
	CameraParameters cam_param;
	float minSize=0.02; // fraction of frame, minimum size of rectangle
	pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
	int skipCount=1; // only process frames ==0 mod this
	int skipPhase=0;

	int wid=1280, ht=720;
	const char *dictionary="TAG25h9";
	for (int argi=1; argi<argc; argi++) {
		if (0==strcmp(argv[argi],"--gui")) showGUI=true;
		else if (0==strcmp(argv[argi],"--res")) sscanf(argv[++argi],"%dx%d",&wid,&ht);
		else if (0==strcmp(argv[argi],"--skip")) sscanf(argv[++argi],"%d",&skipCount);
		else if (0==strcmp(argv[argi],"--cam")) camNo=atoi(argv[++argi]);		
		else if (0==strcmp(argv[argi],"--min")) sscanf(argv[++argi],"%f",&minSize);
		else printf("Unrecognized argument %s\n",argv[argi]);
	}

	//read from camera
	vidcap.open(camNo);
	
	if (wid) vidcap.set(cv::CAP_PROP_FRAME_WIDTH, wid);
	if (ht)  vidcap.set(cv::CAP_PROP_FRAME_HEIGHT, ht);

	//check video is open
	if (!vidcap.isOpened()) {
		cerr<<"Could not open video"<<endl;
		return -1;
	}

	TheIntrinsicFile="camera.yml";

	//read first image to get the dimensions
	vidcap>>TheInputImage;

	//read camera parameters if passed
	if (TheIntrinsicFile!="") {
		cam_param.readFromXMLFile(TheIntrinsicFile);
		cam_param.resize(TheInputImage.size());
	}
	//Configure other parameters
	aruco::MarkerDetector::Params params;
	
//	if (ThePyrDownLevel>0)
//		params.pyrDown(ThePyrDownLevel);
	params.setCornerRefinementMethod(aruco::CORNER_SUBPIX); // more accurate
//	params.setCornerRefinementMethod(aruco::CORNER_LINES); // more reliable?
	params.setDetectionMode(aruco::DM_FAST,0.1); // for distant/small markers (smaller values == smaller markers, but slower too)
	MarkerDetector MDetector(dictionary); // dictionary of tags recognized

	if (showGUI) {
		//Create gui
		cv::namedWindow("in",1);
	}

#ifdef __unix__
	signal(SIGINT,quit_signal_handler); // listen for ctrl-C
#endif
	unsigned int framecount=0;
	uint32_t vidcap_count=0;

	//capture until press ESC or until the end of the video
	while (vidcap.grab()) {
		if (!vidcap.retrieve( TheInputImage) || !vidcap.isOpened()) {
			std::cout<<"ERROR!  Camera "<<camNo<<" no longer connected!\n";
			std::cerr<<"ERROR!  Camera "<<camNo<<" no longer connected!\n";
			exit(1);
		}
		if (quit_signal) exit(0);

		// Skip frames (do no processing)
		skipPhase=(skipPhase+1)%skipCount;
		if (skipPhase!=0) continue;

		double tick = (double)getTickCount();//for checking the speed
		//Detection of markers in the image passed
		MDetector.detect(TheInputImage,TheMarkers,cam_param,1.0,true);
		
		//check the speed by calculating the mean speed of all iterations
		AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
		AvrgTime.second++;
		cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

		// Locations extracted from different markers:
		enum {n_locs=8};
		location_binary locs[n_locs];
				
		for (unsigned int i=0; i<TheMarkers.size(); i++) {
		    if (i>=n_locs) break;
			Marker &marker=TheMarkers[i];
			const marker_info_t &mi=get_marker_info(marker.id);
			extract_location(locs[i],marker);
		}
		/*
		// Extract lowest-slot location
		location_binary bin; // invalid by default
		for (int i=0;i<n_locs;i++) {
			if (locs[i].valid) { 
				bin=locs[i];
				break;
			}
		}

		// Dump to disk		
		static uint32_t bin_count=0;
		bin.count=bin_count++;
		bin.vidcap_count=vidcap_count;
		FILE *fbin=fopen("marker.bin","rb+"); // "r" mode avoids file truncation
		if (fbin==NULL) { // file doesn't exist (yet)
			fbin=fopen("marker.bin","w"); // create the file
			if (fbin==NULL) { // 
				printf("Error creating marker.bin output file (disk full?  permissions?)");
				exit(1);
			}
		} 
		fwrite(&bin,sizeof(bin),1,fbin); // atomic(?) file write
		fclose(fbin);
		*/

		bool vidcap=false;
		// if ((framecount++%32) == 0) vidcap=true;
		if (showGUI || vidcap) {
			//print marker info and draw the markers in image
			TheInputImage.copyTo(TheInputImageCopy);
			for (unsigned int i=0; i<TheMarkers.size(); i++) {
				Marker &marker=TheMarkers[i];
				// cout<<TheMarkers[i]<<endl;
				marker.draw(TheInputImageCopy,Scalar(0,0,255),1);

				//draw a 3d cube on each marker if there is 3d info
				if (  cam_param.isValid()) {
					CvDrawingUtils::draw3dCube(TheInputImageCopy,marker,cam_param,1,true);
					CvDrawingUtils::draw3dAxis(TheInputImageCopy,marker,cam_param);
					//draw_marker_gui_2D(TheInputImageCopy,Scalar(255,255,0),marker);
				}

			}

			if (true) {
				//print other rectangles that contains invalid markers
				for (unsigned int i=0; i<MDetector.getCandidates().size(); i++) {
					aruco::Marker m( MDetector.getCandidates()[i],999);
					m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
				}
			}

			if (vidcap) { // write to disk
				std::vector<int> params;
				params.push_back(cv::IMWRITE_JPEG_QUALITY);
				params.push_back(30); // <- low quality, save disk space and network bandwidth
				cv::imwrite("vidcap_next.jpg",TheInputImageCopy,params); // dump JPEG
				int ignore;
				ignore=system("mv -f vidcap_next.jpg vidcap.jpg"); // atomic(?) file replace
				ignore=system("cp vidcap.jpg vidcaps/`date '+%F__%H_%M_%S__%N'`.jpg"); // telemetry log
				vidcap_count++;
			}
		}
		if (showGUI) {
			//show input with augmented information and  the thresholded image
			cv::imshow("in",TheInputImageCopy);
			// cv::imshow("thres",MDetector.getThresholdedImage());

			char key=cv::waitKey(1);//wait for key to be pressed
			if (key=='q' || key=='x' || key==0x13) exit(0);
		} /* end showGUI */
	} /* end frame loop */

	} catch (std::exception &ex) {
		cout<<"Vision/ArUco exception: "<<ex.what()<<endl;
	}

}

