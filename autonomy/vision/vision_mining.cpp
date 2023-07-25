/*
This is the computer vision system, reading color and depth data
from a realsense camera, and writing data used by the localizer.

    This version specialized for mining only.


From the color images, we extract aruco marker locations.

From the depth images, we extract drivable / non-drivable areas.
*/
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"
#include "aurora/kinematics.h"
#include "aurora/kinematic_links.cpp"

#include "vision/realsense_camera.hpp"
#include "vision/realsense_camera.cpp"

#include "vision/aruco_detector.hpp"
#include "vision/aruco_detector.cpp"
#include "vision/aruco_watcher.hpp"

// Obstacle detection
#include "vision/grid.hpp"
#include "vision/grid.cpp"
#include "vision/erode.hpp"

using namespace aurora;

/* Project current depth data onto mining_depth stripe */
void project_depth_to_mining(const realsense_camera_capture &cap,
    const aurora::robot_coord3D &view3D,
    mining_depth &mining)
{
    // printf("Camera view: "); view3D.print();
    mining.camera_coords=view3D;
    const float depth_calibration_scale_factor=1.0f; // fudge factor to match real distances
    const float sanity_distance_min = 0.5; // mostly parts of robot if they're too close
    const float sanity_distance_max = 7.5; // depth gets ratty if it's too far out
    const float sanity_Z_max = 3.0; // ignore ceiling (with wide error band for tilt)
    const float sanity_Z_min = -1.0; // ignore invalid too-low
    //const int realsense_left_start=30; // invalid data left of here
    
    int L=cap.depth_w/2;
    int R=L+1;
    //for (int y = 0; y < cap.depth_h; y++)
    for (int d=0;d<mining_depth::ndepth;d++) //< vertical samples across image
    for (int x = L; x < R; x++)
    {
        vec3 spot(0,0,0);
        int y=d*(cap.depth_h-1)/(mining_depth::ndepth-1);
        float depth=cap.get_depth_m(x,y);
        if (depth>sanity_distance_min && depth<sanity_distance_max) 
        { // value seems in-range
            depth *= depth_calibration_scale_factor;
        
            vec3 cam = cap.project_3D(depth,x,y);
            vec3 world = view3D.world_from_local(cam);
            
            if (world.z<sanity_Z_max && world.z>sanity_Z_min)
            {
                spot=world;
            }
        }
        mining.depth[d]=spot;
    }   
}

int main(int argc,const char *argv[]) {
    int show_GUI=0;
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_marker_reports_depth(); // for reporting aruco markers
    MAKE_exchange_backend_state(); // for joint angles
    MAKE_exchange_mining_depth(); // for viewed depth data

    // res=720; fps=30; // <- 200% of gaming laptop CPU
    // res=540; fps=60; // <- 220% of gaming laptop CPU
    // res=540; fps=30; // <- 100% of gaming laptop CPU
    
    int res=480; // camera's requested vertical resolution
    int fps=5; // camera's frames per second 
    int downscale=2; // size scale factor to save network bandwidth
    bool aruco=true; // look for computer vision markers in RGB data
    bool obstacle=true; // look for obstacles/driveable areas in depth data
    int erode=3; // image erosion passes
    float minSize=0.05; // fraction of image for aruco markers
    for (int argi=1;argi<argc;argi++) {
      std::string arg=argv[argi];
      if (arg=="--gui") show_GUI++;
      else if (arg=="--res") res=atoi(argv[++argi]); // manual resolution
      else if (arg=="--fps") fps=atoi(argv[++argi]); // manual framerate
      else if (arg=="--no-aruco") aruco=false; 
      else if (arg=="--no-obstacle") obstacle=false; 
      else if (arg=="--erode") erode=atoi(argv[++argi]);
      
      else {
        std::cerr<<"Unknown argument '"<<arg<<"'.  Exiting.\n";
        return 1;
      }
    }

    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    
    printf("Connecting to realsense camera...\n");
    realsense_camera cam(res,fps);
    printf("Connected.\n");
    
    aruco_detector *detector=0;
    if (aruco) {
        detector = new aruco_detector(minSize);
    }

    while (true) {
        // Grab data from realsense
        realsense_camera_capture cap(cam);
        // If the two captures dont have the same data do not draw the obsticles.
        // Maybe solution is to iterate over the two realsense scene.
        // Helper script maybe define an way to compare in realsense.h
        // ex: cap.blend(last);

        
        // Run aruco marker detection on color image
        if (aruco)
        {
            vision_marker_watcher watcher;
            detector->find_markers(cap.color_image,watcher,show_GUI);
            if (watcher.found_markers()>0) { // only write if we actually saw something.
                exchange_marker_reports_depth.write_begin()=watcher.reports;
                exchange_marker_reports_depth.write_end();
            }
        }
        
        // Run obstacle detection on depth image
        if (obstacle) {
            if (erode) erode_depth(cap,erode);
            
            // Project to mining
            robot_link_coords links(exchange_backend_state.read().joint);
            const robot_coord3D &view3D=links.coord3D(link_depthcam);
            
            mining_depth mining;
            project_depth_to_mining(cap,view3D,mining);
            exchange_mining_depth.write_begin() = mining;
            exchange_mining_depth.write_end();
            
        }
        
        // Show debug GUI
        if (show_GUI) {
            const cv::Mat &src=cap.color_image;
            cv::Mat img;
            cv::resize(src,img,cv::Size(src.cols/downscale,src.rows/downscale));
            imshow("Color image",img);
            //imshow("Depth image",8.0f*cap.depth_image); // scale up brightness
        }

        
        if (show_GUI) {
            int key = cv::waitKey(1);  
            if (key == 27 || key=='q')  
                break;  
        }
        // Store The previous copy of the data before grabbing new
        realsense_camera_capture last(cap);
    }
    return 0;
}
