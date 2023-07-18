/*
This is the computer vision system, reading color and depth data
from a realsense camera, and writing data used by the localizer.

This version specialized to capture the depth data to an STL file.


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

// Obstacle detection
#include "vision/grid.hpp"
#include "vision/grid.cpp"

using namespace aurora;

/* Fill out computer vision marker reports, based on observations from aruco */
class vision_marker_watcher {
public:
    aurora::vision_marker_reports reports;
    int index;
    
    // Return the number of valid markers seen
    int found_markers() { return index; }
    
    vision_marker_watcher() :index(0)
    {
    }
    
    void found_marker(const cv::Mat &matrix4x4,const aruco::Marker &marker,int ID)
    {
        printf("  MARKER %d: ",ID);
        
        double size=18.0; // physical size of marker, in centimeters
        if (ID==6) {  // ghost
            size=116.7; // <- big fabric, 1.5 meters including whitespace 
        }
        else if (ID==14) {  // small cat marker (debugging)
            size=14.5; 
        }
        else if (ID==13) {  // large mullet
            size=25; 
        } 
        else if (ID==7) {  // mask
            size=5; 
        }
        else {
            printf("Ignoring--unknown number %d\n",ID);
            return;
        }
        
        if (index>=aurora::vision_marker_report::max_count) {
            printf("Ignoring--too many reports already\n");
            return;
        }
        aurora::vision_marker_report &report=reports[index];
        index++;
        
        report.markerID=ID;
        report.coords.origin=size*extract_row(matrix4x4,3);
        
        // Swap Aruco Y-out Z-down, to camera coords Y-down Z-out
        report.coords.X=extract_row(matrix4x4,0);
        report.coords.Y=-extract_row(matrix4x4,2); 
        report.coords.Z=extract_row(matrix4x4,1);
        
        float min_area=120.0*120.0; // pixel area onscreen to reach 0% confidence
        report.coords.percent=90.0*(1.0-min_area/(min_area+marker.getArea()));
        
        report.coords.print();
    }
private:
    // Extract a row of this OpenCV 4x4 matrix, as a 3D vector
    vec3 extract_row(const cv::Mat &matrix4x4,int row) {
        return vec3(
            matrix4x4.at<float>(0,row),
            matrix4x4.at<float>(1,row),
            matrix4x4.at<float>(2,row)
        );
    }
};

/* Erode depth data: increase black space around missing data, for reliability*/
void erode_depth(realsense_camera_capture &cap,int erode_depth) {
    // erode the depth image here, to trim back depth sparkles
    cv::Mat depth_eroded(cv::Size(cap.depth_w, cap.depth_h), CV_16U);
    cv::Mat element=getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2*erode_depth+1, 2*erode_depth+1),
        cv::Point(erode_depth,erode_depth));
    cv::erode(cap.depth_image,depth_eroded,element);

    //depth_raw=depth_eroded; // swap back (easy, but systematic low-depth bias)

    // Zero out depths for all pixels that were eroded
    for (int y = 0; y < cap.depth_h; y++)
    for (int x = 0; x < cap.depth_w; x++) {
        if (0==depth_eroded.at<realsense_camera_capture::depth_t>(y,x))
            cap.depth_image.at<realsense_camera_capture::depth_t>(y,x)=0;
    }
}

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


/* Convert this x,y pixel coordinate to a 3D world-coords location,
   or an invalid location if the depth is invalid. */
vec3 world_from_depth(const realsense_camera_capture &cap,
    const aurora::robot_coord3D &view3D,
    int x,int y)
{
    const float sanity_distance_min = 0.5; // mostly parts of robot if they're too close
    const float sanity_distance_max = 10.5; // depth gets ratty if it's too far out

    vec3 world(0,-20.0,-20.0); // assume invalid depth
    
    float depth=cap.get_depth_m(x,y);    
    if (depth>sanity_distance_min && depth<sanity_distance_max) 
    { // value seems in-range        
        vec3 cam = cap.project_3D(depth,x,y);
        world = view3D.world_from_local(cam);
    }
    return world;
}

/* Write these three vertices to this ascii STL file as a triangle */
void write_stl_triangle(FILE *f,
    const vec3 &A,const vec3 &B,const vec3 &C)
{
    fprintf(f,"facet normal 0.0 0.0 0.0\n outer loop\n");
    
    fprintf(f,"  vertex %.4f %.4f %.4f\n", A.x, A.y, A.z);
    fprintf(f,"  vertex %.4f %.4f %.4f\n", B.x, B.y, B.z);
    fprintf(f,"  vertex %.4f %.4f %.4f\n", C.x, C.y, C.z);
    
    fprintf(f," endloop\nendfaced\n");
}


/* Write this depth data into an STL file on disk */
void write_depth_to_STL(const realsense_camera_capture &cap,
    const aurora::robot_coord3D &view3D,
    const char *stlFileName="out.stl")
{
    FILE *f=fopen(stlFileName,"w");
    if (f==NULL) return;
    fprintf(f,"solid DEPTHCAM\n");
    
    for (int y = 0; y < cap.depth_h-1; y++)
    for (int x = 0; x < cap.depth_w-1; x++)
    {
        write_stl_triangle(f,
            world_from_depth(cap,view3D, x,y),
            world_from_depth(cap,view3D, x+1,y),
            world_from_depth(cap,view3D, x,y+1)
        );

        write_stl_triangle(f,
            world_from_depth(cap,view3D, x,y+1),
            world_from_depth(cap,view3D, x+1,y),
            world_from_depth(cap,view3D, x+1,y+1)
        ); 
    }   
    
    fprintf(f,"endsolid DEPTHCAM\n");
    fclose(f);
    printf("Wrote depth image to %s\n",stlFileName);
}

int main(int argc,const char *argv[]) {
    int show_GUI=0;
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_marker_reports(); // for reporting aruco markers
    MAKE_exchange_backend_state(); // for joint angles
    MAKE_exchange_mining_depth(); // for viewed depth data

    // res=720; fps=30; // <- 200% of gaming laptop CPU
    // res=540; fps=60; // <- 220% of gaming laptop CPU
    // res=540; fps=30; // <- 100% of gaming laptop CPU
    
    int res=480; // camera's requested vertical resolution
    int fps=5; // camera's frames per second 
    bool aruco=true; // look for computer vision markers in RGB data
    int erode=1; // image erosion passes (remove bad data around depth discontinuities)
    for (int argi=1;argi<argc;argi++) {
      std::string arg=argv[argi];
      if (arg=="--gui") show_GUI++;
      else if (arg=="--res") res=atoi(argv[++argi]); // manual resolution
      else if (arg=="--fps") fps=atoi(argv[++argi]); // manual framerate
      else if (arg=="--no-aruco") aruco=false; 
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
        detector = new aruco_detector();
    }
    
    int frame_count=0;

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
                exchange_marker_reports.write_begin()=watcher.reports;
                exchange_marker_reports.write_end();
            }
        }
        
        // Grab depth image (once it's stable)
        if (frame_count++ > 40) {
            if (erode) erode_depth(cap,erode);
            
            // Project to frame coordinates
            const robot_base &robot = exchange_backend_state.read();
            robot_link_coords links(robot.joint);
            const robot_coord3D &view3D=links.coord3D(link_depthcam);
            
            write_depth_to_STL(cap,view3D);
            break;
            
            /*
            mining_depth mining;
            project_depth_to_mining(cap,view3D,mining);
            exchange_mining_depth.write_begin() = mining;
            exchange_mining_depth.write_end();
            */
            
        }
        
        // Show debug GUI
        if (show_GUI) {
            imshow("Color image",cap.color_image);
            imshow("Depth image",8.0f*cap.depth_image); // scale up brightness
        }

        
        if (show_GUI) {
            int key = cv::waitKey(1);  
            if (key == 27 || key=='q')  
                break;  
        }
        // Store The previous copy of the data before grabbing new
        //realsense_camera_capture last(cap);
    }
    return 0;
}