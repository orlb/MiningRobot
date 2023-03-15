/*
 Extract calibration data from Intel RealSense camera's builtin data.
 
For the RealSense D435 (smaller):
 At res=480:
 depth Intrinsics: wid ht 640.0 480.0  ppx/y 319.0 246.3  fx/y 382.6 382.6
 color Intrinsics: wid ht 640.0 480.0  ppx/y 317.1 248.8  fx/y 613.8 613.9

 At res=720:
 depth Intrinsics: wid ht 1280.0 720.0  ppx/y 638.3 370.5  fx/y 637.6 637.6
 color Intrinsics: wid ht 1280.0 720.0  ppx/y 635.6 373.1  fx/y 920.7 920.8

For the RealSense D455 (bigger):
 

 
*/
#include <librealsense2/rs.hpp>  
#include <opencv2/opencv.hpp>  
#include "vision/realsense_camera.hpp"
#include "vision/realsense_camera.cpp"

// Print a realsense depth or color frame's calibration profile
template <class frame_t>
void print_profile(const char *what,const frame_t &frame)
{
    printf("%s ",what);
    const rs2::stream_profile &profile = frame.get_profile();
    const rs2::video_stream_profile &video = profile.as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics = video.get_intrinsics();
        
    printf("Intrinsics: wid ht %.1f %.1f  ppx/y %.1f %.1f  fx/y %.1f %.1f\n",
        (double)intrinsics.width, (double)intrinsics.height, 
        (double)intrinsics.ppx, (double)intrinsics.ppy, 
        (double)intrinsics.fx, (double)intrinsics.fy);
    printf("  model: %d\n", intrinsics.model);
    for (int i=0;i<=4;i++) printf("  coeff[%d]=%f\n",i,intrinsics.coeffs[i]);
}

int main() {
    int res=720, fps=15;
    printf("Connecting to camera...\n");
    realsense_camera cam(res,fps);
    
    printf("Capturing frame...\n");
    realsense_camera_capture cap(cam);

    print_profile("depth",cap.depth_frame);
    print_profile("color",cap.color_frame);
    return 0;
}

