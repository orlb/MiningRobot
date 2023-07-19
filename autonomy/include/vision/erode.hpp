#ifndef __AURORA_VISION_ERODE_HPP
#define __AURORA_VISION_ERODE_HPP 1

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

#endif

