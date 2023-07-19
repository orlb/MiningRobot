/*

Watches detected ArUco markers, and stashes them for lunatic data exchange.

*/
#ifndef __AURORA_VISION_ARUCO_WATCHER
#define __AURORA_VISION_ARUCO_WATCHER 1

// Reference size for markers (localizer can override this, just sets scale by default)
const float vision_marker_size = 0.305; // dimensions (meters) of marker black area



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
        
        double size=vision_marker_size; // physical size of marker, in meters
        
        if (index>=aurora::vision_marker_report::max_count) {
            printf("   Ignoring--too many reports already\n");
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

#endif

