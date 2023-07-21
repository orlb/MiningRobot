/*
 The localizer figures out where the robot is located in 3D space, based on:
    - Wheel encoders
    - Computer vision markers

 Matt Perry & Orion Lawlor, 2019-2023 (Public Domain)
*/
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"
#include "aurora/kinematics.h"
#include "aurora/kinematic_links.cpp"

/*
 These are the installed computer vision marker locations on the field.
    x,y are in meters
    angle is in degrees, facing along the +X direction of the marker
*/
const aurora::vision_marker_reports knownMarkers {
    aurora::vision_marker_report(5.0, 10.0, 180.0, 2), //blocky, facing out toward drive area
    aurora::vision_marker_report(0.0, 3.0, 83.0, 6), //ghost, behind charge area
    aurora::vision_marker_report(12.0, 5.0, -90.0, 17), //descending bird, behind the pit
    //aurora::vision_marker_report(field_x_trough_center, 00.0, 180.0, 2), //fabric
    };

void marker_update_robot_pos(aurora::robot_loc2D & currentPos, const aurora::robot_coord3D & currentReportCoord,const int32_t markerID)
{
    for(aurora::vision_marker_report known : knownMarkers){
         if ( markerID == known.markerID){
            float weight=0.2; // <- blend in this much of the new report (higher=faster, more jitter).  FIXME: should depend on the confidence value or range.
            
            vec3 diff = known.coords.origin - currentReportCoord.origin;
            float diffwt = weight;
            currentPos.x += diff.x*diffwt;
            currentPos.y+= diff.y*diffwt;

            
            float anglediff = aurora::angle_signed_diff(known.coords.extract_angle(), currentReportCoord.extract_angle());
            float anglediffwt = weight;
            currentPos.angle += anglediff*anglediffwt;
            if (0) 
            {
                std::cout << anglediff << "= angle diff \n";
                std::cout << known.coords.extract_angle() << " = known angle, " <<  currentReportCoord.extract_angle() << " = reported angle \n";
            }
            
            // If the differences are small, we've converged and should have more confidence.
            if (length(diff)<0.3) currentPos.percent+=5.0;
            else currentPos.percent *=0.98;
            
            if (fabs(anglediff)<2.0) currentPos.percent+=3.0;
            else currentPos.percent *=0.99;
            
            if (currentPos.percent>100.0f) currentPos.percent=100.0f;
        }
    }
   
}

/// Update this robot position based on the computer vision markers seen
void update_from_markers(aurora::robot_loc2D &pos,const aurora::robot_coord3D &camera,const aurora::vision_marker_reports &reports,bool print)
{
    for (aurora::vision_marker_report report : reports)
        if (report.is_valid())
        {
            // Put the marker in world coordinates
            aurora::robot_coord3D marker_coords=camera.compose(report.coords);
            marker_coords.percent = report.coords.percent; //<- camera is essentially fixed here
            
            // Sanity-check marker coordinates
            if (marker_coords.Y.z<0.7) { // not sane
                if (print) { printf("     Invalid marker%d: ",report.markerID); marker_coords.print(); }
            }
            else 
            { 
                // Re-estimate robot position from marker position
                marker_update_robot_pos(pos,marker_coords,report.markerID);
                if (print) { printf("     Marker%d: ",report.markerID); marker_coords.print(); }
            }
        }
}


// Move the robot based on wheel encoder ticks
aurora::robot_loc2D move_robot_encoder(const aurora::robot_loc2D &pos,const aurora::drive_encoders &encoderchange)
{
    float wheelbase=1.9; // width in meters between tire centers (effective, including slip: actual is 1.05 meters)
    
    // Don't move if the encoders are stopped (save CPU and confidence loss)
    if (encoderchange.left == 0 && encoderchange.right==0) return pos;
    
    aurora::robot_loc2D new2D=pos;
    
    // Extract position and orientation from absolute location
    //Interesting issue, the pos used in the new iteration is not 3d cords. the vec3 is a a 3d cord stuff?
    vec3 P=vec3(pos.x, pos.y,0.0); // position of robot (center of wheels)
    double ang_rads=pos.angle*M_PI/180.0; // 2D rotation of robot

// Reconstruct coordinate system and wheel locations 
    vec3 FW=vec3(cos(ang_rads),sin(ang_rads),0.0); // forward vector
    vec3 UP=vec3(0,0,1); // up vector
    vec3 LR=FW.cross(UP); // left-to-right vector
    vec3 wheel[2];
    wheel[0]=P-0.5*wheelbase*LR;
    wheel[1]=P+0.5*wheelbase*LR;

//How does wheels vs tracks work?
// Move wheels forward by specified amounts
    float maxjump=3.0f; // < avoids huge jumps due to startup
    if (fabs(encoderchange.left<maxjump) && fabs(encoderchange.right<maxjump)) 
    {
        wheel[0]+=FW*encoderchange.left;
        wheel[1]+=FW*encoderchange.right;
    }

// Extract new robot position and orientation
    P=(wheel[0]+wheel[1])*0.5;
    LR=normalize(wheel[1]-wheel[0]);
    FW=UP.cross(LR);
    ang_rads=atan2(FW.y,FW.x);

// Put back into merged absolute location
    new2D.angle=180.0/M_PI*ang_rads;
    new2D.x=P.x; new2D.y=P.y;
    
    // Lose a little confidence due to error accumulating, especially on turns
    float turn=fabs(encoderchange.left-encoderchange.right);
    new2D.percent=pos.percent*(1.0-0.0001f-0.001f*turn);
    
    return new2D;
}


// Initialize the obstacle view field for a clean startup
void reinitialize_field(aurora::field_drivable &field) {
    // Zero out most of the field
    field.clear(aurora::field_unknown);
    
    const int scale=aurora::field_drivable::GRIDSIZE;
    // Fill in the known obstacles, like the scoring trough
    for (int y=field_y_trough_start/scale;y<field_y_trough_end/scale;y++)
    for (int x=field_x_trough_start/scale;x<field_x_trough_end/scale;x++)
        if (field.in_bounds(x,y))
            field.at(x,y)=aurora::field_fixed;
}

int main() {
    
    //Data sources need to read from, these are defined in lunatic.h
    MAKE_exchange_drive_commands();
    MAKE_exchange_drive_encoders();
    MAKE_exchange_marker_reports_depth();
    MAKE_exchange_marker_reports_webcam();
    MAKE_exchange_backend_state();

    //Data source needed to write to, these are defined in lunatic.h
    MAKE_exchange_plan_current();
    MAKE_exchange_obstacle_view();
    
    // Zero out detected obstacles and such (we have no localization, so they're old)
    MAKE_exchange_field_drivable();
    reinitialize_field(exchange_field_drivable.write_begin());
    exchange_field_drivable.write_end();
    //zero put encoder
    aurora::drive_encoders blank = {0,0};
    exchange_drive_encoders.write_begin()=blank;
    exchange_drive_encoders.write_end();  
    
    // Define our start configuration
    aurora::robot_loc2D pos;
    pos.x = 1.0;
    pos.y = 1.0; // start location
    pos.angle=0.0f;
    pos.percent=1.0f; //<- placeholder, so we can see it change
    
    aurora::drive_encoders lastencoder={0.0,0.0};
    int printcount=0; // <- moderate printing pace, for easier debugging
    bool loc_changed=true;
    while (true) {
        bool print=false;
        if ((printcount++%30)==0) print=true;
        
        aurora::drive_commands currentdrive = exchange_drive_commands.read();
     
    // Update position based on new encoder values
        aurora::drive_encoders currentencoder = exchange_drive_encoders.read();
        aurora::drive_encoders encoder_change = currentencoder - lastencoder;
        if (encoder_change.left!=0 || encoder_change.right!=0) {
            loc_changed=true;
        }
        aurora::robot_loc2D new2D=move_robot_encoder(pos,encoder_change);
        pos=new2D;
        lastencoder = currentencoder;
        if (loc_changed) {
            exchange_plan_current.write_begin() = pos;
            exchange_plan_current.write_end();
            loc_changed=false;
        }
        
        // FIXME: incorporate gyro data here?


        // Create camera coordinate transform
	// Camera wants robot's X axis (rotated by 90)
	aurora::robot_loc2D  camera2D=pos;
	camera2D.angle -= 90.0f; // robot Y instead of X axis
        aurora::robot_coord3D robot3D=camera2D.get3D();
        
        // If you see a newly updated aruco marker, incorporate it into your likely position
        aurora::robot_link_coords robot_links(exchange_backend_state.read().joint,robot3D);
        if (exchange_marker_reports_depth.updated()) {
            const aurora::robot_coord3D &camera=robot_links.coord3D(aurora::link_depthcam);
            update_from_markers(pos,camera,exchange_marker_reports_depth.read(),print);
            loc_changed=true;
        }
        if (exchange_marker_reports_webcam.updated()) {
            const aurora::robot_coord3D &camera=robot_links.coord3D(aurora::link_drivecam);
            update_from_markers(pos,camera,exchange_marker_reports_webcam.read(),print);
            loc_changed=true;
        }
            
        if (print) { printf("Robot: "); pos.print(); }
        

        // Limit our cycle rate (to save CPU)
        aurora::data_exchange_sleep(30);

    }
    return 0;
}
