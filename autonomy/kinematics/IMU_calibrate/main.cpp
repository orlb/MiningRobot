/* Perform IMU calibration, to reduce vertigo and skew.

Collect data in reference configuration, with each joint at local zero.

Add the collected offsets to slot_A1 and slot_F1 manually.
*/
#include "aurora/lunatic.h"

double g_a=9.8f;

class IMU_calibrator {
public:
    vec3 accSum=vec3(0,0,0), rateSum=vec3(0,0,0);
    int count=0;
    
    void add(const nanoslot_IMU_state &s) {
        accSum+=s.local; rateSum+=s.rate;
        count++;
    }
    
    void printv(const char *what,const vec3 &v) const {
        printf(" %s(%.4f,%.4f,%.4f) ", what,v.x,v.y,v.z);
    }
    
    void print(const char *what,vec3 gravity=vec3(0.0,0.0,-1.0)) {
        printf("  %7s   ",what);
        printv("acc vec3", accSum*(1.0/count)*(1.0/g_a)+gravity);
        printv(", gyro vec3",rateSum*(1.0/count));
        printf("\n");
    }
};


int main() {
    MAKE_exchange_nanoslot();
    
    IMU_calibrator frame, fork, dump, boom, stick, tool;
    
    for (int i=0;i<30*10;i++) {
        const nanoslot_exchange &nano=exchange_nanoslot.read();
        
        frame.add(nano.slot_F1.state.frame);
        fork.add(nano.slot_F1.state.fork);
        dump.add(nano.slot_F1.state.dump);
        boom.add(nano.slot_F1.state.boom);
        stick.add(nano.slot_A1.state.stick);
        tool.add(nano.slot_A1.state.tool);
        
        aurora::data_exchange_sleep(40);
    }
    
    
    frame.print("frame");
    fork.print("fork");
    dump.print("dump");
    boom.print("boom");
    stick.print("stick");
    tool.print("tool");    
}


