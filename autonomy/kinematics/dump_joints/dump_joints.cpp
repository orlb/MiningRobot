/* Debug print the joint angles, in degrees, in a spreadsheet friendly format */
#include <stdio.h>
#include "aurora/lunatic.h"

int main() {
    MAKE_exchange_backend_state();
    
    while (true) {
        if (exchange_backend_state.updated()) {
            aurora::backend_state backend=exchange_backend_state.read();
            
            robot_joint_state &j=backend.joint;
            printf("angles\tFD\t%5.1f\t%5.1f\tBSTS\t%5.1f\t%5.1f\t%5.1f\t%5.1f\n",
                j.angle.fork, j.angle.dump,   j.angle.boom, j.angle.stick, j.angle.tilt, j.angle.spin);
            fflush(stdout);
        }
        
        aurora::data_exchange_sleep(100);
    }
}

