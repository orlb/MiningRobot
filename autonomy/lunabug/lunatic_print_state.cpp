/* Debug print the backend's state variables */
#include "aurora/lunatic.h"

int main() {
    MAKE_exchange_backend_state();
    
    aurora::backend_state last;
    while (true) {
        if (exchange_backend_state.updated()) {
            last=exchange_backend_state.read();
            
            printf("%.3f: LR %.2f %.2f\n", 
                last.cur_time, last.power.left, last.power.right);
        }
        
        aurora::data_exchange_sleep(100);
    }
}


