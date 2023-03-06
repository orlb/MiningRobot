/* Debug print the backend's encoder output */
#include "aurora/lunatic.h"

int main() {
    MAKE_exchange_backend_state();
    
    aurora::backend_state last;
    while (true) {
        if (exchange_backend_state.updated()) {
            last=exchange_backend_state.read();
            printf("%.3f\n", last.cur_time);
        }
        
        aurora::data_exchange_sleep(100);
    }
}


