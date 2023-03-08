/* Debug print the backend's encoder output */
#include "aurora/lunatic.h"
#include "nlohmann/json.hpp"    // json input/output
#include <chrono>               // system_clock::time_point(), system_clock::now()
#include <string>               // string()
#include <sstream>              // stringstream()
// #include <ofstream>             // create, open, and close files,
// #include <fstream>              
#include <filesystem>           // create_directory()
#include <ctime>                // put_time(), locattime()
#include <iomanip>              
#include <iostream>             // cout, cin, endl

using std::string;
using std::istringstream;
using std::stringstream;
using std::cin;
using std::cout;
using std::endl;

int main() {

    // Initialize the data capture for the drive encoders
    MAKE_exchange_drive_encoders();
    aurora::drive_encoders last;
    last.left   =   last.right=0.0f;

    // Create directory for the captured data
    const string& data_storage_location     =   "/tmp/data_exchange/data_capture";
    std::filesystem::create_directory(data_storage_location);

    while (true) {

        // If there has been a change to the drive encoders, update terminal printout text for clarity
        if (exchange_drive_encoders.updated()) printf("+");

        // Calculate amount of echange in drive encoders
        aurora::drive_encoders cur      =   exchange_drive_encoders.read();
        aurora::drive_encoders change   =   cur - last;

        // Print change of drive encoders to terminal
        change.print();

        // Capture current time and convert to string format
        // Create variables
        const auto now          = std::chrono::high_resolution_clock::now();
        const auto now_         = std::chrono::system_clock::to_time_t(now);
        const auto ms           = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        string ms_formatted     = std::to_string(ms.count());
        stringstream curr_time;

        // Format milliseconds to three digits
        while (ms_formatted.length() < 3) {
            ms_formatted = ms_formatted.insert(0, "0");
        }


        // Format variables into file name
        curr_time << std::put_time(std::localtime(&now_), "%Y/%m/%d_%H:%M:%S_");
        std::string curr_filename = "lunatic_data_" + curr_time.str() + ms_formatted;

        // Test that curr_filename is accurate
        cout << curr_filename << endl;

        // Create a new file in the tmp dir

        // Old code to be reused
        // Process the provided file by writing its data into an output file
        // void processOutputFile(const string& filename, const vector< pair< string, int>>& data) {
        // 
        //     ofstream fout(filename);
        // 
        //     if (!fout) {
        //         cout << "Error opening output file" << endl;
        //         exit(0);
        //     }
        // 
        //     // Ensure the longest length of any word is represented in the column format for each word
        //     int len = (int)(data.size());
        // 
        //     int longestLen = 0;
        // 
        //     for (int i = 0; i < len; i++) {
        //         if (longestLen < (int)data.at(i).first.size()) {
        //             longestLen = (int)data.at(i).first.size();
        //         }
        //     }
        // 
        //     for (int i = 0; i < len; i++) {
        //         fout << left << setw(longestLen) << data.at(i).first << " " << setw(5) << right << data.at(i).second << endl;
        //     }
        // 
        // }
        
        last=cur;
        
        aurora::data_exchange_sleep(100);
    }
}


