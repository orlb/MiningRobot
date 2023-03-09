/* Debug print the backend's encoder output */
#include "aurora/lunatic.h"
#include "nlohmann/json.hpp"    // json input/output
#include <chrono>               // system_clock::time_point(), system_clock::now()
#include <string>               // string()
#include <sstream>              // stringstream()
#include <fstream>              // create, open, and close files,
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
using std::ofstream;

using json = nlohmann::json;

int main() {

    // Initialize the data capture for the drive encoders
    MAKE_exchange_drive_encoders();
    aurora::drive_encoders last;
    last.left   =   last.right=0.0f;

    // Create directory for the captured data
    const string& data_storage_location     =   "/tmp/data_exchange/data_capture/";
    std::filesystem::create_directory(data_storage_location);

    while (true) {

        // If there has been a change to the drive encoders, update terminal printout text for clarity
        if (exchange_drive_encoders.updated()) printf("+");

        // Calculate amount of echange in drive encoders
        aurora::drive_encoders cur      =   exchange_drive_encoders.read();
        aurora::drive_encoders change   =   cur - last;

        // Print change of drive encoders to terminal
        change.print();

        // The following is necessary code, but I'm stuck right now due to not remembering a lot of the finer
        // details regarding iterators, pointers, etc.

        // // Make sure the number of files in the data_storage_location is not excessive
        // int count_files = 0;
        // int max_num_files = 25;
        // for (auto& tmp : std::filesystem::directory_iterator(data_storage_location)) {
        //     count_files++;
        // }
        // cout << "Total files: " << count_files << endl;
        // while (count_files > max_num_files) {
        //     const auto & curr_path = std::filesystem::directory_iterator(data_storage_location).next();
        //     std::filesystem::remove(curr_path);
        //     count_files--;
        // }
        // cout << "Total files: " << count_files << endl;



        // Capture current time and convert to string format
        // Create variables
        const auto now          = std::chrono::high_resolution_clock::now();
        const auto now_         = std::chrono::system_clock::to_time_t(now);
        const auto ms           = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        string ms_formatted     = std::to_string(ms.count());
        stringstream curr_time_file;
        stringstream curr_time_json;

        // Format milliseconds to three digits
        while (ms_formatted.length() < 3) {
            ms_formatted = ms_formatted.insert(0, "0");
        }

        // Format variables into current time string
        curr_time_file << std::put_time(std::localtime(&now_), "%Y_%m_%d_%H_%M_%S_");
        curr_time_json << std::put_time(std::localtime(&now_), "%Y/%m/%d %H:%M:%S:");

        // Craft the json output
        json output_stream;
        output_stream["timestamp"] = curr_time_json.str() + ms_formatted;
        std::string curr_filename = "lunatic_data_" + curr_time_file.str() + ms_formatted;

        // Test that json is formatted properly:
    
        string test = output_stream.dump();
        cout << test << endl;


        // Test that curr_filename is accurate
        cout << curr_filename << endl;

        // Create path to new file
        const string& curr_path = data_storage_location + curr_filename + ".json";
        cout << curr_path << endl;

        // Create a new file in the tmp dir
        ofstream fout(curr_path);
     
        if (!fout) {
            cout << "Error opening output file" << endl;
            exit(-1);
        }

        fout << output_stream; 
        fout.close();

        last=cur;
        
        aurora::data_exchange_sleep(100);
    }
}


