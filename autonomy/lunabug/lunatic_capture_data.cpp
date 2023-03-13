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

// Create a new output file for json data streaming
ofstream createNewFile();

// Capture the current time and return in string format
string capture_time();

// Capture the current date and return in string format
string capture_date();

// Find and replace within string (function copied from stackoverflow: https://tinyurl.com/48fvpu6n via Czarek Tomcza)
std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace);

// Set data storage location
const string& data_storage_location = "/tmp/data_exchange/data_capture/";

int main() {

    // Initialize data capture for the drive encoders
    MAKE_exchange_drive_encoders();
    aurora::drive_encoders last;
    last.left   =   last.right  =   0.0f;

    // Create ofstream stream for json data streaming
    ofstream fout = createNewFile();

    // Initiate json structure in output stream
    fout << "{}";

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

        // Craft the json output
        json output_stream;
        output_stream["date"] = curr_date_json.str();
        output_stream["time"] = curr_time_json.str() + ms_formatted;
        output_stream["left"] = std::to_string(last.left);
        output_stream["right"] = std::to_string(last.right);

        // Test that json is formatted properly:
    
        string test = output_stream.dump();
        cout << test << endl;



        fout << output_stream; 
        fout.close();

        last=cur;
        
        aurora::data_exchange_sleep(100);
    }
}

ofstream createNewFile() {

    // Create directory for the captured data
    std::filesystem::create_directory(data_storage_location);

    // Capture time
    string time_file = capture_time();

    // Capture date
    string date_file = capture_date();

    // Convert vars to file format
    time_file = ReplaceStringInPlace(time_file, ":", "_");
    date_file = ReplaceStringInPlace(date_file, "\/", "_");

    // Craft filename
    stringstream filename;
    filename << date_file + "_" + time_file + ".json";

    // Create filename
    std::string filename = "lunatic_data_" + filename.str();

    // Test that curr_filename is accurate
    cout << curr_filename << endl;

    // Create path to new file
    const string& curr_path = data_storage_location + curr_filename;
    cout << curr_path << endl;

    // Create a new file in the tmp dir
    ofstream fout(curr_path);
    
    if (!fout) {
        cout << "Error opening output file" << endl;
        exit(-1);
    }

    return fout;


}

string capture_time() {

    // Capture current time
    const auto file_timestamp          = std::chrono::high_resolution_clock::now();
    const auto file_timestamp_         = std::chrono::system_clock::to_time_t(file_timestamp);
    const auto file_ms           = std::chrono::duration_cast<std::chrono::milliseconds>(file_timestamp.time_since_epoch()) % 1000;

    // Convert current time to string format
    string file_ms_formatted     = std::to_string(file_ms.count());

    string ms_formatted     = std::to_string(ms.count());
    stringstream curr_time_json;
    stringstream curr_date_json;

    // Format milliseconds to three digits
    while (ms_formatted.length() < 3) {
        ms_formatted = ms_formatted.insert(0, "0");
    }

    // Format variables into current time string
    curr_time_json << std::put_time(std::localtime(&now_), "%H:%M:%S:"); curr_date_json << std::put_time(std::localtime(&now_), "%Y/%m/%d");
}

string capture_date();

// Find and replace within string (function copied from stackoverflow: https://tinyurl.com/48fvpu6n via Czarek Tomcza)
std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace) {

        size_t pos = 0;
            while ((pos = subject.find(search, pos)) != std::string::npos) {
                         subject.replace(pos, search.length(), replace);
                                  pos += replace.length();
                                      }
                return subject;

}
