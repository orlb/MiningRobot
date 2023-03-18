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
#include <pqxx/pqxx>            // postgresql database library

using std::string;
using std::istringstream;
using std::stringstream;
using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::to_string;

using json = nlohmann::json;

// Create a new output file for json data streaming
string createNewFileName();

// Create a new output file for json data streaming
ofstream createNewFile(const string& filename, const string& location);

// Capture the current time and return in string format
string capture_time();

// Capture the current date and return in string format
string capture_date();

// Find and replace within string (function copied from stackoverflow: https://tinyurl.com/48fvpu6n via Czarek Tomcza)
std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace);

// Set data storage location
const string& data_location = "/tmp/data_exchange/data_capture/";

// Error message for loss of a previously established database connection
const string& db_disconnect_msg = "Connection to the database is lost.";

int main() {

    // Establish connection to postgresql database
    pqxx::connection psql_conn(" \
        dbname=test_cpp \
        user=postgres \
        password=asdf \
        hostaddr=127.0.0.1 \
        port=5432 \
        target_session_attrs=read-write"
    );

    if (!psql_conn.is_open()) {
        cout << "Connection to database could not be established." << endl;

        return 0;
    }

    try {

        if (conn.is_open()) {

            pqxx::work w(conn);

            w.exec("CREATE TABLE IF NOT EXISTS test_conn ( \
                test_col_1 VARCHAR ( 50 ) NOT NULL, \
                test_col_2 VARCHAR ( 50 ) NOT NULL \
                );"
            );
            w.commit();

        } else {
            cout << db_disconnect_msg << endl;
        }

    } catch (const std::exception& e) {
        cout << e.what() << endl;
    }

    // Initialize data capture for the drive encoders
    // Variables are of type float and provide total distance driven by each side of robot
    MAKE_exchange_drive_encoders();
    aurora::drive_encoders last;
    last.left   =   last.right  =   0.0f;

    // From robot_base.h (included in lunatic.h), obtain info from class robot_base
    MAKE_exchange_backend_state();

    // Import new var state
    backend_state state;

    // Establish initial values of state variable
    // (These values will update automatically from now on)
    state = exchange_backend_state.read();

    // List of joints, in sequential order, from base to furthest point of arm
    // Vars are all of type float
    // These are all angles reconstructed from the inertial measurement units (IMUs)
    cout << state.joint.angle.fork << endl;
    cout << state.joint.angle.dump << endl;
    cout << state.joint.angle.boom << endl;
    cout << state.joint.angle.stick << endl;
    cout << state.joint.angle.tilt << endl;
    cout << state.joint.angle.spin << endl;

    // This is the power being sent to the motor, not necessarily position
    // Vars are all of type float
    // Values run from -1 to +1, and indicate full backward to full forward 
    // The first two indicate power to the drive motors
    cout << state.power.left << endl;
    cout << state.power.right << endl;
    // These variables indicate power to the joints
    cout << state.power.fork << endl;
    cout << state.power.dump << endl;
    cout << state.power.boom << endl;
    cout << state.power.stick << endl;
    cout << state.power.tilt << endl;
    cout << state.power.spin << endl;
    // This var represents power level sent to tool
    cout << state.power.tool << endl;

    // The state.state variable is one int
    cout << state.state << endl;

    // state.sensor is obsolete and therefore currently omitted
    // Later will include info such as battery voltage

    // The state.loc variables represent an estimate of location 
    // Values are of type float
    // Variables (x, y) are in meters
    cout << state.loc.x << endl;
    cout << state.loc.y << endl;
    // Variable (angle) are in degrees
    cout << state.loc.angle << endl;


    // Obsolete code, may be deleted
    // Create ofstream stream for json data streaming
    string filename = createNewFileName();
    ofstream fout = createNewFile(filename, data_location);

    // Initiate json structure in output stream
    fout << "{}";

    // Close output stream
    fout.close();

    // Set path to file
    string data_path = data_location + filename;

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
        output_stream["date"]   = capture_date();
        output_stream["time"]   = capture_time();
        output_stream["left"]   = to_string(last.left);
        output_stream["right"]  = to_string(last.right);

        // Test that json is formatted properly:
    
        string test = output_stream.dump();
        cout << test << endl;

        // Open data_location
        std::fstream file(data_path);

        if (!file) {
            std::cerr << "Failed to open data file" << endl;
            return 0;
        }

        // Write json output
        file << output_stream; 
        fout.close();

        // Reset 
        last=cur;
        aurora::data_exchange_sleep(100);
    }
}

string createNewFileName() {

    // Capture time
    string time_file = capture_time();

    // Capture date
    string date_file = capture_date();

    // Convert vars to file format
    time_file = ReplaceStringInPlace(time_file, ":", "_");
    date_file = ReplaceStringInPlace(date_file, "/", "_");

    // Craft filename
    stringstream filename;
    filename << "lunatic_data_" << date_file << "_" << time_file << ".json";

    // Check filename accuracy
    cout << filename << endl;

    return filename.str();

}

// Create new file
ofstream createNewFile(const string& filename, const string& location) {

    // Create directory for the captured data
    std::filesystem::create_directory(location);

    // Create path to new file
    const string& curr_path = location + filename;
    cout << curr_path << endl;

    // Create a new file in the tmp dir
    ofstream fout(curr_path);
    
    if (!fout) {
        cout << "Error opening output file" << endl;
        exit(-1);
    }

    return fout;

}

// Capture current time
string capture_time() {

    // Capture current time
    const auto timestamp           = std::chrono::high_resolution_clock::now();
    const auto timestamp_          = std::chrono::system_clock::to_time_t(timestamp);
    const auto ms                  = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch()) % 1000;

    // Convert current time to string format
    string ms_formatted            = std::to_string(ms.count());
    stringstream time_output;

    // Format milliseconds to three digits
    while (ms_formatted.length() < 3) {
        ms_formatted = ms_formatted.insert(0, "0");
    }

    // Format variables into current time string
    time_output << std::put_time(std::localtime(&timestamp_), "%H:%M:%S:") << ms_formatted;

    return time_output.str();
}

// Capture current date
string capture_date() {
    // Capture current date
    const auto datestamp           = std::chrono::high_resolution_clock::now();
    const auto datestamp_          = std::chrono::system_clock::to_time_t(timestamp);

    // Convert current time to string format
    stringstream date_output;

    // Format variables into current time string
    date_output << std::put_time(std::localtime(&datestamp_), "%Y/%m/%d") << ms_formatted;

    return date_output.str();

}

// Find and replace within string (function copied from stackoverflow: https://tinyurl.com/48fvpu6n via Czarek Tomcza)
std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace) {

    // Set initial position at 0
    size_t pos = 0;

    // Search through subject and replace wherever char is found
    while ((pos = subject.find(search, pos)) != std::string::npos) {
        subject.replace(pos, search.length(), replace);
            pos += replace.length();
    }

    return subject;

}
