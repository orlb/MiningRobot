#include "aurora/lunatic.h"
#include "nlohmann/json.hpp"    // json input/output
#include <chrono>               // system_clock::time_point(), system_clock::now()
#include <string>               // string()
#include <sstream>              // stringstream()
#include <ctime>                // put_time(), locattime()
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

// Capture the current time and return in string format
string capture_time();

// Capture the current date and return in string format
string capture_date();

// Find and replace within string (function copied from stackoverflow: https://tinyurl.com/48fvpu6n via Czarek Tomcza)
std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace);

// Error message for loss of a previously established database connection
const string& db_disconnect_msg = "Connection to the database is lost.";

// Prepare pqxx transactions
void prepare_transactions(pqxx::connection &psql_conn);

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

    // Create the database table, if it does not already exist
    try {

        if (psql_conn.is_open()) {

            pqxx::work w(psql_conn);

            // TO DO
            // Establish variables for names of db, columns, etc.
            // (Table is to contain most data in a single json string
            // and, likely, also a timestamp column to indicate 
            // time data reached the database
            w.exec("\
                CREATE TABLE IF NOT EXISTS test_conn ( \
                id SERIAL NOT NULL PRIMARY KEY, \
                robot_json JSONB NOT NULL, \
                created_at TIMESTAMP NOT NULL DEFAULT NOW(), \
                );"
            );

            w.commit();

        } else {

            cout << db_disconnect_msg << endl;

        }

    } catch (const std::exception& e) {

        cout << e.what() << endl;
        
        return 0;

    }

    // From robot_base.h (included in lunatic.h), obtain info from class robot_base

    // Initialize data capture for the drive encoders
    MAKE_exchange_drive_encoders();
    aurora::drive_encoders last;
    last.left   =   last.right  =   0.0f;

    // Initialize data capture for the backend state
    MAKE_exchange_backend_state();
    aurora::backend_state state;
    state = exchange_backend_state.read();

    while (true) {

        // Update the backend data
        exchange_drive_encoders.updated();
        state = exchange_backend_state.read();

        // TO DO Simplify code by putting all of the below variable/json building into a separate function
        // and returning the result as final string/json value

        // Calculate amount of change in drive encoders
        // Drive_encoder data are of type float and provide total distance driven by each side of robot
        aurora::drive_encoders cur      =   exchange_drive_encoders.read();
        aurora::drive_encoders change   =   cur - last;

        // Craft the json output
        json output_json;

        // Capture epoch time
        output_json["epoch_time"]   = capture_epoch();

        // Capture drive encoder change data
        output_json["drive_encoder_left"]   = change.left;
        output_json["drive_encoder_right"]  = change.right;

        // List of joints, in sequential order, from base to furthest point of arm
        // Vars are all of type float
        // These are all angles reconstructed from the inertial measurement units (IMUs)
        output_json["fork"]  = state.joint.angle.fork;
        output_json["dump"]  = state.joint.angle.dump;
        output_json["boom"]  = state.joint.angle.boom;
        output_json["stick"] = state.joint.angle.stick;
        output_json["tilt"]  = state.joint.angle.tilt;
        output_json["spin"]  = state.joint.angle.spin;

        // This is the power being sent to the motor, not necessarily position
        // Vars are all of type float
        // Values run from -1 to +1, and indicate full backward to full forward 
        // The first two indicate power to the drive motors
        output_json["power_left"]     = state.power.left;
        output_json["power_right"]    = state.power.right;
        // These variables indicate power to the joints
        output_json["power_fork"]     = state.power.fork;
        output_json["power_dump"]     = state.power.dump;
        output_json["power_boom"]     = state.power.boom;
        output_json["power_stick"]    = state.power.stick;
        output_json["power_tilt"]     = state.power.tilt;
        output_json["power_spin"]     = state.power.spin;
        // This var represents power level sent to tool
        output_json["power_tool"]     = state.power.tool;

        // The state.state variable is one int
        output_json["state_state"]     = state.state;

        // state.sensor is obsolete and therefore currently omitted
        // Later will include info such as battery voltage

        // The state.loc variables represent an estimate of location 
        // Values are of type float
        // Variables (x, y) are in meters
        output_json["loc_x"]          = state.loc.x;
        output_json["loc_y"]          = state.loc.y;
        // Variable (angle) is in degrees
        output_json["loc_angle"]      = state.loc.angle;

        // Test that json is formatted properly:
    
        cout << output_json.dump() << endl;

        // TO DO Prep insert command for database

        // Reset 
        last=cur;
        aurora::data_exchange_sleep(500);
    }
}

// Capture current epoch time
uint capture_epoch() {

    // Capture current time
    const auto timestamp           = std::chrono::high_resolution_clock::now();
    // const auto timestamp_          = std::chrono::system_clock::to_time_t(timestamp);
    // const auto ms                  = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch()) % 1000;
    const auto epoch_time          = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch());


    uint result = epoch_time.count();

    // Convert current time to string format
    // string ms_formatted            = std::to_string(ms.count());

    // Format milliseconds to three digits
    // while (ms_formatted.length() < 3) {
    //     ms_formatted = ms_formatted.insert(0, "0");
    // }

    // Format variables into current time string
    // time_output << std::put_time(std::localtime(&timestamp_), "%H:%M:%S:") << ms_formatted;

    // return time_output.str();

    return result;
}

// // Capture current date
// string capture_date() {
//     // Capture current date
//     const auto datestamp           = std::chrono::high_resolution_clock::now();
//     const auto datestamp_          = std::chrono::system_clock::to_time_t(datestamp);
// 
//     // Convert current time to string format
//     stringstream date_output;
// 
//     // Format variables into current time string
//     date_output << std::put_time(std::localtime(&datestamp_), "%Y/%m/%d");
// 
//     return date_output.str();
// 
// }

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

// Prepare pqxx transactions
void prepare_transactions(pqxx::connection &psql_conn) {
    
    // Prepare statement to insert data into table
    psql_conn.prepare(
            "insert_data",
            "INSERT INTO $1 ( $2 ) VALUES ( '$3'::jsonb );"
    );

};

