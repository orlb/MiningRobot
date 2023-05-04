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
uint capture_epoch();

// Find and replace within string (function copied from stackoverflow: https://tinyurl.com/48fvpu6n via Czarek Tomcza)
std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace);

// Error message for loss of a previously established database connection
const string& db_disconnect_msg = "Connection to the database is lost.";

// TO DO: Debug function
// // Prepare pqxx transactions
// void prepare_transactions(pqxx::connection &psql_conn);

// TO DO: Debug function
// // Execute insert json data
// pqxx::result execute_insert(pqxx::transaction_base &t, std::string table_name, std::string col_name, std::string json_data);

int main() {

    // Establish connection to postgresql database
    // TO-DO Establish credentials
    // TO DO: Read file .dbpass and create string
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

            w.exec("\
                CREATE TABLE IF NOT EXISTS test_conn ( \
                id SERIAL NOT NULL PRIMARY KEY, \
                robot_json JSONB NOT NULL, \
                created_at TIMESTAMP NOT NULL DEFAULT NOW()\
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

    // // Prepare the default psql transactions
    // prepare_transactions(psql_conn);

    while (true) {

        // Craft the json output
        json output_json;

        // TO DO Simplify code by putting all of the below variable/json building into a separate function
        // and returning the result as final string or json value

        // Capture epoch time
        output_json["epoch_time"]   = capture_epoch();

        // Update the backend data
        exchange_drive_encoders.updated();
        state = exchange_backend_state.read();

        // Calculate amount of change in drive encoders
        // Drive_encoder data are of type float and provide total distance driven by each side of robot
        aurora::drive_encoders cur      =   exchange_drive_encoders.read();
        aurora::drive_encoders change   =   cur - last;

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

        
        stringstream output_assembled;
        output_assembled << "INSERT INTO test_conn ";
        output_assembled << " ( robot_json )  VALUES  ('";
        output_assembled << output_json.dump();
        output_assembled << "');";

        string output = output_assembled.str();

        // TO DO Prep insert command for database

        try {

            pqxx::work w(psql_conn);

            w.exec(output);

            w.commit();
                

            // TO DO: Correct functions
            // pqxx::work t(psql_conn);
            // 
            // pqxx::result res = execute_insert(t, "test_conn", "robot_json", output_json.dump());
            // 
            // t.commit();

        } catch (const std::exception& e) {

            cout << e.what() << endl;
            
            return 0;

        }

        // Reset 
        last=cur;
        aurora::data_exchange_sleep(500);
    }
}

// Capture current epoch time
uint capture_epoch() {

    // Capture current time
    const auto timestamp           = std::chrono::high_resolution_clock::now();
    const auto epoch_time          = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch());

    // Convert to uint
    uint result = epoch_time.count();

    return result;
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

// TO DO: Create policy function to round different aspects of numbers to set number of decimal places
// Policies: angles, power levels, 

// TO DO: Debug these functions
// // Prepare pqxx transactions
// void prepare_transactions(pqxx::connection &psql_conn) {
//     
//     // Prepare statement to insert data into table
//     psql_conn.prepare(
//             "insert_data",
//             "INSERT INTO $1 ( $2 ) VALUES ( '$3'::jsonb );"
//     );
// 
// };

// TO DO: Debug function
// // Execute insert json data
// pqxx::result execute_insert(pqxx::transaction_base &t, std::string table_name, std::string col_name, std::string json_data) {
// 
//     return t.exec_prepared("insert_data", table_name, col_name, json_data);
// 
// }
