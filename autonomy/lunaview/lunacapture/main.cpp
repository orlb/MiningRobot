#include "aurora/lunatic.h"
#include "nlohmann/json.hpp"    // json input/output
#include <chrono>               // system_clock::time_point(), system_clock::now()
#include <string>               // string()
#include <sstream>              // stringstream()
#include <ctime>                // put_time(), locattime()
#include <iostream>             // cout, cin, endl
#include <pqxx/pqxx>            // postgresql database library
#include <iomanip>              // setprecision
#include <cmath>                // round()

#include "lunacapture.hpp"

using std::string;
using std::istringstream;
using std::stringstream;
using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::to_string;

using json = nlohmann::json;

// Error message for loss of a previously established database connection
const string& db_disconnect_msg = "Connection to the database is lost.";

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
                robot_json JSON NOT NULL, \
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
    
    MAKE_exchange_nanoslot();
    nanoslot_exchange nano;

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
        state = exchange_backend_state.read();
        nano = exchange_nanoslot.read();

        // Calculate amount of change in drive encoders
        // Drive_encoder data are of type float and provide total distance driven by each side of robot
        aurora::drive_encoders cur      =   exchange_drive_encoders.read();
        aurora::drive_encoders change   =   cur - last;

        // Capture drive encoder change data
        output_json["drive_encoder_left"]   = change.left;
        output_json["drive_encoder_right"]  = change.right;
        
        // Output tool vibration on each axis
        output_json["vibe"]  = length(nano.slot_A1.state.tool.vibe);

        // List of joints, in sequential order, from base to furthest point of arm
        // Vars are all of type float
        // These are all angles reconstructed from the inertial measurement units (IMUs)
        output_json["fork"]  = std::round(state.joint.angle.fork * 100) / 100;
        output_json["dump"]  = std::round(state.joint.angle.dump * 100) / 100;
        output_json["boom"]  = std::round(state.joint.angle.boom * 100) / 100;
        output_json["stick"] = std::round(state.joint.angle.stick * 100) / 100;
        output_json["tilt"]  = std::round(state.joint.angle.tilt * 100) / 100;
        output_json["spin"]  = std::round(state.joint.angle.spin * 100) / 100;

        // This is the power being sent to the motor, not necessarily position
        // Vars are all of type float
        // Values run from -1 to +1, and indicate full backward to full forward 
        // The first two indicate power to the drive motors
        output_json["power_left"]     = std::round(state.power.left * 100) / 100;
        output_json["power_right"]    = std::round(state.power.right * 100) / 100;
        // These variables indicate power to the joints
        output_json["power_fork"]     = std::round(state.power.fork * 100) / 100;
        output_json["power_dump"]     = std::round(state.power.dump * 100) / 100;
        output_json["power_boom"]     = std::round(state.power.boom * 100) / 100;
        output_json["power_stick"]    = std::round(state.power.stick * 100) / 100;
        output_json["power_tilt"]     = std::round(state.power.tilt * 100) / 100;
        output_json["power_spin"]     = std::round(state.power.spin * 100) / 100;
        // This var represents power level sent to tool
        output_json["power_tool"]     = std::round(state.power.tool * 100) / 100;

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
        output_json["loc_angle"]      = std::round(state.loc.angle * 100) / 100;

        // Test that json is formatted properly:
    
        cout << output_json.dump() << endl;
        cout << endl;

        
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
