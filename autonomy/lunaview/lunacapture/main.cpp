/*
 Read robot data from the lunatic data exchange, and write it to a postgres database.

 Original by Bryan Beus, 2023-04 (Public Domain)
*/

#include "aurora/lunatic.h"
#include "nlohmann/json.hpp"    // json input/output
#include "lunacapture.hpp"

#include <string>               // string()
#include <sstream>              // stringstream()
#include <ctime>                // put_time(), locattime()
#include <iostream>             // cout, cin, endl
#include <pqxx/pqxx>            // postgresql database library
#include <iomanip>              // setprecision
#include <cmath>                // round()
#include <fstream>              // ifstream()

using std::string;
using std::istringstream;
using std::stringstream;
using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::to_string;
using std::ifstream;

using json = nlohmann::json;

// Error message for loss of a previously established database connection
const string& db_conn_err_msg = "Connection to the database is lost.";

int main() {
    //int sleep_ms = 500; // time in milliseconds between database writes (small = bigger database but better data resolution)
    int sleep_ms = 30; // high resolution mode
    
    int verbose=10; // <- print this many demo values
    
    // Set variables to obtain database password
    std::ifstream file_dbpass (".dbpass");
    std::string dbpass;

    // Obtain database password
    if (file_dbpass.is_open()) {
        file_dbpass >> dbpass;
    }

    // Establish connection to postgresql database
    pqxx::connection psql_conn(" \
        dbname=test_cpp \
        user=postgres \
        password=" + dbpass + " \
        hostaddr=127.0.0.1 \
        port=5432 \
        target_session_attrs=read-write"
    );

    // If the connection does not initiate, send error message and stop the script
    if (!psql_conn.is_open()) {
        cout << db_conn_err_msg << endl;
        return 0;
    }

    // Create the database table, if it does not already exist
    try {

        if (psql_conn.is_open()) {

            pqxx::work w(psql_conn);

            w.exec("\
                CREATE TABLE IF NOT EXISTS test_conn ( \
                id SERIAL NOT NULL PRIMARY KEY, \
                instance_num INT NOT NULL, \
                robot_json JSON NOT NULL, \
                created_at TIMESTAMP NOT NULL DEFAULT NOW()\
                );"
            );

            w.commit();

        } else {

            cout << db_conn_err_msg << endl;

        }

    } catch (const std::exception& e) {

        cout << e.what() << endl;

        return 0;

    }

    // Set default instance_num to 0
    int instance_num = 0;

    // Check if there's an existing previous instance_num entry
    // If so, iterate +1 over the last instance_num
    try {
        pqxx::work w(psql_conn);
        pqxx::result r = w.exec("SELECT * FROM test_conn ORDER BY created_at DESC LIMIT 1;");

        // Test, if there is data in the returned result, then iterate
        // If not, then keep the instance_num at 0
        if (!r.empty()) {
            stringstream instance_num_str;
            instance_num_str << r[0][1].c_str();

            instance_num_str >> instance_num;
            ++instance_num;
        }

    } catch (const std::exception &e) {

        cout << e.what() << endl;
        return 0;

    }

    cout << "Session instance number: " << instance_num << endl;
    cout << endl;

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

    while (true) {

        if (exchange_backend_state.updated() || exchange_nanoslot.updated())
        {
            // Craft the json output
            json output_json;

            // Capture epoch time
            output_json["epoch_time"]   = capture_epoch();

            // Update the backend data
            state = exchange_backend_state.read();
            nano = exchange_nanoslot.read();

            // Calculate amount of change in drive encoders
            // Drive_encoder data are of type float and provide total distance driven by each side of robot
            aurora::drive_encoders cur      =   exchange_drive_encoders.read();
            aurora::drive_encoders change   =   cur - last;
            last=cur;

            // Capture drive encoder change data
            output_json["drive_encoder_left"]   = round_decimal(change.left);
            output_json["drive_encoder_right"]  = round_decimal(change.right);

            // Output tool vibration on each axis (in m/s^2)
            output_json["tool_vibe"]  = round_decimal(length(nano.slot_A1.state.tool.vibe));
            output_json["frame_vibe"]  = round_decimal(length(nano.slot_F1.state.frame.vibe));
            
            // Output load cell loads (in kgf)
            output_json["load_dump"] = round_decimal(nano.slot_F1.state.load_R);
            output_json["load_tool"] = round_decimal(nano.slot_A1.state.load_R);

            // List of joint angles, in sequential order, from base to furthest point of arm
            // Vars are all of type float, units degrees
            // These are all angles reconstructed from the inertial measurement units (IMUs)
            output_json["fork"]  = round_decimal(state.joint.angle.fork);
            output_json["dump"]  = round_decimal(state.joint.angle.dump);
            output_json["boom"]  = round_decimal(state.joint.angle.boom);
            output_json["stick"] = round_decimal(state.joint.angle.stick);
            output_json["tilt"]  = round_decimal(state.joint.angle.tilt);
            output_json["spin"]  = round_decimal(state.joint.angle.spin);

            // This is the power being sent to the motor, not necessarily position
            // Vars are all of type float
            // Values run from -1 to +1, and indicate full backward to full forward
            // The first two indicate power to the drive motors
            output_json["power_left"]     = round_decimal(state.power.left);
            output_json["power_right"]    = round_decimal(state.power.right);
            // These variables indicate power to the joints
            output_json["power_fork"]     = round_decimal(state.power.fork);
            output_json["power_dump"]     = round_decimal(state.power.dump);
            output_json["power_boom"]     = round_decimal(state.power.boom);
            output_json["power_stick"]    = round_decimal(state.power.stick);
            output_json["power_tilt"]     = round_decimal(state.power.tilt);
            output_json["power_spin"]     = round_decimal(state.power.spin);
            // This var represents power level sent to tool
            output_json["power_tool"]     = round_decimal(state.power.tool);

            // The state.state variable is one int
            output_json["state_state"]     = state.state;

            // state.sensor is obsolete and therefore currently omitted
            // Later will include info such as battery voltage

            // The state.loc variables represent an estimate of location
            // Values are of type float
            // Variables (x, y) are in meters
            output_json["loc_x"]          = round_decimal(state.loc.x);
            output_json["loc_y"]          = round_decimal(state.loc.y);
            // Variable (angle) is in degrees
            output_json["loc_angle"]      = round_decimal(state.loc.angle);

            // Test that json is formatted properly:
            if (verbose>0)
            {
                cout << output_json.dump() << endl;
                cout << endl;
                verbose--;
            }

            stringstream output_assembled;
            output_assembled << "INSERT INTO test_conn ";
            output_assembled << " ( instance_num, robot_json )  VALUES  ( ";
            output_assembled << instance_num << ", '"; 
            output_assembled << output_json.dump();
            output_assembled << "');";

            string output = output_assembled.str();

            try {

                pqxx::work w(psql_conn);

                w.exec(output);

                w.commit();

            } catch (const std::exception& e) {

                cout << e.what() << endl;

                return 0;

            }

        }
        
        aurora::data_exchange_sleep(sleep_ms);
    }
}
