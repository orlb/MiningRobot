/*
 Read robot data from the lunatic data exchange, and write it to a postgres database.
 
 To add new data columns:
    - Create table with the column and datatype 
    - Add to data_columns string below
    - Write the actual data to the SQL query below

 Original by Bryan Beus, 2023-04 (Public Domain)
*/

#include "aurora/lunatic.h"     // aurora header file
#include "lunacapture.hpp"      // lunacapture header file

int main() {
    // time in milliseconds between database writes (small = bigger database but better data resolution)
    int sleep_ms = 500; // Low resolution mode
    //int sleep_ms = 100; // Normal resolution mode
    //int sleep_ms = 30; // High resolution mode
    
    // Print this many demo values
    int verbose=10; 

    // Error message for loss of a previously established database connection
    const string& db_conn_err_msg = "Connection to the database is lost.";

    // Set variables to obtain database password
    std::ifstream file_dbpass (".dbpass");
    std::string dbpass;

    // Obtain database password
    if (file_dbpass.is_open()) {
        file_dbpass >> dbpass;
    }

    file_dbpass.close();

    // Establish connection to postgresql database
    pqxx::connection psql_conn(" \
        dbname=test_cpp \
        user=postgres \
        password=" + dbpass + " \
        hostaddr=10.10.10.2 \
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
                    epoch_time BIGINT NOT NULL, \
                    robot_json JSON NOT NULL, \
                    drive_encoder_left FLOAT NOT NULL, \
                    drive_encoder_right FLOAT NOT NULL, \
                    tool_vibe FLOAT NOT NULL, \
                    frame_vibe FLOAT NOT NULL, \
                    load_dump FLOAT NOT NULL, \
                    load_tool FLOAT NOT NULL, \
                    fork FLOAT NOT NULL, \
                    dump FLOAT NOT NULL, \
                    boom FLOAT NOT NULL, \
                    stick FLOAT NOT NULL, \
                    tilt FLOAT NOT NULL, \
                    spin FLOAT NOT NULL, \
                    power_left FLOAT NOT NULL, \
                    power_right FLOAT NOT NULL, \
                    power_fork FLOAT NOT NULL, \
                    power_dump FLOAT NOT NULL, \
                    power_boom FLOAT NOT NULL, \
                    power_stick FLOAT NOT NULL, \
                    power_tilt FLOAT NOT NULL, \
                    power_spin FLOAT NOT NULL, \
                    power_tool FLOAT NOT NULL, \
                    state_state INT NOT NULL, \
                    cell_mine FLOAT NOT NULL, \
                    charge_mine FLOAT NOT NULL, \
                    cell_drive FLOAT NOT NULL, \
                    charge_drive FLOAT NOT NULL, \
                    accum_drive FLOAT NOT NULL, \
                    total_drive FLOAT NOT NULL, \
                    accum_scoop FLOAT NOT NULL, \
                    total_scoop FLOAT NOT NULL, \
                    loc_x FLOAT NOT NULL, \
                    loc_y FLOAT NOT NULL, \
                    loc_angle FLOAT NOT NULL, \
                    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()\
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

        // Test if there is data in the returned result, then increase value by 1
        // If there is no data, then keep the instance_num at 0
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

    // Initialize data capture for nanoslot
    MAKE_exchange_nanoslot();
    nanoslot_exchange nano;

    while (true) {

        if (exchange_backend_state.updated() || exchange_nanoslot.updated())
        {
            // Craft JSON output
            json output_json;

            output_json["sample_title"] = "sample goes here";

            // Update the backend data
            state = exchange_backend_state.read();
            nano = exchange_nanoslot.read();

            // Calculate amount of change in drive encoders
            // Drive_encoder data are of type float and provide total distance driven by each side of robot
            aurora::drive_encoders cur      =   exchange_drive_encoders.read();
            aurora::drive_encoders change   =   cur - last;
            last=cur;

            // Test that json is formatted properly:
            if (verbose>0)
            {
                cout << output_json.dump() << endl;
                cout << endl;
                verbose--;
            }

            // Data columns go into the SQL query
            const static std::string data_columns = "instance_num, epoch_time, robot_json, drive_encoder_left, drive_encoder_right, tool_vibe, frame_vibe, load_dump, load_tool, fork, dump, boom, stick, tilt, spin, power_left, power_right, power_fork, power_dump, power_boom, power_stick, power_tilt, power_spin, power_tool, state_state, cell_mine, charge_mine, cell_drive,   charge_drive,  accum_drive, total_drive, accum_scoop, total_scoop,  loc_x, loc_y, loc_angle";

            stringstream output_assembled;
            output_assembled << "INSERT INTO test_conn ( " << data_columns << " )";
            output_assembled << " VALUES  ( ";

            // Insert instance_num
            output_assembled << instance_num << ", ";                                        //  instance_num

            // Capture epoch time
            output_assembled << capture_epoch() << ", ";                                            //  epoch_time

            // Insert json output
            output_assembled << "'" << output_json.dump() << "', ";                         //  robot_json

            // Capture drive encoder change data
            output_assembled << round_decimal(change.left) << ", ";                                 //  drive_encoder_left
            output_assembled << round_decimal(change.right) << ", ";                                //  drive_encoder_right

            // Output tool vibration on each axis (in m/s^2)
            output_assembled << round_decimal(length(nano.slot_A1.state.tool.vibe)) << ", ";        //  tool_vibe
            output_assembled << round_decimal(length(nano.slot_F1.state.frame.vibe)) << ", ";       //  frame_vibe

            // Output load cell loads (in kgf)
            output_assembled << round_decimal(nano.slot_F1.state.load_R) << ", ";                   //  load_dump
            output_assembled << round_decimal(nano.slot_A1.state.load_R) << ", ";                   //  load_tool

            // List of joint angles, in sequential order, from base to furthest point of arm
            // Vars are all of type float, units degrees
            // These are all angles reconstructed from the inertial measurement units (IMUs)
            output_assembled << round_decimal(state.joint.angle.fork) << ", ";                      //  fork,
            output_assembled << round_decimal(state.joint.angle.dump) << ", ";                      //  dump
            output_assembled << round_decimal(state.joint.angle.boom) << ", ";                      //  boom
            output_assembled << round_decimal(state.joint.angle.stick) << ", ";                     //  stick
            output_assembled << round_decimal(state.joint.angle.tilt) << ", ";                      //  tilt
            output_assembled << round_decimal(state.joint.angle.spin) << ", ";                      //  spin

            // This is the power being sent to the motor, not necessarily position
            // Vars are all of type float
            // Values run from -1 to +1, and indicate full backward to full forward
            // The first two indicate power to the drive motors
            output_assembled << round_decimal(state.power.left) << ", ";                            //  power_left
            output_assembled << round_decimal(state.power.right) << ", ";                           //  power_right

            // These variables indicate power to the joints
            output_assembled << round_decimal(state.power.fork) << ", ";                            //  power_fork
            output_assembled << round_decimal(state.power.dump) << ", ";                            //  power_dump
            output_assembled << round_decimal(state.power.boom) << ", ";                            //  power_boom
            output_assembled << round_decimal(state.power.stick) << ", ";                           //  power_stick
            output_assembled << round_decimal(state.power.tilt) << ", ";                            //  power_tilt
            output_assembled << round_decimal(state.power.spin) << ", ";                            //  power_spin

            // This var represents power level sent to tool
            output_assembled << round_decimal(state.power.tool) << ", ";                            //  power_tool

            // The state.state variable is one int
            output_assembled << state.state << ", ";                                                //  state_state

            // Cell voltages (volts) and charge (percent)
            output_assembled << round_decimal(state.sensor.cell_M) << ", ";                         // cell_mine
            output_assembled << round_decimal(state.sensor.charge_M) << ", ";                       // charge_mine, 
            output_assembled << round_decimal(state.sensor.cell_D) << ", ";                         // cell_drive,   
            output_assembled << round_decimal(state.sensor.charge_D) << ", ";                       // charge_drive,  

            output_assembled << round_decimal(state.accum.drive) << ", ";                           // accum_drive, 
            output_assembled << round_decimal(state.accum.drive_total) << ", ";                     // total_drive, 
            output_assembled << round_decimal(state.accum.scoop) << ", ";                           // accum_scoop, 
            output_assembled << round_decimal(state.accum.scoop_total) << ", ";                     // total_scoop,


            // The state.loc variables represent an estimate of location
            // Values are of type float
            // Variables (x, y) are in meters
            output_assembled << round_decimal(state.loc.x) << ", ";                                 //  loc_x
            output_assembled << round_decimal(state.loc.y) << ", ";                                 //  loc_y

            // Variable (angle) is in degrees
            output_assembled << round_decimal(state.loc.angle) << " ";                             //  loc_angle
            
            output_assembled << ")";

            string output = output_assembled.str();

            if (verbose > 0) {
                cout << output << endl;
            }

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
