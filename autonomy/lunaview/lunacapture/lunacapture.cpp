#include <chrono>               // system_clock::time_point(), system_clock::now()
#include <string>               // string()
#include <sstream>              // stringstream()
#include <ctime>                // put_time(), locattime()
#include <iostream>             // cout, cin, endl
#include <iomanip>              // setprecision

#include "lunacapture.hpp"	// Include lunacapture header file

using std::string;
using std::istringstream;
using std::stringstream;
using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::to_string;

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
