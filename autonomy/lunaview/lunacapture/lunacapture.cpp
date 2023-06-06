#include "lunacapture.hpp"	// Include lunacapture header file

#include <chrono>               // system_clock::time_point(), system_clock::now()
#include <string>               // string()
#include <sstream>              // stringstream()
#include <ctime>                // put_time(), locattime()
#include <iostream>             // cout, cin, endl
#include <iomanip>              // setprecision
#include <cmath>                // round()


using std::string;
using std::istringstream;
using std::stringstream;
using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::to_string;

// To Do: Return as uint64_t
// Capture current epoch time
uint64_t capture_epoch() {

    // Capture current time
    const auto timestamp           = std::chrono::high_resolution_clock::now();
    const auto epoch_time          = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch());

    // Convert to uint
    uint64_t result = epoch_time.count();

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

// Round this value to two decimal places
double round_decimal(double x)
{
    return std::round(x * 100) / 100;
}
