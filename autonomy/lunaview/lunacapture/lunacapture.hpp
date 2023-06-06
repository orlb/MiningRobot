/*
 * lunacapture.hpp
 * Aurora Robotics
 * University of Alaska - Fairbanks
 * Project by Bryan Beus
 * Under supervision of Dr. Orion Lawlor
 * Declaration file for LunaCapture
 * Captures data from mining Robot "Excahauler"
 */

#ifndef LUNACAPTURE_HPP
#define LUNACAPTURE_HPP

#include <chrono>               // system_clock::time_point(), system_clock::now()
#include <string>               // string()
#include <sstream>              // stringstream()
#include <ctime>                // put_time(), locattime()
#include <iostream>             // cout, cin, endl
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

// Capture the current time and return in string format
uint64_t capture_epoch();

// Find and replace within string (function copied from stackoverflow: https://tinyurl.com/48fvpu6n via Czarek Tomcza)
std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace);

double round_decimal(double x);

#endif
