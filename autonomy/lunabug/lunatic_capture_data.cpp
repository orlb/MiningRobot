/* Debug print the backend's encoder output */
#include "aurora/lunatic.h"
#include "nlohmann/json.hpp"

// Temporary libraries

#include <string>
// #include <fstream>
// #include <sstream>
// #include <iomanip>
// #include <iostream>
#include <filesystem>
// #include <stdlib.h>
// 
// using json = nlohmann::json;
// using std::cin;
// using std::cout;
// using std::endl;
// using std::vector;
using std::string;
// using std::ofstream;
// using std::ifstream;
// using std::istringstream;

// namespace fs = std::filesystem;

int main() {

    MAKE_exchange_drive_encoders();
    
    aurora::drive_encoders last;

    last.left=last.right=0.0f;

    const string data_storage_location = "/tmp/data_exchange/data_capture";

    std::filesystem::create_directory(data_storage_location);

    while (true) {
        if (exchange_drive_encoders.updated()) printf("+");
        aurora::drive_encoders cur=exchange_drive_encoders.read();
        
        aurora::drive_encoders change = cur - last;
        change.print();

        // Create a new file in the tmp dir



        // Old code to be reused
        // Process the provided file by writing its data into an output file
        // void processOutputFile(const string& filename, const vector< pair< string, int>>& data) {
        // 
        //     ofstream fout(filename);
        // 
        //     if (!fout) {
        //         cout << "Error opening output file" << endl;
        //         exit(0);
        //     }
        // 
        //     // Ensure the longest length of any word is represented in the column format for each word
        //     int len = (int)(data.size());
        // 
        //     int longestLen = 0;
        // 
        //     for (int i = 0; i < len; i++) {
        //         if (longestLen < (int)data.at(i).first.size()) {
        //             longestLen = (int)data.at(i).first.size();
        //         }
        //     }
        // 
        //     for (int i = 0; i < len; i++) {
        //         fout << left << setw(longestLen) << data.at(i).first << " " << setw(5) << right << data.at(i).second << endl;
        //     }
        // 
        // }
        
        last=cur;
        
        aurora::data_exchange_sleep(100);
    }
}


