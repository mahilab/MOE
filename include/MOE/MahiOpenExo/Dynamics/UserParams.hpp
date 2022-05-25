#pragma once


namespace moe {
    // Stores the parameters associated with the specific subject
    struct UserParams{
        int forearm_location = 3; // Forearm location starting at the distal part of the robot
        int cw_location = 4;      // CW location starting closest to the robot
        int shoulder_location = 0;  // angle in degrees
        // Function to get user params from a json
        static UserParams get_from_json(std::string filename) {
            json userParams;
            std::ifstream userParamsFile(filename);
            userParamsFile >> userParams;

            UserParams newParams;
            newParams.forearm_location = userParams["forearm_location"].get<int>();
            newParams.cw_location = userParams["cw_location"].get<int>();
            newParams.shoulder_location = userParams["shoulder_location"].get<int>();
            
            return newParams;
        }
    };
}