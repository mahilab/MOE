#pragma once


namespace moe {
    // Stores the parameters associated with the specific subject
    struct SubjectParameters{

    int forearm_location = 3; // Forearm location starting at the distal part of the robot
    int cw_location = 4;      // CW location starting closest to the robot
    double shoulder_ang = 0;  // angle in degrees
    };
}