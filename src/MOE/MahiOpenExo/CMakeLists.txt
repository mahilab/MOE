target_sources(moe
    PRIVATE
    Joint.cpp
    JointHardware.cpp
    JointVirtual.cpp
    MahiOpenExoHardware.cpp
    MahiOpenExoVirtual.cpp
    Moe.cpp
)

add_subdirectory(Dynamics)