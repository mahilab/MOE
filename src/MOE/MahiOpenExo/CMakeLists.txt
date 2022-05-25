target_sources(moe
    PRIVATE
    Joint.cpp
    JointHardware.cpp
    JointVirtual.cpp
    MahiOpenExoHardware.cpp
    MahiOpenExoVirtual.cpp
    MahiOpenExo.cpp
)

add_subdirectory(Dynamics)