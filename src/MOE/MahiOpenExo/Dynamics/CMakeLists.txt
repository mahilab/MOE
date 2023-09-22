target_sources(moe
    PRIVATE
    G.cpp
    M.cpp
    V.cpp
    Friction.cpp
    MoeDynamicModel.cpp)

if (WITH_MAHI_MPC)
    target_sources(moe
        PRIVATE
        cas_G.cpp
        cas_G_m.cpp
        cas_M.cpp
        cas_V.cpp
        cas_V_m.cpp
        cas_Friction.cpp)
endif()