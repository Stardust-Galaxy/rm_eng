#ifndef RM_ENG_JOINT_STATE_HPP
#define RM_ENG_JOINT_STATE_HPP
#include <array>
#include <string>
struct __attribute__ ((packed)) joint_states {
    uint8_t header;
    std::array<double, 7> positions;
};
#endif