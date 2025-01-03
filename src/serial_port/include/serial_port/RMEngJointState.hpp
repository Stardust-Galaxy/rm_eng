#ifndef RM_ENG_JOINT_STATE_HPP
#define RM_ENG_JOINT_STATE_HPP
#include <array>
#include <string>
struct __attribute__ ((packed)) joint_states {
    uint8_t header;
    std::array<uint16_t, 6> positions;
};
#endif