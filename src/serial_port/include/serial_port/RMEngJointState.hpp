#ifndef RM_ENG_JOINT_STATE_HPP
#define RM_ENG_JOINT_STATE_HPP
#include <array>
#include <string>
struct __attribute__ ((packed)) joint_states {
    uint8_t header;
    std::array<uint16_t, 6> positions;
};

struct  joint_states_ {
    int16_t mode;
    int16_t yaw_joint_1;
    int16_t pitch_joint_1;
    int16_t pitch_joint_2;
    int16_t roll_joint_1;
    int16_t pitch_joint_3;
    int16_t roll_joint_2;
};
#endif