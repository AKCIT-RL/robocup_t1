#ifndef BOOSTER_ROBOTICS_SDK_B1_INTERNAL_API_CONST_HPP
#define BOOSTER_ROBOTICS_SDK_B1_INTERNAL_API_CONST_HPP

#include <string>

namespace booster_internal {
namespace robot {
namespace b1 {

enum SquatDirection {
    kSquatDown = -1,
    kSquatUp = 1,
};

enum SquatSide {
    kLeft = 0,
    kRight = 1,
};

enum ControlMode {
    kDefaultMode = 0,
    kAudienceMode = 1,
    kForbiddenMode = 2
};

}
}
} // namespace booster_internal::robot::b1

#endif // BOOSTER_ROBOTICS_SDK_B1_INTERNAL_API_CONST_HPP