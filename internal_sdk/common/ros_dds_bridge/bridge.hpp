#ifndef __BOOSTER_ROBOTICS_SDK_BRIDGE_HPP__
#define __BOOSTER_ROBOTICS_SDK_BRIDGE_HPP__

#include <string>

namespace booster {
namespace robot {

class RosDdsBridge {
public:
    static std::string GetBoosterDomainParticipantname(const std::string &participant_name);
    static std::string GetBoosterDomainTopicName(const std::string &topic_name);
    static std::string GetBoosterDomainMsgTypeName(const std::string &msg_type_name);
};

}
} // namespace booster::robot

#endif