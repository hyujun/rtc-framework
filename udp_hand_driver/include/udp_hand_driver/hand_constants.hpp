#ifndef UDP_HAND_DRIVER_HAND_CONSTANTS_HPP_
#define UDP_HAND_DRIVER_HAND_CONSTANTS_HPP_

#include <string>
#include <vector>

namespace udp_hand_driver {

inline constexpr int kNumHandMotors = 10;

inline const std::vector<std::string> kDefaultHandMotorNames = {
    "thumb_cmc_aa", "thumb_cmc_fe",  "thumb_mcp_fe",  "index_mcp_aa",  "index_mcp_fe",
    "index_dip_fe", "middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe", "ring_mcp_fe"};

inline const std::vector<std::string> kDefaultFingertipNames = {"thumb", "index", "middle", "ring"};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_HAND_CONSTANTS_HPP_
