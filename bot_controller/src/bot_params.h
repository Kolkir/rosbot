#ifndef BOT_PARAMS_H
#define BOT_PARAMS_H

namespace ros {
class NodeHandle;
}

struct BotParams {
  double wheel_radius = 0.0;  // meters
  double wheels_base = 0.0;   // meters
  double max_rpm = 0.0;
};

BotParams ReadBotParams(ros::NodeHandle& node_handler);

#endif  // BOT_PARAMS_H
