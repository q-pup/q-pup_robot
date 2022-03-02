#include "qpup_utils/qpup_utils.hpp"

#include "ros/ros.h"

namespace qpup_utils {

std::string getLoggerName() {
  std::string logger_name = ros::this_node::getName();
  std::size_t start_of_name_index = logger_name.rfind('/') + 1;
  logger_name = logger_name.substr(start_of_name_index);

  return logger_name;
}
}  // namespace qpup_utils
