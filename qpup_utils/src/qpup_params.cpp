#include "qpup_utils/qpup_params.hpp"
#include "ros/console.h"

namespace qpup_utils {

std::string getLoggerName(ros::NodeHandle &nh) {
  const std::string &nh_namespace{nh.getNamespace()};
  std::size_t start_of_name_index = nh_namespace.rfind('/') + 1;

  std::string logger_name = nh_namespace.substr(start_of_name_index);
  if (logger_name.empty()) {
    static unsigned int number_of_unnamed_loggers = 0;
    std::string fallback_logger_name{"UNNAMED_LOGGER_" +
                                     std::to_string(number_of_unnamed_loggers)};
    ROS_ERROR_STREAM_NAMED("qpup_params",
                           "Failed to construct a valid logger name from "
                           "NodeHandle. Using fallback of \""
                               << fallback_logger_name << "\"");

    logger_name = fallback_logger_name;
    number_of_unnamed_loggers++;
  }

  return logger_name;
}

std::string getLoggerName() {
  std::string logger_name = ros::this_node::getName();
  std::size_t start_of_name_index = logger_name.rfind('/') + 1;
  logger_name = logger_name.substr(start_of_name_index);

  return logger_name;
}

} // namespace qpup_utils
