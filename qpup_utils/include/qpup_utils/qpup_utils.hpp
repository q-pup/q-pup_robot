#pragma once

#include <string>

namespace qpup_utils {

/** getLoggerName - Determines a rosconsole logger name by using the executing node's name. This is the name passed to
 * ros::init
 *
 * @return derived logger name
 */
std::string getLoggerName();

}  // namespace qpup_utils
