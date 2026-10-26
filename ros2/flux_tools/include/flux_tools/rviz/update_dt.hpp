#ifndef FLUX_TOOLS_RVIZ_UPDATE_DT_HPP
#define FLUX_TOOLS_RVIZ_UPDATE_DT_HPP

#include <chrono>

namespace flux_tools::rviz
{

// Display::update() takes float seconds up to rviz_common 14 (Jazzy) and nanoseconds from 15
// (Lyrical). CMake sets the macro from the version it found.
#ifdef FLUX_TOOLS_CHRONO_UPDATE
using UpdateDt = std::chrono::nanoseconds;
#else
using UpdateDt = float;
#endif

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS_RVIZ_UPDATE_DT_HPP
