#ifndef FLUX2_TOOLS_RVIZ_UPDATE_DT_HPP
#define FLUX2_TOOLS_RVIZ_UPDATE_DT_HPP

#include <chrono>

namespace flux2_tools::rviz
{

// Display::update() takes float seconds up to rviz_common 14 (Jazzy) and nanoseconds from 15
// (Lyrical). CMake sets the macro from the version it found.
#ifdef FLUX2_TOOLS_CHRONO_UPDATE
using UpdateDt = std::chrono::nanoseconds;
#else
using UpdateDt = float;
#endif

}  // namespace flux2_tools::rviz

#endif  // FLUX2_TOOLS_RVIZ_UPDATE_DT_HPP
