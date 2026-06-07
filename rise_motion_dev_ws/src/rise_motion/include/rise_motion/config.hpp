#pragma once

// if used as part of rise-os-core set the environment variable MOTION_USE_EXTERNAL_CONFIG=1 to use the config package

#if EXTERNAL_CONFIG_AVAILABLE

#include <config/rise_configurations.hpp>

namespace rise_motion::config
{
    inline constexpr int num_motors = static_variables_GroupMotorDefinitions::SIZE;
}

#else

namespace rise_motion::config
{
    inline constexpr int num_motors = 1;
}

#endif
