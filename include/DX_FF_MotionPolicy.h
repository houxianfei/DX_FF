#pragma once

// SDK-wide motion speed. Rebuild the SDK after changing this value.
// Backends translate it to their motor protocol during connection setup.
#ifndef FF_MOTION_SPEED
#define FF_MOTION_SPEED 1023
#endif

static_assert(FF_MOTION_SPEED >= 1 && FF_MOTION_SPEED <= 1023,
              "FF_MOTION_SPEED must be in 1..1023");
