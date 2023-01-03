#pragma once

#include <cstdlib>
namespace hs {
namespace core {

using InvocationID =
    u_int64_t; // Needs to never run out! At 240 fps running 100000 systems, I'm counting 24372 years. Probably enough.

} // namespace core
} // namespace hs