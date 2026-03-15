// SiliconForge — Name Pool Implementation
#include "core/name_pool.hpp"

namespace sf {

NamePool& global_name_pool() {
    static NamePool pool;
    return pool;
}

} // namespace sf
