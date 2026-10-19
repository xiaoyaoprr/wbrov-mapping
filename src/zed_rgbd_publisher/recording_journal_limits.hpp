#pragma once
#include <cstddef>

namespace wbrov_recording {
// Small JSON records only, never images. 512 records represented about 17 s
// at 30 Hz and overflowed during a measured 28 s metadata write stall.
// Bound the backlog explicitly while allowing more than four minutes at 30 Hz.
constexpr std::size_t journal_capacity = 8192;
constexpr std::size_t journal_record_byte_limit = 2048;
}
