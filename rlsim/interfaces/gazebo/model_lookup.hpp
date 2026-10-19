#pragma once
#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace rlsim {
inline std::optional<size_t> find_gazebo_model(const std::vector<std::string>& names,
                                             const std::string& selected,
                                             size_t poses, size_t twists) {
    auto found = std::find(names.begin(), names.end(), selected);
    if (selected.empty() || found == names.end()) return std::nullopt;
    auto index = static_cast<size_t>(found - names.begin());
    if (index >= poses || index >= twists) return std::nullopt;
    return index;
}
} // namespace rlsim
