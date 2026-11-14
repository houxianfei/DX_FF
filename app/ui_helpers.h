#pragma once
#include "ff_gui.h"
#include <algorithm>
namespace ff_gui {
namespace {
inline bool inputString(const char *label, std::string &value, std::size_t capacity = 256) {
  std::vector<char> buffer(std::max(capacity, value.size() + 1), '\0');
  std::copy(value.begin(), value.end(), buffer.begin());
  if (!ImGui::InputText(label, buffer.data(), buffer.size()))
    return false;
  value = buffer.data();
  return true;
}
template <typename T> bool eraseAt(std::vector<T> &items, int index) {
  if (index < 0 || static_cast<size_t>(index) >= items.size()) return false;
  items.erase(items.begin() + index); return true;
}
inline void tooltip(const char *text) {
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort |
                           ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 36.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}
} // namespace
} // namespace ff_gui
