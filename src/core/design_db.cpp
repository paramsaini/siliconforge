// SiliconForge — Design Database Implementation
#include "core/design_db.hpp"
#include <stdexcept>
#include <algorithm>

namespace sf {

DesignState DesignDatabase::dummy_;

void DesignDatabase::create(const std::string& name, const std::string& tech) {
    DesignState ds;
    ds.design_name = name;
    ds.technology = tech;
    ds.stage = DesignState::INIT;
    ds.version = 0;
    ds.log.push_back("Design '" + name + "' created, tech=" + tech);
    designs_[name] = std::move(ds);
}

DesignState& DesignDatabase::get(const std::string& name) {
    auto it = designs_.find(name);
    if (it == designs_.end())
        throw std::runtime_error("Design not found: " + name);
    return it->second;
}

const DesignState& DesignDatabase::get(const std::string& name) const {
    auto it = designs_.find(name);
    if (it == designs_.end()) return dummy_;
    return it->second;
}

bool DesignDatabase::exists(const std::string& name) const {
    return designs_.count(name) > 0;
}

void DesignDatabase::remove(const std::string& name) {
    designs_.erase(name);
}

void DesignDatabase::set_stage(const std::string& name, DesignState::Stage stage) {
    auto& ds = get(name);
    ds.stage = stage;
    static const char* stage_names[] = {
        "INIT", "RTL_PARSED", "SYNTHESIZED", "PLACED", "ROUTED", "VERIFIED", "SIGNED_OFF"
    };
    ds.log.push_back("Stage → " + std::string(stage_names[(int)stage]));
}

void DesignDatabase::log(const std::string& name, const std::string& msg) {
    get(name).log.push_back(msg);
}

int DesignDatabase::snapshot(const std::string& name) {
    auto& ds = get(name);
    ds.version++;
    ds.log.push_back("Snapshot v" + std::to_string(ds.version));
    return ds.version;
}

DesignDbStats DesignDatabase::stats() const {
    DesignDbStats s;
    s.designs = (int)designs_.size();
    for (auto& [name, ds] : designs_) {
        s.total_gates += (int)ds.netlist.num_gates();
        s.total_nets += (int)ds.netlist.num_nets();
        s.total_cells += (int)ds.physical.cells.size();
    }
    s.message = std::to_string(s.designs) + " designs, " +
                std::to_string(s.total_gates) + " gates, " +
                std::to_string(s.total_cells) + " cells";
    return s;
}

std::vector<std::string> DesignDatabase::list() const {
    std::vector<std::string> names;
    for (auto& [name, ds] : designs_) names.push_back(name);
    std::sort(names.begin(), names.end());
    return names;
}

std::string DesignDatabase::summary(const std::string& name) const {
    auto& ds = get(name);
    std::ostringstream ss;
    static const char* stage_names[] = {
        "INIT", "RTL_PARSED", "SYNTHESIZED", "PLACED", "ROUTED", "VERIFIED", "SIGNED_OFF"
    };
    ss << "Design: " << ds.design_name << "\n"
       << "Technology: " << ds.technology << "\n"
       << "Stage: " << stage_names[(int)ds.stage] << "\n"
       << "Version: " << ds.version << "\n"
       << "Gates: " << ds.netlist.num_gates() << "\n"
       << "Nets: " << ds.netlist.num_nets() << "\n"
       << "Log (" << ds.log.size() << " entries):\n";
    int shown = std::min((int)ds.log.size(), 5);
    for (int i = (int)ds.log.size() - shown; i < (int)ds.log.size(); ++i)
        ss << "  " << ds.log[i] << "\n";
    return ss.str();
}

} // namespace sf
