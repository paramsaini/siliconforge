#pragma once
// SiliconForge — Report Generator
// Produces comprehensive design reports: timing, power, area, DRC summary.
// Generates structured text reports suitable for review and sign-off.

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include "timing/power.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <ctime>

namespace sf {

class ReportGenerator {
public:
    ReportGenerator(const std::string& design_name = "design")
        : name_(design_name) {}

    // Add sections
    void add_timing(const StaResult& sta);
    void add_power(const PowerResult& pwr);
    void add_area(const Netlist& nl, const PhysicalDesign& pd);
    void add_drc_summary(int violations, int warnings);
    void add_custom(const std::string& title, const std::string& content);

    // Generate final report
    std::string generate() const;

    // Summary line counts
    int section_count() const { return (int)sections_.size(); }

private:
    std::string name_;
    struct Section {
        std::string title;
        std::string content;
    };
    std::vector<Section> sections_;

    static std::string separator(int width = 60);
    static std::string header(const std::string& title, int width = 60);
};

} // namespace sf
