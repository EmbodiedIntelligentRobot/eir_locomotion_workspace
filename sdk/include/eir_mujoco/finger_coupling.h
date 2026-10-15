#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace MJ_BRIDGE
{

struct FingerCouplingJoint
{
    std::string curve_name;
    std::string active_joint_name;
    std::string passive_joint_name;
};

struct FingerCouplingValue
{
    double passive_position{0.0};
    double slope{0.0};
};

class FingerCouplingTable
{
public:
    bool loadCsv(const std::string& path, std::string* error = nullptr);
    bool empty() const noexcept;
    bool hasCurve(const std::string& curve_name) const;
    double openActivePosition(const std::string& curve_name) const;
    double closedActivePosition(const std::string& curve_name) const;
    double minActivePosition(const std::string& curve_name) const;
    double maxActivePosition(const std::string& curve_name) const;
    double minPassivePosition(const std::string& curve_name) const;
    double maxPassivePosition(const std::string& curve_name) const;
    std::optional<FingerCouplingValue> evaluate(
        const std::string& curve_name, double active_position) const;

private:
    struct Sample
    {
        double active_position{0.0};
        double passive_position{0.0};
    };

    struct Curve
    {
        double open_active_position{0.0};
        double closed_active_position{0.0};
        double min_passive_position{0.0};
        double max_passive_position{0.0};
        std::vector<Sample> samples;
    };

    const Curve& curve(const std::string& curve_name) const;
    std::unordered_map<std::string, Curve> curves_;
};

}  // namespace MJ_BRIDGE
