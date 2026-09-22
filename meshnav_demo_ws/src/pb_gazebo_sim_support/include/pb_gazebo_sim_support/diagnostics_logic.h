// Copyright 2026. Licensed under Apache-2.0.
//
// Pure, engine-free logic for ContactDiagnostics.  Kept out of the system plugin so
// it can be unit tested without a running simulation, in the same spirit as
// pb_terminal_controller's terminal_logic.h.

#ifndef PB_GAZEBO_SIM_SUPPORT__DIAGNOSTICS_LOGIC_H_
#define PB_GAZEBO_SIM_SUPPORT__DIAGNOSTICS_LOGIC_H_

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <sdf/Element.hh>

namespace pb_gazebo_sim_support
{
/// \brief Read a boolean switch out of SDF.
///
/// Default-off is a property of this parser, not of the model file: an absent
/// element returns `_default`, and anything that is not an explicit true/false
/// spelling also returns `_default` rather than guessing.  A diagnostic that
/// instrumented the simulation because of a typo would be worse than one that
/// silently stayed off.
inline bool DiagnosticsEnabledFromElement(
    const std::shared_ptr<const sdf::Element> &_sdf, const std::string &_name,
    bool _default)
{
  if (_sdf == nullptr || !_sdf->HasElement(_name))
  {
    return _default;
  }
  std::string value = _sdf->Get<std::string>(_name);
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "true" || value == "1")
  {
    return true;
  }
  if (value == "false" || value == "0")
  {
    return false;
  }
  return _default;
}

/// \brief Parse a whitespace- or comma-separated joint list.
///
/// A single `<joints>` string rather than repeated `<joint>` elements: the configure
/// hook receives a *const* sdf element and sdformat's element-iteration API is
/// non-const, so a repeated element cannot be read without casting the constness
/// away.  A plain string keeps the read const-correct and the parsing testable.
inline std::vector<std::string> ParseJointList(const std::string &_value)
{
  std::vector<std::string> names;
  std::string current;
  for (char c : _value)
  {
    if (std::isspace(static_cast<unsigned char>(c)) || c == ',')
    {
      if (!current.empty())
      {
        names.push_back(current);
        current.clear();
      }
    }
    else
    {
      current.push_back(c);
    }
  }
  if (!current.empty())
  {
    names.push_back(current);
  }
  return names;
}

/// \brief Documented layout of the joint-state diagnostic message.
///
/// `[velocity * N, position * N, applied_force * N]`, joints in the configured
/// order.  Fixed here so the plugin and the analyzer cannot drift apart.
inline std::vector<double> JointStateLayout(const std::vector<double> &_velocity,
                                            const std::vector<double> &_position,
                                            const std::vector<double> &_force)
{
  std::vector<double> out;
  out.reserve(_velocity.size() + _position.size() + _force.size());
  out.insert(out.end(), _velocity.begin(), _velocity.end());
  out.insert(out.end(), _position.begin(), _position.end());
  out.insert(out.end(), _force.begin(), _force.end());
  return out;
}

/// \brief True when a requested force is large enough to expect motion.
///
/// Used to label a sample "commanded" in the diagnostics trace.  The threshold is
/// deliberately tiny: the question is only whether the drive asked for anything.
inline bool RequestsMotion(double _forceMagnitude, double _threshold = 1.0e-3)
{
  return std::abs(_forceMagnitude) > _threshold;
}
}  // namespace pb_gazebo_sim_support

#endif  // PB_GAZEBO_SIM_SUPPORT__DIAGNOSTICS_LOGIC_H_
