// Loader for test/data/spine-link-vectors.txt (format described in that file's header).
#pragma once

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace spine::test
{

struct Vector
{
  std::string stream;  // odom | pose | trig
  std::string name;
  std::map<std::string, std::string> fields;  // raw text, parsed per field type by the caller
  std::vector<std::uint8_t> expect;
};

inline std::vector<std::uint8_t> parse_hex(const std::string & hex)
{
  if (hex.size() % 2 != 0) {
    throw std::runtime_error("odd-length hex");
  }
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

inline std::int64_t as_i64(const std::string & s) {return std::stoll(s);}

// strtod parses both decimal and C99 hex-float exactly.
inline double as_f64(const std::string & s)
{
  char * end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') {
    throw std::runtime_error("bad float64 '" + s + "'");
  }
  return v;
}

inline std::vector<Vector> load_vectors(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::vector<Vector> out;
  std::string line;
  while (std::getline(in, line)) {
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::istringstream ss(line);
    Vector v;
    ss >> v.stream >> v.name;
    std::string tok;
    while (ss >> tok) {
      const auto eq = tok.find('=');
      if (eq == std::string::npos) {
        throw std::runtime_error("bad token '" + tok + "' in " + v.name);
      }
      const auto key = tok.substr(0, eq);
      const auto val = tok.substr(eq + 1);
      if (key == "expect") {
        v.expect = parse_hex(val);
      } else {
        v.fields[key] = val;
      }
    }
    out.push_back(std::move(v));
  }
  return out;
}

}  // namespace spine::test
