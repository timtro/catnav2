#include "JsonNMPCLogger.hpp"

#include <string>

#include "Obstacle.hpp"

using namespace std::string_literals;

JsonLogger::JsonLogger(const std::string outputFilePath) {
  logFile.open(outputFilePath);
  logFile << "[\n";
}

JsonLogger::~JsonLogger() {
  logFile << "\n]\n";
  logFile.close();
}

void JsonLogger::log(const std::string& s) {
  logFile << sep << s;
  sep = separator;
}

std::string to_string(const InfoFlag f) {
  if (f == InfoFlag::OK)
    return "OK";
  else if (f == InfoFlag::TargetReached)
    return "TargetReached";
  else if (f == InfoFlag::NoTarget)
    return "NoTarget";
  else if (f == InfoFlag::MissingWorldData)
    return "MissingWorldData";
  else if (f == InfoFlag::STOP)
    return "STOP";
  else if (f == InfoFlag::Null)
    return "Null";
  else
    return "ERROR: UNRECOGNIZED INFO FLAG";
}

std::string to_string(const Target t) {
  return "([" + std::to_string(t.position.x) + ", "
         + std::to_string(t.position.y) + "], " + std::to_string(t.tolerance)
         + ")";
}

// clang-format off
std::string to_json(ob::Point o) {
  return "{\"type\": \"Point\", "s
    + "\"x\": " + std::to_string(o.position.x) + ", "
    + "\"y\": " + std::to_string(o.position.y) + ", "
    + "\"pwr\": " + std::to_string(o.pwr) + ", "
    + "\"epsilon\": " + std::to_string(o.epsilon) + "}";
}
// clang-format on

std::string to_json(ob::Null) { return R"({"type": "Null"})"; }

std::string to_json(ob::Obstacle ob) {
  return std::visit([](const auto& o) { return to_json(o); }, ob);
}

std::string ob_vec_to_json_array(const std::vector<ob::Obstacle>& obs) {
  std::string output = "[";
  constexpr auto separator = ", ";
  const auto* sep = "";
  for (const auto& ob : obs) {
    output += sep + to_json(ob);
    sep = separator;
  }
  return output + "]";
}
