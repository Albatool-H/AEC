#pragma once
#include <string>
#include <ostream>
#include <memory>

namespace ns3 {
namespace ndn {

class ControlTracer
{
public:
  static void Install(const std::string& filename);
  static void Destroy();

  static void LogTx(double time, const std::string& node,
                    const std::string& pktType, const std::string& msgType,
                    const std::string& name);

  static void LogRx(double time, const std::string& node,
                    const std::string& pktType, const std::string& msgType,
                    const std::string& name);

private:
  static std::shared_ptr<std::ostream> s_os;
  static bool s_headerPrinted;
};

}
}
