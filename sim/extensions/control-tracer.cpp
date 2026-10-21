#include "control-tracer.hpp"

#include <fstream>

namespace ns3 {
namespace ndn {

std::shared_ptr<std::ostream> ControlTracer::s_os;
bool ControlTracer::s_headerPrinted = false;

void
ControlTracer::Install(const std::string& filename)
{
  auto os = std::make_shared<std::ofstream>();
  os->open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
  s_os = os;
  s_headerPrinted = false;
}

void
ControlTracer::Destroy()
{
  s_os.reset();
  s_headerPrinted = false;
}

void
ControlTracer::LogTx(double time, const std::string& node,
                     const std::string& pktType, const std::string& msgType,
                     const std::string& name)
{
  if (!s_os) return;
  if (!s_headerPrinted) {
    (*s_os) << "Time\tNode\tDirection\tPacket\tMessageType\tName\n";
    s_headerPrinted = true;
  }
  (*s_os) << time << "\t" << node << "\t" << "TX" << "\t"
          << pktType << "\t" << msgType << "\t" << name << "\n";
}

void
ControlTracer::LogRx(double time, const std::string& node,
                     const std::string& pktType, const std::string& msgType,
                     const std::string& name)
{
  if (!s_os) return;
  if (!s_headerPrinted) {
    (*s_os) << "Time\tNode\tDirection\tPacket\tMessageType\tName\n";
    s_headerPrinted = true;
  }
  (*s_os) << time << "\t" << node << "\t" << "RX" << "\t"
          << pktType << "\t" << msgType << "\t" << name << "\n";
}

}
}
