#include "probcache-tsi-tag.hpp"

namespace ns3 {
namespace ndn {

TypeId
ProbCacheTsiTag::GetTypeId()
{
  static TypeId tid =
    TypeId("ns3::ndn::ProbCacheTsiTag").SetParent<Tag>().AddConstructor<ProbCacheTsiTag>();
  return tid;
}

TypeId
ProbCacheTsiTag::GetInstanceTypeId() const
{
  return ProbCacheTsiTag::GetTypeId();
}

uint32_t
ProbCacheTsiTag::GetSerializedSize() const
{
  return sizeof(uint32_t);
}

void
ProbCacheTsiTag::Serialize(TagBuffer i) const
{
  i.WriteU32(m_tsi);
}

void
ProbCacheTsiTag::Deserialize(TagBuffer i)
{
  m_tsi = i.ReadU32();
}

void
ProbCacheTsiTag::Print(std::ostream& os) const
{
  os << m_tsi;
}

}
}
