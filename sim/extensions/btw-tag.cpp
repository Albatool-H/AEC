#include "btw-tag.hpp"

namespace ns3 {
namespace ndn {

TypeId
BtwMaxTag::GetTypeId()
{
  static TypeId tid =
    TypeId("ns3::ndn::BtwMaxTag").SetParent<Tag>().AddConstructor<BtwMaxTag>();
  return tid;
}

TypeId
BtwMaxTag::GetInstanceTypeId() const
{
  return BtwMaxTag::GetTypeId();
}

uint32_t
BtwMaxTag::GetSerializedSize() const
{
  return sizeof(uint64_t);
}

void
BtwMaxTag::Serialize(TagBuffer i) const
{
  i.WriteU64(m_max);
}

void
BtwMaxTag::Deserialize(TagBuffer i)
{
  m_max = i.ReadU64();
}

void
BtwMaxTag::Print(std::ostream& os) const
{
  os << m_max;
}

}
}
