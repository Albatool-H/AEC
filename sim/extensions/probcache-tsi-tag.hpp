#ifndef PROBCACHE_TSI_TAG_HPP
#define PROBCACHE_TSI_TAG_HPP

#include "ns3/tag.h"

namespace ns3 {
namespace ndn {

class ProbCacheTsiTag : public Tag {
public:
  static TypeId GetTypeId();

  ProbCacheTsiTag() : m_tsi(0) {}
  ~ProbCacheTsiTag() {}

  void     Set(uint32_t tsi) { m_tsi = tsi; }
  uint32_t Get() const       { return m_tsi; }

  virtual TypeId   GetInstanceTypeId() const;
  virtual uint32_t GetSerializedSize() const;
  virtual void     Serialize(TagBuffer i) const;
  virtual void     Deserialize(TagBuffer i);
  virtual void     Print(std::ostream& os) const;

private:
  uint32_t m_tsi;
};

}
}

#endif
