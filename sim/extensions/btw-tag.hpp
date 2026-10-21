#ifndef BTW_TAG_HPP
#define BTW_TAG_HPP

#include "ns3/tag.h"

namespace ns3 {
namespace ndn {

class BtwMaxTag : public Tag {
public:
  static constexpr double SCALE = 1e9;

  static TypeId GetTypeId();

  BtwMaxTag() : m_max(0) {}
  ~BtwMaxTag() {}

  void     Set(uint64_t v) { m_max = v; }
  uint64_t Get() const     { return m_max; }

  virtual TypeId   GetInstanceTypeId() const;
  virtual uint32_t GetSerializedSize() const;
  virtual void     Serialize(TagBuffer i) const;
  virtual void     Deserialize(TagBuffer i);
  virtual void     Print(std::ostream& os) const;

private:
  uint64_t m_max;
};

}
}

#endif
