#pragma once

#include "ns3/ndnSIM/apps/ndn-producer.hpp"

namespace ns3 {
namespace ndn {

class NoCacheProducer : public Producer
{
public:
  static TypeId GetTypeId();
  void OnInterest(std::shared_ptr<const Interest> interest) override;

  uint64_t GetHitsTotal() const { return m_hitsTotal; }

private:
  template <typename T>
  T GetAttributeAs(const std::string& name) const;

  uint64_t m_hitsTotal = 0;
};

}
}
