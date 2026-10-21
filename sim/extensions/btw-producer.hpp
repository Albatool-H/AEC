#ifndef BTW_PRODUCER_HPP
#define BTW_PRODUCER_HPP

#include "ns3/ndnSIM/apps/ndn-producer.hpp"

namespace ns3 {
namespace ndn {

class BtwProducer : public Producer
{
public:
  static TypeId GetTypeId();

  void OnInterest(std::shared_ptr<const Interest> interest) override;

private:
  template <typename T>
  T GetAttributeAs(const std::string& name) const;
};

}
}

#endif
