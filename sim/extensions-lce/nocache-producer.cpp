#include "nocache-producer.hpp"
#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/nstime.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE("ndn.NoCacheProducer");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(NoCacheProducer);

TypeId
NoCacheProducer::GetTypeId()
{
  static TypeId tid = TypeId("ns3::ndn::NoCacheProducer")
    .SetGroupName("Ndn")
    .SetParent<Producer>()
    .AddConstructor<NoCacheProducer>();
  return tid;
}

void
NoCacheProducer::OnInterest(std::shared_ptr<const Interest> interest)
{

  App::OnInterest(interest);
  if (!m_active) return;

  Name dataName(interest->getName());
  auto data = std::make_shared<Data>();
  data->setName(dataName);
  data->setFreshnessPeriod(::ndn::time::milliseconds(GetAttributeAs<TimeValue>("Freshness").Get().GetMilliSeconds()));
  uint32_t payloadSize = GetAttributeAs<UintegerValue>("PayloadSize").Get();
  data->setContent(std::make_shared< ::ndn::Buffer>(payloadSize));

  data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);

  StackHelper::getKeyChain().sign(*data);

  ++m_hitsTotal;

  NS_LOG_INFO("NoCacheProducer reply " << data->getName());
  m_transmittedDatas(data, this, m_face);
  m_face->onReceiveData(*data);
}

template <typename T>
T NoCacheProducer::GetAttributeAs(const std::string& name) const
{
  T v;
  GetAttribute(name, v);
  return v;
}

}
}
