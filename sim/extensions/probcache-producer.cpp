#include "probcache-producer.hpp"
#include "probcache-tsi-tag.hpp"

#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/ndnSIM/model/ndn-ns3.hpp"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.hpp"

#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/nstime.h"
#include "ns3/log.h"
#include "ns3/packet.h"

NS_LOG_COMPONENT_DEFINE("ndn.ProbCacheProducer");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(ProbCacheProducer);

TypeId
ProbCacheProducer::GetTypeId()
{
  static TypeId tid = TypeId("ns3::ndn::ProbCacheProducer")
    .SetGroupName("Ndn")
    .SetParent<Producer>()
    .AddConstructor<ProbCacheProducer>();
  return tid;
}

void
ProbCacheProducer::OnInterest(std::shared_ptr<const Interest> interest)
{

  App::OnInterest(interest);
  if (!m_active) return;

  uint32_t tsi = 0;
  auto reqNs3 = interest->getTag<Ns3PacketTag>();
  if (reqNs3 != nullptr) {
    FwHopCountTag hc;
    if (reqNs3->getPacket()->PeekPacketTag(hc))
      tsi = hc.Get();
  }

  auto data = std::make_shared<Data>(interest->getName());
  data->setFreshnessPeriod(::ndn::time::milliseconds(
      GetAttributeAs<TimeValue>("Freshness").Get().GetMilliSeconds()));
  uint32_t payloadSize = GetAttributeAs<UintegerValue>("PayloadSize").Get();
  data->setContent(std::make_shared< ::ndn::Buffer>(payloadSize));
  StackHelper::getKeyChain().sign(*data);

  Ptr<Packet> packet = Convert::ToPacket(*data);
  ProbCacheTsiTag tsiTag;
  packet->RemovePacketTag(tsiTag);
  tsiTag.Set(tsi);
  packet->AddPacketTag(tsiTag);
  auto tagged = Convert::FromPacket<Data>(packet);

  NS_LOG_INFO("ProbCacheProducer reply " << tagged->getName() << " TSI(c)=" << tsi);
  m_transmittedDatas(tagged, this, m_face);
  m_face->onReceiveData(*tagged);
}

template <typename T>
T ProbCacheProducer::GetAttributeAs(const std::string& name) const
{
  T v;
  GetAttribute(name, v);
  return v;
}

}
}
