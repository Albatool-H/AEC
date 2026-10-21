#include "btw-producer.hpp"
#include "btw-tag.hpp"

#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/ndnSIM/model/ndn-ns3.hpp"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"

#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/nstime.h"
#include "ns3/log.h"
#include "ns3/packet.h"

NS_LOG_COMPONENT_DEFINE("ndn.BtwProducer");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(BtwProducer);

TypeId
BtwProducer::GetTypeId()
{
  static TypeId tid = TypeId("ns3::ndn::BtwProducer")
    .SetGroupName("Ndn")
    .SetParent<Producer>()
    .AddConstructor<BtwProducer>();
  return tid;
}

void
BtwProducer::OnInterest(std::shared_ptr<const Interest> interest)
{
  App::OnInterest(interest);
  if (!m_active) return;

  uint64_t recordedMax = 0;
  auto reqNs3 = interest->getTag<Ns3PacketTag>();
  if (reqNs3 != nullptr) {
    BtwMaxTag bt;
    if (reqNs3->getPacket()->PeekPacketTag(bt))
      recordedMax = bt.Get();
  }

  auto data = std::make_shared<Data>(interest->getName());
  data->setFreshnessPeriod(::ndn::time::milliseconds(
      GetAttributeAs<TimeValue>("Freshness").Get().GetMilliSeconds()));
  uint32_t payloadSize = GetAttributeAs<UintegerValue>("PayloadSize").Get();
  data->setContent(std::make_shared< ::ndn::Buffer>(payloadSize));
  StackHelper::getKeyChain().sign(*data);

  Ptr<Packet> packet = Convert::ToPacket(*data);
  BtwMaxTag dataTag;
  dataTag.Set(recordedMax);
  packet->AddPacketTag(dataTag);
  auto tagged = Convert::FromPacket<Data>(packet);

  NS_LOG_INFO("BtwProducer reply " << tagged->getName() << " recordedMax=" << recordedMax);
  m_transmittedDatas(tagged, this, m_face);
  m_face->onReceiveData(*tagged);
}

template <typename T>
T BtwProducer::GetAttributeAs(const std::string& name) const
{
  T v;
  GetAttribute(name, v);
  return v;
}

}
}
