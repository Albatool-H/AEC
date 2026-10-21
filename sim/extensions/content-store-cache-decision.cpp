#include "content-store-cache-decision.hpp"
#include "probcache-tsi-tag.hpp"
#include "btw-tag.hpp"

#include "ns3/ndnSIM/model/ndn-ns3.hpp"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.hpp"

#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/packet.h"

#include <algorithm>
#include <cmath>

namespace ns3 {
namespace ndn {
namespace cs {

NS_OBJECT_ENSURE_REGISTERED(ContentStoreCacheDecision);

TypeId
ContentStoreCacheDecision::GetTypeId()
{
  static TypeId tid =
    TypeId("ns3::ndn::cs::CacheDecision")
      .SetGroupName("Ndn")
      .SetParent<super>()
      .AddConstructor<ContentStoreCacheDecision>()
      .AddAttribute("Mode", "Caching strategy: lcd | probcache | btw",
                    StringValue("lcd"),
                    MakeStringAccessor(&ContentStoreCacheDecision::m_mode),
                    MakeStringChecker())
      .AddAttribute("Betweenness", "This node's pre-computed C_B in [0,1] (btw only).",
                    DoubleValue(0.0),
                    MakeDoubleAccessor(&ContentStoreCacheDecision::m_betweenness),
                    MakeDoubleChecker<double>(0.0));
  return tid;
}

ContentStoreCacheDecision::ContentStoreCacheDecision()
  : m_mode("lcd"), m_betweenness(0.0)
{
}

bool
ContentStoreCacheDecision::Add(shared_ptr<const Data> )
{
  return false;
}

bool
ContentStoreCacheDecision::ForceAdd(shared_ptr<const Data> data)
{
  return super::Add(data);
}

shared_ptr<Data>
ContentStoreCacheDecision::Lookup(shared_ptr<const Interest> interest)
{

  uint64_t btwRunningMax = 0;
  if (m_mode == "btw") {
    const uint64_t mine = static_cast<uint64_t>(std::llround(m_betweenness * BtwMaxTag::SCALE));
    btwRunningMax = mine;
    auto inTag = interest->getTag<Ns3PacketTag>();
    if (inTag != nullptr) {
      Ptr<const Packet> pkt = inTag->getPacket();
      BtwMaxTag bt;
      uint64_t cur = 0;
      if (pkt->PeekPacketTag(bt)) cur = bt.Get();
      btwRunningMax = std::max(cur, mine);
      if (btwRunningMax != cur) {
        BtwMaxTag nb;
        nb.Set(btwRunningMax);
        pkt->AddPacketTag(nb);
      }
    }
  }

  shared_ptr<Data> copy = super::Lookup(interest);
  if (copy == nullptr)
    return copy;

  Ptr<Packet> packet = Convert::ToPacket(*copy);

  if (m_mode == "btw") {

    BtwMaxTag dataTag;
    dataTag.Set(btwRunningMax);
    packet->AddPacketTag(dataTag);
  }
  else {

    FwHopCountTag hop;
    packet->RemovePacketTag(hop);
    packet->AddPacketTag(FwHopCountTag());
    if (m_mode == "probcache") {

      uint32_t reqHops = 0;
      auto reqNs3 = interest->getTag<Ns3PacketTag>();
      if (reqNs3 != nullptr) {
        FwHopCountTag hc;
        if (reqNs3->getPacket()->PeekPacketTag(hc)) reqHops = hc.Get();
      }
      ProbCacheTsiTag tsi;
      packet->RemovePacketTag(tsi);
      tsi.Set(reqHops);
      packet->AddPacketTag(tsi);
    }
  }

  auto restamped = Convert::FromPacket<Data>(packet);
  return make_shared<Data>(*restamped);
}

}
}
}
