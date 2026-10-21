#include "cache-decision-strategy.hpp"
#include "probcache-tsi-tag.hpp"
#include "btw-tag.hpp"

#include "ns3/ndnSIM/NFD/daemon/fw/strategy-registry.hpp"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.hpp"

#include <boost/functional/hash.hpp>
#include <cmath>

namespace nfd {
namespace fw {

const Name CacheDecisionStrategy::STRATEGY_NAME(
  "ndn:/localhost/nfd/strategy/cache-decision/%FD%01");

NFD_REGISTER_STRATEGY(CacheDecisionStrategy);

CacheDecisionStrategy::CacheDecisionStrategy(Forwarder& forwarder, const Name& name)
  : BestRouteStrategy2(forwarder, name)
  , m_rng(ns3::CreateObject<ns3::UniformRandomVariable>())
{
}

void
CacheDecisionStrategy::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                             const Face& inFace, const Data& data)
{

  uint32_t x = 0, c = 0;
  uint64_t recordedMax = 0;
  auto ns3tag = data.getTag<ns3::ndn::Ns3PacketTag>();
  if (ns3tag != nullptr) {
    auto pkt = ns3tag->getPacket();
    ns3::ndn::FwHopCountTag hc;
    if (pkt->PeekPacketTag(hc)) x = hc.Get();
    ns3::ndn::ProbCacheTsiTag tsi;
    if (pkt->PeekPacketTag(tsi)) c = tsi.Get();
    ns3::ndn::BtwMaxTag bt;
    if (pkt->PeekPacketTag(bt)) recordedMax = bt.Get();
  }

  bool doCache = false;
  if (m_mode == "lcd") {
    doCache = (x == 1);
  }
  else if (m_mode == "probcache") {
    if (c >= 1 && x >= 1 && x <= c) {
      double p = (static_cast<double>(c - x + 1) / m_tTw)
               * (static_cast<double>(x) / static_cast<double>(c));
      if (p > 1.0) p = 1.0;
      if (p < 0.0) p = 0.0;
      doCache = (m_rng->GetValue() < p);
    }
  }
  else if (m_mode == "btw") {
    uint64_t mine = static_cast<uint64_t>(std::llround(m_betweenness * ns3::ndn::BtwMaxTag::SCALE));
    doCache = (recordedMax > 0 && mine == recordedMax);
  }
  else if (m_mode == "poolhome") {

    if (m_cacheTable != nullptr && !m_cacheTable->empty()) {
      size_t bucket = boost::hash<std::string>()(data.getName().toUri()) % m_cacheTable->size();
      doCache = ((*m_cacheTable)[bucket] == m_selfId);
    }
  }

  if (doCache && m_cs != nullptr) {

    auto copy = make_shared<Data>(data);
    copy->removeTag<ns3::ndn::Ns3PacketTag>();
    m_cs->ForceAdd(copy);
  }
}

}
}
