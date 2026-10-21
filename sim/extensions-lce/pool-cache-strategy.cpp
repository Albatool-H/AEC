#include "pool-cache-strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/strategy-registry.hpp"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"
#include "ns3/log.h"

#include <boost/functional/hash.hpp>

namespace nfd {
namespace fw {

NFD_LOG_INIT("PoolCacheGatewayStrategy");

const Name PoolCacheGatewayStrategy::STRATEGY_NAME(
  "ndn:/localhost/nfd/strategy/pool-cache-gateway/%FD%01");

NFD_REGISTER_STRATEGY(PoolCacheGatewayStrategy);

PoolCacheGatewayStrategy::PoolCacheGatewayStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder, name)
{
}

void
PoolCacheGatewayStrategy::afterReceiveInterest(const Face& inFace,
                                                const Interest& interest,
                                                shared_ptr<fib::Entry> fibEntry,
                                                shared_ptr<pit::Entry> pitEntry)
{
  const ::ndn::Name& name = interest.getName();

  if (name.size() < 2 || name.get(0).toUri() != "u") {
    if (fibEntry) {
      const auto& nexthops = fibEntry->getNextHops();
      for (auto it = nexthops.begin(); it != nexthops.end(); ++it) {
        if (it->getFace()->getId() != inFace.getId()) {
          this->sendInterest(pitEntry, it->getFace());
          return;
        }
      }
    }
    this->rejectPendingInterest(pitEntry);
    return;
  }

  if (m_cacheTablePtr == nullptr || m_cacheTablePtr->empty() || !m_appFace) {
    NFD_LOG_WARN("PoolCacheGatewayStrategy[" << m_switchID
                 << "] called before SetCacheTable/SetAppFace; rejecting "
                 << name);
    this->rejectPendingInterest(pitEntry);
    return;
  }
  const std::vector<std::string>& tbl = *m_cacheTablePtr;

  ::ndn::Name inner = name.getSubName(1);
  size_t bucket = boost::hash<std::string>()(inner.toUri()) % tbl.size();
  const std::string& designated = tbl[bucket];

  if (designated == m_switchID) {
    ++m_wrapCount;
    this->sendInterest(pitEntry, m_appFace);
    return;
  }

  ::ndn::Name wrapped("/cache");
  wrapped.append(designated);
  for (size_t i = 0; i < inner.size(); ++i) wrapped.append(inner.get(i));

  auto wInt = std::make_shared<::ndn::Interest>(wrapped);
  wInt->setInterestLifetime(interest.getInterestLifetime());
  if (auto ns3tag = interest.getTag<::ns3::ndn::Ns3PacketTag>()) {
    wInt->setTag(ns3tag);
  }
  wInt->setNonce(interest.getNonce());

  ++m_wrapCount;

  NFD_LOG_DEBUG("PoolCacheGatewayStrategy[" << m_switchID
                << "] wrap " << name << " -> " << wrapped
                << " bucket=" << bucket << " D=" << designated);

  this->getForwarder().onInterest(*m_appFace, *wInt);
}

}
}
