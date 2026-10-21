#ifndef CACHE_DECISION_STRATEGY_HPP
#define CACHE_DECISION_STRATEGY_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/best-route-strategy2.hpp"
#include "ns3/random-variable-stream.h"
#include "ns3/ptr.h"

#include "content-store-cache-decision.hpp"

#include <string>
#include <vector>

namespace nfd {
namespace fw {

class CacheDecisionStrategy : public BestRouteStrategy2
{
public:
  CacheDecisionStrategy(Forwarder& forwarder, const Name& name = STRATEGY_NAME);

  virtual void
  beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                        const Face& inFace, const Data& data) DECL_OVERRIDE;

  void SetMode(const std::string& m)            { m_mode = m; }
  void SetContentStore(ns3::Ptr<ns3::ndn::cs::ContentStoreCacheDecision> cs) { m_cs = cs; }
  void SetBetweenness(double b)                 { m_betweenness = b; }
  void SetTargetTimeWindow(double t)            { m_tTw = t; }

  void SetCacheTable(const std::vector<std::string>* t) { m_cacheTable = t; }
  void SetSelfId(const std::string& s)                  { m_selfId = s; }

  static const Name STRATEGY_NAME;

private:
  std::string                                        m_mode = "lcd";
  ns3::Ptr<ns3::ndn::cs::ContentStoreCacheDecision>  m_cs;
  double                                             m_betweenness = 0.0;
  double                                             m_tTw = 10.0;
  ns3::Ptr<ns3::UniformRandomVariable>               m_rng;
  const std::vector<std::string>*                    m_cacheTable = nullptr;
  std::string                                        m_selfId;
};

}
}

#endif
