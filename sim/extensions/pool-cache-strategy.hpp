#ifndef POOL_CACHE_STRATEGY_HPP
#define POOL_CACHE_STRATEGY_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/strategy.hpp"

#include <string>
#include <vector>

namespace nfd {
namespace fw {

class PoolCacheGatewayStrategy : public Strategy
{
public:
  PoolCacheGatewayStrategy(Forwarder& forwarder, const Name& name = STRATEGY_NAME);

  virtual void
  afterReceiveInterest(const Face& inFace,
                       const Interest& interest,
                       shared_ptr<fib::Entry> fibEntry,
                       shared_ptr<pit::Entry> pitEntry) DECL_OVERRIDE;

  void
  SetCacheTable(const std::vector<std::string>* tablePtr) { m_cacheTablePtr = tablePtr; }

  void
  SetAppFace(shared_ptr<Face> face) { m_appFace = face; }

  void
  SetSwitchID(const std::string& id) { m_switchID = id; }

  uint64_t GetWrapCount() const { return m_wrapCount; }

  static const Name STRATEGY_NAME;

private:
  const std::vector<std::string>* m_cacheTablePtr = nullptr;
  shared_ptr<Face>                m_appFace;
  std::string                     m_switchID;
  uint64_t                        m_wrapCount = 0;
};

}
}

#endif
