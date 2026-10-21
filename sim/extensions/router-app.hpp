#ifndef ROUTER_APP_HPP
#define ROUTER_APP_HPP

#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/ndnSIM/model/ndn-common.hpp"
#include "ns3/event-id.h"

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

namespace ns3 {
namespace ndn {

class RouterApp : public App
{
public:
  static TypeId GetTypeId();

  void StartApplication() override;
  void StopApplication() override;
  void OnInterest(std::shared_ptr<const Interest> interest) override;
  void OnData(std::shared_ptr<const Data> data) override;

  void SetPool(const std::vector<std::string>& pool) { m_pool = pool; }

  uint64_t GetEvictions() const { return m_evictions; }

private:

  void ServeOrFetch(const Name& inner, const Name& returnName);

  void EmitData(const Name& returnName, std::shared_ptr<const Data> payload,
                bool inheritHop = true);
  void EmitInterest(std::shared_ptr<Interest> interest);
  void CacheStore(const std::string& key, std::shared_ptr<const Data> data);

  void OnPendingTimeout(std::string key);
  void OnGwTimeout(std::string wUri);

  std::string                 m_routerId;
  std::vector<std::string>    m_pool;
  uint32_t                    m_dCacheCap = 100;

  std::unordered_map<std::string, std::shared_ptr<const Data>> m_dCache;
  std::list<std::string>      m_dCacheOrder;
  uint64_t                    m_evictions = 0;

  std::unordered_map<std::string, std::vector<Name>> m_pending;
  std::unordered_map<std::string, EventId>           m_pendingTimer;

  std::unordered_map<std::string, Name>    m_gwPending;
  std::unordered_map<std::string, EventId> m_gwTimer;

  double                      m_pendingTimeout = 1.0;
  uint32_t                    m_nextNonce = 1;
};

}
}

#endif
