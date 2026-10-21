#ifndef CONTENT_STORE_CACHE_DECISION_HPP
#define CONTENT_STORE_CACHE_DECISION_HPP

#include "ns3/ndnSIM/model/cs/content-store-with-stats.hpp"
#include "ns3/ndnSIM/utils/trie/lru-policy.hpp"

#include <string>

namespace ns3 {
namespace ndn {
namespace cs {

class ContentStoreCacheDecision : public ContentStoreWithStats<ndnSIM::lru_policy_traits>
{
public:
  typedef ContentStoreWithStats<ndnSIM::lru_policy_traits> super;

  static TypeId GetTypeId();

  ContentStoreCacheDecision();

  virtual bool
  Add(shared_ptr<const Data> data) override;

  bool
  ForceAdd(shared_ptr<const Data> data);

  virtual shared_ptr<Data>
  Lookup(shared_ptr<const Interest> interest) override;

  double GetBetweenness() const { return m_betweenness; }

private:
  std::string m_mode;
  double      m_betweenness;
};

}
}
}

#endif
