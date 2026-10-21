#include "router-app.hpp"

#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"

#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"

#include <boost/functional/hash.hpp>
#include <algorithm>

NS_LOG_COMPONENT_DEFINE("ndn.RouterApp");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(RouterApp);

TypeId
RouterApp::GetTypeId()
{
  static TypeId tid = TypeId("ns3::ndn::RouterApp")
    .SetGroupName("Ndn")
    .SetParent<App>()
    .AddConstructor<RouterApp>()
    .AddAttribute("RouterId", "This router's id (e.g. S37)",
                  StringValue(""),
                  MakeStringAccessor(&RouterApp::m_routerId),
                  MakeStringChecker())
    .AddAttribute("CacheCapacity", "Single-copy pool cache size (= --cacheSize)",
                  UintegerValue(100),
                  MakeUintegerAccessor(&RouterApp::m_dCacheCap),
                  MakeUintegerChecker<uint32_t>());
  return tid;
}

void
RouterApp::StartApplication()
{
  App::StartApplication();

  FibHelper::AddRoute(GetNode(), "/u", m_face, 0);
  FibHelper::AddRoute(GetNode(), "/pool/" + m_routerId, m_face, 0);
  NS_LOG_INFO("RouterApp [" << m_routerId << "] start, pool=" << m_pool.size()
              << " cap=" << m_dCacheCap);
}

void
RouterApp::StopApplication()
{
  App::StopApplication();
}

void
RouterApp::OnInterest(std::shared_ptr<const Interest> interest)
{
  App::OnInterest(interest);
  if (!m_active) return;
  const Name& name = interest->getName();
  if (name.size() == 0) return;

  if (name.get(0).toUri() == "u") {
    if (m_pool.empty()) return;
    Name inner = name.getSubName(1);
    size_t home = boost::hash<std::string>()(inner.toUri()) % m_pool.size();
    const std::string& designated = m_pool[home];
    if (designated == m_routerId) {
      ServeOrFetch(inner, name);
    }
    else {
      Name wrapped("/pool");
      wrapped.append(designated);
      for (size_t i = 0; i < inner.size(); ++i) wrapped.append(inner.get(i));
      const std::string wUri = wrapped.toUri();
      if (m_gwPending.find(wUri) != m_gwPending.end()) return;
      m_gwPending[wUri] = name;
      m_gwTimer[wUri] = Simulator::Schedule(Seconds(m_pendingTimeout),
                                            &RouterApp::OnGwTimeout, this, wUri);
      auto wInt = std::make_shared<Interest>(wrapped);
      wInt->setInterestLifetime(interest->getInterestLifetime());
      wInt->setNonce(m_nextNonce++);
      NS_LOG_DEBUG("RouterApp [" << m_routerId << "] wrap " << name
                   << " -> " << wrapped << " (home=" << designated << ")");
      EmitInterest(wInt);
    }
    return;
  }

  if (name.size() >= 2 && name.get(0).toUri() == "pool"
      && name.get(1).toUri() == m_routerId) {
    Name inner = name.getSubName(2);
    ServeOrFetch(inner, name);
    return;
  }
}

void
RouterApp::OnData(std::shared_ptr<const Data> data)
{
  App::OnData(data);
  const Name& name = data->getName();
  if (name.size() == 0) return;

  if (name.get(0).toUri() == "pool") {
    const std::string wUri = name.toUri();
    auto it = m_gwPending.find(wUri);
    if (it != m_gwPending.end()) {
      EmitData(it->second, data);
      m_gwPending.erase(it);
      auto t = m_gwTimer.find(wUri);
      if (t != m_gwTimer.end()) { Simulator::Cancel(t->second); m_gwTimer.erase(t); }
    }
    return;
  }

  if (name.size() >= 2 && name.get(0).toUri() == "data" && name.get(1).toUri() == "content") {
    const std::string key = name.toUri();
    auto it = m_pending.find(key);
    if (it == m_pending.end()) return;
    CacheStore(key, data);
    for (const auto& rn : it->second) EmitData(rn, data);
    m_pending.erase(it);
    auto t = m_pendingTimer.find(key);
    if (t != m_pendingTimer.end()) { Simulator::Cancel(t->second); m_pendingTimer.erase(t); }
    return;
  }
}

void
RouterApp::ServeOrFetch(const Name& inner, const Name& returnName)
{
  const std::string key = inner.toUri();
  auto it = m_dCache.find(key);
  if (it != m_dCache.end()) {
    auto o = std::find(m_dCacheOrder.begin(), m_dCacheOrder.end(), key);
    if (o != m_dCacheOrder.end()) { m_dCacheOrder.erase(o); m_dCacheOrder.push_back(key); }

    EmitData(returnName, it->second, false);
    return;
  }
  auto p = m_pending.find(key);
  if (p != m_pending.end()) { p->second.push_back(returnName); return; }
  m_pending[key] = { returnName };
  m_pendingTimer[key] = Simulator::Schedule(Seconds(m_pendingTimeout),
                                            &RouterApp::OnPendingTimeout, this, key);
  auto fetch = std::make_shared<Interest>(inner);
  fetch->setInterestLifetime(::ndn::time::seconds(2));
  fetch->setNonce(m_nextNonce++);
  NS_LOG_DEBUG("RouterApp [" << m_routerId << "] fetch " << inner);
  EmitInterest(fetch);
}

void
RouterApp::OnPendingTimeout(std::string key)
{

  m_pending.erase(key);
  m_pendingTimer.erase(key);
}

void
RouterApp::OnGwTimeout(std::string wUri)
{

  m_gwPending.erase(wUri);
  m_gwTimer.erase(wUri);
}

void
RouterApp::EmitData(const Name& returnName, std::shared_ptr<const Data> payload,
                    bool inheritHop)
{
  auto out = std::make_shared<Data>(returnName);
  out->setFreshnessPeriod(::ndn::time::seconds(2));
  const auto& c = payload->getContent();
  out->setContent(c.value(), c.value_size());

  if (inheritHop) {
    if (auto ns3tag = payload->getTag<ndn::Ns3PacketTag>()) out->setTag(ns3tag);
  }
  out->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
  StackHelper::getKeyChain().sign(*out);
  m_transmittedDatas(out, this, m_face);
  m_face->onReceiveData(*out);
}

void
RouterApp::EmitInterest(std::shared_ptr<Interest> interest)
{
  m_transmittedInterests(interest, this, m_face);
  m_face->onReceiveInterest(*interest);
}

void
RouterApp::CacheStore(const std::string& key, std::shared_ptr<const Data> data)
{
  auto existing = m_dCache.find(key);
  if (existing != m_dCache.end()) {
    auto o = std::find(m_dCacheOrder.begin(), m_dCacheOrder.end(), key);
    if (o != m_dCacheOrder.end()) { m_dCacheOrder.erase(o); m_dCacheOrder.push_back(key); }
    existing->second = data;
    return;
  }
  if (m_dCache.size() >= m_dCacheCap && !m_dCacheOrder.empty()) {
    const std::string victim = m_dCacheOrder.front();
    m_dCacheOrder.pop_front();
    m_dCache.erase(victim);
    ++m_evictions;
  }
  m_dCache[key] = data;
  m_dCacheOrder.push_back(key);
}

}
}
