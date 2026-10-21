#include "switch-app.hpp"
#include "control-tracer.hpp"

#include "ns3/ndnSIM-module.h"
#include "ns3/simulator.h"
#include "ndn-cxx/security/key-chain.hpp"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.hpp"

#include <boost/functional/hash.hpp>

#include <algorithm>
#include <random>
#include <sstream>

NS_LOG_COMPONENT_DEFINE("SwitchApp");

static std::string classifySwMsg(const std::string& nameStr) {
  if (nameStr.find("/heartbeat") != std::string::npos) {
    if (nameStr.rfind("/control/", 0) == 0) return "heartbeat-cc";
    size_t p1 = nameStr.find('/', 10);
    if (p1 != std::string::npos) {
      size_t p2 = nameStr.find('/', p1+1);
      if (p2 != std::string::npos && nameStr[10] == 'S' && nameStr[p1+1] == 'S')
        return "heartbeat-ss";
    }
    return "heartbeat-sc";
  }
  if (nameStr.find("/fib/sync") != std::string::npos) return "fibsync";
  if (nameStr.find("/inactiveneighbour") != std::string::npos) return "inactive-nbr";
  if (nameStr.find("/inactivecontroller") != std::string::npos) return "inactive-ctrl";
  return "control-other";
}

#define WIRELOG(role,id,dir,pkt,name) \
  do { \
    std::string _n = (name); \
    std::string _p = ((std::string(pkt) == "I") ? "Interest" : "Data"); \
    if (std::string(dir) == "TX") \
      ns3::ndn::ControlTracer::LogTx(ns3::Simulator::Now().GetSeconds(), id, _p, classifySwMsg(_n), _n); \
    else \
      ns3::ndn::ControlTracer::LogRx(ns3::Simulator::Now().GetSeconds(), id, _p, classifySwMsg(_n), _n); \
  } while(0)

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED(SwitchApp);

TypeId
SwitchApp::GetTypeId()
{
  static TypeId tid = TypeId("SwitchApp")
    .SetParent<ndn::App>()
    .AddConstructor<SwitchApp>()
    .AddAttribute("SwitchID", "ID of the switch",
                  StringValue("S1"),
                  MakeStringAccessor(&SwitchApp::m_switchID),
                  MakeStringChecker())
    .AddAttribute("IsEdge", "Whether this switch is on the network edge",
                  BooleanValue(false),
                  MakeBooleanAccessor(&SwitchApp::m_edge),
                  MakeBooleanChecker())
    .AddAttribute("HeartbeatFirst", "First heartbeat delay in seconds",
                  DoubleValue(10.0),
                  MakeDoubleAccessor(&SwitchApp::m_hbFirst),
                  MakeDoubleChecker<double>())
    .AddAttribute("HeartbeatInterval", "Periodic heartbeat interval in seconds",
                  DoubleValue(60.0),
                  MakeDoubleAccessor(&SwitchApp::m_hbInterval),
                  MakeDoubleChecker<double>())
    .AddAttribute("HeartbeatTimeout", "Retry timeout between W1/W2/... checks in seconds",
                  DoubleValue(10.0),
                  MakeDoubleAccessor(&SwitchApp::m_hbTimeout),
                  MakeDoubleChecker<double>())
    .AddAttribute("HeartbeatRepeat", "Number of retries before marking INACTIVE",
                  UintegerValue(3),
                  MakeUintegerAccessor(&SwitchApp::m_hbRepeat),
                  MakeUintegerChecker<uint32_t>())
    .AddAttribute("PendingGwTimeout",
                  "Seconds before an unmatched wrap entry is cleared",
                  DoubleValue(10.0),
                  MakeDoubleAccessor(&SwitchApp::m_pendingGwTimeoutS),
                  MakeDoubleChecker<double>())
    .AddAttribute("DPendingTimeout",
                  "Seconds before an unsatisfied producer-fetch entry is cleared at D",
                  DoubleValue(5.0),
                  MakeDoubleAccessor(&SwitchApp::m_dPendingTimeoutS),
                  MakeDoubleChecker<double>());
  return tid;
}

SwitchApp::SwitchApp() {}
SwitchApp::~SwitchApp() {}

void
SwitchApp::StartApplication()
{
  ndn::App::StartApplication();

  m_localPrefix = "/xcontrol/" + m_switchID + "/";
  ndn::FibHelper::AddRoute(GetNode(), m_localPrefix, m_face, 0);

  ndn::FibHelper::AddRoute(GetNode(), "/cache/" + m_switchID + "/", m_face, 0);

  ndn::FibHelper::AddRoute(GetNode(), "/u", m_face, 0);

  NS_LOG_INFO("Switch [" << m_switchID << "]: start t="
              << Simulator::Now().GetSeconds() << "s prefix=" << m_localPrefix
              << " edge=" << (m_edge ? 1 : 0));

  for (auto& n : m_neighbors)   { n.status = Status::ACTIVE; n.misses = 0; }
  for (auto& c : m_controllers) { c.status = Status::ACTIVE; c.misses = 0; }

  m_nbPeriodicEvent   = Simulator::Schedule(Seconds(m_hbFirst), &SwitchApp::SendNeighborsHeartbeats, this);
  m_ctrlPeriodicEvent = Simulator::Schedule(Seconds(m_hbFirst), &SwitchApp::SendControllersHeartbeats, this);
}

void
SwitchApp::StopApplication()
{
  if (m_nbPeriodicEvent.IsRunning())   m_nbPeriodicEvent.Cancel();
  if (m_nbRetryEvent.IsRunning())      m_nbRetryEvent.Cancel();
  if (m_ctrlPeriodicEvent.IsRunning()) m_ctrlPeriodicEvent.Cancel();
  if (m_ctrlRetryEvent.IsRunning())    m_ctrlRetryEvent.Cancel();

  NS_LOG_INFO("Switch [" << m_switchID << "]: stop t="
              << Simulator::Now().GetSeconds() << "s");
  ndn::App::StopApplication();
}

void
SwitchApp::OnInterest(std::shared_ptr<const ndn::Interest> interest)
{
  ndn::App::OnInterest(interest);
  const auto& name = interest->getName();
  WIRELOG("Switch", m_switchID, "RX", "I", name.toUri());
  NS_LOG_INFO("Switch [" << m_switchID << "]: Interest " << name);

  if (name.size() >= 4 && name.get(3).toUri() == "heartbeat") {
    const auto root = name.get(0).toUri();
    if (root == "control" || root == "xcontrol") {
      int hop = 0;
      auto ns3tag = interest->getTag<ndn::Ns3PacketTag>();
      if (ns3tag) {
        ::ns3::ndn::FwHopCountTag hopTag;
        if (ns3tag->getPacket()->PeekPacketTag(hopTag)) hop = hopTag.Get();
      }
      if (hop > 1) {
        NS_LOG_INFO("Switch [" << m_switchID << "]: drop non-neighbor heartbeat "
                    << name << " (hop=" << hop << ")");
        return;
      }
    }
  }

  if (!m_cacheTable.empty() &&
      name.size() >= 2 && name.get(0).toUri() == "u")
  {
    ndn::Name inner = name.getSubName(1);
    size_t bucket = boost::hash<std::string>()(inner.toUri()) % m_cacheTable.size();
    const std::string& designated = m_cacheTable[bucket];

    ++m_bucketDeltas[bucket];

    if (designated == m_switchID) {
      ndn::Name cacheKey("/cache");
      cacheKey.append(m_switchID);
      for (size_t i = 0; i < inner.size(); ++i) cacheKey.append(inner.get(i));
      const std::string cKey = cacheKey.toUri();

      auto cIt = m_dCache.find(cKey);
      if (cIt != m_dCache.end()) {

        auto orderIt = std::find(m_dCacheOrder.begin(), m_dCacheOrder.end(), cKey);
        if (orderIt != m_dCacheOrder.end()) {
          m_dCacheOrder.erase(orderIt);
          m_dCacheOrder.push_back(cKey);
        }
        auto cached = cIt->second;
        auto out = std::make_shared<ndn::Data>(name);
        out->setContent(cached->getContent());
        out->setFreshnessPeriod(cached->getFreshnessPeriod());
        out->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
        ndn::StackHelper::getKeyChain().sign(*out);

        NS_LOG_INFO("Switch [" << m_switchID << "]: d-hit (self) " << name);
        ++m_dHitsSinceHB;
        ++m_dHitTotal;
        ++m_dHitsPerBucketHB[bucket];
        m_transmittedDatas(out, this, m_face);
        m_face->onReceiveData(*out);
        return;
      }

      const std::string innerUri = inner.toUri();
      if (m_dPending.find(innerUri) == m_dPending.end()) {
        auto fetch = std::make_shared<ndn::Interest>(inner);
        fetch->setInterestLifetime(interest->getInterestLifetime());
        if (auto ns3tag = interest->getTag<ndn::Ns3PacketTag>()) fetch->setTag(ns3tag);
        static thread_local std::random_device rd2;
        static thread_local std::mt19937 gen2(rd2());
        fetch->setNonce(gen2());

        DPendingEntry entry;
        entry.wrappedUri = name.toUri();
        entry.timer = Simulator::Schedule(
          Seconds(m_dPendingTimeoutS),
          &SwitchApp::OnDPendingTimeout, this, innerUri);
        m_dPending[innerUri] = entry;
        ++m_dMissTotal;

        NS_LOG_INFO("Switch [" << m_switchID << "]: d-miss fetch (self) " << inner);
        m_transmittedInterests(fetch, this, m_face);
        m_face->onReceiveInterest(*fetch);
      }
      return;
    }

    ndn::Name wrapped("/cache");
    wrapped.append(designated);
    for (size_t i = 0; i < inner.size(); ++i) wrapped.append(inner.get(i));
    const std::string wUri = wrapped.toUri();

    if (m_pendingGw.find(wUri) != m_pendingGw.end()) {
      NS_LOG_INFO("Switch [" << m_switchID << "]: gw-wrap-suppressed (already pending) " << name);
      return;
    }

    auto wInt = std::make_shared<ndn::Interest>(wrapped);
    wInt->setInterestLifetime(interest->getInterestLifetime());
    auto ns3tag = interest->getTag<ndn::Ns3PacketTag>();
    if (ns3tag) wInt->setTag(ns3tag);
    static thread_local std::random_device rd;
    static thread_local std::mt19937 gen(rd());
    wInt->setNonce(gen());

    {
      GwPendingEntry entry;
      entry.originalUri = name.toUri();
      entry.timer = Simulator::Schedule(
        Seconds(m_pendingGwTimeoutS),
        &SwitchApp::OnPendingGwTimeout, this, wUri);
      m_pendingGw[wUri] = entry;
    }

    NS_LOG_INFO("Switch [" << m_switchID << "]: gw-wrap bucket=" << bucket
                << " D=" << designated << " " << name << " -> " << wrapped);
    m_transmittedInterests(wInt, this, m_face);
    m_face->onReceiveInterest(*wInt);
    return;
  }

  if (name.size() >= 2 &&
      name.get(0).toUri() == "cache" &&
      name.get(1).toUri() == m_switchID)
  {
    NS_LOG_INFO("Switch [" << m_switchID << "]: cache-target RX " << name);

    if (name.size() < 4 ||
        name.get(2).toUri() != "data" || name.get(3).toUri() != "content") {
      return;
    }

    const std::string wrappedUri = name.toUri();

    auto cIt = m_dCache.find(wrappedUri);
    if (cIt != m_dCache.end()) {

      auto orderIt = std::find(m_dCacheOrder.begin(), m_dCacheOrder.end(), wrappedUri);
      if (orderIt != m_dCacheOrder.end()) {
        m_dCacheOrder.erase(orderIt);
        m_dCacheOrder.push_back(wrappedUri);
      }
      auto cached = cIt->second;
      auto out = std::make_shared<ndn::Data>(name);
      out->setContent(cached->getContent());
      out->setFreshnessPeriod(cached->getFreshnessPeriod());

      out->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
      ndn::StackHelper::getKeyChain().sign(*out);

      NS_LOG_INFO("Switch [" << m_switchID << "]: d-hit " << name);
      ++m_dHitsSinceHB;
      ++m_dHitTotal;

      if (!m_cacheTable.empty()) {
        ndn::Name inner = name.getSubName(2);
        size_t bucket = boost::hash<std::string>()(inner.toUri()) % m_cacheTable.size();
        ++m_dHitsPerBucketHB[bucket];
      }
      m_transmittedDatas(out, this, m_face);
      m_face->onReceiveData(*out);
      return;
    }

    ndn::Name inner = name.getSubName(2);
    const std::string innerUri = inner.toUri();
    if (m_dPending.find(innerUri) == m_dPending.end()) {
      auto fetch = std::make_shared<ndn::Interest>(inner);
      fetch->setInterestLifetime(interest->getInterestLifetime());
      if (auto ns3tag = interest->getTag<ndn::Ns3PacketTag>()) fetch->setTag(ns3tag);
      static thread_local std::random_device rd;
      static thread_local std::mt19937 gen(rd());
      fetch->setNonce(gen());

      {
        DPendingEntry entry;
        entry.wrappedUri = wrappedUri;
        entry.timer = Simulator::Schedule(
          Seconds(m_dPendingTimeoutS),
          &SwitchApp::OnDPendingTimeout, this, innerUri);
        m_dPending[innerUri] = entry;
      }
      ++m_dMissTotal;

      NS_LOG_INFO("Switch [" << m_switchID << "]: d-miss fetch " << inner);
      m_transmittedInterests(fetch, this, m_face);
      m_face->onReceiveInterest(*fetch);
    }
    return;
  }

  if (name.size() >= 4 &&
      name.get(0).toUri() == "xcontrol" &&
      name.get(1).toUri() == m_switchID &&
      name.get(2).toUri() == "cache-update")
  {
    std::vector<std::string> newTable;
    newTable.reserve(name.size() - 3);
    for (size_t i = 3; i < name.size(); ++i) {
      newTable.push_back(name.get(i).toUri());
    }
    if (!newTable.empty()) {
      SetCacheTable(newTable);
      NS_LOG_INFO("Switch [" << m_switchID << "]: 5b cache-update applied ("
                  << newTable.size() << " buckets)");
    }
    return;
  }

  if (name.size() >= 4 &&
      name.get(0).toUri() == "xcontrol" &&
      name.get(1).toUri() == m_switchID &&
      name.get(3).toUri() == "heartbeat")
  {

    const std::string sender = name.get(2).toUri();
    if (findNeighbor(sender)) {
      ActivateNeighbor(sender);
    } else {
      NeighborInfo info;
      info.neighborID = sender;
      info.status     = Status::ACTIVE;
      info.misses     = 0;
      m_neighbors.push_back(info);
      NS_LOG_INFO("Switch [" << m_switchID << "]: new neighbor "
                  << sender << " (heartbeat-receive)");
      for (const auto& c : m_controllers) {
        if (c.status == Status::INACTIVE) continue;
        const std::string nm = "/xcontrol/" + c.controllerID + "/" + m_switchID
                             + "/newneighbour/" + sender;
        auto rep = std::make_shared<ndn::Interest>(ndn::Name(nm));
        rep->setInterestLifetime(ndn::time::seconds(1)); rep->setMustBeFresh(true);
        WIRELOG("Switch", m_switchID, "TX", "I", rep->getName().toUri());
        m_transmittedInterests(rep, this, m_face);
        m_face->onReceiveInterest(*rep);
      }
    }

    auto data = std::make_shared<ndn::Data>(name);

    data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
    static const std::string content = "active";
    data->setContent(reinterpret_cast<const uint8_t*>(content.data()), content.size());
    data->setFreshnessPeriod(ndn::time::seconds(5));
    ndn::StackHelper::getKeyChain().sign(*data);

    WIRELOG("Switch", m_switchID, "TX", "D", data->getName().toUri());

    m_transmittedDatas(data, this, m_face);

    m_face->onReceiveData(*data);
return;
  }
}

void
SwitchApp::OnData(std::shared_ptr<const ndn::Data> data)
{
  ndn::App::OnData(data);
  const auto& name = data->getName();
  WIRELOG("Switch", m_switchID, "RX", "D", name.toUri());

  if (name.size() >= 2 &&
      name.get(0).toUri() == "data" && name.get(1).toUri() == "content")
  {
    auto pIt = m_dPending.find(name.toUri());
    if (pIt != m_dPending.end()) {

      ndn::Name responseName(pIt->second.wrappedUri);
      auto out = std::make_shared<ndn::Data>(responseName);
      out->setContent(data->getContent());
      out->setFreshnessPeriod(data->getFreshnessPeriod());

      if (auto ns3tag = data->getTag<ndn::Ns3PacketTag>()) out->setTag(ns3tag);
      out->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
      ndn::StackHelper::getKeyChain().sign(*out);

      ndn::Name cacheKey("/cache");
      cacheKey.append(m_switchID);
      for (size_t i = 0; i < name.size(); ++i) cacheKey.append(name.get(i));
      const std::string cKey = cacheKey.toUri();
      if (m_dCache.find(cKey) == m_dCache.end()) {
        if (m_dCache.size() >= m_dCacheCap && !m_dCacheOrder.empty()) {
          const std::string& oldest = m_dCacheOrder.front();
          m_dCache.erase(oldest);
          m_dCacheOrder.pop_front();
          ++m_evictionsSinceHB;
          ++m_evictionsTotal;
        }
        m_dCacheOrder.push_back(cKey);
      }
      m_dCache[cKey] = out;

      NS_LOG_INFO("Switch [" << m_switchID << "]: d-wrap " << name << " -> " << responseName);
      Simulator::Cancel(pIt->second.timer);
      m_dPending.erase(pIt);
      m_transmittedDatas(out, this, m_face);
      m_face->onReceiveData(*out);
      return;
    }
  }

  if (name.size() >= 4 && name.get(0).toUri() == "cache" &&
      name.get(2).toUri() == "data" && name.get(3).toUri() == "content")
  {

    ndn::Name original("/u");
    for (size_t i = 2; i < name.size(); ++i) original.append(name.get(i));

    auto out = std::make_shared<ndn::Data>(original);
    out->setContent(data->getContent());
    out->setFreshnessPeriod(data->getFreshnessPeriod());

    if (auto ns3tag = data->getTag<ndn::Ns3PacketTag>()) out->setTag(ns3tag);

    out->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
    ndn::StackHelper::getKeyChain().sign(*out);

    NS_LOG_INFO("Switch [" << m_switchID << "]: gw-unwrap " << name << " -> " << original);

    auto it = m_pendingGw.find(name.toUri());
    if (it != m_pendingGw.end()) {
      Simulator::Cancel(it->second.timer);
      m_pendingGw.erase(it);
    }

    m_transmittedDatas(out, this, m_face);
    m_face->onReceiveData(*out);
    return;
  }

  if (name.size() >= 4 &&
      name.get(0).toUri() == "xcontrol" &&
      name.get(2).toUri() == m_switchID &&
      name.get(3).toUri() == "heartbeat")
  {
    const std::string peer = name.get(1).toUri();

    if (auto* n = findNeighbor(peer)) {
      n->status = Status::ACTIVE;
      n->misses = 0;
      NS_LOG_INFO("Switch [" << m_switchID << "]: hb-reply from neighbor " << peer);
      return;
    }

    if (auto* c = findController(peer)) {
      c->status = Status::ACTIVE;
      c->misses = 0;

      uint64_t observedEpoch = 0;
      const std::string payload = BlockToString(data->getContent());
      if (ExtractUintAfterKey(payload, "epoch=", observedEpoch)) {
        const bool newEpoch = (observedEpoch > c->lastObservedEpoch);
        c->lastObservedEpoch = observedEpoch;
        c->lastAdvTime = Simulator::Now();

        NS_LOG_INFO("Switch [" << m_switchID << "]: hb-reply from controller "
                    << peer << " epoch=" << observedEpoch
                    << " applied=" << c->lastAppliedEpoch
                    << (newEpoch ? " (new)" : ""));

        if (c->lastAppliedEpoch < observedEpoch) {
          SendFibSync(peer, c->lastAppliedEpoch);
        }
      } else {
        NS_LOG_INFO("Switch [" << m_switchID << "]: hb-reply from controller "
                    << peer << " (no epoch field)");
      }
      return;
    }
  }

  if (name.size() >= 5 &&
      name.get(0).toUri() == "xcontrol" &&
      name.get(2).toUri() == m_switchID &&
      name.get(3).toUri() == "fib" &&
      name.get(4).toUri() == "sync")
  {
    const std::string ctrl = name.get(1).toUri();
    if (auto* c = findController(ctrl)) {
      const std::string payload = BlockToString(data->getContent());

      uint64_t newEpoch = 0;

      if (ExtractUintAfterKey(payload, "apply-epoch=", newEpoch)) {
        c->lastAppliedEpoch = newEpoch;
        c->lastSyncTime = Simulator::Now();

        double pullDelay = 0.0;
        if (!c->lastAdvTime.IsZero())
          pullDelay = (c->lastSyncTime - c->lastAdvTime).GetSeconds();

        NS_LOG_INFO("Switch [" << m_switchID << "]: FIB applied epoch="
                    << newEpoch << " from controller " << ctrl
                    << " pullDelay=" << pullDelay << "s");
      }
      else {
        if (ExtractUintAfterKey(payload, "epoch=", newEpoch)) {
          c->lastAppliedEpoch = newEpoch;
          NS_LOG_INFO("Switch [" << m_switchID << "]: FIB up-to-date epoch="
                      << newEpoch << " with controller " << ctrl);
        }
     }

    }
    return;
  }

  NS_LOG_INFO("Switch [" << m_switchID << "]: Data " << name);
}

void
SwitchApp::SendNeighborsHeartbeats()
{
  bool startedWindow = false;

  for (auto& n : m_neighbors) {
    if (n.status == Status::INACTIVE) continue;

    const std::string nm = "/xcontrol/" + n.neighborID + "/" + m_switchID + "/heartbeat";
    auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
    interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
    WIRELOG("Switch", m_switchID, "TX", "I", interest->getName().toUri());

    m_transmittedInterests(interest, this, m_face);

    m_face->onReceiveInterest(*interest);
    if (n.misses == 0) {
      n.status = Status::W1;
      n.misses = 1;
      startedWindow = true;
    }
  }

  m_nbPeriodicEvent = Simulator::Schedule(Seconds(m_hbInterval),
                                          &SwitchApp::SendNeighborsHeartbeats, this);

  if (startedWindow) {
    if (m_nbRetryEvent.IsRunning()) m_nbRetryEvent.Cancel();
    m_nbRetryEvent = Simulator::Schedule(Seconds(m_hbTimeout),
                                         &SwitchApp::RetryNeighborsHeartbeats, this);
    NS_LOG_INFO("Switch [" << m_switchID << "]: HB(neighbors) window armed for "
                << m_hbRepeat << "x" << m_hbTimeout << "s");
  } else {
    if (m_nbRetryEvent.IsRunning()) m_nbRetryEvent.Cancel();
  }
}

void
SwitchApp::RetryNeighborsHeartbeats()
{
  bool anyPending = false;

  for (auto& n : m_neighbors) {
    if (!(n.status == Status::W1 || n.status == Status::W2 || n.status == Status::W3))
      continue;

    if (n.misses >= m_hbRepeat) {

      n.status = Status::INACTIVE;
      NS_LOG_INFO("Switch [" << m_switchID << "]: Neighbor " << n.neighborID << " INACTIVE");
      for (const auto& c : m_controllers) {
        if (c.status == Status::INACTIVE) continue;
        const std::string nm = "/xcontrol/" + c.controllerID + "/" + m_switchID
                             + "/inactiveneighbour/" + n.neighborID;
        auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
        interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
        WIRELOG("Switch", m_switchID, "TX", "I", interest->getName().toUri());

        m_transmittedInterests(interest, this, m_face);

        m_face->onReceiveInterest(*interest);
}
      continue;
    }

    n.misses += 1;
    n.status = static_cast<Status>(std::min<int>(static_cast<int>(Status::W3),
                                                 static_cast<int>(n.status) + 1));
    const std::string nm = "/xcontrol/" + n.neighborID + "/" + m_switchID + "/heartbeat";
    auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
    interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
    WIRELOG("Switch", m_switchID, "TX", "I", interest->getName().toUri());

    m_transmittedInterests(interest, this, m_face);

    m_face->onReceiveInterest(*interest);
anyPending = true;
  }

  if (anyPending) {
    m_nbRetryEvent = Simulator::Schedule(Seconds(m_hbTimeout),
                                         &SwitchApp::RetryNeighborsHeartbeats, this);
  } else {
    NS_LOG_INFO("Switch [" << m_switchID << "]: HB(neighbors) window complete");
  }
}

void
SwitchApp::SendControllersHeartbeats()
{
  bool startedWindow = false;

  const std::string gwStats = BuildGwStatsComponent();
  const std::string evStats = BuildEvStatsComponent();
  const std::string dhStats = BuildDhStatsComponent();

  for (auto& c : m_controllers) {
    if (c.status == Status::INACTIVE) continue;

    const std::string nm = "/xcontrol/" + c.controllerID + "/" + m_switchID
                         + "/heartbeat/" + gwStats + "/" + evStats + "/" + dhStats;
    auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
    interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
    WIRELOG("Switch", m_switchID, "TX", "I", interest->getName().toUri());

    m_transmittedInterests(interest, this, m_face);

    m_face->onReceiveInterest(*interest);
if (c.misses == 0) {
      c.status = Status::W1;
      c.misses = 1;
      startedWindow = true;
    }
  }

  ResetHbCounters();

  m_ctrlPeriodicEvent = Simulator::Schedule(Seconds(m_hbInterval),
                                            &SwitchApp::SendControllersHeartbeats, this);

  if (startedWindow) {
    if (m_ctrlRetryEvent.IsRunning()) m_ctrlRetryEvent.Cancel();
    m_ctrlRetryEvent = Simulator::Schedule(Seconds(m_hbTimeout),
                                           &SwitchApp::RetryControllersHeartbeats, this);
    NS_LOG_INFO("Switch [" << m_switchID << "]: HB(controllers) window armed for "
                << m_hbRepeat << "x" << m_hbTimeout << "s");
  } else {
    if (m_ctrlRetryEvent.IsRunning()) m_ctrlRetryEvent.Cancel();
  }
}

void
SwitchApp::RetryControllersHeartbeats()
{
  bool anyPending = false;

  for (auto& c : m_controllers) {
    if (!(c.status == Status::W1 || c.status == Status::W2 || c.status == Status::W3))
      continue;

    if (c.misses >= m_hbRepeat) {

      c.status = Status::INACTIVE;
      NS_LOG_INFO("Switch [" << m_switchID << "]: Controller " << c.controllerID << " INACTIVE");

      for (const auto& other : m_controllers) {
        if (other.controllerID == c.controllerID || other.status == Status::INACTIVE) continue;
        const std::string nm = "/xcontrol/" + other.controllerID + "/" + m_switchID
                             + "/inactivecontroller/" + c.controllerID;
        auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
        interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
        WIRELOG("Switch", m_switchID, "TX", "I", interest->getName().toUri());

        m_transmittedInterests(interest, this, m_face);

        m_face->onReceiveInterest(*interest);
}
      continue;
    }

    c.misses += 1;
    c.status = static_cast<Status>(std::min<int>(static_cast<int>(Status::W3),
                                                 static_cast<int>(c.status) + 1));

    const std::string nm = "/xcontrol/" + c.controllerID + "/" + m_switchID + "/heartbeat";
    auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
    interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
    WIRELOG("Switch", m_switchID, "TX", "I", interest->getName().toUri());

    m_transmittedInterests(interest, this, m_face);

    m_face->onReceiveInterest(*interest);
anyPending = true;
  }

  if (anyPending) {
    m_ctrlRetryEvent = Simulator::Schedule(Seconds(m_hbTimeout),
                                           &SwitchApp::RetryControllersHeartbeats, this);
  } else {
    NS_LOG_INFO("Switch [" << m_switchID << "]: HB(controllers) window complete");
  }
}

void
SwitchApp::SendFibSync(const std::string& controllerId, uint64_t lastEpoch)
{
  const std::string nm = "/xcontrol/" + controllerId + "/" + m_switchID
                       + "/fib/sync/" + std::to_string(lastEpoch);
  auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
  interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);

  NS_LOG_INFO("Switch [" << m_switchID << "]: fib/sync -> " << controllerId
              << " lastEpoch=" << lastEpoch);

  WIRELOG("Switch", m_switchID, "TX", "I", interest->getName().toUri());

  m_transmittedInterests(interest, this, m_face);

  m_face->onReceiveInterest(*interest);
}

void SwitchApp::SetCacheTable(const std::vector<std::string>& table)
{
  m_cacheTable = table;
  std::ostringstream oss;
  for (size_t i = 0; i < m_cacheTable.size(); ++i) {
    if (i) oss << ",";
    oss << i << ":" << m_cacheTable[i];
  }
  NS_LOG_DEBUG("Switch [" << m_switchID << "]: cache table [" << oss.str() << "]");
}

void SwitchApp::SendCacheTestInterest(const std::string& targetSwitchID)
{
  if (!m_active) return;
  ndn::Name name("/cache");
  name.append(targetSwitchID).append("test").append(m_switchID);
  auto interest = std::make_shared<ndn::Interest>(name);
  interest->setMustBeFresh(false);
  interest->setInterestLifetime(ndn::time::seconds(5));
  static thread_local std::random_device rd;
  static thread_local std::mt19937 gen(rd());
  interest->setNonce(gen());

  NS_LOG_INFO("Switch [" << m_switchID << "]: cache-test TX " << name);
  m_transmittedInterests(interest, this, m_face);
  m_face->onReceiveInterest(*interest);
}

void SwitchApp::AddNeighbor(const std::string& nID)
{
  if (!findNeighbor(nID)) {
    NeighborInfo info;
    info.neighborID = nID;
    info.status = Status::ACTIVE;
    info.misses = 0;
    m_neighbors.push_back(info);

    NS_LOG_INFO("Switch [" << m_switchID << "]: add neighbor " << nID);
  }
}

void SwitchApp::RemoveNeighbor(const std::string& nID)
{
  auto it = std::remove_if(m_neighbors.begin(), m_neighbors.end(),
                           [&](const NeighborInfo& e){ return e.neighborID == nID; });
  if (it != m_neighbors.end()) {
    m_neighbors.erase(it, m_neighbors.end());
    NS_LOG_INFO("Switch [" << m_switchID << "]: remove neighbor " << nID);
  }
}

void SwitchApp::ActivateNeighbor(const std::string& nID)
{
  if (auto* n = findNeighbor(nID)) {
    const bool wasInactive = (n->status == Status::INACTIVE);
    n->status = Status::ACTIVE;
    n->misses = 0;
    NS_LOG_INFO("Switch [" << m_switchID << "]: neighbor " << nID << " ACTIVE");

    if (wasInactive) {
      for (const auto& c : m_controllers) {
        if (c.status == Status::INACTIVE) continue;
        const std::string nm = "/xcontrol/" + c.controllerID + "/" + m_switchID
                             + "/activeneighbour/" + nID;
        auto rep = std::make_shared<ndn::Interest>(ndn::Name(nm));
        rep->setInterestLifetime(ndn::time::seconds(1)); rep->setMustBeFresh(true);
        WIRELOG("Switch", m_switchID, "TX", "I", rep->getName().toUri());
        m_transmittedInterests(rep, this, m_face);
        m_face->onReceiveInterest(*rep);
      }
    }
  }
}

void SwitchApp::ReactivateAllNeighbors()
{

  std::vector<std::string> ids;
  ids.reserve(m_neighbors.size());
  for (const auto& n : m_neighbors) ids.push_back(n.neighborID);
  for (const auto& id : ids) ActivateNeighbor(id);
  NS_LOG_INFO("Switch [" << m_switchID << "]: ReactivateAllNeighbors over "
              << ids.size() << " neighbor(s)");
}

void SwitchApp::DeactivateNeighbor(const std::string& nID)
{
  if (auto* n = findNeighbor(nID)) {
    n->status = Status::INACTIVE;
    NS_LOG_INFO("Switch [" << m_switchID << "]: neighbor " << nID << " INACTIVE");
  }
}

void SwitchApp::AddController(const std::string& cID)
{
  if (!findController(cID)) {
    ControllerInfo info;
    info.controllerID = cID;
    info.status = Status::ACTIVE;
    info.misses = 0;
    m_controllers.push_back(info);

    NS_LOG_INFO("Switch [" << m_switchID << "]: add controller " << cID);
  }
}

void SwitchApp::RemoveController(const std::string& cID)
{
  auto it = std::remove_if(m_controllers.begin(), m_controllers.end(),
                           [&](const ControllerInfo& e){ return e.controllerID == cID; });
  if (it != m_controllers.end()) {
    m_controllers.erase(it, m_controllers.end());
    NS_LOG_INFO("Switch [" << m_switchID << "]: remove controller " << cID);
  }
}

void SwitchApp::ActivateController(const std::string& cID)
{
  if (auto* c = findController(cID)) {
    c->status = Status::ACTIVE;
    c->misses = 0;
    NS_LOG_INFO("Switch [" << m_switchID << "]: controller " << cID << " ACTIVE");
  }
}

void SwitchApp::DeactivateController(const std::string& cID)
{
  if (auto* c = findController(cID)) {
    c->status = Status::INACTIVE;
    NS_LOG_INFO("Switch [" << m_switchID << "]: controller " << cID << " INACTIVE");
  }
}

void SwitchApp::SetEdge(bool edge)
{
  m_edge = edge;
  NS_LOG_INFO("Switch [" << m_switchID << "]: set edge=" << (edge ? 1 : 0));
}

SwitchApp::ControllerInfo*
SwitchApp::findController(const std::string& cID)
{
  for (auto& c : m_controllers) if (c.controllerID == cID) return &c;
  return nullptr;
}

SwitchApp::NeighborInfo*
SwitchApp::findNeighbor(const std::string& nID)
{
  for (auto& n : m_neighbors) if (n.neighborID == nID) return &n;
  return nullptr;
}

std::string
SwitchApp::BlockToString(const ndn::Block& b)
{
  return std::string(reinterpret_cast<const char*>(b.value()), b.value_size());
}

bool
SwitchApp::ExtractUintAfterKey(const std::string& text, const std::string& key, uint64_t& out)
{
  const auto pos = text.find(key);
  if (pos == std::string::npos) return false;
  size_t i = pos + key.size();
  uint64_t val = 0;
  bool seen = false;
  while (i < text.size() && isdigit(static_cast<unsigned char>(text[i]))) {
    seen = true;
    val = val * 10 + (text[i] - '0');
    ++i;
  }
  if (!seen) return false;
  out = val;
  return true;
}

void
SwitchApp::OnPendingGwTimeout(std::string wrappedUri)
{
  auto it = m_pendingGw.find(wrappedUri);
  if (it == m_pendingGw.end()) return;
  NS_LOG_WARN("Switch [" << m_switchID << "]: gw-pending TIMEOUT "
              << wrappedUri << " (original=" << it->second.originalUri
              << ") -- wrapped Data lost or delayed past " << m_pendingGwTimeoutS << " s");
  m_pendingGw.erase(it);
}

void
SwitchApp::OnDPendingTimeout(std::string innerUri)
{
  auto it = m_dPending.find(innerUri);
  if (it == m_dPending.end()) return;
  NS_LOG_WARN("Switch [" << m_switchID << "]: d-pending TIMEOUT "
              << innerUri << " (wrapped=" << it->second.wrappedUri
              << ") -- producer Data lost; future Interests will retry");
  m_dPending.erase(it);
}

std::string
SwitchApp::BuildGwStatsComponent() const
{

  std::ostringstream oss;
  oss << "gw=";
  const size_t n = m_cacheTable.empty() ? 10 : m_cacheTable.size();
  for (size_t b = 0; b < n; ++b) {
    if (b) oss << ",";
    auto it = m_bucketDeltas.find(b);
    oss << (it != m_bucketDeltas.end() ? it->second : uint64_t{0});
  }
  return oss.str();
}

std::string
SwitchApp::BuildEvStatsComponent() const
{

  std::ostringstream oss;
  oss << "ev=" << m_evictionsSinceHB;
  return oss.str();
}

std::string
SwitchApp::BuildDhStatsComponent() const
{

  std::ostringstream oss;
  oss << "dh=";
  bool first = true;
  for (const auto& kv : m_dHitsPerBucketHB) {
    if (kv.second == 0) continue;
    if (!first) oss << ",";
    oss << kv.first << ":" << kv.second;
    first = false;
  }
  if (first) oss << "_";
  return oss.str();
}

void
SwitchApp::ResetHbCounters()
{
  m_bucketDeltas.clear();
  m_dHitsPerBucketHB.clear();
  m_evictionsSinceHB = 0;
  m_dHitsSinceHB = 0;
}

}
