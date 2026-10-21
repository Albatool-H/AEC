#include "controller-app.hpp"
#include "control-tracer.hpp"

#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/utils/ndn-ns3-packet-tag.hpp"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.hpp"
#include "ndn-cxx/security/key-chain.hpp"
#include "ns3/simulator.h"
#include "ns3/log.h"

#include <sstream>
#include <algorithm>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <limits>
#include "ns3/string.h"

NS_LOG_COMPONENT_DEFINE("ControllerApp");

static std::string classifyCtrlMsg(const std::string& nameStr) {
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
      ns3::ndn::ControlTracer::LogTx(ns3::Simulator::Now().GetSeconds(), id, _p, classifyCtrlMsg(_n), _n); \
    else \
      ns3::ndn::ControlTracer::LogRx(ns3::Simulator::Now().GetSeconds(), id, _p, classifyCtrlMsg(_n), _n); \
  } while(0)

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED(ControllerApp);

TypeId
ControllerApp::GetTypeId()
{
  static TypeId tid = TypeId("ControllerApp")
    .SetParent<ndn::App>()
    .AddConstructor<ControllerApp>()
    .AddAttribute("ControllerID", "ID of the controller",
                  StringValue("C1"),
                  MakeStringAccessor(&ControllerApp::m_controllerId),
                  MakeStringChecker())
    .AddAttribute("HeartbeatFirst", "First heartbeat delay in seconds",
                  DoubleValue(10.0),
                  MakeDoubleAccessor(&ControllerApp::m_hbFirst),
                  MakeDoubleChecker<double>())
    .AddAttribute("HeartbeatInterval", "Periodic heartbeat interval in seconds",
                  DoubleValue(60.0),
                  MakeDoubleAccessor(&ControllerApp::m_hbInterval),
                  MakeDoubleChecker<double>())
    .AddAttribute("HeartbeatTimeout", "Retry timeout between W1/W2/... checks in seconds",
                  DoubleValue(10.0),
                  MakeDoubleAccessor(&ControllerApp::m_hbTimeout),
                  MakeDoubleChecker<double>())
    .AddAttribute("HeartbeatRepeat", "Number of retries before marking INACTIVE",
                  UintegerValue(3),
                  MakeUintegerAccessor(&ControllerApp::m_hbRepeat),
                  MakeUintegerChecker<uint32_t>())

    .AddAttribute("PendingTimeout",
                  "Trigger A: declare dead if any live nbr report is older than this (s)",
                  DoubleValue(120.0),
                  MakeDoubleAccessor(&ControllerApp::m_pendingTimeout),
                  MakeDoubleChecker<double>())
    .AddAttribute("HbSilenceTimeout",
                  "Trigger A: declare dead if no heartbeat received from sw for this long (s)",
                  DoubleValue(180.0),
                  MakeDoubleAccessor(&ControllerApp::m_hbSilenceTimeout),
                  MakeDoubleChecker<double>())

    .AddAttribute("UseRlBandit",
                  "Route OnDead/OnAlive decisions through the VW bandit bridge",
                  BooleanValue(false),
                  MakeBooleanAccessor(&ControllerApp::m_useRlBandit),
                  MakeBooleanChecker())
    .AddAttribute("RlBridgeDir",
                  "Directory under which decision_<id>.req/.resp and feedback files live",
                  StringValue("rl-bridge"),
                  MakeStringAccessor(&ControllerApp::m_rlBridgeDir),
                  MakeStringChecker())
    .AddAttribute("RlPollSeconds",
                  "Wall-clock poll cadence (s) while awaiting VW response",
                  DoubleValue(0.005),
                  MakeDoubleAccessor(&ControllerApp::m_rlPollSeconds),
                  MakeDoubleChecker<double>())
    .AddAttribute("RlPollMaxWait",
                  "Hard timeout per decision when waiting for VW response (s)",
                  DoubleValue(5.0),
                  MakeDoubleAccessor(&ControllerApp::m_rlPollMaxWait),
                  MakeDoubleChecker<double>())

    .AddAttribute("RlWEviction",
                  "Reward weight on (eviction_before - eviction_after)",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&ControllerApp::m_w_eviction),
                  MakeDoubleChecker<double>())
    .AddAttribute("RlWCost",
                  "Reward weight on the live-move cost penalty",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&ControllerApp::m_w_cost),
                  MakeDoubleChecker<double>())
    .AddAttribute("RlPeriodicInterval",
                  "Periodic LIVE-branch decision interval (s). 0 = follow 2 * hbInterval.",
                  DoubleValue(0.0),
                  MakeDoubleAccessor(&ControllerApp::m_rlPeriodicInterval),
                  MakeDoubleChecker<double>());
  return tid;
}

ControllerApp::ControllerApp() {}
ControllerApp::~ControllerApp() {}

void
ControllerApp::StartApplication()
{
  ndn::App::StartApplication();

  m_controlPrefix  = "/control/"  + m_controllerId + "/";
  m_xcontrolPrefix = "/xcontrol/" + m_controllerId + "/";

  ndn::FibHelper::AddRoute(GetNode(), m_controlPrefix,  m_face, 0);
  ndn::FibHelper::AddRoute(GetNode(), m_xcontrolPrefix, m_face, 0);

  NS_LOG_INFO("Controller [" << m_controllerId << "]: start t="
              << Simulator::Now().GetSeconds() << "s control="
              << m_controlPrefix << " xcontrol=" << m_xcontrolPrefix);

  m_hbPeriodicEvent = Simulator::Schedule(Seconds(m_hbFirst),
                                          &ControllerApp::SendControllerHeartbeats, this);

  m_deadTimeoutCheckEvent =
    Simulator::Schedule(Seconds(m_hbFirst + 30.0),
                        &ControllerApp::CheckDeadTimeouts, this);

  if (m_useRlBandit) {
    const double interval = (m_rlPeriodicInterval > 0.0)
                            ? m_rlPeriodicInterval
                            : 2.0 * m_hbInterval;
    m_rlPeriodicEvent =
      Simulator::Schedule(Seconds(m_hbFirst + interval),
                          &ControllerApp::RlPeriodicCheck, this);
  }
}

void
ControllerApp::StopApplication()
{
  if (m_hbPeriodicEvent.IsRunning())     m_hbPeriodicEvent.Cancel();
  if (m_hbRetryEvent.IsRunning())        m_hbRetryEvent.Cancel();
  if (m_deadTimeoutCheckEvent.IsRunning()) m_deadTimeoutCheckEvent.Cancel();
  if (m_rlPeriodicEvent.IsRunning())     m_rlPeriodicEvent.Cancel();
  NS_LOG_INFO("Controller [" << m_controllerId << "]: stop t="
              << Simulator::Now().GetSeconds() << "s");
  ndn::App::StopApplication();
}

ControllerApp::ManagedSwitch*
ControllerApp::getSwitch(const std::string& id)
{
  auto it = m_switches.find(id);
  return (it == m_switches.end()) ? nullptr : &it->second;
}

ControllerApp::CtrlNeighbor*
ControllerApp::getNeighbor(const std::string& id)
{
  for (auto& n : m_neighbors)
    if (n.controllerId == id)
      return &n;
  return nullptr;
}

void
ControllerApp::OnInterest(std::shared_ptr<const ndn::Interest> interest)
{
  ndn::App::OnInterest(interest);
  const auto& name = interest->getName();
  WIRELOG("Controller", m_controllerId, "RX", "I", name.toUri());
  const auto root = (name.size() > 0) ? name.get(0).toUri() : "";
  NS_LOG_INFO("Controller [" << m_controllerId << "]: Interest " << name);

  if (name.size() >= 4 && name.get(3).toUri() == "heartbeat"
      && (root == "control" || root == "xcontrol")) {
    int hop = 0;
    auto ns3tag = interest->getTag<ndn::Ns3PacketTag>();
    if (ns3tag) {
      ::ns3::ndn::FwHopCountTag hopTag;
      if (ns3tag->getPacket()->PeekPacketTag(hopTag)) hop = hopTag.Get();
    }
    if (hop > 1) {
      NS_LOG_INFO("Controller [" << m_controllerId << "]: drop non-neighbor heartbeat "
                  << name << " (hop=" << hop << ")");
      return;
    }
  }

  if (root == "xcontrol") {

    if (name.size() >= 4 &&
        name.get(1).toUri() == m_controllerId &&
        name.get(3).toUri() == "heartbeat")
    {
      const std::string sw = name.get(2).toUri();

      if (getSwitch(sw) != nullptr) {
        m_lastHbReceived[sw] = Simulator::Now();
      }

      ParseSwitchHbStats(name, sw);
      ReplySwitchHeartbeat(name, sw);
      return;
    }

    if (name.size() >= 5 &&
        name.get(1).toUri() == m_controllerId &&
        name.get(3).toUri() == "inactiveneighbour")
    {
      const std::string sw    = name.get(2).toUri();
      const std::string neigh = name.get(4).toUri();

      if (getSwitch(neigh) != nullptr) {
        auto& reporters = m_inactiveReports[neigh];
        if (reporters.empty()) {
          m_pendingSince[neigh] = Simulator::Now();
        }
        reporters.insert(sw);
        auto nbrIt = m_switchNeighborsInCluster.find(neigh);
        size_t liveQuorum = 0;
        if (nbrIt != m_switchNeighborsInCluster.end()) {
          for (const auto& n : nbrIt->second) {
            if (m_deadSwitches.find(n) == m_deadSwitches.end()) ++liveQuorum;
          }
        }
        NS_LOG_INFO("Controller [" << m_controllerId << "]: inactiveneighbour "
                    << neigh << " reported by " << sw
                    << " (" << reporters.size() << "/" << liveQuorum
                    << " live nbrs)");
        if (liveQuorum > 0 && reporters.size() >= liveQuorum
            && m_deadSwitches.find(neigh) == m_deadSwitches.end()) {
          m_deadSwitches.insert(neigh);
          m_pendingSince.erase(neigh);
          OnDeadNode(neigh);
        }
      } else {
        NS_LOG_INFO("Controller [" << m_controllerId
                    << "]: inactiveneighbour (cross-cluster) "
                    << neigh << " reported by " << sw << " -- ignored");
      }

      auto data = std::make_shared<ndn::Data>(name);
      data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
      static const std::string content = "ack: inactive-neighbour";
      data->setContent(reinterpret_cast<const uint8_t*>(content.data()), content.size());
      data->setFreshnessPeriod(ndn::time::seconds(5));
      ndn::StackHelper::getKeyChain().sign(*data);
      WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

      m_transmittedDatas(data, this, m_face);

      m_face->onReceiveData(*data);
return;
    }

    if (name.size() >= 5 &&
        name.get(1).toUri() == m_controllerId &&
        (name.get(3).toUri() == "activeneighbour"
         || name.get(3).toUri() == "newneighbour"))
    {
      const std::string sw    = name.get(2).toUri();
      const std::string peer  = name.get(4).toUri();
      const std::string kind  = name.get(3).toUri();

      if (getSwitch(peer) != nullptr) {
        m_inactiveReports.erase(peer);
        m_pendingSince.erase(peer);

        const bool wasDead = (m_deadSwitches.erase(peer) > 0);
        NS_LOG_INFO("Controller [" << m_controllerId << "]: " << kind << " "
                    << peer << " from " << sw
                    << (wasDead ? " (was dead -> alive)" : ""));
        if (wasDead || kind == "newneighbour") {
          OnAliveNode(peer);
        }
      } else {
        NS_LOG_INFO("Controller [" << m_controllerId << "]: "
                    << kind << " (cross-cluster) " << peer
                    << " from " << sw << " -- ignored");
      }

      auto data = std::make_shared<ndn::Data>(name);
      data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
      static const std::string content = "ack: peer-up";
      data->setContent(reinterpret_cast<const uint8_t*>(content.data()), content.size());
      data->setFreshnessPeriod(ndn::time::seconds(5));
      ndn::StackHelper::getKeyChain().sign(*data);
      WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

      m_transmittedDatas(data, this, m_face);
      m_face->onReceiveData(*data);
      return;
    }

    if (name.size() >= 5 &&
        name.get(1).toUri() == m_controllerId &&
        name.get(3).toUri() == "inactivecontroller")
    {
      const std::string sw   = name.get(2).toUri();
      const std::string down = name.get(4).toUri();

      RecomputeAndBumpEpoch("inactive-controller:" + down + " (via " + sw + ")");

      auto data = std::make_shared<ndn::Data>(name);
      data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
      static const std::string content = "ack: inactive-controller";
      data->setContent(reinterpret_cast<const uint8_t*>(content.data()), content.size());
      data->setFreshnessPeriod(ndn::time::seconds(5));
      ndn::StackHelper::getKeyChain().sign(*data);
      WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

      m_transmittedDatas(data, this, m_face);

      m_face->onReceiveData(*data);
return;
    }

    if (name.size() >= 5 &&
        name.get(1).toUri() == m_controllerId &&
        name.get(3).toUri() == "fib" &&
        name.get(4).toUri() == "sync")
    {
      const std::string sw = name.get(2).toUri();
      uint64_t lastEpoch = 0;
      if (name.size() >= 6) {
        std::stringstream ss(name.get(5).toUri());
        ss >> lastEpoch;
      }
      ReplyFibSync(name, sw, lastEpoch);
      return;
    }
  }

  if (root == "control") {

    if (name.size() >= 4 &&
        name.get(1).toUri() == m_controllerId &&
        name.get(3).toUri() == "heartbeat")
    {
      const std::string src = name.get(2).toUri();
      ReplyControllerHeartbeat(name, src);
      return;
    }

    if (name.size() >= 5 &&
        name.get(1).toUri() == m_controllerId &&
        name.get(3).toUri() == "inactivecontroller")
    {
      const std::string src  = name.get(2).toUri();
      const std::string down = name.get(4).toUri();

      RecomputeControlPlaneOnly("inactive-controller:" + down + " (via " + src + ")");

      auto data = std::make_shared<ndn::Data>(name);
      data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
      static const std::string content = "ack: inactive-controller";
      data->setContent(reinterpret_cast<const uint8_t*>(content.data()), content.size());
      data->setFreshnessPeriod(ndn::time::seconds(5));
      ndn::StackHelper::getKeyChain().sign(*data);
      WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

      m_transmittedDatas(data, this, m_face);

      m_face->onReceiveData(*data);
return;
    }

    if (name.size() >= 5 &&
        name.get(1).toUri() == m_controllerId &&
        name.get(3).toUri() == "fib" &&
        name.get(4).toUri() == "sync")
    {
      const std::string src = name.get(2).toUri();
      uint64_t lastEpoch = 0;
      if (name.size() >= 6) {
        std::stringstream ss(name.get(5).toUri());
        ss >> lastEpoch;
      }
      ReplyControllerFibSync(name, src, lastEpoch);
      return;
    }
  }

  auto data = std::make_shared<ndn::Data>(name);
  data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
  const std::string content = "Controller " + m_controllerId;
  data->setContent(reinterpret_cast<const uint8_t*>(content.data()), content.size());
  data->setFreshnessPeriod(ndn::time::seconds(1));
  ndn::StackHelper::getKeyChain().sign(*data);
  WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

  m_transmittedDatas(data, this, m_face);

  m_face->onReceiveData(*data);
}

void
ControllerApp::OnData(std::shared_ptr<const ndn::Data> data)
{
  ndn::App::OnData(data);
  const auto& name = data->getName();
  WIRELOG("Controller", m_controllerId, "RX", "D", name.toUri());

  if (name.size() >= 4 &&
      name.get(0).toUri() == "control" &&
      name.get(2).toUri() == m_controllerId &&
      name.get(3).toUri() == "heartbeat")
  {
    const std::string nbr = name.get(1).toUri();
    if (auto* n = getNeighbor(nbr)) {
      n->status = PeerStatus::ACTIVE;
      n->misses = 0;

      const ndn::Block& blk = data->getContent();
      std::string payload(reinterpret_cast<const char*>(blk.value()), blk.value_size());
      uint64_t peerEpoch = 0;
      auto pos = payload.find("epoch=");
      if (pos != std::string::npos) {
        size_t i = pos + 6;
        while (i < payload.size() && isdigit(static_cast<unsigned char>(payload[i]))) {
          peerEpoch = peerEpoch * 10 + (payload[i] - '0');
          ++i;
        }
      }

      if (peerEpoch > n->lastObservedEpoch) {
        NS_LOG_INFO("Controller [" << m_controllerId << "]: peer " << nbr
                    << " epoch=" << peerEpoch << " (was " << n->lastObservedEpoch << ")");
        SendControllerFibSync(nbr, n->lastObservedEpoch);
        n->lastObservedEpoch = peerEpoch;
      } else {
        NS_LOG_INFO("Controller [" << m_controllerId << "]: hb-reply from " << nbr
                    << " epoch=" << peerEpoch << " (up to date)");
      }
    }
  }

  if (name.size() >= 5 &&
      name.get(0).toUri() == "control" &&
      name.get(2).toUri() == m_controllerId &&
      name.get(3).toUri() == "fib" &&
      name.get(4).toUri() == "sync")
  {
    const std::string peer = name.get(1).toUri();
    const ndn::Block& blk = data->getContent();
    std::string payload(reinterpret_cast<const char*>(blk.value()), blk.value_size());
    NS_LOG_INFO("Controller [" << m_controllerId << "]: FIB sync reply from " << peer
                << " payload=" << payload);
  }
}

void
ControllerApp::SendControllerHeartbeats()
{
  bool startedWindow = false;

  for (auto& n : m_neighbors) {
    if (n.status == PeerStatus::INACTIVE) continue;

    const std::string nm = "/control/" + n.controllerId + "/" + m_controllerId + "/heartbeat";
    auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
    interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
    WIRELOG("Controller", m_controllerId, "TX", "I", interest->getName().toUri());

    m_transmittedInterests(interest, this, m_face);

    m_face->onReceiveInterest(*interest);
if (n.misses == 0) {
      n.status = PeerStatus::W1;
      n.misses = 1;
      startedWindow = true;
    }
  }

  m_hbPeriodicEvent = Simulator::Schedule(Seconds(m_hbInterval),
                                          &ControllerApp::SendControllerHeartbeats, this);

  if (startedWindow) {
    if (m_hbRetryEvent.IsRunning()) m_hbRetryEvent.Cancel();
    m_hbRetryEvent = Simulator::Schedule(Seconds(m_hbTimeout),
                                         &ControllerApp::RetryControllerHeartbeats, this);
    NS_LOG_INFO("Controller [" << m_controllerId << "]: HB window armed for "
                << m_hbRepeat << "x" << m_hbTimeout << "s");
  } else {
    if (m_hbRetryEvent.IsRunning()) m_hbRetryEvent.Cancel();
  }
}

void
ControllerApp::RetryControllerHeartbeats()
{
  bool anyPending = false;

  for (auto& n : m_neighbors) {
    if (!(n.status == PeerStatus::W1 || n.status == PeerStatus::W2 || n.status == PeerStatus::W3))
      continue;

    if (n.misses >= m_hbRepeat) {

      n.status = PeerStatus::INACTIVE;
      NS_LOG_INFO("Controller [" << m_controllerId << "]: neighbor "
                  << n.controllerId << " INACTIVE");

      RecomputeControlPlaneOnly("inactive-controller:" + n.controllerId + " (self-detected)");

      for (auto& other : m_neighbors) {
        if (other.controllerId == n.controllerId || other.status == PeerStatus::INACTIVE) continue;
        const std::string nm = "/control/" + other.controllerId + "/" + m_controllerId
                             + "/inactivecontroller/" + n.controllerId;
        auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
        interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
        WIRELOG("Controller", m_controllerId, "TX", "I", interest->getName().toUri());

        m_transmittedInterests(interest, this, m_face);

        m_face->onReceiveInterest(*interest);
}
      continue;
    }

    n.misses += 1;
    n.status = static_cast<PeerStatus>(std::min<int>(static_cast<int>(PeerStatus::W3),
                                                     static_cast<int>(n.status) + 1));
    const std::string nm = "/control/" + n.controllerId + "/" + m_controllerId + "/heartbeat";
    auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
    interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);
    WIRELOG("Controller", m_controllerId, "TX", "I", interest->getName().toUri());

    m_transmittedInterests(interest, this, m_face);

    m_face->onReceiveInterest(*interest);
anyPending = true;
  }

  if (anyPending) {
    m_hbRetryEvent = Simulator::Schedule(Seconds(m_hbTimeout),
                                         &ControllerApp::RetryControllerHeartbeats, this);
  } else {
    NS_LOG_INFO("Controller [" << m_controllerId << "]: HB window complete");
  }
}

void
ControllerApp::RecomputeAndBumpEpoch(const std::string& cause)
{
  if (m_dataPlane) {
    m_dataPlane->CalculateRoutes();
  }

  m_fibEpoch += 1;
  auto& stats = m_epochStats[m_fibEpoch];
  stats.createdAt = Simulator::Now();

  NS_LOG_INFO("Controller [" << m_controllerId << "]: FIB epoch "
              << m_fibEpoch << " created t=" << stats.createdAt.GetSeconds()
              << "s cause=" << cause);
}

void
ControllerApp::RecomputeControlPlaneOnly(const std::string& cause)
{
  if (m_controlPlane) {
    m_controlPlane->CalculateRoutes();
  }
  ++m_ctrlEpoch;
  NS_LOG_INFO("Controller [" << m_controllerId << "]: CTRL recompute t="
              << Simulator::Now().GetSeconds() << "s cause=" << cause
              << " ctrlEpoch=" << m_ctrlEpoch);
}

void
ControllerApp::ReplySwitchHeartbeat(const ndn::Name& name, const std::string& swId)
{
  auto data = std::make_shared<ndn::Data>(name);
  data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);

  std::ostringstream oss;
  oss << "active;epoch=" << m_fibEpoch
      << ";ts=" << Simulator::Now().GetSeconds()
      << ";controller=" << m_controllerId;

  const auto now = Simulator::Now();
  if (m_fibEpoch > 0) {
    auto& stats = m_epochStats[m_fibEpoch];
    if (stats.firstAdvAt.find(swId) == stats.firstAdvAt.end()) {
      stats.firstAdvAt[swId] = now;
      const double advDelay = (now - stats.createdAt).GetSeconds();
      NS_LOG_INFO("Controller [" << m_controllerId << "]: epoch-adv hb->" << swId
                  << " epoch=" << m_fibEpoch
                  << " advDelay=" << advDelay << "s");
    }
  }

  const auto payload = oss.str();
  data->setContent(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  data->setFreshnessPeriod(ndn::time::seconds(5));
  ndn::StackHelper::getKeyChain().sign(*data);

  WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

  m_transmittedDatas(data, this, m_face);

  m_face->onReceiveData(*data);
}

void
ControllerApp::ReplyControllerHeartbeat(const ndn::Name& name, const std::string& )
{
  auto data = std::make_shared<ndn::Data>(name);
  data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);

  std::ostringstream oss;
  oss << "active;epoch=" << m_fibEpoch
      << ";ts=" << Simulator::Now().GetSeconds()
      << ";controller=" << m_controllerId;

  const auto payload = oss.str();
  data->setContent(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  data->setFreshnessPeriod(ndn::time::seconds(5));
  ndn::StackHelper::getKeyChain().sign(*data);

  WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

  m_transmittedDatas(data, this, m_face);

  m_face->onReceiveData(*data);
}

void
ControllerApp::ReplyFibSync(const ndn::Name& name, const std::string& swId, uint64_t lastEpoch)
{
  auto data = std::make_shared<ndn::Data>(name);
  data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
  std::ostringstream content;

  if (lastEpoch < m_fibEpoch) {

    content << "apply-epoch=" << m_fibEpoch << ";producers=[";
    if (auto* sw = getSwitch(swId)) {
      for (size_t i = 0; i < sw->producers.size(); ++i) {
        content << sw->producers[i];
        if (i + 1 < sw->producers.size()) content << ",";
      }
      content << "];consumers=[";
      for (size_t i = 0; i < sw->consumers.size(); ++i) {
        content << sw->consumers[i];
        if (i + 1 < sw->consumers.size()) content << ",";
      }
      content << "]";
      sw->lastAppliedEpoch = m_fibEpoch;
      sw->lastSyncTime = Simulator::Now();
    } else {
      content << "];consumers=[]";
    }

    auto& stats = m_epochStats[m_fibEpoch];
    const auto now = Simulator::Now();
    if (stats.firstSyncAt.find(swId) == stats.firstSyncAt.end())
      stats.firstSyncAt[swId] = now;

    const double prop = (stats.createdAt.IsZero()) ? 0.0
                       : (now - stats.createdAt).GetSeconds();

    NS_LOG_INFO("Controller [" << m_controllerId << "]: FIB sync -> " << swId
                << " epoch=" << m_fibEpoch
                << " lastEpoch=" << lastEpoch
                << " propagation=" << prop << "s");
  }
  else {
    content << "up-to-date;epoch=" << m_fibEpoch;
    NS_LOG_INFO("Controller [" << m_controllerId << "]: FIB sync -> " << swId
                << " up-to-date epoch=" << m_fibEpoch
                << " (last=" << lastEpoch << ")");
  }

  const auto payload = content.str();
  data->setContent(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  data->setFreshnessPeriod(ndn::time::seconds(1));
  ndn::StackHelper::getKeyChain().sign(*data);

  WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

  m_transmittedDatas(data, this, m_face);

  m_face->onReceiveData(*data);
}

void
ControllerApp::SendControllerFibSync(const std::string& peerId, uint64_t lastEpoch)
{
  const std::string nm = "/control/" + peerId + "/" + m_controllerId
                       + "/fib/sync/" + std::to_string(lastEpoch);
  auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
  interest->setInterestLifetime(ndn::time::seconds(1)); interest->setMustBeFresh(true);

  NS_LOG_INFO("Controller [" << m_controllerId << "]: fib/sync -> " << peerId
              << " lastEpoch=" << lastEpoch);

  WIRELOG("Controller", m_controllerId, "TX", "I", interest->getName().toUri());

  m_transmittedInterests(interest, this, m_face);
  m_face->onReceiveInterest(*interest);
}

void
ControllerApp::ReplyControllerFibSync(const ndn::Name& name, const std::string& srcCtrl, uint64_t lastEpoch)
{
  auto data = std::make_shared<ndn::Data>(name);
  data->setCachingPolicy(::ndn::nfd::LocalControlHeader::CachingPolicy::NO_CACHE);
  std::ostringstream content;

  if (lastEpoch < m_fibEpoch) {
    content << "epoch=" << m_fibEpoch << ";switches=[";
    bool first = true;
    for (const auto& kv : m_switches) {
      if (!first) content << ",";
      content << kv.first;
      first = false;
    }
    content << "]";
    NS_LOG_INFO("Controller [" << m_controllerId << "]: ctrl FIB sync -> " << srcCtrl
                << " epoch=" << m_fibEpoch << " (was " << lastEpoch << ")");
  }
  else {
    content << "up-to-date;epoch=" << m_fibEpoch;
    NS_LOG_INFO("Controller [" << m_controllerId << "]: ctrl FIB sync -> " << srcCtrl
                << " up-to-date epoch=" << m_fibEpoch);
  }

  const auto payload = content.str();
  data->setContent(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  data->setFreshnessPeriod(ndn::time::seconds(1));
  ndn::StackHelper::getKeyChain().sign(*data);

  WIRELOG("Controller", m_controllerId, "TX", "D", data->getName().toUri());

  m_transmittedDatas(data, this, m_face);
  m_face->onReceiveData(*data);
}

void
ControllerApp::AddSwitch(const std::string& switchId, bool isEdge)
{
  auto& sw = m_switches[switchId];
  sw.switchId = switchId;
  sw.isEdge   = isEdge;
  NS_LOG_INFO("Controller [" << m_controllerId << "]: add switch " << switchId
              << " edge=" << isEdge);
}

void
ControllerApp::RemoveSwitch(const std::string& switchId)
{
  m_switches.erase(switchId);
  NS_LOG_INFO("Controller [" << m_controllerId << "]: remove switch " << switchId);
}

void
ControllerApp::AddProducerToSwitch(const std::string& switchId, const std::string& producerId)
{
  if (auto* sw = getSwitch(switchId)) {
    if (std::find(sw->producers.begin(), sw->producers.end(), producerId) == sw->producers.end())
      sw->producers.push_back(producerId);
    NS_LOG_INFO("Controller [" << m_controllerId << "]: S" << switchId << " +producer " << producerId);
  }
}

void
ControllerApp::RemoveProducerFromSwitch(const std::string& switchId, const std::string& producerId)
{
  if (auto* sw = getSwitch(switchId)) {
    sw->producers.erase(std::remove(sw->producers.begin(), sw->producers.end(), producerId),
                        sw->producers.end());
    NS_LOG_INFO("Controller [" << m_controllerId << "]: S" << switchId << " -producer " << producerId);
  }
}

void
ControllerApp::AddConsumerToSwitch(const std::string& switchId, const std::string& consumerId)
{
  if (auto* sw = getSwitch(switchId)) {
    if (std::find(sw->consumers.begin(), sw->consumers.end(), consumerId) == sw->consumers.end())
      sw->consumers.push_back(consumerId);
    NS_LOG_INFO("Controller [" << m_controllerId << "]: S" << switchId << " +consumer " << consumerId);
  }
}

void
ControllerApp::RemoveConsumerFromSwitch(const std::string& switchId, const std::string& consumerId)
{
  if (auto* sw = getSwitch(switchId)) {
    sw->consumers.erase(std::remove(sw->consumers.begin(), sw->consumers.end(), consumerId),
                        sw->consumers.end());
    NS_LOG_INFO("Controller [" << m_controllerId << "]: S" << switchId << " -consumer " << consumerId);
  }
}

void
ControllerApp::AddNeighborController(const std::string& controllerId)
{
  if (!getNeighbor(controllerId)) {
    CtrlNeighbor nb;
    nb.controllerId = controllerId;
    nb.status       = PeerStatus::ACTIVE;
    nb.misses       = 0;
    m_neighbors.push_back(nb);
    NS_LOG_INFO("Controller [" << m_controllerId << "]: add neighbor " << controllerId);
  }
}

void
ControllerApp::AddKnownController(const std::string& controllerId)
{
  if (std::find(m_knownControllers.begin(), m_knownControllers.end(), controllerId)
      == m_knownControllers.end()) {
    m_knownControllers.push_back(controllerId);
    NS_LOG_INFO("Controller [" << m_controllerId << "]: add known " << controllerId);
  }
}

void
ControllerApp::AddSwitchNeighbor(const std::string& sw, const std::string& nbr)
{

  m_switchNeighborsInCluster[sw].insert(nbr);
}

void
ControllerApp::CheckDeadTimeouts()
{
  const Time now = Simulator::Now();
  const Time pendingTo = Seconds(m_pendingTimeout);
  const Time hbSilenceTo = Seconds(m_hbSilenceTimeout);

  std::vector<std::string> candidates;
  for (const auto& kv : m_switches) candidates.push_back(kv.first);

  for (const auto& sw : candidates) {
    if (m_deadSwitches.count(sw)) continue;

    auto repIt = m_inactiveReports.find(sw);
    const bool hasLiveReporter = (repIt != m_inactiveReports.end()
                                  && !repIt->second.empty());

    auto hbIt = m_lastHbReceived.find(sw);
    const bool hbSilent = (hbIt == m_lastHbReceived.end())
                          ? true
                          : (now - hbIt->second) > hbSilenceTo;

    bool fire = false;
    if (hasLiveReporter) {
      auto psIt = m_pendingSince.find(sw);
      const Time firstReport = (psIt == m_pendingSince.end()) ? now : psIt->second;
      if ((now - firstReport) > pendingTo && hbSilent) {
        fire = true;
      }
    }

    if (fire) {
      NS_LOG_INFO("Controller [" << m_controllerId << "]: CheckDeadTimeouts "
                  << "TRIGGER_A " << sw << " at t=" << now.GetSeconds()
                  << "s (pending>" << m_pendingTimeout
                  << "s, hbSilent>" << m_hbSilenceTimeout << "s)");
      m_deadSwitches.insert(sw);
      m_pendingSince.erase(sw);
      OnDeadNode(sw);
    }
  }

  m_deadTimeoutCheckEvent =
    Simulator::Schedule(Seconds(30.0),
                        &ControllerApp::CheckDeadTimeouts, this);
}

void
ControllerApp::OnDeadNode(const std::string& sw)
{

  NS_LOG_INFO("Controller [" << m_controllerId << "]: OnDeadNode " << sw);
  const bool tableChanged = RecomputeAfterDeadNode(sw);
  RecomputeAndBumpEpoch("dead:" + sw);
  if (tableChanged) {
    PushCacheTableToAllManagedSwitches();
    ResetTelemetrySincePublish();
  }
}

void
ControllerApp::OnAliveNode(const std::string& sw)
{

  NS_LOG_INFO("Controller [" << m_controllerId << "]: OnAliveNode " << sw);
  const bool tableChanged = RecomputeAfterAliveNode(sw);
  RecomputeAndBumpEpoch("alive:" + sw);
  if (tableChanged) {
    PushCacheTableToAllManagedSwitches();
    ResetTelemetrySincePublish();
  }
}

void
ControllerApp::RlPeriodicCheck()
{
  if (!m_useRlBandit) return;

  std::vector<std::string> live;
  for (const auto& kv : m_switches) {
    if (m_deadSwitches.count(kv.first) == 0) live.push_back(kv.first);
  }
  if (!live.empty()) {
    static auto rng = CreateObject<UniformRandomVariable>();
    const std::string s = live[rng->GetInteger(0, live.size() - 1)];
    NS_LOG_INFO("Controller [" << m_controllerId
                << "]: 5d periodic LIVE-branch trigger arrival=" << s);
    const bool tableChanged = RecomputeAfterAliveNode(s);
    if (tableChanged) {
      RecomputeAndBumpEpoch("periodic:" + s);
      PushCacheTableToAllManagedSwitches();
      ResetTelemetrySincePublish();
    }
  }

  const double interval = (m_rlPeriodicInterval > 0.0)
                          ? m_rlPeriodicInterval
                          : 2.0 * m_hbInterval;
  m_rlPeriodicEvent = Simulator::Schedule(Seconds(interval),
                                          &ControllerApp::RlPeriodicCheck, this);
}

void
ControllerApp::RemoveNeighborController(const std::string& controllerId)
{
  m_neighbors.erase(std::remove_if(m_neighbors.begin(), m_neighbors.end(),
                    [&](const CtrlNeighbor& n){ return n.controllerId == controllerId; }),
                    m_neighbors.end());
  NS_LOG_INFO("Controller [" << m_controllerId << "]: remove neighbor " << controllerId);
}

void
ControllerApp::ActivateNeighborController(const std::string& controllerId)
{
  if (auto* n = getNeighbor(controllerId)) {
    n->status = PeerStatus::ACTIVE;
    n->misses = 0;
    NS_LOG_INFO("Controller [" << m_controllerId << "]: neighbor " << controllerId << " ACTIVE");
  }
}

void
ControllerApp::DeactivateNeighborController(const std::string& controllerId)
{
  if (auto* n = getNeighbor(controllerId)) {
    n->status = PeerStatus::INACTIVE;
    NS_LOG_INFO("Controller [" << m_controllerId << "]: neighbor " << controllerId << " INACTIVE");
  }
}

void
ControllerApp::SetRoutingHelpers(ndn::GlobalRoutingHelper* control,
                                 ndn::GlobalRoutingHelper* data)
{
  m_controlPlane = control;
  m_dataPlane    = data;
}

void
ControllerApp::SetClusterCacheTable(const std::vector<std::string>& table)
{
  m_clusterCacheTable = table;
  m_cacheBuckets = table.size();
  m_aggPopularity.assign(m_cacheBuckets, 0);
  m_popSincePublish.assign(m_cacheBuckets, 0);

  for (auto& kv : m_switches) {
    kv.second.gatewayWraps.assign(m_cacheBuckets, 0);
    kv.second.gatewayWrapsSincePublish.assign(m_cacheBuckets, 0);
  }
  NS_LOG_INFO("Controller [" << m_controllerId << "]: cluster cache table seeded "
              << "(" << m_cacheBuckets << " buckets)");
}

void
ControllerApp::ResetTelemetrySincePublish()
{

  std::fill(m_popSincePublish.begin(), m_popSincePublish.end(), 0);
  for (auto& kv : m_switches) {
    auto& ms = kv.second;
    std::fill(ms.gatewayWrapsSincePublish.begin(),
              ms.gatewayWrapsSincePublish.end(), 0);
    ms.dHitsSincePublish.clear();
    ms.evictionsSincePublish = 0;
  }
  NS_LOG_INFO("Controller [" << m_controllerId << "]: telemetry reset after publish");
}

namespace {

static bool startsWith(const std::string& s, const std::string& p) {
  return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
}

static std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == sep) { out.push_back(cur); cur.clear(); }
    else          { cur.push_back(c); }
  }
  out.push_back(cur);
  return out;
}

}

void
ControllerApp::ParseSwitchHbStats(const ndn::Name& name, const std::string& swId)
{

  if (name.size() < 7) return;

  auto* ms = getSwitch(swId);
  if (!ms) return;

  const std::string gwComp = name.get(4).toUri();
  const std::string evComp = name.get(5).toUri();
  const std::string dhComp = name.get(6).toUri();

  if (startsWith(gwComp, "gw=")) {
    auto parts = split(gwComp.substr(3), ',');
    if (ms->gatewayWraps.size() < parts.size()) {
      ms->gatewayWraps.assign(parts.size(), 0);
    }
    if (ms->gatewayWrapsSincePublish.size() < parts.size()) {
      ms->gatewayWrapsSincePublish.assign(parts.size(), 0);
    }
    if (m_aggPopularity.size() < parts.size()) {
      m_aggPopularity.assign(parts.size(), 0);
      m_cacheBuckets = parts.size();
    }
    if (m_popSincePublish.size() < parts.size()) {
      m_popSincePublish.assign(parts.size(), 0);
    }
    for (size_t b = 0; b < parts.size(); ++b) {
      uint64_t v = 0;
      try { v = std::stoull(parts[b]); } catch (...) { v = 0; }

      ms->gatewayWraps[b] = v;
      m_aggPopularity[b] += v;

      ms->gatewayWrapsSincePublish[b] += v;
      m_popSincePublish[b]            += v;
    }
  }

  if (startsWith(evComp, "ev=")) {
    uint64_t e = 0;
    try { e = std::stoull(evComp.substr(3)); } catch (...) { e = 0; }
    ms->evictions = e;
    ms->evictionsSincePublish += e;
  }

  ms->dHitsPerBucket.clear();
  if (startsWith(dhComp, "dh=") && dhComp.size() > 3 && dhComp[3] != '_') {
    for (const auto& pair : split(dhComp.substr(3), ',')) {
      auto colon = pair.find(':');
      if (colon == std::string::npos) continue;
      try {
        size_t b = std::stoul(pair.substr(0, colon));
        uint64_t h = std::stoull(pair.substr(colon + 1));
        ms->dHitsPerBucket[b] = h;
        ms->dHitsSincePublish[b] += h;
      } catch (...) {  }
    }
  }
}

bool
ControllerApp::RecomputeAfterDeadNode(const std::string& dead)
{
  if (m_clusterCacheTable.empty()) return false;

  if (m_useRlBandit) FinalizePendingBanditDecisions("dead-trigger");

  std::vector<size_t> orphans;
  for (size_t b = 0; b < m_clusterCacheTable.size(); ++b) {
    if (m_clusterCacheTable[b] == dead) orphans.push_back(b);
  }
  if (orphans.empty()) return false;

  std::vector<std::string> live;
  for (const auto& kv : m_switches) {
    const std::string& sid = kv.first;
    if (sid == dead) continue;
    if (m_deadSwitches.count(sid)) continue;
    live.push_back(sid);
  }
  if (live.empty()) {
    NS_LOG_WARN("Controller [" << m_controllerId << "]: RecomputeAfterDeadNode "
                << dead << " but no live alternative; table unchanged");
    return false;
  }

  bool changed = false;
  for (size_t b : orphans) {
    std::string best;
    if (m_useRlBandit) {
      best = BanditChooseTarget(b, live);
      if (best.empty()) {

        NS_LOG_WARN("Controller [" << m_controllerId << "]: bandit returned "
                    "no choice for orphan bucket=" << b << "; left pointing "
                    "at dead switch " << m_clusterCacheTable[b]);
        continue;
      }
    } else {

      auto bucketsOwned = [&](const std::string& s) -> size_t {
        size_t n = 0;
        for (const auto& host : m_clusterCacheTable) if (host == s) ++n;
        return n;
      };
      auto totalPop = [&](const std::string& s) -> uint64_t {
        uint64_t sum = 0;
        for (size_t i = 0; i < m_clusterCacheTable.size(); ++i) {
          if (m_clusterCacheTable[i] == s && i < m_popSincePublish.size())
            sum += m_popSincePublish[i];
        }
        return sum;
      };
      double bestFit = -1.0;
      for (const auto& s : live) {
        const double fit = 1.0 / (1.0 + double(totalPop(s)) + double(bucketsOwned(s)));
        if (fit > bestFit) { bestFit = fit; best = s; }
      }
    }
    NS_LOG_INFO("Controller [" << m_controllerId << "]: "
                << (m_useRlBandit ? "5d-bandit" : "5b-heuristic")
                << " reassign bucket=" << b
                << " " << m_clusterCacheTable[b] << " -> " << best);
    m_clusterCacheTable[b] = best;
    changed = true;
  }
  return changed;
}

bool
ControllerApp::RecomputeAfterAliveNode(const std::string& alive)
{
  if (m_clusterCacheTable.empty()) return false;

  if (m_useRlBandit) FinalizePendingBanditDecisions("alive-trigger");

  if (m_popSincePublish.empty()) {
    NS_LOG_INFO("Controller [" << m_controllerId << "]: no telemetry yet, skip alive-swap");
    return false;
  }

  if (m_useRlBandit) {

    std::vector<size_t> cands;
    for (size_t b = 0;
         b < m_popSincePublish.size() && b < m_clusterCacheTable.size();
         ++b) {
      cands.push_back(b);
    }
    size_t chosen = 0;
    if (!BanditChooseBucketForAlive(alive, cands, chosen)) {
      NS_LOG_WARN("Controller [" << m_controllerId
                  << "]: bandit returned no choice for alive=" << alive
                  << "; cache table unchanged");
      return false;
    }
    if (chosen == std::numeric_limits<size_t>::max()) {
      NS_LOG_INFO("Controller [" << m_controllerId
                  << "]: 5d-bandit picked null (no swap)");
      return false;
    }
    NS_LOG_INFO("Controller [" << m_controllerId << "]: 5d-bandit swap bucket="
                << chosen << " " << m_clusterCacheTable[chosen] << " -> " << alive);
    m_clusterCacheTable[chosen] = alive;
    return true;
  }

  long long bestGap = -1;
  size_t    victim  = 0;
  for (size_t b = 0;
       b < m_popSincePublish.size() && b < m_clusterCacheTable.size();
       ++b) {
    const std::string& host = m_clusterCacheTable[b];
    uint64_t dHits = 0;
    auto it = m_switches.find(host);
    if (it != m_switches.end()) {
      auto dh = it->second.dHitsSincePublish.find(b);
      if (dh != it->second.dHitsSincePublish.end()) dHits = dh->second;
    }
    const long long gap = (long long) m_popSincePublish[b] - (long long) dHits;
    if (gap > bestGap) { bestGap = gap; victim = b; }
  }
  if (bestGap <= 0) {
    NS_LOG_INFO("Controller [" << m_controllerId << "]: no starving bucket, skip alive-swap");
    return false;
  }

  const std::string& victimHost = m_clusterCacheTable[victim];
  size_t hostBuckets = 0;
  for (const auto& h : m_clusterCacheTable) if (h == victimHost) ++hostBuckets;
  if (hostBuckets < 2) {
    NS_LOG_INFO("Controller [" << m_controllerId << "]: victim bucket=" << victim
                << " host " << victimHost << " has only " << hostBuckets
                << " bucket(s), skip alive-swap");
    return false;
  }

  NS_LOG_INFO("Controller [" << m_controllerId << "]: 5b-swap bucket=" << victim
              << " " << victimHost << " -> " << alive
              << " (gap=" << bestGap << ")");
  m_clusterCacheTable[victim] = alive;
  return true;
}

void
ControllerApp::PushCacheTableToSwitch(const std::string& sw)
{

  std::string nm = "/xcontrol/" + sw + "/cache-update";
  for (const auto& host : m_clusterCacheTable) {
    nm += "/" + host;
  }
  auto interest = std::make_shared<ndn::Interest>(ndn::Name(nm));
  interest->setInterestLifetime(ndn::time::seconds(1));
  interest->setMustBeFresh(true);

  WIRELOG("Controller", m_controllerId, "TX", "I", interest->getName().toUri());
  m_transmittedInterests(interest, this, m_face);
  m_face->onReceiveInterest(*interest);
}

void
ControllerApp::PushCacheTableToAllManagedSwitches()
{
  for (const auto& kv : m_switches) {
    PushCacheTableToSwitch(kv.first);
  }
}

std::string
ControllerApp::BuildVwRequest(const std::string& shared_tag,
                              const std::vector<std::string>& action_features) const
{
  std::ostringstream os;
  os << "shared |" << shared_tag << "\n";
  for (const auto& af : action_features) {
    os << "|Cand " << af << "\n";
  }
  return os.str();
}

uint64_t
ControllerApp::WriteBanditRequest(const std::string& payload,
                                  std::string& outPath)
{
  const uint64_t id = ++m_rlDecisionId;

  const std::string dir = m_rlBridgeDir + "/" + m_controllerId;
  ::mkdir(m_rlBridgeDir.c_str(), 0755);
  ::mkdir(dir.c_str(), 0755);

  std::ostringstream nameStream;
  nameStream << dir << "/decision_" << id;
  const std::string base = nameStream.str();
  const std::string tmp  = base + ".req.tmp";
  const std::string req  = base + ".req";

  {
    std::ofstream f(tmp);
    f << payload;
  }

  ::rename(tmp.c_str(), req.c_str());

  outPath = req;
  return id;
}

bool
ControllerApp::ReadBanditResponse(uint64_t decisionId,
                                  int&   outChosenIdx,
                                  double& outProb) const
{
  const std::string dir = m_rlBridgeDir + "/" + m_controllerId;
  std::ostringstream nameStream;
  nameStream << dir << "/decision_" << decisionId << ".resp";
  const std::string path = nameStream.str();

  const auto deadline =
    std::chrono::steady_clock::now()
    + std::chrono::milliseconds(static_cast<int>(m_rlPollMaxWait * 1000));
  const auto sleepFor =
    std::chrono::milliseconds(static_cast<int>(m_rlPollSeconds * 1000));

  while (std::chrono::steady_clock::now() < deadline) {
    struct stat st;
    if (::stat(path.c_str(), &st) == 0 && st.st_size > 0) {
      std::ifstream f(path);
      if (f >> outChosenIdx >> outProb) {

        ::unlink(path.c_str());
        return true;
      }
    }
    std::this_thread::sleep_for(sleepFor);
  }
  NS_LOG_WARN("Controller [" << m_controllerId << "]: bandit response "
              "timeout for decision " << decisionId);
  return false;
}

void
ControllerApp::WriteBanditFeedback(uint64_t decisionId,
                                   int chosenIdx,
                                   double reward,
                                   double probability) const
{
  const std::string dir = m_rlBridgeDir + "/" + m_controllerId;
  std::ostringstream nameStream;
  nameStream << dir << "/feedback_" << decisionId;
  const std::string base = nameStream.str();
  const std::string tmp  = base + ".req.tmp";
  const std::string req  = base + ".req";

  {
    std::ofstream f(tmp);
    f << chosenIdx << ' ' << reward << ' ' << probability << "\n";
  }
  ::rename(tmp.c_str(), req.c_str());
}

double
ControllerApp::LiveMoveCost(size_t b) const
{
  if (b >= m_clusterCacheTable.size()) return 10.0;
  const std::string& host = m_clusterCacheTable[b];
  size_t hostBuckets = 0;
  for (const auto& h : m_clusterCacheTable) if (h == host) ++hostBuckets;
  if (hostBuckets <= 1) return 10.0;
  return 1.0 / double(hostBuckets);
}

std::string
ControllerApp::BanditChooseTarget(size_t orphanBucket,
                                  const std::vector<std::string>& candidates)
{
  if (candidates.empty()) return std::string();

  auto bucketsOwned = [&](const std::string& s) -> size_t {
    size_t n = 0;
    for (const auto& host : m_clusterCacheTable) if (host == s) ++n;
    return n;
  };
  auto totalPop = [&](const std::string& s) -> uint64_t {
    uint64_t sum = 0;
    for (size_t i = 0; i < m_clusterCacheTable.size(); ++i) {
      if (m_clusterCacheTable[i] == s && i < m_popSincePublish.size())
        sum += m_popSincePublish[i];
    }
    return sum;
  };

  std::vector<std::string> actionLines;
  actionLines.reserve(candidates.size());
  for (const auto& s : candidates) {
    const double load_inv  = 1.0 / (1.0 + double(totalPop(s)));
    const double count_inv = 1.0 / (1.0 + double(bucketsOwned(s)));
    std::ostringstream af;
    af << "load_inv:" << load_inv << " count_inv:" << count_inv;
    actionLines.push_back(af.str());
  }

  std::ostringstream shared;
  shared << "Branch dead orphan_b=" << orphanBucket;
  const std::string payload = BuildVwRequest(shared.str(), actionLines);

  std::string path;
  const uint64_t id = WriteBanditRequest(payload, path);

  int    chosenIdx = -1;
  double prob      = 1.0;
  if (!ReadBanditResponse(id, chosenIdx, prob)) {
    return std::string();
  }
  if (chosenIdx < 0 || static_cast<size_t>(chosenIdx) >= candidates.size()) {
    NS_LOG_WARN("Controller [" << m_controllerId << "]: bandit returned "
                "out-of-range idx=" << chosenIdx
                << " (candidates=" << candidates.size() << ")");
    return std::string();
  }
  const std::string& chosen = candidates[chosenIdx];

  BanditPendingDecision pd;
  pd.id           = id;
  pd.payload      = payload;
  pd.chosenIdx    = chosenIdx;
  pd.probability  = prob;
  pd.branchTag    = "dead";
  pd.tSent        = Simulator::Now();
  pd.liveMoveCost = 0.0;
  pd.bucket       = orphanBucket;
  pd.srcHost      = (orphanBucket < m_clusterCacheTable.size())
                    ? m_clusterCacheTable[orphanBucket] : std::string();
  pd.dstHost      = chosen;

  pd.evictionBefore = EvictionTotal();
  m_banditPending.push_back(std::move(pd));

  return chosen;
}

bool
ControllerApp::BanditChooseBucketForAlive(const std::string& aliveSwitch,
                                          const std::vector<size_t>& candidates,
                                          size_t& outBucket)
{
  if (candidates.empty()) return false;

  std::vector<std::string> actionLines;
  actionLines.reserve(candidates.size() + 1);
  for (size_t b : candidates) {
    long long gap = 0;
    if (b < m_popSincePublish.size() && b < m_clusterCacheTable.size()) {
      const std::string& host = m_clusterCacheTable[b];
      uint64_t dHits = 0;
      auto it = m_switches.find(host);
      if (it != m_switches.end()) {
        auto dh = it->second.dHitsSincePublish.find(b);
        if (dh != it->second.dHitsSincePublish.end()) dHits = dh->second;
      }
      gap = (long long) m_popSincePublish[b] - (long long) dHits;
    }
    const double neg_cost = -LiveMoveCost(b);
    std::ostringstream af;
    af << "gap:" << gap << " neg_cost:" << neg_cost;
    actionLines.push_back(af.str());
  }

  actionLines.push_back(std::string("gap:0 neg_cost:0"));

  std::ostringstream shared;
  shared << "Branch live arrival=" << aliveSwitch;
  const std::string payload = BuildVwRequest(shared.str(), actionLines);

  std::string path;
  const uint64_t id = WriteBanditRequest(payload, path);

  int    chosenIdx = -1;
  double prob      = 1.0;
  if (!ReadBanditResponse(id, chosenIdx, prob)) {
    return false;
  }
  if (chosenIdx < 0 || static_cast<size_t>(chosenIdx) > candidates.size()) {
    NS_LOG_WARN("Controller [" << m_controllerId << "]: bandit returned "
                "out-of-range idx=" << chosenIdx
                << " (candidates+null=" << candidates.size() + 1 << ")");
    return false;
  }

  BanditPendingDecision pd;
  pd.id          = id;
  pd.payload     = payload;
  pd.chosenIdx   = chosenIdx;
  pd.probability = prob;
  pd.branchTag   = "live";
  pd.tSent       = Simulator::Now();
  pd.dstHost     = aliveSwitch;

  if (static_cast<size_t>(chosenIdx) == candidates.size()) {
    outBucket          = std::numeric_limits<size_t>::max();
    pd.bucket          = outBucket;
    pd.liveMoveCost    = 0.0;
    pd.srcHost         = std::string();
  } else {
    const size_t b = candidates[chosenIdx];
    outBucket          = b;
    pd.bucket          = b;
    pd.liveMoveCost    = LiveMoveCost(b);
    pd.srcHost         = (b < m_clusterCacheTable.size())
                         ? m_clusterCacheTable[b] : std::string();
  }

  pd.evictionBefore = EvictionTotal();
  m_banditPending.push_back(std::move(pd));
  return true;
}

void
ControllerApp::RunningStd::update(double x)
{

  ++n;
  const double delta = x - mean;
  mean += delta / double(n);
  const double delta2 = x - mean;
  m2 += delta * delta2;
}

double
ControllerApp::RunningStd::sd() const
{
  if (n < 2) return 1.0;
  const double var = m2 / double(n - 1);
  return (var > 0.0) ? std::sqrt(var) : 1.0;
}

uint64_t
ControllerApp::EvictionTotal() const
{
  uint64_t sum = 0;
  for (const auto& kv : m_switches) sum += kv.second.evictionsSincePublish;
  return sum;
}

void
ControllerApp::FinalizePendingBanditDecisions(const char* cause)
{
  if (m_banditPending.empty()) return;

  const uint64_t evictionAfter = EvictionTotal();

  for (auto& pd : m_banditPending) {

    const double dEv = double(pd.evictionBefore) - double(evictionAfter);

    m_sdEviction.update(dEv);

    const double normEv = dEv / m_sdEviction.sd();

    const double r = m_w_eviction * normEv - m_w_cost * pd.liveMoveCost;

    NS_LOG_INFO("Controller [" << m_controllerId
                << "]: bandit feedback id=" << pd.id
                << " branch=" << pd.branchTag
                << " bucket=" << (pd.bucket == std::numeric_limits<size_t>::max()
                                  ? std::string("null") : std::to_string(pd.bucket))
                << " dEv=" << dEv
                << " normEv=" << normEv
                << " cost=" << pd.liveMoveCost
                << " r=" << r
                << " cause=" << cause);

    WriteBanditFeedback(pd.id, pd.chosenIdx, r, pd.probability);
  }

  m_banditPending.clear();
}

}
