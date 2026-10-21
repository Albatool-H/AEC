#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ndnSIM-module.h"

#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-l3-rate-tracer.hpp"
#include "ns3/ndnSIM/utils/topology/annotated-topology-reader.hpp"

#include <boost/functional/hash.hpp>

#include "controller-app.hpp"
#include "switch-app.hpp"
#include "pool-cache-strategy.hpp"
#include "cache-decision-strategy.hpp"
#include "content-store-cache-decision.hpp"

#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/forwarder.hpp"
#include "ns3/ndnSIM/NFD/daemon/table/strategy-choice.hpp"
#include "control-tracer.hpp"
#include "nocache-producer.hpp"

#include "ns3/ndnSIM/helper/ndn-link-control-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-strategy-choice-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"

#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/model/ndn-net-device-face.hpp"
#include "ns3/ndnSIM/model/ndn-global-router.hpp"
#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/point-to-point-net-device.h"
#include "ns3/random-variable-stream.h"
#include "ns3/names.h"
#include "ns3/log.h"
#include "ns3/channel.h"
#include "ns3/net-device.h"

#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <memory>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("aec");

namespace {

using Link = std::pair<Ptr<Node>, Ptr<Node>>;

static std::vector<std::string> gDataPrefixes;
static std::map<std::string, std::string> gSwitchToCtrl;
static std::map<std::string, Ptr<Node>>   gCtrlNodes;

static std::string ControllerIdForSwitch(Ptr<Node> sw)
{
  auto it = gSwitchToCtrl.find(Names::FindName(sw));
  return (it == gSwitchToCtrl.end()) ? std::string() : it->second;
}

static std::shared_ptr<nfd::Face>
FindFaceToNeighbor(Ptr<Node> from, Ptr<Node> to)
{
  using namespace ns3::ndn;
  Ptr<L3Protocol> l3 = from->GetObject<L3Protocol>();
  if (!l3) return nullptr;
  for (const auto& f : l3->getForwarder()->getFaceTable()) {
    auto ndf = std::dynamic_pointer_cast<NetDeviceFace>(f);
    if (!ndf) continue;
    Ptr<NetDevice> nd = ndf->GetNetDevice();
    if (!nd) continue;
    Ptr<Channel> ch = nd->GetChannel();
    if (!ch || ch->GetNDevices() != 2) continue;
    Ptr<Node> other = (ch->GetDevice(0)->GetNode() == from)
                      ? ch->GetDevice(1)->GetNode()
                      : ch->GetDevice(0)->GetNode();
    if (other == to) return f;
  }
  return nullptr;
}

static void PurgeDataFibToward(Ptr<Node> from, Ptr<Node> to)
{
  using namespace ns3::ndn;
  auto face = FindFaceToNeighbor(from, to);
  if (!face) return;
  for (const auto& p : gDataPrefixes) {
    FibHelper::RemoveRoute(from, p, face);
  }
}

static std::vector<Link>
BuildSwitchLinks(const std::vector<Ptr<Node>>& switches)
{
  std::set<std::string> swNames;
  for (auto& s : switches) swNames.insert(Names::FindName(s));

  std::set<Ptr<Channel>> seen;
  std::vector<Link> out;

  for (auto& s : switches) {
    for (uint32_t i = 0; i < s->GetNDevices(); ++i) {
      auto p2p = s->GetDevice(i)->GetObject<PointToPointNetDevice>();
      if (!p2p) continue;
      auto ch = p2p->GetChannel();
      if (!ch || seen.count(ch) || ch->GetNDevices() != 2) continue;
      seen.insert(ch);

      auto n0 = ch->GetDevice(0)->GetNode();
      auto n1 = ch->GetDevice(1)->GetNode();
      std::string name0 = Names::FindName(n0);
      std::string name1 = Names::FindName(n1);
      if (swNames.count(name0) && swNames.count(name1))
        out.emplace_back(n0, n1);
    }
  }
  return out;
}

static void ForceActiveForLink(const Link& e)
{
  const std::string a = Names::FindName(e.first);
  const std::string b = Names::FindName(e.second);
  auto aSw = e.first->GetApplication(0) ? e.first->GetApplication(0)->GetObject<SwitchApp>() : nullptr;
  auto bSw = e.second->GetApplication(0) ? e.second->GetApplication(0)->GetObject<SwitchApp>() : nullptr;

  if (aSw && !b.empty()) aSw->ActivateNeighbor(b);
  if (bSw && !a.empty()) bSw->ActivateNeighbor(a);
}

static std::set<std::pair<uint32_t, uint32_t>> gDownLinks;
static std::map<uint32_t, ns3::ndn::GlobalRouter::IncidencyList> gFullAdjacency;

static std::pair<uint32_t, uint32_t> LinkKey(Ptr<Node> a, Ptr<Node> b)
{
  uint32_t x = a->GetId();
  uint32_t y = b->GetId();
  return x < y ? std::make_pair(x, y) : std::make_pair(y, x);
}

static void RebuildAdjacency(Ptr<Node> node)
{
  auto gr = node->GetObject<ns3::ndn::GlobalRouter>();
  if (!gr) return;
  auto full = gFullAdjacency.find(node->GetId());
  if (full == gFullAdjacency.end())
    full = gFullAdjacency.emplace(node->GetId(), gr->GetIncidencies()).first;

  auto& adjacency = gr->GetIncidencies();
  adjacency.clear();
  for (const auto& entry : full->second) {
    Ptr<Node> peer = std::get<2>(entry)->GetObject<Node>();
    if (peer && gDownLinks.count(LinkKey(node, peer))) continue;
    adjacency.push_back(entry);
  }
}

static void SetLinkRoutable(const Link& e, bool up)
{
  if (up) gDownLinks.erase(LinkKey(e.first, e.second));
  else    gDownLinks.insert(LinkKey(e.first, e.second));
  RebuildAdjacency(e.first);
  RebuildAdjacency(e.second);
}

static void BringUpLinks(std::vector<Link> links)
{
  std::set<std::string> toBump;
  for (const auto& e : links) {
    ns3::ndn::LinkControlHelper::UpLink(e.first, e.second);
    NS_LOG_INFO("NDN-LINK: UP   " << Names::FindName(e.first)
                << "<->" << Names::FindName(e.second)
                << " t=" << Simulator::Now().GetSeconds() << "s");
    ForceActiveForLink(e);
    SetLinkRoutable(e, true);
    const std::string ca = ControllerIdForSwitch(e.first);
    const std::string cb = ControllerIdForSwitch(e.second);
    if (!ca.empty()) toBump.insert(ca);
    if (!cb.empty()) toBump.insert(cb);
  }

  for (const auto& id : toBump) {
    auto it = gCtrlNodes.find(id);
    if (it == gCtrlNodes.end()) continue;
    auto app = it->second->GetApplication(0)->GetObject<ControllerApp>();
    if (app) app->RecomputeAndBumpEpoch("link-up");
  }
}

struct FailCtx {
  std::vector<Link>          all;
  uint32_t                   failLinks;
  Time                       downTime;
  Time                       period;
  Ptr<UniformRandomVariable> rng;
};

static void FailRandomLinks(FailCtx ctx)
{
  if (ctx.all.empty() || ctx.failLinks == 0) {
    Simulator::Schedule(ctx.period, &FailRandomLinks, ctx);
    return;
  }

  const uint32_t L = std::min<uint32_t>(ctx.failLinks, ctx.all.size());
  std::set<uint32_t> pick;
  while (pick.size() < L) pick.insert(ctx.rng->GetInteger(0, ctx.all.size() - 1));

  std::vector<Link> chosen;
  chosen.reserve(L);
  for (auto idx : pick) chosen.push_back(ctx.all[idx]);

  for (const auto& e : chosen) {
    ns3::ndn::LinkControlHelper::FailLink(e.first, e.second);
    NS_LOG_INFO("NDN-LINK: DOWN " << Names::FindName(e.first)
                << "<->" << Names::FindName(e.second)
                << " t=" << Simulator::Now().GetSeconds() << "s");
    SetLinkRoutable(e, false);
    PurgeDataFibToward(e.first,  e.second);
    PurgeDataFibToward(e.second, e.first);
  }

  Simulator::Schedule(ctx.downTime, &BringUpLinks, chosen);
  Simulator::Schedule(ctx.period, &FailRandomLinks, ctx);
}

struct PoolStatsCtx {
  std::vector<Ptr<Node>> switches;
  std::vector<Ptr<Node>> producers;
  std::ofstream*         poolFile  = nullptr;
  std::ofstream*         prodFile  = nullptr;
  Time                   period;

  std::map<std::string, Ptr<ControllerApp>> prodCtrl;
};

static std::map<std::string, uint64_t> gPrevDHit;
static std::map<std::string, uint64_t> gPrevDMiss;
static std::map<std::string, uint64_t> gPrevEvict;
static std::map<std::string, uint64_t> gPrevProdHits;

static void RlRecordConsumerDelay(Ptr<ControllerApp> ctrl, Ptr<ns3::ndn::App> ,
                                  uint32_t , Time delay,
                                  uint32_t , int32_t hopCount)
{
  if (ctrl) ctrl->RecordConsumerDelivery(delay.GetSeconds(), hopCount);
}

static void DumpPoolStats(PoolStatsCtx ctx)
{
  using namespace ns3::ndn;
  double t = Simulator::Now().GetSeconds();

  if (ctx.poolFile && ctx.poolFile->is_open()) {
    for (const auto& sw : ctx.switches) {
      std::string sname = Names::FindName(sw);
      auto app = sw->GetApplication(0)
                 ? sw->GetApplication(0)->GetObject<SwitchApp>() : nullptr;
      if (!app) continue;
      uint64_t h = app->GetDHitTotal();
      uint64_t m = app->GetDMissTotal();
      uint64_t e = app->GetEvictionsTotal();
      uint64_t dh = h - gPrevDHit[sname];
      uint64_t dm = m - gPrevDMiss[sname];
      uint64_t de = e - gPrevEvict[sname];
      gPrevDHit[sname] = h;
      gPrevDMiss[sname] = m;
      gPrevEvict[sname] = e;

      if (dh + dm + de > 0) {
        *ctx.poolFile << t << '\t' << sname << '\t' << dh << '\t' << dm << '\t' << de << '\n';
      }
    }
    ctx.poolFile->flush();
  }

  if (ctx.prodFile && ctx.prodFile->is_open()) {
    for (const auto& p : ctx.producers) {
      std::string pname = Names::FindName(p);
      auto app = p->GetApplication(0)
                 ? p->GetApplication(0)->GetObject<NoCacheProducer>() : nullptr;
      if (!app) continue;
      uint64_t total = app->GetHitsTotal();
      uint64_t delta = total - gPrevProdHits[pname];
      gPrevProdHits[pname] = total;
      if (delta > 0) {
        *ctx.prodFile << t << '\t' << pname << '\t' << delta << '\n';
        auto itc = ctx.prodCtrl.find(pname);
        if (itc != ctx.prodCtrl.end() && itc->second) itc->second->RecordProducerHits(delta);
      }
    }
    ctx.prodFile->flush();
  }

  Simulator::Schedule(ctx.period, &DumpPoolStats, ctx);
}

struct NodeFailCtx {
  std::vector<Ptr<Node>>     switches;
  uint32_t                   deadNodes = 0;
  Time                       downTime;
  Time                       period;
  Ptr<UniformRandomVariable> rng;
};

static std::vector<Link> SwitchSwitchLinks(Ptr<Node> sw)
{
  std::vector<Link> out;
  std::set<Ptr<Channel>> seen;
  for (uint32_t d = 0; d < sw->GetNDevices(); ++d) {
    auto ch = sw->GetDevice(d)->GetChannel();
    if (!ch || ch->GetNDevices() != 2 || seen.count(ch)) continue;
    seen.insert(ch);
    auto other = (ch->GetDevice(0)->GetNode() == sw)
                 ? ch->GetDevice(1)->GetNode() : ch->GetDevice(0)->GetNode();
    auto oname = Names::FindName(other);
    if (!oname.empty() && oname[0] == 'S') out.emplace_back(sw, other);
  }
  return out;
}

static void RestoreNodes(std::vector<Ptr<Node>> chosen)
{
  for (const auto& sw : chosen) {
    auto links = SwitchSwitchLinks(sw);

    BringUpLinks(links);
    auto app = sw->GetApplication(0)
               ? sw->GetApplication(0)->GetObject<SwitchApp>() : nullptr;
    if (app) app->ReactivateAllNeighbors();
    NS_LOG_INFO("NODE-UP: " << Names::FindName(sw)
                << " t=" << Simulator::Now().GetSeconds() << "s");
  }
}

static void FailRandomNodes(NodeFailCtx ctx)
{
  if (ctx.switches.empty() || ctx.deadNodes == 0) {
    Simulator::Schedule(ctx.period, &FailRandomNodes, ctx);
    return;
  }

  const uint32_t N = std::min<uint32_t>(ctx.deadNodes, ctx.switches.size());
  std::set<uint32_t> pick;
  while (pick.size() < N) pick.insert(ctx.rng->GetInteger(0, ctx.switches.size() - 1));

  std::vector<Ptr<Node>> chosen;
  chosen.reserve(N);
  for (auto idx : pick) chosen.push_back(ctx.switches[idx]);

  for (const auto& sw : chosen) {
    NS_LOG_INFO("NODE-DOWN: " << Names::FindName(sw)
                << " t=" << Simulator::Now().GetSeconds() << "s");
    auto links = SwitchSwitchLinks(sw);
    for (const auto& e : links) {
      ns3::ndn::LinkControlHelper::FailLink(e.first, e.second);
      SetLinkRoutable(e, false);
      PurgeDataFibToward(e.first,  e.second);
      PurgeDataFibToward(e.second, e.first);
    }
  }

  Simulator::Schedule(ctx.downTime, &RestoreNodes, chosen);
  Simulator::Schedule(ctx.period, &FailRandomNodes, ctx);
}

}

namespace ns3 {

static void
ConfigurePoolCacheStrategyOnAllSwitches(std::vector<Ptr<Node>> switches)
{
  for (auto& sw : switches) {
    auto swApp = sw->GetApplication(0)->GetObject<SwitchApp>();
    if (!swApp) continue;
    auto l3 = sw->GetObject<ndn::L3Protocol>();
    if (!l3 || !l3->getForwarder()) continue;
    auto& fwd = *l3->getForwarder();
    auto& s = fwd.getStrategyChoice().findEffectiveStrategy("/u");
    auto* ourStrat = dynamic_cast<nfd::fw::PoolCacheGatewayStrategy*>(&s);
    if (!ourStrat) {
      NS_LOG_WARN("PoolCacheGatewayStrategy not effective for /u on "
                  << Names::FindName(sw));
      continue;
    }
    ourStrat->SetCacheTable(&swApp->GetCacheTable());
    ourStrat->SetAppFace(swApp->GetFace());
    ourStrat->SetSwitchID(Names::FindName(sw));
    NS_LOG_INFO("PoolCacheGatewayStrategy configured on "
                << Names::FindName(sw)
                << " (cache-table size=" << swApp->GetCacheTable().size() << ")");
  }
}

static void
ConfigurePoolHomeStrategy(std::vector<Ptr<Node>> switches, std::string dataPrefix)
{
  for (auto& sw : switches) {
    auto swApp = sw->GetApplication(0)
                 ? sw->GetApplication(0)->GetObject<SwitchApp>() : nullptr;
    if (!swApp) continue;
    auto l3 = sw->GetObject<ndn::L3Protocol>();
    if (!l3 || !l3->getForwarder()) continue;
    auto& s = l3->getForwarder()->getStrategyChoice().findEffectiveStrategy(dataPrefix);
    auto* strat = dynamic_cast<nfd::fw::CacheDecisionStrategy*>(&s);
    if (!strat) {
      NS_LOG_WARN("poolhome: CacheDecisionStrategy not effective on "
                  << Names::FindName(sw));
      continue;
    }
    strat->SetMode("poolhome");
    strat->SetContentStore(
      DynamicCast<ns3::ndn::cs::ContentStoreCacheDecision>(sw->GetObject<ndn::ContentStore>()));
    strat->SetCacheTable(&swApp->GetCacheTable());
    strat->SetSelfId(Names::FindName(sw));
    NS_LOG_INFO("poolhome configured on " << Names::FindName(sw)
                << " (cache-table size=" << swApp->GetCacheTable().size() << ")");
  }
}

int main(int argc, char* argv[])
{
  double      simTime       = 600.0;
  double      consRate      = 30.0;
  std::string dataStrategy  = "best";
  double      dataLifeTime  = 1.0;
  double      retxMs        = 20.0;
  std::string topoFile      = "topology/iot-topology.txt";
  std::string metaFile      = "topology/iot-meta.json";
  std::string dataPrefix    = "/data/content/";
  uint32_t    cacheSize     = 100;

  double   hbFirst    = 10.0;
  uint32_t hbRepeat   = 3;
  double   hbTimeout  = 10.0;
  double   hbInterval = 60.0;

  double   pendingTimeout   = 0.0;
  double   hbSilenceTimeout = 0.0;

  bool        useRlBandit   = false;
  std::string rlBridgeDir   = "rl-bridge";
  double      rlPollSeconds = 0.005;
  double      rlPollMaxWait = 5.0;

  double      rlPeriodicInterval = 0.0;

  double      rlWProducer = 1.0;
  double      rlWHop      = 1.0;
  double      rlWDelay    = 1.0;
  double      rlWEviction = 1.0;
  double      rlWCost     = 1.0;

  uint32_t failLinks  = 0;
  uint32_t deadNodes  = 0;
  double   downTime   = 0.0;
  double   failPeriod = 0.0;
  double   failStart  = -1.0;

  uint32_t catalogSize = 500000;
  double   zipfAlpha   = 1.6;
  double   zipfQ       = 0.0;
  uint32_t cacheBuckets = 10;

  std::string consumerModel = "zipf";
  uint32_t    totalRequests = 5000;

  double      consumerStartOffset = 0.0;

  double      consumerEndTime = 0.0;

  bool        useConsumerSideWrap = false;

  bool        useStrategyWrap     = false;

  uint32_t    rngSeed = 1;

  CommandLine cmd;
  cmd.AddValue("simTime",       "Simulation time (s)", simTime);
  cmd.AddValue("consRate",      "Consumer rate (Interest/s)", consRate);
  cmd.AddValue("dataStrategy",  "Forwarding strategy: asf | multicast | best", dataStrategy);
  cmd.AddValue("dataLifeTime",  "Interest lifetime (s)", dataLifeTime);
  cmd.AddValue("retxMs",        "Retx timer (ms)", retxMs);
  cmd.AddValue("topoFile",      "Topology file path", topoFile);
  cmd.AddValue("metaFile",      "Metadata JSON path", metaFile);
  cmd.AddValue("dataPrefix",    "Data prefix", dataPrefix);
  cmd.AddValue("cacheSize",     "CS size per node (0=disabled)", cacheSize);
  cmd.AddValue("hbFirst",       "First heartbeat delay (s)", hbFirst);
  cmd.AddValue("hbRepeat",      "Heartbeat retries before INACTIVE", hbRepeat);
  cmd.AddValue("hbTimeout",     "Heartbeat retry timeout (s)", hbTimeout);
  cmd.AddValue("hbInterval",    "Heartbeat interval (s)", hbInterval);
  cmd.AddValue("pendingTimeout",   "5d Trigger A: pending nbr-report timeout (s)", pendingTimeout);
  cmd.AddValue("hbSilenceTimeout", "5d Trigger A: controller-side HB silence timeout (s)", hbSilenceTimeout);
  cmd.AddValue("useRlBandit",      "5d: route OnDead/OnAlive through VW bandit file bridge", useRlBandit);
  cmd.AddValue("rlBridgeDir",      "5d: dir under which decision_*.req/.resp/feedback files live", rlBridgeDir);
  cmd.AddValue("rlPollSeconds",    "5d: wall-clock poll cadence while awaiting VW response (s)", rlPollSeconds);
  cmd.AddValue("rlPollMaxWait",    "5d: hard timeout per decision waiting for VW response (s)", rlPollMaxWait);
  cmd.AddValue("rlPeriodicInterval","5d: periodic LIVE-branch decision interval (s). 0 = 2 x hbInterval.", rlPeriodicInterval);
  cmd.AddValue("rlWProducer", "Reward weight: producer-hit signal", rlWProducer);
  cmd.AddValue("rlWHop",      "Reward weight: hop signal",          rlWHop);
  cmd.AddValue("rlWDelay",    "Reward weight: delay signal",        rlWDelay);
  cmd.AddValue("rlWEviction", "Reward weight: eviction signal",     rlWEviction);
  cmd.AddValue("rlWCost",     "Reward weight: live-move cost",      rlWCost);
  cmd.AddValue("failLinks",     "Random switch-switch links to fail per round", failLinks);
  cmd.AddValue("deadNodes",     "Random switches to fully kill per round (all switch-switch links down)", deadNodes);
  cmd.AddValue("downTime",      "Link/node down duration (s)", downTime);
  cmd.AddValue("failPeriod",    "Failure injection period (s)", failPeriod);
  cmd.AddValue("failStart",     "First failure round time (s, default=failPeriod)", failStart);
  cmd.AddValue("catalogSize",   "Number of distinct content items (Zipf catalog)", catalogSize);
  cmd.AddValue("zipfAlpha",     "Zipf popularity exponent (s parameter)", zipfAlpha);
  cmd.AddValue("zipfQ",         "Zipf-Mandelbrot rank-improvement q (0=pure Zipf)", zipfQ);
  cmd.AddValue("cacheBuckets",  "Pool-cache hash buckets per cluster", cacheBuckets);
  cmd.AddValue("consumerModel", "Consumer model: zipf | iot-oneshot", consumerModel);
  cmd.AddValue("totalRequests", "Total one-shot interests (only used when consumerModel=iot-oneshot)", totalRequests);
  cmd.AddValue("consumerStartOffset", "Seconds before first consumer fires (SDN warm-up)", consumerStartOffset);
  cmd.AddValue("consumerEndTime",     "Last time (s) any consumer may fire; 0=use simTime", consumerEndTime);
  cmd.AddValue("useConsumerSideWrap", "Consumer-side wrap (mysimulation parity); skips the gateway /u/ wrap layer", useConsumerSideWrap);
  cmd.AddValue("useStrategyWrap",     "Install PoolCacheGatewayStrategy on /u/ so wrap happens inside NFD afterReceiveInterest (3c spike)", useStrategyWrap);
  cmd.AddValue("seed", "5e: RngSeedManager seed (1..N for multi-seed averaging)", rngSeed);
  cmd.Parse(argc, argv);

  RngSeedManager::SetSeed(rngSeed);

  LogComponentEnable("aec", LOG_LEVEL_INFO);
  LogComponentEnable("ControllerApp", LOG_LEVEL_INFO);
  LogComponentEnable("SwitchApp", LOG_LEVEL_INFO);

  AnnotatedTopologyReader topoReader("", 1.0);
  topoReader.SetFileName(topoFile);
  topoReader.Read();

  std::vector<Ptr<Node>> controllers;
  std::vector<Ptr<Node>> switches;
  std::vector<Ptr<Node>> producers;
  std::vector<Ptr<Node>> consumers;
  std::map<std::string, Ptr<Node>> nodeMap;

  for (auto it = NodeList::Begin(); it != NodeList::End(); ++it) {
    Ptr<Node> node = *it;
    std::string name = Names::FindName(node);
    nodeMap[name] = node;

    if (name.size() > 0) {
      if (name[0] == 'C') controllers.push_back(node);
      else if (name[0] == 'S') switches.push_back(node);
      else if (name[0] == 'P') producers.push_back(node);
      else if (name[0] == 'U') consumers.push_back(node);
    }
  }

  NS_LOG_INFO("Topology: " << controllers.size() << " controllers, "
              << switches.size() << " switches, "
              << producers.size() << " producers, "
              << consumers.size() << " consumers");

  std::map<std::string, std::vector<std::string>> clusterSwitches;

  std::map<std::string, std::string> consumerCluster;
  std::map<std::string, std::string> producerCluster;
  std::ifstream metaIn(metaFile);
  if (metaIn.good()) {
    std::string metaContent((std::istreambuf_iterator<char>(metaIn)),
                             std::istreambuf_iterator<char>());

    for (auto& ctrl : controllers) {
      std::string cname = Names::FindName(ctrl);
      std::string key = "\"" + cname + "\": [";
      auto pos = metaContent.find(key);
      if (pos == std::string::npos) continue;

      auto start = metaContent.find('[', pos);
      auto end = metaContent.find(']', start);
      if (start == std::string::npos || end == std::string::npos) continue;

      std::string arr = metaContent.substr(start + 1, end - start - 1);
      std::vector<std::string> sws;
      size_t p = 0;
      while ((p = arr.find('"', p)) != std::string::npos) {
        auto p2 = arr.find('"', p + 1);
        if (p2 == std::string::npos) break;
        sws.push_back(arr.substr(p + 1, p2 - p - 1));
        p = p2 + 1;
      }
      clusterSwitches[cname] = sws;
    }

    auto parseRoleClusterMap =
      [&](const std::string& fieldKey, std::map<std::string, std::string>& out) {
        std::string key = "\"" + fieldKey + "\": {";
        auto pos = metaContent.find(key);
        if (pos == std::string::npos) return;
        auto start = metaContent.find('{', pos);
        auto end = metaContent.find('}', start);
        if (start == std::string::npos || end == std::string::npos) return;
        std::string body = metaContent.substr(start + 1, end - start - 1);
        size_t p = 0;
        while ((p = body.find('"', p)) != std::string::npos) {
          auto p2 = body.find('"', p + 1);
          if (p2 == std::string::npos) break;
          std::string name = body.substr(p + 1, p2 - p - 1);
          auto colon = body.find(':', p2);
          if (colon == std::string::npos) break;
          size_t numStart = colon + 1;
          while (numStart < body.size() &&
                 (body[numStart] == ' ' || body[numStart] == '\t' || body[numStart] == '\n'))
            numStart++;
          size_t numEnd = numStart;
          while (numEnd < body.size() && body[numEnd] >= '0' && body[numEnd] <= '9') numEnd++;
          if (numStart < numEnd) {
            int idx = std::stoi(body.substr(numStart, numEnd - numStart));
            out[name] = "C" + std::to_string(idx);
          }
          p = numEnd;
        }
      };
    parseRoleClusterMap("consumer_cluster", consumerCluster);
    parseRoleClusterMap("producer_cluster", producerCluster);
  }

  std::set<std::string> poolCacheSwitchNames;
  for (const auto& kv : clusterSwitches) {
    for (const auto& sname : kv.second) poolCacheSwitchNames.insert(sname);
  }

  ndn::StackHelper poolHelper;
  poolHelper.SetOldContentStore("ns3::ndn::cs::CacheDecision",
                                "MaxSize", std::to_string(cacheSize),
                                "Mode", "poolhome");
  ndn::StackHelper noCacheHelper;
  noCacheHelper.SetOldContentStore("ns3::ndn::cs::Nocache");
  std::vector<Ptr<Node>> poolCacheNodes;
  for (auto it = NodeList::Begin(); it != NodeList::End(); ++it) {
    Ptr<Node> n = *it;
    std::string nname = Names::FindName(n);
    bool isPoolCache = (!nname.empty() && nname[0] == 'S'
                        && poolCacheSwitchNames.count(nname) > 0);
    if (isPoolCache) { poolHelper.Install(n); poolCacheNodes.push_back(n); }
    else             noCacheHelper.Install(n);
  }
  NS_LOG_INFO("CS: " << poolCacheSwitchNames.size() << " pool-cache switches with CS="
              << cacheSize << "; all other nodes CS=0");

  ndn::AppHelper ctrlHelper("ControllerApp");
  ctrlHelper.SetAttribute("HeartbeatFirst",    DoubleValue(hbFirst));
  ctrlHelper.SetAttribute("HeartbeatInterval", DoubleValue(hbInterval));
  ctrlHelper.SetAttribute("HeartbeatTimeout",  DoubleValue(hbTimeout));
  ctrlHelper.SetAttribute("HeartbeatRepeat",   UintegerValue(hbRepeat));

  const double effPendingTo   = (pendingTimeout   > 0.0) ? pendingTimeout   : hbInterval;
  const double effHbSilenceTo = (hbSilenceTimeout > 0.0) ? hbSilenceTimeout : hbInterval;
  ctrlHelper.SetAttribute("PendingTimeout",    DoubleValue(effPendingTo));
  ctrlHelper.SetAttribute("HbSilenceTimeout",  DoubleValue(effHbSilenceTo));

  ctrlHelper.SetAttribute("UseRlBandit",       BooleanValue(useRlBandit));
  ctrlHelper.SetAttribute("RlBridgeDir",       StringValue(rlBridgeDir));
  ctrlHelper.SetAttribute("RlPollSeconds",     DoubleValue(rlPollSeconds));
  ctrlHelper.SetAttribute("RlPollMaxWait",     DoubleValue(rlPollMaxWait));
  ctrlHelper.SetAttribute("RlPeriodicInterval",DoubleValue(rlPeriodicInterval));
  ctrlHelper.SetAttribute("RlWProducer",       DoubleValue(rlWProducer));
  ctrlHelper.SetAttribute("RlWHop",            DoubleValue(rlWHop));
  ctrlHelper.SetAttribute("RlWDelay",          DoubleValue(rlWDelay));
  ctrlHelper.SetAttribute("RlWEviction",       DoubleValue(rlWEviction));
  ctrlHelper.SetAttribute("RlWCost",           DoubleValue(rlWCost));

  for (auto& ctrl : controllers) {
    std::string cname = Names::FindName(ctrl);
    ctrlHelper.SetAttribute("ControllerID", StringValue(cname));
    auto apps = ctrlHelper.Install(ctrl);
    apps.Start(Seconds(0));
    apps.Stop(Seconds(simTime));
  }

  for (auto& ctrl : controllers) {
    auto app = ctrl->GetApplication(0)->GetObject<ControllerApp>();
    if (!app) continue;
    std::string cname = Names::FindName(ctrl);

    for (auto& other : controllers) {
      std::string oname = Names::FindName(other);
      if (oname == cname) continue;
      app->AddKnownController(oname);
    }

    for (uint32_t d = 0; d < ctrl->GetNDevices(); ++d) {
      auto ch = ctrl->GetDevice(d)->GetChannel();
      if (!ch || ch->GetNDevices() != 2) continue;
      auto other = (ch->GetDevice(0)->GetNode() == ctrl)
                   ? ch->GetDevice(1)->GetNode() : ch->GetDevice(0)->GetNode();
      auto oname = Names::FindName(other);
      if (!oname.empty() && oname[0] == 'C') {
        app->AddNeighborController(oname);
      }
    }
  }

  std::map<std::string, std::string> swToCluster;
  for (auto& kv : clusterSwitches) {
    for (auto& swName : kv.second) swToCluster[swName] = kv.first;
  }
  std::set<std::string> edgeSwitches;
  for (auto& sw : switches) {
    std::string sname = Names::FindName(sw);
    auto myCluster = swToCluster[sname];
    for (uint32_t d = 0; d < sw->GetNDevices(); ++d) {
      auto ch = sw->GetDevice(d)->GetChannel();
      if (!ch || ch->GetNDevices() != 2) continue;
      auto other = (ch->GetDevice(0)->GetNode() == sw)
                   ? ch->GetDevice(1)->GetNode() : ch->GetDevice(0)->GetNode();
      auto oname = Names::FindName(other);
      if (oname.size() > 0 && oname[0] == 'S' &&
          swToCluster[oname] != myCluster) {
        edgeSwitches.insert(sname);
        break;
      }
    }
  }

  for (auto& kv : clusterSwitches) {
    auto ctrlNode = nodeMap[kv.first];
    auto ctrlApp = ctrlNode->GetApplication(0)->GetObject<ControllerApp>();
    for (auto& swName : kv.second) {
      bool isEdge = edgeSwitches.count(swName) > 0;
      ctrlApp->AddSwitch(swName, isEdge);
    }
  }

  ndn::AppHelper swHelper("SwitchApp");
  swHelper.SetAttribute("HeartbeatFirst",    DoubleValue(hbFirst));
  swHelper.SetAttribute("HeartbeatInterval", DoubleValue(hbInterval));
  swHelper.SetAttribute("HeartbeatTimeout",  DoubleValue(hbTimeout));
  swHelper.SetAttribute("HeartbeatRepeat",   UintegerValue(hbRepeat));

  for (auto& sw : switches) {
    std::string sname = Names::FindName(sw);
    swHelper.SetAttribute("SwitchID", StringValue(sname));
    auto apps = swHelper.Install(sw);
    apps.Start(Seconds(0));
    apps.Stop(Seconds(simTime));

    auto swApp = sw->GetApplication(0)->GetObject<SwitchApp>();

    if (edgeSwitches.count(sname)) swApp->SetEdge(true);

    swApp->SetCacheCapacity(cacheSize);

    for (auto& kv : clusterSwitches) {
      auto& sws = kv.second;
      if (std::find(sws.begin(), sws.end(), sname) != sws.end()) {
        swApp->AddController(kv.first);
        break;
      }
    }

    const std::string& myCluster = swToCluster[sname];
    auto myCtrlNode = nodeMap.count(myCluster) ? nodeMap[myCluster] : nullptr;
    auto myCtrlApp  = myCtrlNode
                      ? myCtrlNode->GetApplication(0)->GetObject<ControllerApp>()
                      : nullptr;
    for (uint32_t d = 0; d < sw->GetNDevices(); ++d) {
      auto dev = sw->GetDevice(d);
      auto ch = dev->GetChannel();
      if (!ch || ch->GetNDevices() != 2) continue;
      auto otherDev = (ch->GetDevice(0)->GetNode() == sw)
                      ? ch->GetDevice(1) : ch->GetDevice(0);
      auto otherNode = otherDev->GetNode();
      std::string otherName = Names::FindName(otherNode);
      if (otherName.size() > 0 && otherName[0] == 'S') {
        swApp->AddNeighbor(otherName);
        if (myCtrlApp && swToCluster.count(otherName)
            && swToCluster[otherName] == myCluster) {
          myCtrlApp->AddSwitchNeighbor(sname, otherName);
        }
      }
    }
  }

  std::map<std::string, std::vector<std::string>> clusterTables;
  {
    std::set<std::string> mappedGlobally;
    auto rng = CreateObject<UniformRandomVariable>();
    for (auto& kv : clusterSwitches) {
      const auto& cname   = kv.first;
      const auto& members = kv.second;
      if (members.empty()) continue;
      std::vector<std::string> table;
      table.reserve(cacheBuckets);
      for (const auto& s : members) {
        if (table.size() == cacheBuckets) break;
        if (!mappedGlobally.count(s)) table.push_back(s);
      }
      while (table.size() < cacheBuckets) {
        table.push_back(members[rng->GetInteger(0, members.size() - 1)]);
      }
      std::ostringstream oss;
      for (size_t i = 0; i < table.size(); ++i) {
        if (i) oss << ",";
        oss << i << ":" << table[i];
      }
      NS_LOG_INFO("Cluster " << cname << " cache table [" << oss.str() << "]");
      for (const auto& s : members) {
        auto it = nodeMap.find(s);
        if (it == nodeMap.end()) continue;
        auto swApp = it->second->GetApplication(0)->GetObject<SwitchApp>();
        if (!swApp) continue;
        if (swApp->GetCacheTable().empty()) swApp->SetCacheTable(table);
      }
      for (const auto& s : table) mappedGlobally.insert(s);
      clusterTables[cname] = table;

      auto ctrlIt = nodeMap.find(cname);
      if (ctrlIt != nodeMap.end()) {
        auto ctrlApp = ctrlIt->second->GetApplication(0)->GetObject<ControllerApp>();
        if (ctrlApp) {
          ctrlApp->SetClusterCacheTable(table);
        }
      }
    }
  }

  auto attachedSwitch = [&](Ptr<Node> sensor) -> std::string {
    for (uint32_t d = 0; d < sensor->GetNDevices(); ++d) {
      auto ch = sensor->GetDevice(d)->GetChannel();
      if (!ch || ch->GetNDevices() != 2) continue;
      auto other = (ch->GetDevice(0)->GetNode() == sensor)
                   ? ch->GetDevice(1)->GetNode() : ch->GetDevice(0)->GetNode();
      auto oname = Names::FindName(other);
      if (oname.size() > 0 && oname[0] == 'S') return oname;
    }
    return std::string();
  };

  ndn::AppHelper prodHelper("ns3::ndn::NoCacheProducer");
  prodHelper.SetAttribute("PayloadSize", StringValue("1024"));
  prodHelper.SetAttribute("Freshness", TimeValue(Seconds(2)));
  prodHelper.SetPrefix(dataPrefix);
  for (auto& prod : producers) {
    prodHelper.Install(prod).Start(Seconds(0));
    CreateObject<UniformRandomVariable>();
    std::string pname = Names::FindName(prod);
    std::string swName = attachedSwitch(prod);
    if (!swName.empty()) {
      auto ctrlId = swToCluster[swName];
      auto ctrlApp = nodeMap[ctrlId]->GetApplication(0)->GetObject<ControllerApp>();
      ctrlApp->AddProducerToSwitch(swName, pname);
    }
    NS_LOG_INFO("Producer on " << pname << " via " << swName);
  }

  const std::string consPrefix = "/u" + dataPrefix;

  for (size_t i = 0; i < consumers.size(); ++i) {
    std::string uname  = Names::FindName(consumers[i]);
    std::string swName = attachedSwitch(consumers[i]);
    if (!swName.empty()) {
      auto ctrlId  = swToCluster[swName];
      auto ctrlApp = nodeMap[ctrlId]->GetApplication(0)->GetObject<ControllerApp>();
      ctrlApp->AddConsumerToSwitch(swName, uname);
    }
    NS_LOG_INFO("Consumer on " << uname << " via " << swName);
  }

  double endTime = (consumerEndTime > 0.0) ? consumerEndTime : simTime;

  if (consumerModel == "zipf") {
    ndn::AppHelper consHelper("ns3::ndn::ConsumerZipfMandelbrot");
    consHelper.SetPrefix(consPrefix);
    consHelper.SetAttribute("Frequency",        DoubleValue(consRate));
    consHelper.SetAttribute("LifeTime",         StringValue(std::to_string(dataLifeTime) + "s"));
    consHelper.SetAttribute("RetxTimer",        StringValue(std::to_string(retxMs) + "ms"));
    consHelper.SetAttribute("NumberOfContents", UintegerValue(catalogSize));
    consHelper.SetAttribute("s",                DoubleValue(zipfAlpha));
    consHelper.SetAttribute("q",                DoubleValue(zipfQ));
    for (size_t i = 0; i < consumers.size(); ++i) {
      auto app = consHelper.Install(consumers[i]);
      app.Start(Seconds(consumerStartOffset + 0.1 * i));
      app.Stop(Seconds(endTime));
    }
  }
  else if (consumerModel == "iot-oneshot") {

    Ptr<ZipfRandomVariable> popRand = CreateObject<ZipfRandomVariable>();
    popRand->SetAttribute("N",     IntegerValue(catalogSize));
    popRand->SetAttribute("Alpha", DoubleValue(zipfAlpha));
    Ptr<ExponentialRandomVariable> startGap = CreateObject<ExponentialRandomVariable>();
    double window  = std::max(1.0, endTime - consumerStartOffset);
    double meanGap = (totalRequests > 0) ? (window / (double) totalRequests) : 0.1;
    startGap->SetAttribute("Mean",  DoubleValue(meanGap));
    startGap->SetAttribute("Bound", DoubleValue(50.0 * meanGap));

    ndn::AppHelper consHelper("ns3::ndn::IOTOneShotConsumer");
    consHelper.SetAttribute("LifeTime",  StringValue(std::to_string(dataLifeTime) + "s"));
    consHelper.SetAttribute("RetxTimer", StringValue(std::to_string(retxMs) + "ms"));

    double startTime = consumerStartOffset + 0.1;
    uint32_t installed = 0;
    for (uint32_t i = 0; i < totalRequests; ++i) {

      if (startTime > simTime) break;
      uint32_t contentID = (uint32_t) (popRand->GetValue() - 1);
      if (contentID >= catalogSize) contentID = catalogSize - 1;

      std::string thisPrefix = consPrefix;
      if (useConsumerSideWrap) {
        Ptr<Node> consNode = consumers[i % consumers.size()];

        std::string consName = Names::FindName(consNode);
        std::string clusterId;
        auto itCC = consumerCluster.find(consName);
        if (itCC != consumerCluster.end()) {
          clusterId = itCC->second;
        } else {

          std::string swName = attachedSwitch(consNode);
          if (swToCluster.count(swName)) clusterId = swToCluster[swName];
        }
        auto itTable = clusterTables.find(clusterId);
        if (itTable != clusterTables.end() && !itTable->second.empty()) {
          ndn::Name inner(dataPrefix);
          inner.appendSequenceNumber(contentID);
          size_t bucket = boost::hash<std::string>()(inner.toUri()) % itTable->second.size();
          const std::string& designated = itTable->second[bucket];
          thisPrefix = "/cache/" + designated + dataPrefix;
        } else {
          NS_LOG_WARN("Consumer-side wrap: no cluster table for consumer "
                      << consName << " (clusterId='" << clusterId
                      << "'); falling back to /u/ -- gateway wrap will fire.");
        }
      }
      consHelper.SetPrefix(thisPrefix);
      consHelper.SetAttribute("StartSeq",   IntegerValue(contentID));
      consHelper.SetAttribute("MaxContent", IntegerValue(contentID + 1));
      Ptr<Node> cNode = consumers[i % consumers.size()];
      ApplicationContainer capp = consHelper.Install(cNode);
      capp.Start(Seconds(startTime));

      if (useRlBandit) {
        auto itC = consumerCluster.find(Names::FindName(cNode));
        if (itC != consumerCluster.end()) {
          auto itN = nodeMap.find(itC->second);
          if (itN != nodeMap.end() && itN->second->GetApplication(0)) {
            Ptr<ControllerApp> cc =
              itN->second->GetApplication(0)->GetObject<ControllerApp>();
            if (cc)
              capp.Get(0)->TraceConnectWithoutContext(
                "FirstInterestDataDelay", MakeBoundCallback(&RlRecordConsumerDelay, cc));
          }
        }
      }
      startTime += startGap->GetValue();
      installed++;
    }
    NS_LOG_INFO("Installed " << installed << "/" << totalRequests
                             << " IOTOneShotConsumer instances across "
                             << consumers.size() << " consumer nodes "
                             << "[firing window " << consumerStartOffset
                             << "s..~" << endTime << "s, "
                             << (useConsumerSideWrap ? "consumer-side wrap" : "gateway wrap") << "]");
  }
  else {
    NS_FATAL_ERROR("Unknown --consumerModel: " << consumerModel);
  }

  ndn::GlobalRoutingHelper controlPlane;
  NodeContainer ctrlContainer;
  for (auto& c : controllers) ctrlContainer.Add(c);
  controlPlane.Install(ctrlContainer);
  for (auto& ctrl : controllers) {
    std::string cname = Names::FindName(ctrl);
    controlPlane.AddOrigins("/control/" + cname + "/", ctrl);
  }
  controlPlane.CalculateRoutes();

  ndn::GlobalRoutingHelper dataPlane;
  NodeContainer dataNodes;
  for (auto& n : switches)  dataNodes.Add(n);
  for (auto& n : producers) dataNodes.Add(n);
  for (auto& n : consumers) dataNodes.Add(n);
  dataPlane.Install(dataNodes);

  for (auto& sw : switches) {
    std::string sname = Names::FindName(sw);
    dataPlane.AddOrigins("/xcontrol/" + sname + "/", sw);

    dataPlane.AddOrigins("/cache/" + sname + "/", sw);

    dataPlane.AddOrigins("/u/", sw);
  }
  for (auto& prod : producers) {
    dataPlane.AddOrigins(dataPrefix, prod);
  }
  dataPlane.CalculateRoutes();

  for (auto& kv : clusterSwitches) {
    auto ctrlNode = nodeMap[kv.first];
    if (!ctrlNode) continue;
    const std::string& cname = kv.first;
    for (auto& swName : kv.second) {
      auto swNode = nodeMap[swName];
      if (!swNode) continue;
      ndn::FibHelper::AddRoute(ctrlNode, "/xcontrol/" + swName + "/", swNode,   1);
      ndn::FibHelper::AddRoute(swNode,   "/xcontrol/" + cname  + "/", ctrlNode, 1);
    }
  }

  std::string strat = "/localhost/nfd/strategy/best-route";
  if (dataStrategy == "asf") strat = "/localhost/nfd/strategy/asf";
  else if (dataStrategy == "multicast") strat = "/localhost/nfd/strategy/multicast";
  ndn::StrategyChoiceHelper::InstallAll("/data",  strat);
  ndn::StrategyChoiceHelper::InstallAll("/cache", strat);
  ndn::StrategyChoiceHelper::InstallAll("/u",     strat);

  for (auto& sw : poolCacheNodes)
    ndn::StrategyChoiceHelper::Install<nfd::fw::CacheDecisionStrategy>(sw, dataPrefix);
  Simulator::Schedule(Seconds(0.0001), &ConfigurePoolHomeStrategy, poolCacheNodes, dataPrefix);

  if (useStrategyWrap) {
    for (auto& sw : switches) {
      ndn::StrategyChoiceHelper::Install<nfd::fw::PoolCacheGatewayStrategy>(sw, "/u");
    }
    Simulator::Schedule(Seconds(0.0001), &ConfigurePoolCacheStrategyOnAllSwitches, switches);
  }

  for (auto& ctrl : controllers) {
    ctrl->GetApplication(0)->GetObject<ControllerApp>()
        ->SetRoutingHelpers(&controlPlane, &dataPlane);
  }

  gSwitchToCtrl.clear();
  for (auto& kv : clusterSwitches) {
    for (auto& swName : kv.second) gSwitchToCtrl[swName] = kv.first;
  }
  gCtrlNodes.clear();
  for (auto& ctrl : controllers) gCtrlNodes[Names::FindName(ctrl)] = ctrl;
  gDataPrefixes.clear();
  gDataPrefixes.push_back(dataPrefix);

  ndn::AppDelayTracer::InstallAll("app-delays-trace.txt");
  ndn::L3RateTracer::InstallAll("l3-rate-trace.txt", Seconds(1.0));
  ndn::ControlTracer::Install("control-trace.txt");

  static std::ofstream poolFile("d_pool_stats.csv");
  static std::ofstream prodFile("producer_hits.csv");
  poolFile << "Time\tSwitchID\tDHits\tDMisses\tEvictions\n";
  prodFile << "Time\tProducerID\tHits\n";
  PoolStatsCtx psCtx;
  psCtx.switches  = switches;
  psCtx.producers = producers;
  psCtx.poolFile  = &poolFile;
  psCtx.prodFile  = &prodFile;
  psCtx.period    = Seconds(1.0);

  if (useRlBandit) {
    for (const auto& p : producers) {
      std::string pname = Names::FindName(p);
      auto itC = producerCluster.find(pname);
      if (itC == producerCluster.end()) continue;
      auto itN = nodeMap.find(itC->second);
      if (itN == nodeMap.end() || !itN->second->GetApplication(0)) continue;
      psCtx.prodCtrl[pname] = itN->second->GetApplication(0)->GetObject<ControllerApp>();
    }
  }
  Simulator::Schedule(psCtx.period, &DumpPoolStats, psCtx);

  auto ssLinks = BuildSwitchLinks(switches);
  NS_LOG_INFO("NDN-LINK: cataloged " << ssLinks.size() << " switch-switch links");

  if (failLinks > 0 && downTime > 0.0 && failPeriod > 0.0) {
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    FailCtx ctx;
    ctx.all       = ssLinks;
    ctx.failLinks = failLinks;
    ctx.downTime  = Seconds(downTime);
    ctx.period    = Seconds(failPeriod);
    ctx.rng       = rng;

    double firstAt = (failStart > 0.0) ? failStart : failPeriod;
    Simulator::Schedule(Seconds(firstAt), &FailRandomLinks, ctx);
    NS_LOG_INFO("FAIL-INJECTOR: X=" << failLinks
                << " Y=" << downTime << "s Z=" << failPeriod << "s"
                << " firstAt=" << firstAt << "s");
  }

  if (deadNodes > 0 && downTime > 0.0 && failPeriod > 0.0) {
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    NodeFailCtx ctx;
    ctx.switches  = switches;
    ctx.deadNodes = deadNodes;
    ctx.downTime  = Seconds(downTime);
    ctx.period    = Seconds(failPeriod);
    ctx.rng       = rng;

    double firstAt = (failStart > 0.0) ? failStart : failPeriod;
    Simulator::Schedule(Seconds(firstAt), &FailRandomNodes, ctx);
    NS_LOG_INFO("NODE-INJECTOR: K=" << deadNodes
                << " Y=" << downTime << "s Z=" << failPeriod << "s"
                << " firstAt=" << firstAt << "s");
  }

  {
    auto schedTest = [&](double t, const std::string& src, const std::string& dst) {
      auto it = nodeMap.find(src);
      if (it == nodeMap.end()) return;
      auto srcApp = it->second->GetApplication(0)->GetObject<SwitchApp>();
      if (!srcApp) return;
      Simulator::Schedule(Seconds(t), &SwitchApp::SendCacheTestInterest, srcApp, dst);
    };
    schedTest(15.0, "S0",  "S1");
    schedTest(15.2, "S0",  "S99");
    schedTest(15.5, "S50", "S55");
  }

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}

}

int main(int argc, char* argv[]) { return ns3::main(argc, argv); }
