#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ndnSIM-module.h"

#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-l3-rate-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-cs-tracer.hpp"
#include "ns3/ndnSIM/utils/topology/annotated-topology-reader.hpp"

#include "ns3/ndnSIM/helper/ndn-link-control-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-strategy-choice-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"

#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/model/ndn-net-device-face.hpp"
#include "ns3/point-to-point-net-device.h"
#include "ns3/random-variable-stream.h"
#include "ns3/names.h"
#include "ns3/log.h"

#include "content-store-cache-decision.hpp"
#include "cache-decision-strategy.hpp"
#include "probcache-producer.hpp"
#include "btw-producer.hpp"
#include "router-app.hpp"

#include "ns3/ndnSIM/helper/ndn-strategy-choice-helper.hpp"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"

#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("iot-baseline-ndn");

namespace {

using Link = std::pair<Ptr<Node>, Ptr<Node>>;
static ns3::ndn::GlobalRoutingHelper* gDataPlane = nullptr;
static std::vector<std::string>       gDataPrefixes;

static std::map<std::string, uint64_t> gBaselineEvictTotal;
static std::map<std::string, uint64_t> gBaselineEvictPrev;
static std::ofstream                   gBaselineCsFile;

static void
OnBaselineCsEvict(std::string nodeName,
                  ns3::Ptr<const ns3::ndn::cs::Entry> entry,
                  ns3::Time when)
{
  (void) entry; (void) when;
  ++gBaselineEvictTotal[nodeName];
}

struct BaselineCsStatsCtx {
  ns3::NodeContainer routers;
  ns3::Time          period;
};

static void
DumpBaselineCsStats(BaselineCsStatsCtx ctx)
{
  double t = ns3::Simulator::Now().GetSeconds();
  for (uint32_t i = 0; i < ctx.routers.GetN(); ++i) {
    std::string name = ns3::Names::FindName(ctx.routers.Get(i));
    if (name.empty()) name = std::to_string(ctx.routers.Get(i)->GetId());
    uint64_t total = gBaselineEvictTotal[name];
    uint64_t prev  = gBaselineEvictPrev[name];
    uint64_t delta = total - prev;
    gBaselineEvictPrev[name] = total;
    if (delta > 0) {
      gBaselineCsFile << t << '\t' << name << '\t' << delta << '\n';
    }
  }
  gBaselineCsFile.flush();
  ns3::Simulator::Schedule(ctx.period, &DumpBaselineCsStats, ctx);
}

struct PoolEvictCtx {
  std::vector<std::string>                       names;
  std::vector<ns3::Ptr<ns3::ndn::RouterApp>>     apps;
  ns3::Time                                      period;
};
static std::map<std::string, uint64_t> gPoolEvictPrev;
static void
DumpPoolCsStats(PoolEvictCtx ctx)
{
  double t = ns3::Simulator::Now().GetSeconds();
  for (size_t i = 0; i < ctx.apps.size(); ++i) {
    uint64_t total = ctx.apps[i]->GetEvictions();
    uint64_t delta = total - gPoolEvictPrev[ctx.names[i]];
    gPoolEvictPrev[ctx.names[i]] = total;
    if (delta > 0) gBaselineCsFile << t << '\t' << ctx.names[i] << '\t' << delta << '\n';
  }
  gBaselineCsFile.flush();
  ns3::Simulator::Schedule(ctx.period, &DumpPoolCsStats, ctx);
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
    if (other == to) return ndf;
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

static void BringUpLinks(std::vector<Link> links)
{
  for (const auto& e : links) {
    ns3::ndn::LinkControlHelper::UpLink(e.first, e.second);
    NS_LOG_INFO("NDN-LINK: UP   " << Names::FindName(e.first)
                << "<->" << Names::FindName(e.second)
                << " t=" << Simulator::Now().GetSeconds() << "s");
    if (gDataPlane) gDataPlane->Install(e.first);
  }
  if (gDataPlane) gDataPlane->CalculateRoutes();
}

static void RestoreNodes(std::vector<Ptr<Node>> chosen)
{
  for (const auto& sw : chosen) {
    auto links = SwitchSwitchLinks(sw);
    BringUpLinks(links);
    NS_LOG_INFO("NODE-UP: " << Names::FindName(sw)
                << " t=" << Simulator::Now().GetSeconds() << "s");
  }
}

struct NodeFailCtx {
  std::vector<Ptr<Node>>     switches;
  uint32_t                   deadNodes = 0;
  Time                       downTime;
  Time                       period;
  Ptr<UniformRandomVariable> rng;
};

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
      PurgeDataFibToward(e.first,  e.second);
      PurgeDataFibToward(e.second, e.first);
    }
  }

  Simulator::Schedule(ctx.downTime, &RestoreNodes, chosen);
  Simulator::Schedule(ctx.period, &FailRandomNodes, ctx);
}

}

namespace ns3 {

int main(int argc, char* argv[])
{
  double      simTime       = 400.0;
  double      consRate      = 30.0;
  std::string dataStrategy  = "best";
  double      dataLifeTime  = 1.0;
  double      retxMs        = 20.0;
  std::string topoFile      = "topology/iot-topology-noctrl.txt";
  std::string dataPrefix    = "/data/content/";
  uint32_t    cacheSize     = 100;

  std::string cacheStrategy = "lce";
  std::string btwFile       = "";

  uint32_t failLinks  = 0;
  uint32_t deadNodes  = 0;
  double   downTime   = 0.0;
  double   failPeriod = 0.0;
  double   failStart  = -1.0;

  uint32_t catalogSize = 500000;
  double   zipfAlpha   = 1.6;
  double   zipfQ       = 0.0;
  std::string metaFile = "topology/iot-meta.json";

  std::string consumerModel = "zipf";
  uint32_t    totalRequests = 5000;

  double      consumerStartOffset = 0.0;
  double      consumerEndTime     = 0.0;

  uint32_t    rngSeed = 1;

  CommandLine cmd;
  cmd.AddValue("simTime",      "Simulation time (s)", simTime);
  cmd.AddValue("consRate",     "Consumer rate (Interest/s)", consRate);
  cmd.AddValue("dataStrategy", "Forwarding strategy: asf | multicast | best", dataStrategy);
  cmd.AddValue("dataLifeTime", "Interest lifetime (s)", dataLifeTime);
  cmd.AddValue("retxMs",       "Retx timer (ms)", retxMs);
  cmd.AddValue("topoFile",     "Topology file path", topoFile);
  cmd.AddValue("dataPrefix",   "Data prefix", dataPrefix);
  cmd.AddValue("cacheSize",    "LRU CS size on each router", cacheSize);
  cmd.AddValue("cacheStrategy","Caching strategy: lce | lcd | probcache | btw", cacheStrategy);
  cmd.AddValue("btwFile",      "Betweenness TSV (name<TAB>C_B) for --cacheStrategy=btw", btwFile);
  cmd.AddValue("failLinks",    "Random switch-switch links to fail per round", failLinks);
  cmd.AddValue("deadNodes",    "Random routers to fully kill per round", deadNodes);
  cmd.AddValue("downTime",     "Down duration (s)", downTime);
  cmd.AddValue("failPeriod",   "Failure injection period (s)", failPeriod);
  cmd.AddValue("failStart",    "First failure round time (s)", failStart);
  cmd.AddValue("catalogSize",  "Zipf catalog size", catalogSize);
  cmd.AddValue("metaFile",     "Cluster membership JSON for poolcache", metaFile);
  cmd.AddValue("zipfAlpha",    "Zipf exponent s", zipfAlpha);
  cmd.AddValue("zipfQ",        "Zipf q", zipfQ);
  cmd.AddValue("consumerModel","Consumer model: zipf | iot-oneshot", consumerModel);
  cmd.AddValue("totalRequests","Total one-shot interests (only used when consumerModel=iot-oneshot)", totalRequests);
  cmd.AddValue("consumerStartOffset","Seconds before first consumer fires (warm-up)", consumerStartOffset);
  cmd.AddValue("consumerEndTime",    "Last time (s) any consumer may fire; 0=use simTime", consumerEndTime);
  cmd.AddValue("seed", "5e: RngSeedManager seed (1..N for multi-seed averaging)", rngSeed);
  cmd.Parse(argc, argv);

  RngSeedManager::SetSeed(rngSeed);

  LogComponentEnable("iot-baseline-ndn", LOG_LEVEL_INFO);

  AnnotatedTopologyReader topoReader("", 1.0);
  topoReader.SetFileName(topoFile);
  topoReader.Read();

  std::vector<Ptr<Node>> routers, producers, consumers;
  for (auto it = NodeList::Begin(); it != NodeList::End(); ++it) {
    Ptr<Node> node = *it;
    std::string name = Names::FindName(node);
    if (name.empty()) continue;
    if      (name[0] == 'S') routers.push_back(node);
    else if (name[0] == 'P') producers.push_back(node);
    else if (name[0] == 'U') consumers.push_back(node);
  }
  NS_LOG_INFO("Baseline NDN: " << routers.size() << " routers, "
              << producers.size() << " producers, "
              << consumers.size() << " consumers");

  NS_LOG_INFO("Caching strategy: " << cacheStrategy);
  bool customStrat = (cacheStrategy == "lcd" || cacheStrategy == "probcache"
                      || cacheStrategy == "btw");
  bool poolcache = (cacheStrategy == "poolcache");
  ndn::StackHelper lruHelper;
  if (cacheStrategy == "lce") {
    lruHelper.SetOldContentStore("ns3::ndn::cs::Stats::Lru", "MaxSize", std::to_string(cacheSize));
  } else if (customStrat) {

    lruHelper.SetOldContentStore("ns3::ndn::cs::CacheDecision",
                                 "MaxSize", std::to_string(cacheSize),
                                 "Mode", cacheStrategy);
  } else if (poolcache) {

    lruHelper.SetOldContentStore("ns3::ndn::cs::Nocache");
  } else {
    NS_FATAL_ERROR("Unknown --cacheStrategy=" << cacheStrategy);
  }

  std::map<std::string, double> btwMap;
  if (cacheStrategy == "btw") {
    if (btwFile.empty()) NS_FATAL_ERROR("--cacheStrategy=btw requires --btwFile");
    std::ifstream bf(btwFile);
    if (!bf) NS_FATAL_ERROR("cannot open btwFile " << btwFile);
    std::string nm; double cb;
    while (bf >> nm >> cb) btwMap[nm] = cb;
    NS_LOG_INFO("Btw: loaded " << btwMap.size() << " centralities from " << btwFile);
  }

  ndn::StackHelper noCacheHelper;
  noCacheHelper.SetOldContentStore("ns3::ndn::cs::Nocache");
  for (auto& r : routers)   lruHelper.Install(r);
  for (auto& p : producers) noCacheHelper.Install(p);
  for (auto& c : consumers) noCacheHelper.Install(c);

  for (auto& r : routers) {
    std::string name = Names::FindName(r);
    if (name.empty()) name = std::to_string(r->GetId());
    Ptr<ndn::ContentStore> cs = r->GetObject<ndn::ContentStore>();

    if (cs && !poolcache) {
      cs->TraceConnectWithoutContext("WillRemoveEntry",
          MakeBoundCallback(&OnBaselineCsEvict, name));

      if (cacheStrategy == "btw") {
        cs->SetAttribute("Betweenness", DoubleValue(btwMap.count(name) ? btwMap[name] : 0.0));
      }
    }
  }

  if (customStrat) {
    for (auto& r : routers)
      ndn::StrategyChoiceHelper::Install<nfd::fw::CacheDecisionStrategy>(r, dataPrefix);
    for (auto& r : routers) {
      std::string name = Names::FindName(r);
      Ptr<ndn::L3Protocol> l3 = r->GetObject<ndn::L3Protocol>();
      if (!l3 || !l3->getForwarder()) continue;
      auto& s = l3->getForwarder()->getStrategyChoice().findEffectiveStrategy(dataPrefix);
      auto* strat = dynamic_cast<nfd::fw::CacheDecisionStrategy*>(&s);
      if (!strat) { NS_LOG_WARN("CacheDecisionStrategy not effective on " << name); continue; }
      strat->SetMode(cacheStrategy);
      strat->SetTargetTimeWindow(10.0);
      strat->SetContentStore(
        DynamicCast<ns3::ndn::cs::ContentStoreCacheDecision>(r->GetObject<ndn::ContentStore>()));
      if (cacheStrategy == "btw")
        strat->SetBetweenness(btwMap.count(name) ? btwMap[name] : 0.0);
    }
  }

  std::vector<Ptr<ns3::ndn::RouterApp>> poolApps;
  if (poolcache) {
    std::map<std::string, std::vector<std::string>> clusterMembers;
    std::map<std::string, std::string>              routerCluster;
    {
      std::ifstream metaIn(metaFile);
      if (!metaIn.good())
        NS_FATAL_ERROR("poolcache: cannot open meta file " << metaFile);
      std::string meta((std::istreambuf_iterator<char>(metaIn)),
                       std::istreambuf_iterator<char>());
      auto parseArray = [&](size_t from) {
        std::vector<std::string> out;
        auto a = meta.find('[', from), e = meta.find(']', a);
        if (a == std::string::npos || e == std::string::npos) return out;
        std::string arr = meta.substr(a + 1, e - a - 1);
        size_t p = 0;
        while ((p = arr.find('"', p)) != std::string::npos) {
          auto p2 = arr.find('"', p + 1);
          if (p2 == std::string::npos) break;
          out.push_back(arr.substr(p + 1, p2 - p - 1));
          p = p2 + 1;
        }
        return out;
      };

      std::vector<std::string> cnames = parseArray(meta.find("\"controllers\""));
      for (const auto& cn : cnames) {
        auto pos = meta.find("\"" + cn + "\": [");
        if (pos == std::string::npos) continue;
        std::vector<std::string> sws = parseArray(pos);
        clusterMembers[cn] = sws;
        for (const auto& sw : sws) routerCluster[sw] = cn;
      }
    }
    for (auto& r : routers) {
      const std::string rid = Names::FindName(r);
      ndn::AppHelper rh("ns3::ndn::RouterApp");
      rh.SetAttribute("RouterId",      StringValue(rid));
      rh.SetAttribute("CacheCapacity", UintegerValue(cacheSize));
      auto apps = rh.Install(r);
      apps.Start(Seconds(0));
      auto app = DynamicCast<ns3::ndn::RouterApp>(apps.Get(0));
      if (!app) continue;
      std::vector<std::string> pool;
      auto it = routerCluster.find(rid);
      if (it != routerCluster.end()) pool = clusterMembers[it->second];
      if (pool.empty()) pool = { rid };
      app->SetPool(pool);
      poolApps.push_back(app);
    }
    NS_LOG_INFO("Pool cache (cluster-local): " << poolApps.size()
                << " RouterApps across " << clusterMembers.size() << " clusters");
  }

  std::string prodTypeId = (cacheStrategy == "probcache") ? "ns3::ndn::ProbCacheProducer"
                         : (cacheStrategy == "btw")       ? "ns3::ndn::BtwProducer"
                         :                                  "ns3::ndn::Producer";
  ndn::AppHelper prodHelper(prodTypeId);
  prodHelper.SetAttribute("PayloadSize",
                          StringValue(prodTypeId == "ns3::ndn::Producer" ? "1027" : "1024"));
  prodHelper.SetAttribute("Freshness",   TimeValue(Seconds(2)));
  prodHelper.SetPrefix(dataPrefix);
  for (auto& prod : producers) {
    prodHelper.Install(prod).Start(Seconds(0));
    CreateObject<UniformRandomVariable>();
    NS_LOG_INFO("Producer on " << Names::FindName(prod));
  }

  double endTime = (consumerEndTime > 0.0) ? consumerEndTime : simTime;

  std::string consumerPrefix = poolcache ? ("/u" + dataPrefix) : dataPrefix;

  if (consumerModel == "zipf") {
    ndn::AppHelper consHelper("ns3::ndn::ConsumerZipfMandelbrot");
    consHelper.SetPrefix(consumerPrefix);
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
      NS_LOG_INFO("Consumer on " << Names::FindName(consumers[i]));
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
    consHelper.SetPrefix(consumerPrefix);
    consHelper.SetAttribute("LifeTime", StringValue(std::to_string(dataLifeTime) + "s"));
    consHelper.SetAttribute("RetxTimer", StringValue(std::to_string(retxMs) + "ms"));

    double startTime = consumerStartOffset + 0.1;
    uint32_t installed = 0;
    for (uint32_t i = 0; i < totalRequests; ++i) {

      if (startTime > simTime) break;
      uint32_t contentID = (uint32_t) (popRand->GetValue() - 1);
      if (contentID >= catalogSize) contentID = catalogSize - 1;
      consHelper.SetAttribute("StartSeq",   IntegerValue(contentID));
      consHelper.SetAttribute("MaxContent", IntegerValue(contentID + 1));
      consHelper.Install(consumers[i % consumers.size()]).Start(Seconds(startTime));
      startTime += startGap->GetValue();
      installed++;
    }
    NS_LOG_INFO("Installed " << installed << "/" << totalRequests
                             << " IOTOneShotConsumer instances across "
                             << consumers.size() << " consumer nodes "
                             << "[firing window " << consumerStartOffset
                             << "s..~" << endTime << "s]");
  }
  else {
    NS_FATAL_ERROR("Unknown --consumerModel: " << consumerModel);
  }

  ndn::GlobalRoutingHelper dataPlane;
  NodeContainer all;
  for (auto& r : routers)   all.Add(r);
  for (auto& p : producers) all.Add(p);
  for (auto& c : consumers) all.Add(c);
  dataPlane.Install(all);
  for (auto& prod : producers) dataPlane.AddOrigins(dataPrefix, prod);
  if (poolcache) {

    for (auto& r : routers) {
      dataPlane.AddOrigins("/u", r);
      dataPlane.AddOrigins("/pool/" + Names::FindName(r), r);
    }
  }
  dataPlane.CalculateRoutes();
  gDataPlane = &dataPlane;
  gDataPrefixes.clear();
  gDataPrefixes.push_back(dataPrefix);

  std::string strat = "/localhost/nfd/strategy/best-route";
  if      (dataStrategy == "asf")       strat = "/localhost/nfd/strategy/asf";
  else if (dataStrategy == "multicast") strat = "/localhost/nfd/strategy/multicast";
  ndn::StrategyChoiceHelper::InstallAll("/data", strat);

  if (deadNodes > 0 && downTime > 0.0 && failPeriod > 0.0) {
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    NodeFailCtx ctx;
    ctx.switches  = routers;
    ctx.deadNodes = deadNodes;
    ctx.downTime  = Seconds(downTime);
    ctx.period    = Seconds(failPeriod);
    ctx.rng       = rng;
    double firstAt = (failStart > 0.0) ? failStart : failPeriod;
    Simulator::Schedule(Seconds(firstAt), &FailRandomNodes, ctx);
    NS_LOG_INFO("NODE-INJECTOR: K=" << deadNodes << " Y=" << downTime << "s "
                << "Z=" << failPeriod << "s firstAt=" << firstAt << "s");
  }

  ndn::AppDelayTracer::InstallAll("app-delays-trace.txt");
  ndn::L3RateTracer::InstallAll("l3-rate-trace.txt", Seconds(1.0));

  ndn::CsTracer::InstallAll("cs-trace.txt", Seconds(1.0));

  gBaselineCsFile.open("baseline_cs_stats.csv");
  gBaselineCsFile << "Time\tNodeID\tEvictionsDelta\n";
  if (poolcache) {
    PoolEvictCtx ctx;
    for (auto& r : routers) ctx.names.push_back(Names::FindName(r));
    ctx.apps   = poolApps;
    ctx.period = Seconds(1.0);
    Simulator::Schedule(ctx.period, &DumpPoolCsStats, ctx);
  } else {
    BaselineCsStatsCtx ctx;
    ctx.routers = NodeContainer();
    for (auto& r : routers) ctx.routers.Add(r);
    ctx.period = Seconds(1.0);
    Simulator::Schedule(ctx.period, &DumpBaselineCsStats, ctx);
  }

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}

}

int main(int argc, char* argv[]) { return ns3::main(argc, argv); }
