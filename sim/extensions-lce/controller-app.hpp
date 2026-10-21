#pragma once
#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/ndnSIM/helper/ndn-global-routing-helper.hpp"
#include "ns3/core-module.h"

#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>

namespace ns3 {

class ControllerApp : public ndn::App
{
public:
  static TypeId GetTypeId();
  ControllerApp();
  ~ControllerApp() override;

  void StartApplication() override;
  void StopApplication() override;
  void OnInterest(std::shared_ptr<const ndn::Interest> interest) override;
  void OnData(std::shared_ptr<const ndn::Data> data) override;

  void AddSwitch(const std::string& switchId, bool isEdge);
  void RemoveSwitch(const std::string& switchId);
  void AddProducerToSwitch(const std::string& switchId, const std::string& producerId);
  void RemoveProducerFromSwitch(const std::string& switchId, const std::string& producerId);
  void AddConsumerToSwitch(const std::string& switchId, const std::string& consumerId);
  void RemoveConsumerFromSwitch(const std::string& switchId, const std::string& consumerId);

  void AddNeighborController(const std::string& controllerId);
  void RemoveNeighborController(const std::string& controllerId);
  void ActivateNeighborController(const std::string& controllerId);
  void DeactivateNeighborController(const std::string& controllerId);

  void AddSwitchNeighbor(const std::string& sw, const std::string& nbr);

  void AddKnownController(const std::string& controllerId);
  const std::vector<std::string>& GetKnownControllers() const { return m_knownControllers; }

  void SetRoutingHelpers(ndn::GlobalRoutingHelper* control,
                         ndn::GlobalRoutingHelper* data);

  void RecomputeAndBumpEpoch(const std::string& cause);

  void RecomputeControlPlaneOnly(const std::string& cause);

private:

  enum class PeerStatus { ACTIVE = 0, W1 = 1, W2 = 2, W3 = 3, INACTIVE = 4 };

  struct CtrlNeighbor {
    std::string controllerId;
    PeerStatus  status = PeerStatus::ACTIVE;
    uint32_t    misses = 0;
    uint64_t    lastObservedEpoch = 0;
  };

  struct ManagedSwitch {
    std::string switchId;
    bool        isEdge = false;
    std::vector<std::string> producers;
    std::vector<std::string> consumers;

    uint64_t lastAppliedEpoch = 0;
    Time     lastSyncTime;

    std::vector<uint64_t>                     gatewayWraps;
    std::unordered_map<size_t, uint64_t>      dHitsPerBucket;
    uint64_t                                  evictions = 0;

    std::vector<uint64_t>                     gatewayWrapsSincePublish;
    std::unordered_map<size_t, uint64_t>      dHitsSincePublish;
    uint64_t                                  evictionsSincePublish = 0;
  };

  struct EpochStats {
    Time createdAt;
    std::unordered_map<std::string, Time> firstAdvAt;
    std::unordered_map<std::string, Time> firstSyncAt;
  };

private:
  ManagedSwitch* getSwitch(const std::string& id);
  CtrlNeighbor*  getNeighbor(const std::string& id);

  void ReplySwitchHeartbeat(const ndn::Name& name, const std::string& swId);
  void ReplyControllerHeartbeat(const ndn::Name& name, const std::string& srcCtrl);
  void ReplyFibSync(const ndn::Name& name, const std::string& swId, uint64_t lastEpoch);
  void ReplyControllerFibSync(const ndn::Name& name, const std::string& srcCtrl, uint64_t lastEpoch);
  void SendControllerFibSync(const std::string& peerId, uint64_t lastEpoch);

  void SendControllerHeartbeats();
  void RetryControllerHeartbeats();

private:
  std::string m_controllerId;
  double      m_hbFirst   = 10.0;
  double      m_hbInterval= 60.0;
  double      m_hbTimeout = 10.0;
  uint32_t    m_hbRepeat  = 3;

  std::string m_controlPrefix;
  std::string m_xcontrolPrefix;

  ndn::GlobalRoutingHelper* m_controlPlane = nullptr;
  ndn::GlobalRoutingHelper* m_dataPlane    = nullptr;

  std::map<std::string, ManagedSwitch> m_switches;
  std::vector<CtrlNeighbor>            m_neighbors;
  std::vector<std::string>             m_knownControllers;

  uint64_t m_fibEpoch = 0;
  uint64_t m_ctrlEpoch = 0;
  std::map<uint64_t, EpochStats> m_epochStats;

  std::map<std::string, std::set<std::string>> m_switchNeighborsInCluster;

  std::map<std::string, std::set<std::string>> m_inactiveReports;

  std::map<std::string, Time> m_pendingSince;
  std::map<std::string, Time> m_lastHbReceived;

  double m_pendingTimeout   = 120.0;
  double m_hbSilenceTimeout = 180.0;

  EventId m_deadTimeoutCheckEvent;
  void    CheckDeadTimeouts();

  bool        m_useRlBandit  = false;
  std::string m_rlBridgeDir;
  double      m_rlPollSeconds = 0.005;
  double      m_rlPollMaxWait = 5.0;
  uint64_t    m_rlDecisionId  = 0;

  double      m_rlPeriodicInterval = 0.0;
  EventId     m_rlPeriodicEvent;
  void        RlPeriodicCheck();

  std::string BuildVwRequest(const std::string& shared_tag,
                             const std::vector<std::string>& action_features) const;

  uint64_t WriteBanditRequest(const std::string& payload,
                              std::string& outPath);

  bool ReadBanditResponse(uint64_t decisionId,
                          int&   outChosenIdx,
                          double& outProb) const;

  void WriteBanditFeedback(uint64_t decisionId,
                           int chosenIdx,
                           double reward,
                           double probability) const;

  struct BanditPendingDecision {
    uint64_t    id;
    std::string payload;
    int         chosenIdx;
    double      probability;
    std::string branchTag;
    Time        tSent;
    double      liveMoveCost;
    size_t      bucket;

    std::string srcHost;
    std::string dstHost;

    uint64_t evictionBefore = 0;
  };
  std::vector<BanditPendingDecision> m_banditPending;

  struct RunningStd {
    uint64_t n    = 0;
    double   mean = 0.0;
    double   m2   = 0.0;
    void   update(double x);
    double sd() const;
  };
  RunningStd m_sdEviction;

  double m_w_eviction = 1.0;
  double m_w_cost     = 1.0;

  uint64_t EvictionTotal() const;

  void FinalizePendingBanditDecisions(const char* cause);

  double LiveMoveCost(size_t b) const;

  std::string BanditChooseTarget(size_t                            orphanBucket,
                                 const std::vector<std::string>&   candidates);
  bool        BanditChooseBucketForAlive(const std::string&        aliveSwitch,
                                         const std::vector<size_t>& candidates,
                                         size_t&                   outBucket);

  std::set<std::string> m_deadSwitches;

  void OnDeadNode(const std::string& sw);
  void OnAliveNode(const std::string& sw);

public:
  void SetClusterCacheTable(const std::vector<std::string>& table);
  const std::vector<std::string>& GetClusterCacheTable() const { return m_clusterCacheTable; }

private:

  std::vector<std::string> m_clusterCacheTable;
  size_t                   m_cacheBuckets = 10;

  std::vector<uint64_t>    m_aggPopularity;
  std::vector<uint64_t>    m_popSincePublish;

  void ResetTelemetrySincePublish();

  void ParseSwitchHbStats(const ndn::Name& name, const std::string& swId);

  bool RecomputeAfterDeadNode(const std::string& dead);
  bool RecomputeAfterAliveNode(const std::string& alive);

  void PushCacheTableToSwitch(const std::string& sw);
  void PushCacheTableToAllManagedSwitches();

private:
  EventId m_hbPeriodicEvent;
  EventId m_hbRetryEvent;
};

}
