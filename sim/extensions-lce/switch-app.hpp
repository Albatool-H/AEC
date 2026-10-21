#pragma once
#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/core-module.h"

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

namespace ns3 {

class SwitchApp : public ndn::App
{
public:
  static TypeId GetTypeId();
  SwitchApp();
  ~SwitchApp() override;

  void StartApplication() override;
  void StopApplication() override;
  void OnInterest(std::shared_ptr<const ndn::Interest> interest) override;
  void OnData(std::shared_ptr<const ndn::Data> data) override;

  void AddNeighbor(const std::string& nID);
  void RemoveNeighbor(const std::string& nID);
  void ActivateNeighbor(const std::string& nID);
  void DeactivateNeighbor(const std::string& nID);

  void AddController(const std::string& cID);
  void RemoveController(const std::string& cID);
  void ActivateController(const std::string& cID);
  void DeactivateController(const std::string& cID);

  void SetEdge(bool edge);
  void SendFibSync(const std::string& controllerId, uint64_t lastEpoch);

  void SetCacheTable(const std::vector<std::string>& table);
  const std::vector<std::string>& GetCacheTable() const { return m_cacheTable; }

  void SendCacheTestInterest(const std::string& targetSwitchID);

  void SetCacheCapacity(uint32_t n) { m_dCacheCap = n; }

  void ReactivateAllNeighbors();

  auto GetFace() const -> decltype(m_face) { return m_face; }

private:

  enum class Status { ACTIVE = 0, W1 = 1, W2 = 2, W3 = 3, INACTIVE = 4 };

  struct NeighborInfo {
    std::string neighborID;
    Status      status = Status::ACTIVE;
    uint32_t    misses = 0;
  };

  struct ControllerInfo {
    std::string controllerID;
    Status      status = Status::ACTIVE;
    uint64_t    lastObservedEpoch = 0;
    uint64_t    lastAppliedEpoch  = 0;
    Time        lastAdvTime;
    Time        lastSyncTime;
    uint32_t    misses = 0;
  };

private:
  ControllerInfo* findController(const std::string& cID);
  NeighborInfo*   findNeighbor(const std::string& nID);

  static std::string BlockToString(const ndn::Block& b);
  static bool ExtractUintAfterKey(const std::string& text, const std::string& key, uint64_t& out);

  void SendNeighborsHeartbeats();
  void RetryNeighborsHeartbeats();
  void SendControllersHeartbeats();
  void RetryControllersHeartbeats();

  std::string BuildGwStatsComponent() const;
  std::string BuildEvStatsComponent() const;
  std::string BuildDhStatsComponent() const;
  void        ResetHbCounters();

private:
  std::string m_switchID;
  bool        m_edge = false;
  double      m_hbFirst    = 10.0;
  double      m_hbInterval = 60.0;
  double      m_hbTimeout  = 10.0;
  uint32_t    m_hbRepeat   = 3;

  std::string m_localPrefix;

  std::vector<NeighborInfo>   m_neighbors;
  std::vector<ControllerInfo> m_controllers;

  std::vector<std::string>    m_cacheTable;

  struct GwPendingEntry {
    std::string  originalUri;
    ns3::EventId timer;
  };
  std::unordered_map<std::string, GwPendingEntry> m_pendingGw;

  struct DPendingEntry {
    std::string  wrappedUri;
    ns3::EventId timer;
  };
  std::unordered_map<std::string, DPendingEntry> m_dPending;

  double m_pendingGwTimeoutS = 10.0;
  double m_dPendingTimeoutS  = 5.0;

  void OnPendingGwTimeout(std::string wrappedUri);
  void OnDPendingTimeout(std::string innerUri);

  std::unordered_map<std::string, std::shared_ptr<ndn::Data>> m_dCache;
  std::list<std::string>      m_dCacheOrder;
  uint32_t                    m_dCacheCap = 100;

  uint64_t                                  m_dHitsSinceHB = 0;
  std::unordered_map<size_t, uint64_t>      m_dHitsPerBucketHB;
  std::unordered_map<size_t, uint64_t>      m_bucketDeltas;
  uint64_t                                  m_evictionsSinceHB = 0;

  uint64_t                                  m_dHitTotal      = 0;
  uint64_t                                  m_dMissTotal     = 0;
  uint64_t                                  m_evictionsTotal = 0;

public:
  uint64_t GetDHitTotal()    const { return m_dHitTotal;     }
  uint64_t GetDMissTotal()   const { return m_dMissTotal;    }
  uint64_t GetEvictionsTotal() const { return m_evictionsTotal; }

  EventId m_nbPeriodicEvent;
  EventId m_nbRetryEvent;
  EventId m_ctrlPeriodicEvent;
  EventId m_ctrlRetryEvent;
};

}
