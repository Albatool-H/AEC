#pragma once
#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/ndnSIM-module.h"
#include "ns3/core-module.h"
#include "ns3/log.h"

#include <string>
#include <vector>

namespace ns3 {

enum class BSStatus { ACTIVE = 0, INACTIVE };

struct ProducerInfo {
  std::string producerID;
  BSStatus    status { BSStatus::ACTIVE };
};

struct ConsumerInfo {
  std::string consumerID;
  BSStatus    status { BSStatus::ACTIVE };
};

struct LinkedSwitch {
  std::string switchID;
};

class BSApp : public ndn::App
{
public:
  static TypeId GetTypeId();

  BSApp();
  ~BSApp() override;

  void StartApplication() override;
  void StopApplication() override;
  void OnInterest(std::shared_ptr<const ndn::Interest> interest) override;
  void OnData(std::shared_ptr<const ndn::Data> data) override;

  void SetLinkedSwitch(const std::string& sID);
  const std::string& GetLinkedSwitch() const;

  void AddProducer(const std::string& pID);
  void RemoveProducer(const std::string& pID);

  void AddConsumer(const std::string& cID);
  void RemoveConsumer(const std::string& cID);

  const std::vector<ProducerInfo>& GetProducers() const { return m_producers; }
  const std::vector<ConsumerInfo>& GetConsumers() const { return m_consumers; }

private:
  std::string m_bsID;
  std::string m_prefix;

  LinkedSwitch m_linkedSwitch;
  std::vector<ProducerInfo> m_producers;
  std::vector<ConsumerInfo> m_consumers;
};

}
