#include "bs-app.hpp"
#include "ns3/ndnSIM-module.h"
#include "ns3/core-module.h"
#include "ns3/simulator.h"
#include "ns3/log.h"

#include <algorithm>
#include <string>

NS_LOG_COMPONENT_DEFINE("BSApp");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED(BSApp);

TypeId
BSApp::GetTypeId()
{
  static TypeId tid = TypeId("BSApp")
    .SetParent<ndn::App>()
    .AddConstructor<BSApp>()
    .AddAttribute("BSID", "ID of the base station",
                  StringValue("BS1"),
                  MakeStringAccessor(&BSApp::m_bsID),
                  MakeStringChecker());
  return tid;
}

BSApp::BSApp() {}
BSApp::~BSApp() {}

void
BSApp::StartApplication()
{
  ndn::App::StartApplication();

  m_prefix = "/bs/" + m_bsID + "/";
  ndn::FibHelper::AddRoute(GetNode(), m_prefix, m_face, 0);

  NS_LOG_INFO("BS [" << m_bsID << "]: start t=" << Simulator::Now().GetSeconds()
              << "s prefix=" << m_prefix);
}

void
BSApp::StopApplication()
{
  NS_LOG_INFO("BS [" << m_bsID << "]: stop t=" << Simulator::Now().GetSeconds() << "s");
  ndn::App::StopApplication();
}

void
BSApp::OnInterest(std::shared_ptr<const ndn::Interest> interest)
{
  ndn::App::OnInterest(interest);
  NS_LOG_INFO("BS [" << m_bsID << "]: Interest " << interest->getName());
}

void
BSApp::OnData(std::shared_ptr<const ndn::Data> data)
{
  ndn::App::OnData(data);
  NS_LOG_INFO("BS [" << m_bsID << "]: Data " << data->getName());
}

void
BSApp::SetLinkedSwitch(const std::string& sID)
{
  m_linkedSwitch.switchID = sID;
  NS_LOG_INFO("BS [" << m_bsID << "]: linked-switch=" << sID);
}

const std::string&
BSApp::GetLinkedSwitch() const
{
  return m_linkedSwitch.switchID;
}

void
BSApp::AddProducer(const std::string& pID)
{
  auto it = std::find_if(m_producers.begin(), m_producers.end(),
                         [&](const ProducerInfo& e){ return e.producerID == pID; });
  if (it == m_producers.end()) {
    ProducerInfo info;
    info.producerID = pID;
    info.status = BSStatus::ACTIVE;
    m_producers.push_back(info);

    NS_LOG_INFO("BS [" << m_bsID << "]: +producer " << pID);
  }
}

void
BSApp::RemoveProducer(const std::string& pID)
{
  auto it = std::remove_if(m_producers.begin(), m_producers.end(),
                           [&](const ProducerInfo& e){ return e.producerID == pID; });
  if (it != m_producers.end()) {
    m_producers.erase(it, m_producers.end());
    NS_LOG_INFO("BS [" << m_bsID << "]: -producer " << pID);
  }
}

void
BSApp::AddConsumer(const std::string& cID)
{
  auto it = std::find_if(m_consumers.begin(), m_consumers.end(),
                         [&](const ConsumerInfo& e){ return e.consumerID == cID; });
  if (it == m_consumers.end()) {
    ConsumerInfo info;
    info.consumerID = cID;
    info.status = BSStatus::ACTIVE;
    m_consumers.push_back(info);

    NS_LOG_INFO("BS [" << m_bsID << "]: +consumer " << cID);
  }
}

void
BSApp::RemoveConsumer(const std::string& cID)
{
  auto it = std::remove_if(m_consumers.begin(), m_consumers.end(),
                           [&](const ConsumerInfo& e){ return e.consumerID == cID; });
  if (it != m_consumers.end()) {
    m_consumers.erase(it, m_consumers.end());
    NS_LOG_INFO("BS [" << m_bsID << "]: -consumer " << cID);
  }
}

}
