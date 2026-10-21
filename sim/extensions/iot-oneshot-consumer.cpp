/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2011-2015  Regents of the University of California.
 *
 * This file is part of ndnSIM. See AUTHORS for complete list of ndnSIM authors and
 * contributors.
 *
 * ndnSIM is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * ndnSIM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ndnSIM, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 **/

#include "iot-oneshot-consumer.hpp"

#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/integer.h"

NS_LOG_COMPONENT_DEFINE("ndn.IOTOneShotConsumer");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(IOTOneShotConsumer);

TypeId
IOTOneShotConsumer::GetTypeId()
{
  static TypeId tid =
    TypeId("ns3::ndn::IOTOneShotConsumer")
      .SetGroupName("Ndn")
      .SetParent<Consumer>()
      .AddConstructor<IOTOneShotConsumer>()

      .AddAttribute("Frequency", "Frequency of interest packets",
                    StringValue("1.0"),
                    MakeDoubleAccessor(&IOTOneShotConsumer::m_frequency),
                    MakeDoubleChecker<double>())

      .AddAttribute("Randomize",
                    "Type of send time randomization: none (default), uniform, exponential",
                    StringValue("none"),
                    MakeStringAccessor(&IOTOneShotConsumer::SetRandomize,
                                       &IOTOneShotConsumer::GetRandomize),
                    MakeStringChecker())

      .AddAttribute("MaxContent", "Maximum contents to request (sets seqMax)",
                    IntegerValue(std::numeric_limits<uint32_t>::max()),
                    MakeIntegerAccessor(&IOTOneShotConsumer::m_seqMax),
                    MakeIntegerChecker<uint32_t>())

      .AddTraceSource("ReceivedDataTrace",
                      "Trace called every time there is a data coming",
                      MakeTraceSourceAccessor(&IOTOneShotConsumer::m_receivedDataTrace));

  return tid;
}

IOTOneShotConsumer::IOTOneShotConsumer()
  : m_frequency(1.0)
  , m_firstTime(true)
{
  NS_LOG_FUNCTION_NOARGS();
  m_seqMax = std::numeric_limits<uint32_t>::max();
}

IOTOneShotConsumer::~IOTOneShotConsumer() = default;

void
IOTOneShotConsumer::ScheduleNextPacket()
{
  if (m_firstTime) {
    m_sendEvent = Simulator::Schedule(Seconds(0.0), &Consumer::SendPacket, this);
    m_firstTime = false;
  }
  else if (!m_sendEvent.IsRunning()) {
    m_sendEvent =
      Simulator::Schedule((m_random == nullptr) ? Seconds(1.0 / m_frequency)
                                                : Seconds(m_random->GetValue()),
                          &Consumer::SendPacket, this);
  }
}

void
IOTOneShotConsumer::SetRandomize(const std::string& value)
{
  if (value == "uniform") {
    m_random = CreateObject<UniformRandomVariable>();
    m_random->SetAttribute("Min", DoubleValue(0.0));
    m_random->SetAttribute("Max", DoubleValue(2 * 1.0 / m_frequency));
  }
  else if (value == "exponential") {
    m_random = CreateObject<ExponentialRandomVariable>();
    m_random->SetAttribute("Mean", DoubleValue(1.0 / m_frequency));
    m_random->SetAttribute("Bound", DoubleValue(50 * 1.0 / m_frequency));
  }
  else {
    m_random = nullptr;
  }
  m_randomType = value;
}

std::string
IOTOneShotConsumer::GetRandomize() const
{
  return m_randomType;
}

void
IOTOneShotConsumer::OnData(std::shared_ptr<const Data> contentObject)
{
  Consumer::OnData(contentObject);
  m_receivedDataTrace(contentObject);
}

}
}
