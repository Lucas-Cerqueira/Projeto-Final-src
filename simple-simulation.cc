/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Junling Bu <linlinjavaer@gmail.com>
 *
 */
/**
 * This example shows basic construction of an 802.11p node.  Two nodes
 * are constructed with 802.11p devices, and by default, one node sends a single
 * packet to another node (the number of packets and interval between
 * them can be configured by command-line arguments).  The example shows
 * typical usage of the helper classes for this mode of WiFi (where "OCB" refers
 * to "Outside the Context of a BSS")."
 */

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/netanim-module.h"

#include <iostream>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

/*
 * In WAVE module, there is no net device class named like "Wifi80211pNetDevice",
 * instead, we need to use Wifi80211pHelper to create an object of
 * WifiNetDevice class.
 *
 * usage:
 *  NodeContainer nodes;
 *  NetDeviceContainer devices;
 *  nodes.Create (2);
 *  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
 *  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
 *  wifiPhy.SetChannel (wifiChannel.Create ());
 *  NqosWaveMacHelper wifi80211pMac = NqosWave80211pMacHelper::Default();
 *  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
 *  devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);
 *
 * The reason of not providing a 802.11p class is that most of modeling
 * 802.11p standard has been done in wifi module, so we only need a high
 * MAC class that enables OCB mode.
 */

class SimulationMonitor
{
  private:
    double logInterval; // seconds
    const char* logFilename;

    Ptr<Socket> m_source;
    Ptr<Socket> m_receiver;

    // Packets counters
    uint32_t txPacketsCount;
    uint32_t rxPacketsCount;
    uint32_t cumulativeTxPacketsCount;
    uint32_t cumulativeRxPacketsCount;

    // Bytes counters
    uint32_t txBytesCount;
    uint32_t rxBytesCount;
    uint32_t cumulativeTxBytesCount;
    uint32_t cumulativeRxBytesCount;

    void SentPacket (Ptr<Socket> socket, uint32_t packetSize);
    void ReceivedPacket (Ptr<Socket> socket);

  public:
    SimulationMonitor(double logInterval, const char* logFilename);

    void Start(Ptr<Socket> source, Ptr<Socket> receiver);
    void CalculateMetrics();
    // void StopAndSaveResults();
    //
    // uint32_t GetTxPacketsCount();
    // uint32_t GetRxPacketsCount();
    // uint32_t GetCumulativeTxPacketsCount();
    // uint32_t GetCumulativeRxPacketsCount();
    //
    // uint32_t GetTxBytesCount();
    // uint32_t GetRxBytesCount();
    // uint32_t GetCumulativeTxBytesCount();
    // uint32_t GetCumulativeRxBytesCount();
};

void SimulationMonitor::SentPacket (Ptr<Socket> socket, uint32_t packetSize)
{
  //std::cout << "Packet sent: " << packetSize << "bytes" << std::endl;
  txPacketsCount++;
  cumulativeTxPacketsCount++;
  txBytesCount += packetSize;
  cumulativeTxBytesCount += packetSize;
}

void SimulationMonitor::ReceivedPacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet = socket->Recv ();
  while (packet)
    {
      rxPacketsCount++;
      cumulativeRxPacketsCount++;
      rxBytesCount += packet->GetSize();
      cumulativeRxBytesCount += packet->GetSize();
      packet = socket->Recv ();
    }
}

SimulationMonitor::SimulationMonitor (double logInterval, const char* logFilename)
  : logInterval (logInterval),
    logFilename (logFilename),
    txPacketsCount (0),
    rxPacketsCount (0),
    cumulativeTxPacketsCount (0),
    cumulativeRxPacketsCount (0),
    txBytesCount (0),
    rxBytesCount (0),
    cumulativeTxBytesCount (0),
    cumulativeRxBytesCount (0)
{
  std::cout << "Creating new SimulationMonitor" << std::endl;
}

void SimulationMonitor::Start(Ptr<Socket> source, Ptr<Socket> receiver)
{
  m_source = source;
  m_receiver = receiver;

  m_source->SetDataSentCallback (MakeCallback (&SimulationMonitor::SentPacket, this));
  m_receiver->SetRecvCallback (MakeCallback (&SimulationMonitor::ReceivedPacket, this));

  this->CalculateMetrics();
}

void SimulationMonitor::CalculateMetrics()
{
  if (txPacketsCount == 0)
  {
    Simulator::Schedule (Seconds (this->logInterval), &SimulationMonitor::CalculateMetrics, this);
    return;
  }

  double kbps = rxBytesCount * 8.0 / 1000.0;
  double per = (float)(txPacketsCount - rxPacketsCount)/(float)txPacketsCount;
  double ber = (float)(txBytesCount - rxBytesCount)/(float)txBytesCount;

  std::cout << "Time: " << (Simulator::Now ()).GetSeconds () << "s" << "\t"
  << "PER: " << per << "\t"
  << "BER: " << ber << "\t"
  << kbps << " kbps" << std::endl;

  std::cout << "Position: " << m_source->GetNode()->GetObject<MobilityModel>()->GetPosition() << std::endl;

  txPacketsCount = 0;
  rxPacketsCount = 0;
  txBytesCount = 0;
  rxBytesCount = 0;
  Simulator::Schedule (Seconds (this->logInterval), &SimulationMonitor::CalculateMetrics, this);
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void CheckThroughput()
{
  std::cout << "CheckThroughput" << std::endl;
  Simulator::Schedule (Seconds (1.0), &CheckThroughput);
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 1000;
  double interval = 0.1; // 10 times per second
  double totalSimTime = numPackets * interval + 1;
  bool verbose = false;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);


  NodeContainer c;
  c.Create (2);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // Set Tx Power
  // "Users who want to obtain longer range should configure attributes “TxPowerStart”, “TxPowerEnd” and “TxPowerLevels” of the YansWifiPhy class by themselves."
  // wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  // wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));
  // wavePhy.Set ("TxPowerStart",DoubleValue (m_txp));
  // wavePhy.Set ("TxPowerEnd", DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerStart", DoubleValue (15.0) );
  wifiPhy.Set ("TxPowerEnd", DoubleValue (15.0) );

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
  {
    wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
  }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  // Tracing
  wifiPhy.EnablePcap ("wave-simple-80211p", devices);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 5.0, 0.0));
  positionAlloc->Add (Vector (500.0, -5.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);

  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // mobility.Install (c.Get (0));

  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (c.Get (0));
  Ptr<ConstantVelocityMobilityModel> constVelocityModel = c.Get (0)->GetObject<ConstantVelocityMobilityModel> ();
  constVelocityModel->SetVelocity(Vector (5.0, 0.0, 0.0));

  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (c.Get (1));
  constVelocityModel = c.Get (1)->GetObject<ConstantVelocityMobilityModel> ();
  constVelocityModel->SetVelocity(Vector (-5.0, 0.0, 0.0));

  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  //recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  //source->SetDataSentCallback (MakeCallback (&SentPacket));
  source->Connect (remote);

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (0.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);

  SimulationMonitor simMonitor (1.0, "teste.csv");
  simMonitor.Start(source, recvSink);

  AnimationInterface anim ("animation.xml");
  //anim.EnableWifiPhyCounters(Seconds (0.0), Seconds(totalSimTime));
  //anim.SetMobilityPollInterval(Seconds (0.5));
  //anim.SkipPacketTracing();

  //CheckThroughput();
  Simulator::Stop (Seconds (totalSimTime));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
