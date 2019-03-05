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

#include "ns3/gnuplot.h"
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

#include "ns3/attribute.h"
#include "ns3/random-variable-stream.h"

#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include "ns3/dual-log-distance-propagation-loss-model.h"
#include "ns3/two-ray-propagation-loss-model.h"
#include "ns3/intersection-propagation-loss-model.h"

#include "ns3/rng-seed-manager.h"

using namespace ns3;

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
    Gnuplot2dDataset m_output;

    double logInterval; // seconds
    const char* logFilename;

    NodeContainer m_nodeC;

    Ptr<Socket> m_source;
    Ptr<Socket> m_receiver;

    double distance;

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

    void WriteCsvHeader ();

  public:
    SimulationMonitor (double logInterval, const char* logFilename);

    //void Start(Ptr<Socket> source, Ptr<Socket> receiver);
    void Start (NodeContainer nodeC, Ptr<Socket> source, Ptr<Socket> receiver);
    void CalculateMetrics ();

    double GetNodeRelDistance ();

    Gnuplot2dDataset GetGnuOutput ();
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

SimulationMonitor::SimulationMonitor (double logInterval, const char* logFilename)
  : m_output("Teste plot"),
    logInterval (logInterval),
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
  m_output.SetStyle (Gnuplot2dDataset::LINES);
  std::cout << "Creating new SimulationMonitor" << std::endl;
  WriteCsvHeader();
}

void SimulationMonitor::WriteCsvHeader ()
{
  std::ofstream out (logFilename);
  out << "Time," <<
    "TxPos," <<
    "RxPos," <<
    "RelDistance," <<
    "TxVel," <<
    "RxVel," <<
    "PDR," <<
    "ReceiveRate" <<
    std::endl;
  out.close ();
}

void SimulationMonitor::SentPacket (Ptr<Socket> socket, uint32_t packetSize)
{
  //std::cout << "Packet sent: " << packetSize << "bytes" << std::endl;
  txPacketsCount++;
  cumulativeTxPacketsCount++;
  txBytesCount += packetSize;
  cumulativeTxBytesCount += packetSize;
}

double SimulationMonitor::GetNodeRelDistance()
{
  Ptr<MobilityModel> model1 = m_nodeC.Get(0)->GetObject<MobilityModel>();
  Ptr<MobilityModel> model2 = m_nodeC.Get(1)->GetObject<MobilityModel>();
  double newDistance = model1->GetDistanceFrom (model2);
  // Approaching each other
  if (newDistance < distance)
  {
    distance = newDistance;
    return distance;
  }
  // Getting farther from each other
  distance = newDistance;
  return -distance;
}

void SimulationMonitor::ReceivedPacket (Ptr<Socket> socket)
{
  //std::cout << "Packet received!" << std::endl;
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

void SimulationMonitor::Start(NodeContainer nodeC, Ptr<Socket> source, Ptr<Socket> receiver)
{
  m_nodeC = nodeC;
  m_source = source;
  m_receiver = receiver;

  m_source->SetDataSentCallback (MakeCallback (&SimulationMonitor::SentPacket, this));
  m_receiver->SetRecvCallback (MakeCallback (&SimulationMonitor::ReceivedPacket, this));

  Ptr<MobilityModel> model1 = m_nodeC.Get(0)->GetObject<MobilityModel>();
  Ptr<MobilityModel> model2 = m_nodeC.Get(1)->GetObject<MobilityModel>();
  distance = model1->GetDistanceFrom (model2);

  this->CalculateMetrics();
}

void SimulationMonitor::CalculateMetrics()
{
  if (txPacketsCount == 0)
  {
    Simulator::Schedule (Seconds (this->logInterval), &SimulationMonitor::CalculateMetrics, this);
    return;
  }

  Ptr<MobilityModel> tx_mob = m_nodeC.Get(0)->GetObject<MobilityModel>();
  Ptr<MobilityModel> rx_mob = m_nodeC.Get(1)->GetObject<MobilityModel>();

  double kbps = rxBytesCount * 8.0 / 1000.0;
  double pdr = (float)(rxPacketsCount)/(float)txPacketsCount;
  // double ber = (float)(txBytesCount - rxBytesCount)/(float)txBytesCount;

  double distance = GetNodeRelDistance();

  std::ofstream out (logFilename, std::ios::app);

  // std::cout << "Time: " << (Simulator::Now ()).GetSeconds () << "s" << "\t"
  // << "Distance: " << distance << "m" << "\t"
  // << "PER: " << per << "\t"
  // << kbps << " kbps" << std::endl;

  // NS_LOG_UNCOND ("Time: " << (Simulator::Now ()).GetSeconds ()
  //     << ", TxPos: " << tx_mob->GetPosition()
  //     << ", RxPos: " << rx_mob->GetPosition()
  //     << ", RelDistance: " << distance
  //     << ", TxVel: " << tx_mob->GetVelocity()
  //     << ", RxVel: " << rx_mob->GetVelocity()
  //     << ", PDR: " << pdr
  //     << ", Kbps: " << kbps);

  out << (Simulator::Now ()).GetSeconds () << ","
      << tx_mob->GetPosition() << ","
      << rx_mob->GetPosition() << ","
      << distance << ","
      << tx_mob->GetVelocity() << ","
      << rx_mob->GetVelocity() << ","
      << pdr << ","
      << kbps << std::endl;

  out.close ();

  //std::cout << "Position: " << m_source->GetNode()->GetObject<MobilityModel>()->GetPosition() << std::endl;

  //m_output.Add ((Simulator::Now ()).GetSeconds (), per);
  // m_output.Add (distance, per);

  txPacketsCount = 0;
  rxPacketsCount = 0;
  txBytesCount = 0;
  rxBytesCount = 0;
  Simulator::Schedule (Seconds (this->logInterval), &SimulationMonitor::CalculateMetrics, this);
}

Gnuplot2dDataset SimulationMonitor::GetGnuOutput()
{
  return m_output;
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

  uint32_t fading = 0;
  uint32_t packetSize = 400; // bytes
  double totalSimTime = 150; // seconds


  double awgnVariance = -1;
  // double awgnVariance = 0;
  double interval = 0.1; // 10 times per second
  uint32_t numPackets = (int) (totalSimTime / interval);
  // double totalSimTime = numPackets * interval + 1;
  double freq = 5.9e9;
  double txPower = 20;

  bool verbose = false;

  // Vector pos_vehA (0.0, 5.0, 1.8);
  // Vector pos_vehB (10.0, 5.0, 1.8);

  // Intersection test
  double dt = 60;

  Vector vel_vehA (0, 0.0, 0.0);
  Vector vel_vehB (-30.0/3.6, 0.0, 0.0);

  std::string logFile;

  int n_run = 0;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("logFile", "file to log the metrics", logFile);
  cmd.AddValue ("dt", "distance to intersection", dt);
  cmd.AddValue ("run", "number of the run", n_run);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  Vector pos_vehA (0.0, 0.0, 1.5);
  Vector pos_vehB (500.0, dt, 1.5);

  // Set the run number
  RngSeedManager::SetRun(n_run + numPackets + txPower + dt);

  if (n_run != 0)
  {
    std::ostringstream oss;
    oss << logFile << "_" << n_run;
    logFile = oss.str();
  }

  NodeContainer c;
  c.Create (2);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();

  // Set Tx Power
  // "Users who want to obtain longer range should configure attributes “TxPowerStart”, “TxPowerEnd” and “TxPowerLevels” of the YansWifiPhy class by themselves."
  // wavePhy.Set ("TxPowerStart",DoubleValue (m_txp));
  // wavePhy.Set ("TxPowerEnd", DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower) );
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower) );

  // std::string lossModelName;
  YansWifiChannelHelper wifiChannel;
  // Intersection model
  wifiChannel.AddPropagationLoss ("ns3::IntersectionPropagationLossModel",
                                  "IntersectionCenter",
                                  Vector2DValue (Vector2D (0, dt)),
                                  "Frequency", DoubleValue (freq),
                                  "MinDistance", DoubleValue (1),
                                  "RxStreetWidth", DoubleValue (23),
                                  "IsSuburban", BooleanValue (false));
  if (awgnVariance != 0)
    awgnVariance = std::pow(4.1, 2.0);

  if (awgnVariance != -1)
  {
    NS_LOG_UNCOND("Adicionou ruido com variância " << awgnVariance);
    std::ostringstream ostr;
    ostr << "ns3::NormalRandomVariable[Mean=0.0|Variance=" << awgnVariance << "]";
    std::string randomString = ostr.str();
    wifiChannel.AddPropagationLoss ("ns3::RandomPropagationLossModel",
                                    "Variable", StringValue(randomString));
  }

  // Setup propagation models
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

  // Propagation loss models are additive.
  if (fading != 0)
    {
      // Nakagami model accounts for the variations in signal strength due to multipath fading.
      // The model does not account for the path loss due to the distance traveled by the signal, hence for typical simulation usage it is recommended to
      // consider using it in combination with other models that take into account this aspect.
      /* Parameters:
        * Distance1: Beginning of the second distance field. Default is 80m.
        * Distance2: Beginning of the third distance field. Default is 200m.
        * m0: m0 for distances smaller than Distance1. Default is 1.5.
        * m1: m1 for distances smaller than Distance2. Default is 0.75.
        * m2: m2 for distances greater than Distance2. Default is 0.75.
      */
      // If m0 = m1 = m2 = 1, the Nakagami-m distribution equals the Rayleigh distribution.
      wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
    }

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
  positionAlloc->Add (pos_vehA);
  positionAlloc->Add (pos_vehB);
  mobility.SetPositionAllocator (positionAlloc);

  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // mobility.Install (c.Get (0));

  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (c.Get (0));
  Ptr<ConstantVelocityMobilityModel> constVelocityModel_A = c.Get (0)->GetObject<ConstantVelocityMobilityModel> ();
  constVelocityModel_A->SetVelocity (vel_vehA);

  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (c.Get (1));
  Ptr<ConstantVelocityMobilityModel> constVelocityModel_B = c.Get (1)->GetObject<ConstantVelocityMobilityModel> ();
  constVelocityModel_B->SetVelocity (vel_vehB);

  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  // NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (1), tid); // Vehicle B
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  //recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (0), tid); // Vehicle A
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  //source->SetDataSentCallback (MakeCallback (&SentPacket));
  source->Connect (remote);

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (0.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);

  SimulationMonitor simMonitor (1.0, logFile.c_str ());
  simMonitor.Start(c, source, recvSink);

  Simulator::Stop (Seconds (totalSimTime));
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
