/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Junling Bu <linlinjavaer@gmail.com>
 */
#include "ns3/command-line.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/seq-ts-header.h"

#include "ns3/wifi-80211p-helper.h"
#include "ns3/wifi-net-device.h"

#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"

#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"

#include "ns3/string.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/data-rate.h"

#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"

using namespace ns3;
/**
 * This simulation is to show the routing service of WaveNetDevice described in IEEE 09.4.
 *
 * note: although txPowerLevel is supported now, the "TxPowerLevels"
 * attribute of YansWifiPhy is 1 which means phy devices only support 1
 * levels. Thus, if users want to control txPowerLevel, they should set
 * these attributes of YansWifiPhy by themselves..
 */
class WaveNetDeviceExample
{
public:
  /// Send WSMP example function
  void SendWsmpExample (void);

  /// Send IP example function
  void SendIpExample (void);

  /// Send WSA example
  void SendWsaExample (void);

private:
  /**
   * Send one WSMP packet function
   * \param channel the channel to use
   * \param seq the sequence
   */
  void SendOneWsmpPacket (uint32_t channel, uint32_t seq);
  /**
   * Send IP packet function
   * \param seq the sequence
   * \param ipv6 true if IPV6
   */
  void SendIpPacket (uint32_t seq, bool ipv6);
  /**
   * Receive function
   * \param dev the device
   * \param pkt the packet
   * \param mode the mode
   * \param sender the sender address
   * \returns true if successful
   */
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  /**
   * Receive VSA function
   * \param pkt the packet
   * \param address the address
   * \returns true if successful
   */
  bool ReceiveVsa (Ptr<const Packet> pkt,const Address & address, uint32_t, uint32_t);
  /// Create WAVE nodes function
  void CreateWaveNodes (void);

  NodeContainer nodes; ///< the nodes
  NetDeviceContainer devices; ///< the devices
  Ptr<PacketSink> sink;
  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
};
void
WaveNetDeviceExample::CreateWaveNodes (void)
{
  nodes = NodeContainer ();
  nodes.Create (2);

  InternetStackHelper internet;
  internet.Install (nodes);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = waveChannel.Create ();

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	wifiPhy.SetErrorRateModel("ns3::NistErrorRateModel");
	wifiPhy.Set ("Frequency", UintegerValue (5900));
	wifiPhy.Set ("TxPowerStart", DoubleValue(23));
	wifiPhy.Set ("TxPowerEnd", DoubleValue (23));
	wifiPhy.Set ("ChannelNumber", UintegerValue (180));
	wifiPhy.Set ("ChannelWidth", UintegerValue (10));
	wifiPhy.SetChannel (channel);
	wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);


	//WAVE Helper
	YansWavePhyHelper wavePhy = YansWavePhyHelper::Default ();
	wavePhy.SetErrorRateModel("ns3::NistErrorRateModel");
	wavePhy.Set ("Frequency", UintegerValue (5900));
	wavePhy.Set ("TxPowerStart", DoubleValue(23));
	wavePhy.Set ("TxPowerEnd", DoubleValue (23));
	wavePhy.Set ("ChannelNumber", UintegerValue (180));
	wavePhy.Set ("ChannelWidth", UintegerValue (10));
	wavePhy.SetChannel (channel);
	wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();

  std::string phyMode = "OfdmRate6MbpsBW10MHz";
  Wifi80211pHelper wifi80211pHelper = Wifi80211pHelper::Default ();
  wifi80211pHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                            "DataMode", StringValue (phyMode),
                                            "ControlMode", StringValue (phyMode));

  WaveHelper waveHelper = WaveHelper::Default ();
  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (phyMode),
                                      "ControlMode", StringValue (phyMode));

  devices = wifi80211pHelper.Install (wifiPhy, wifi80211pMac, nodes);
  //devices = waveHelper.Install (wavePhy, waveMac, nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  // for (uint32_t i = 0; i != devices.GetN (); ++i)
  //   {
  //     Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
  //     device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
  //     //device->SetWaveVsaCallback (MakeCallback (&WaveNetDeviceExample::ReceiveVsa, this));
  //   }

  // Tracing
  //wavePhy.EnablePcap ("wave-simple-device", devices);
}

bool
WaveNetDeviceExample::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  std::cout << "receive a packet: " << std::endl
            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
            << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s," << std::endl
            << "  recvTime = " << Now ().GetSeconds () << "s," << std::endl
            << "  protocol = 0x" << std::hex << mode << std::dec  << std::endl;
  return true;
}

void
WaveNetDeviceExample::SendIpPacket (uint32_t seq, bool ipv6)
{
  Ptr<WifiNetDevice> sender = DynamicCast<WifiNetDevice> (devices.Get (0));
  Ptr<WifiNetDevice> receiver = DynamicCast<WifiNetDevice> (devices.Get (1));
  // Ptr<WaveNetDevice> sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  // Ptr<WaveNetDevice> receiver = DynamicCast<WaveNetDevice> (devices.Get (1));

  const Address dest = receiver->GetAddress ();
  // send IPv4 packet or IPv6 packet
  const static uint16_t IPv4_PROT_NUMBER = 0x0800;
  const static uint16_t IPv6_PROT_NUMBER = 0x86DD;
  uint16_t protocol = ipv6 ? IPv6_PROT_NUMBER : IPv4_PROT_NUMBER;
  Ptr<Packet> p  = Create<Packet> (100);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  sender->Send (p, dest, protocol);
}

void
WaveNetDeviceExample::SendIpExample ()
{
  uint16_t port = 9;
  double duration = 10;
  CreateWaveNodes ();
  // Configure channel access
  // Ptr<WaveNetDevice> sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  // Ptr<WaveNetDevice> receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  // const SchInfo schInfo = SchInfo (CH180, false, EXTENDED_CONTINUOUS);
  // Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, sender, schInfo);
  // Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo);
  // Simulator::Schedule (Seconds (duration), &WaveNetDevice::StopSch, sender, CH180);
  // Simulator::Schedule (Seconds (duration), &WaveNetDevice::StopSch, receiver, CH180);
  // // Register txprofile
  // const TxProfile txProfile = TxProfile (CH180);
  // Simulator::Schedule (Seconds (0.0), &WaveNetDevice::RegisterTxProfile, sender, txProfile);
  // Simulator::Schedule (Seconds (duration), &WaveNetDevice::DeleteTxProfile, sender, CH180);

  // Setup node 0 as transmitter and node 1 as sink
  Ptr<Node> txNode = nodes.Get(0);
  Ptr<Node> sinkNode = nodes.Get(1);
  Ptr<Ipv4> ipv4 = sinkNode->GetObject<Ipv4>();
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (ipv4->GetAddress(1,0).GetLocal(), port));
  ApplicationContainer sinkApp = sinkHelper.Install (sinkNode);
  sink = StaticCast<PacketSink> (sinkApp.Get(0));

  OnOffHelper txHelper ("ns3::TcpSocketFactory", InetSocketAddress (ipv4->GetAddress(1,0).GetLocal(), port));
  txHelper.SetAttribute ("PacketSize", UintegerValue (400));
  txHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  txHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  txHelper.SetAttribute ("DataRate", DataRateValue (DataRate ("6Mbps")));
  ApplicationContainer txApp = txHelper.Install (txNode);

  // Start the Applications
  sinkApp.Start (Seconds (0.0));
  txApp.Start (Seconds (2.0));
  sinkApp.Stop (Seconds (duration));
  txApp.Stop (Seconds (duration));

  flowMonitor = flowHelper.InstallAll();

  std::cout << "All setup" << std::endl;

  Simulator::Stop (Seconds (duration));
  Simulator::Run ();

  flowMonitor->CheckForLostPackets();
  flowMonitor->SerializeToXmlFile("wave-simple-device.xml", true, true);

  Simulator::Destroy ();
}

int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  WaveNetDeviceExample example;
  //std::cout << "run WAVE WSMP routing service case:" << std::endl;
  //example.SendWsmpExample ();
  //std::cout << "run WAVE IP routing service case:" << std::endl;
  example.SendIpExample ();
  //std::cout << "run WAVE WSA routing service case:" << std::endl;
  //example.SendWsaExample ();
  return 0;
}
