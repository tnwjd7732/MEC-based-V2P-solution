/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)s
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
 * Author: SooJeong Bang <sujeong98of@naver.com>
 *
 * Modications for MEC simulation purposes by
 * Mikko Majanen <mikko.majanen@vtt.fi>
 * 
 * This program is based on Mikko Majanen's MEC testing code <mikko.majanen@vtt.fi>
 * Copyright (c) 2018 VTT Technical Research Centre of Finland Ltd.
 */

#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/trace-helper.h"
//#include "ns3/gtk-config-store.h"

#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"

#include "ns3/netanim-module.h"
#include "ns3/ipv4-nix-vector-helper.h"
#include "ns3/ipv4-list-routing-helper.h"

using namespace ns3;

/**
 * Sample simulation script for LTE+MEC+EPC. A MEC server is attached to each
 * eNodeB (or PGW). MEC can run both UDP and TCP based applications.
 *
 */



// Prints actual position and velocity when a course change event occurs
static void
CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  // Prints position and velocities
  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}


NS_LOG_COMPONENT_DEFINE ("LteMecExample");

int
main (int argc, char *argv[])
{

   
  LogComponentEnable("EpcEnbApplication", LOG_LEVEL_ALL);
  LogComponentEnable("MecServerApplication", LOG_LEVEL_ALL);
  LogComponentEnable("PointToPointEpcHelper", LOG_LEVEL_ALL);
  LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
  LogComponentEnable("LteMecExample", LOG_LEVEL_ALL);
  LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

  for(int i=0;i<argc;i++){
    NS_LOG_LOGIC("argv["<<i<<"] = " << argv[i]);
  }


  uint16_t numberOfNodes = 8; //UE 명수 
  uint16_t numberOfPed = 5; //UE 명수 
  double simTime = 25.1; 
  double distance = 60.0; //enodeB간 거리
  double interPacketInterval = 100; //0.1초마다 1개의 패킷 전송
  std::string traceFile; //커맨드 라인에서 tracefile(SUMO-tcl) 받아옴 - 파일의 이름
  std::string logFile;

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue("traceFile", "Ns2 movement trace file", traceFile);
  cmd.AddValue("numberOfNodes", "Number of UE nodes", numberOfNodes);
  cmd.AddValue("numberOfPed", "Number of Pedestrian nodes", numberOfPed);
  cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("logFile", "Log file", logFile);
  cmd.AddValue("distance", "Distance between eNBs [m]", distance);
  cmd.AddValue("interPacketInterval", "Inter packet interval [ms])", interPacketInterval);
  cmd.Parse(argc, argv); 



  cmd.Parse (argc,argv);

  // Check command line arguments
  if (traceFile.empty () || numberOfNodes <= 0 || numberOfPed <= 0|| simTime <= 0 || logFile.empty ())
    {
      std::cout << "Usage of " << argv[0] << " :\n\n"
      "./waf --run \"ns2-mobility-trace"
      " --traceFile=src/mobility/examples/default.ns_movements"
      " --numberOfNodes=2 --numberOfPed=1 --simTime=100.0 --logFile=ns2-mob.log --distance=60.0 --interPacketInterval=1000\" \n\n"
      "NOTE: ns2-traces-file could be an absolute or relative path. You could use the file default.ns_movements\n"
      "      included in the same directory of this example file.\n\n"
      "NOTE 2: Number of nodes present in the trace file must match with the command line argument and must\n"
      "        be a positive number. Note that you must know it before to be able to load it.\n\n"
      "NOTE 3: Duration must be a positive number. Note that you must know it before to be able to load it.\n\n";

      return 0;
    }
    
// open log file for output
  std::ofstream os;
  os.open (logFile.c_str ());
  //ltehelper, epc로 LTE 기본환경 셋
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> (); 
  lteHelper->SetEpcHelper (epcHelper);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();
  


  // parse again so you can override default values from the command line
  cmd.Parse(argc, argv);
  
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  Ipv4NixVectorHelper nixRouting;
  Ipv4StaticRoutingHelper staticRouting;
  Ipv4ListRoutingHelper list;
  
  list.Add(staticRouting, 0);
  list.Add(nixRouting, 10);



  // Create a single RemoteHost
  //실험에서 UE-eNB-MEC server는이므로 사실상 remoteHost로 가는 패킷 없음
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.SetRoutingHelper(list);
  internet.Install (remoteHostContainer);




  // Create the Internet
  PointToPointHelper p2ph; //pgw-remotehost간 p2p link(그게Internet?) 생성과 attach
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost); //pgw --- remoteHost


  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0"); //host addres가  .0.0.0
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices); //pgwremotehost
  // interface 0: pgw addr, interface 1: remote host
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1); //pgw (ip: 1.0.0.1)
  NS_LOG_LOGIC("remote host addr: " << remoteHostAddr);
  Ipv4Address pgwAddr = internetIpIfaces.GetAddress(0);//remotehost (ip: 1.0.0.2)
  NS_LOG_LOGIC("PGW addr: " << pgwAddr);


  //Routing
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);


  NodeContainer ueNodes; 
  NodeContainer enbNodes;
  enbNodes.Create(1); //only one eNB 
  Ptr<Node> enb = enbNodes.Get(0);
  ueNodes.Create(numberOfNodes);

  // Install Mobility Model (enb node)
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector(70.0, 180.0, 10));
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator(positionAlloc);
  enbmobility.Install(enbNodes);

/*
  // Install Mobility Model (UE node) -> replace SUMO tracefile
  Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i<numberOfNodes; i++) {
    uePositionAlloc->Add (Vector (1.0+10*i, 170.0, 1.5));
  }
  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  uemobility.SetPositionAllocator(uePositionAlloc);
  uemobility.Install(ueNodes);
*/

 // Create Ns2MobilityHelper with the specified trace log file as parameter
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
  ns2.Install (); // configure movements for each node, while reading trace file

  // Configure callback for logging
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeBoundCallback (&CourseChange, &os));

  // Install LTE Devices to the nodes (LTE 통신가능!)
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

//  p2ph.EnablePcapAll("0302");
  
  // Install the IP stack on the UEs 
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
  {
    Ptr<Node> ueNode = ueNodes.Get (u); 
    // Set the default gateway for the UE 
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
  }


  // Attach one UE per eNodeB
  for (uint16_t i = 0; i < numberOfNodes; i++)
  {
    //attach one UE per eNB:
    //lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(i));
    // side effect: the default EPS bearer will be activated

    //attach all UEs to the same eNB:
    lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(0));
  }
  
  
  //보행자와 차량 그룹으로 분류 
  NodeContainer pedNodes; //pedestrian과 vehicle모두 UE이지만, 다른 application을 설치하고 다르게 관리하기 위해서 두개의 node container로 구
  NodeContainer vehNodes;
  
  for(int u=0; u< numberOfNodes - numberOfPed; u++) {
    Ptr<Node> vehicle = ueNodes.Get (u);
    vehNodes.Add(vehicle);
  }

  for(int u=numberOfNodes-numberOfPed; u<numberOfNodes; u++){
    Ptr<Node> pedestrian = ueNodes.Get (u);
    pedNodes.Add(pedestrian);
  
  }

  

//=======================================application==================================================

  // Install and start applications and packet sinks

  uint16_t udpPort = 2000;
  uint16_t udpPort_mec = 1200;
    uint16_t udpPort_veh = 3200;
  
  ApplicationContainer pedclientApps; //MEC server에게 보내는 pedestrian app
  ApplicationContainer mecclientApps; //Vehicle에게 보내는 MEC app
  ApplicationContainer serverApps; //pedestrian에게 받는 MEC app
  ApplicationContainer vehserverApps; //MEC에게 받는 vehicle app
  ApplicationContainer vehclientApps; //통신 초기 등록을 위해 한번만 보내는 app

  bool useMecOnEnb = true; //true --> MEC on eNB 
  bool useMecOnPgw = false;//true --> MEC on PGW (if not on eNB) 
  //if both above are false, no MEC, packet sinks are on remote host

 Ptr<Node> targetHost;
    Ipv4Address targetAddr;
    if(useMecOnEnb){
      targetHost = epcHelper->GetMecNode(); //TODO: works only for 1 eNB !
      targetAddr = epcHelper->GetMecAddr();
    }
    else if(useMecOnPgw){ 
      targetHost = pgw;
      targetAddr = pgwAddr;
    }
    else {
      targetHost = remoteHost; 
      targetAddr = remoteHostAddr;
    }
    
    
  for (uint32_t u = 0; u < pedNodes.GetN (); ++u)
  {
  //해당 노드(vehicle or pedestrian..)만의 port num을 가지고 serverapp과의 커넥션
    ++udpPort;
    PacketSinkHelper udpPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), udpPort));

 

    //targetHost(MEC server)에 udp sinkhelper를 모두 설치
    serverApps.Add (udpPacketSinkHelper.Install (targetHost)); //타겟 호스트(엣지 서버)에게 PacketSink install


    NS_LOG_LOGIC("targetHostAddr = " << targetAddr);
    UdpClientHelper ulClient (InetSocketAddress(targetAddr, udpPort)); 
    ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval))); //100msec마다 패킷 전송
    // 이 부분을 My Algorithm을 추가해서 interPacketInterval을 구하도록 해야함!! 동적으로 변화하는 report timing
    ulClient.SetAttribute ("MaxPackets", UintegerValue(1000000)); 
    ulClient.SetAttribute ("PacketSize", UintegerValue(140)); 

    pedclientApps.Add (ulClient.Install (pedNodes.Get(u))); //지금 가져 UE node에 client app설치 UDP app
}

  for (uint32_t u = 0; u < vehNodes.GetN (); ++u)
  {
  //해당 노드(vehicle or pedestrian..)만의 port num을 가지고 serverapp과의 커넥션
    ++udpPort_veh;
    PacketSinkHelper udpPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), udpPort_veh));

 

    //targetHost(MEC server)에 udp sinkhelper를 모두 설치
    serverApps.Add (udpPacketSinkHelper.Install (targetHost)); //타겟 호스트(엣지 서버)에게 PacketSink install


    NS_LOG_LOGIC("targetHostAddr = " << targetAddr);
    UdpClientHelper vehClient (InetSocketAddress(targetAddr, udpPort_veh)); 
    vehClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval))); //100msec마다 패킷 전송
    // 이 부분을 My Algorithm을 추가해서 interPacketInterval을 구하도록 해야함!! 동적으로 변화하는 report timing
    vehClient.SetAttribute ("MaxPackets", UintegerValue(1000000)); 
    vehClient.SetAttribute ("PacketSize", UintegerValue(140)); 

    vehclientApps.Add (vehClient.Install (vehNodes.Get(u))); //지금 가져 UE node에 client app설치 UDP app
}


 for(uint32_t i=0; i< vehNodes.GetN(); ++i) {
 

    //MEC Client app - 엣지 서버가 차량들에게 broadcast하는 application
    ++udpPort_mec;
    PacketSinkHelper udpPacketSinkHelper_veh ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), udpPort_mec));
    //Ptr<Node> targetVeh;
    //targetVeh = vehNodes.Get(i); //TODO: works only for 1 eNB !

    vehserverApps.Add (udpPacketSinkHelper_veh.Install (vehNodes.Get(i))); //타겟 호스트(엣지 서버)에게 PacketSink install
    NS_LOG_LOGIC(i<<"번째 자동차에 서버 앱 설치 완료 ");
    
    UdpClientHelper mecClient (InetSocketAddress(ueIpIface.GetAddress (i), udpPort_mec)); 
    
    mecClient.SetAttribute ("Interval", TimeValue (MilliSeconds(100))); //100msec마다 패킷 전송
    mecClient.SetAttribute ("MaxPackets", UintegerValue(1000000)); 
    mecClient.SetAttribute ("PacketSize", UintegerValue(150)); 

    mecclientApps.Add (mecClient.Install (targetHost)); //지금 가져 UE node에 client app설치 UDP app
  }



  mecclientApps.Start (Seconds (1.0)); //client only send
  mecclientApps.Stop (Seconds (simTime)); //client only sent

 
  pedclientApps.Start (Seconds (0.01)); //client only send
  pedclientApps.Stop (Seconds (simTime)); //client only sent
  
  vehclientApps.Start (Seconds (0.01)); //client only send
  vehclientApps.Stop (Seconds (simTime)); //client only sent
  
  vehserverApps.Start (Seconds (0.01)); //server only receive
  serverApps.Start (Seconds (0.01)); //server only receive

  lteHelper->EnableTraces ();

//================================================application======================================================
  /*
  // Uncomment to enable PCAP tracing
  //p2ph.EnablePcapAll("lte-mec");
  */

  // Flow monitor
  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  flowMonitor = flowHelper.InstallAll();


  Simulator::Stop(Seconds(simTime));
  
  AnimationInterface anim("0301anim.xml");
  Simulator::Run();

  /*GtkConfigStore config;
  config.ConfigureAttributes();*/

  flowMonitor->SerializeToXmlFile("flow_stats_mec.xml", true, true);

  Simulator::Destroy();
  os.close (); // close log file
  return 0;

}
