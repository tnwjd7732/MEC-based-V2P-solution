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
 
#include <string>
#include <vector>
#include <sstream>
#include <iterator>
#include <iostream>
#include <map>
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
//#include "ns3/database.h"

using namespace ns3;

 //extern void PrintDataBase(void);

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

//Modify packet data field - UE(pedestrian, vehicle -> current position, velocity, angle)
static void
ModifyPacketData_UE (Ptr<Node> node, UdpClientHelper udpclient, Ptr <Application> app )
{
   
   Ptr<MobilityModel> mymobility=node->GetObject<MobilityModel>();  //get mobility model

   uint32_t ID=node->GetId();
   std::string nodeID=std::to_string(ID);
   
   Vector myvelocity = mymobility -> GetVelocity(); //get Velocity vector
   Vector myposition = mymobility -> GetPosition(); //get Position vector
   
   
   //Literally, sending_data is the data to be snet in a packet
   std::string sending_data; 
   std::string pos_x, pos_y, velo_x, velo_y;
   
   pos_x=std::to_string(myposition.x);
   pos_y=std::to_string(myposition.y);
   velo_x=std::to_string(myvelocity.x);
   velo_y=std::to_string(myvelocity.y);
   
   //Each data element in the packet is separated by "/"
   sending_data.append(nodeID); sending_data.append("/");
   sending_data.append(pos_x); sending_data.append("/");
   sending_data.append(pos_y); sending_data.append("/");
 
  // We don't use position_z in Collision Prediction Algorithm
  // sending_data.append(std::to_string(myposition.z)); sending_data.append("/"); 
  
   sending_data.append(velo_x); sending_data.append("/");
   sending_data.append(velo_y); 

   udpclient.SetFill(app, sending_data);

}

//Modify packet data field (MEC server send to vehicle => ALL DATA FROM MEC DB)
static void
ModifyPacketData_MEC (UdpClientHelper udpclient, Ptr <Application> app )
{   
   std::map<int32_t, std::vector<double>>::iterator it;
   //Literally, sending_data is the data to be snet in a packet
   std::string sending_data;
   
   std::string nodeID;
   std::string pos_x, pos_y, velo_x, velo_y;
   
   
   for(it=mecDb.begin(); it!=mecDb.end(); it++){
      nodeID=std::to_string(it->first);
      pos_x=std::to_string(it->second[0]);
      pos_y=std::to_string(it->second[1]);
      velo_x=std::to_string(it->second[2]);
      velo_y=std::to_string(it->second[3]);   
      
      //Each data element in the packet is separated by "/"
      sending_data.append(nodeID); sending_data.append("/");
      sending_data.append(pos_x); sending_data.append("/");
      sending_data.append(pos_y); sending_data.append("/");
      sending_data.append(velo_x); sending_data.append("/");
      sending_data.append(velo_y); sending_data.append("/");
   }
    
   udpclient.SetFill(app, sending_data);
}

void
PrintDataBase ()
{

//checking mecDb data!

  std::map<int32_t, std::vector<double>>::iterator it;
  std::cout << "Start MEC Database print at "<< Simulator::Now()<<std::endl;
  for(it=mecDb.begin(); it!=mecDb.end(); it++){
     std::cout << "key: "<< it->first<<std::endl;
     for(uint32_t i=0; i<5; i++){
        std::cout <<"value: "<< it->second[i] <<std::endl;
     }
   }
   std::cout << "Finish MEC Database print at "<< Simulator::Now()<<std::endl;

}

//이 함수는 100mesec마다 호출되어서 MEC database에 저장된 데이터 중 시스템에 참여하지 않는 보행자의 데이터를 제거
//(100msec 너무 긴시간인가? 서버니까 더 자주 체크해도 될까?)
//임계값은 실험 코드 초안 작성 후, 보행자 report 주기로 계산될 수 있는 최대값으로 변경 예정 
void
CheckLastModifiedTime () 
{
  std::map<int32_t, std::vector<double>>::iterator it;
  for(it=mecDb.begin(); it!=mecDb.end(); it++){
     
     double lastUpdateTime = (Simulator::Now ().GetSeconds ())-(it->second[4]); //마지막 업데이트 시간
     
     double threshold = 2.0;
     //마지막 업데이트로부터 지금까지 임계값(default=2.0 sec)이상 지났으면
     //보행자가 이 시스템에 참여하지 않는 상태로 간주하고 map에서 해당 보행자 데이터 제거 
     if (lastUpdateTime>threshold){ 
        mecDb.erase(it->first);
        std::cout<<it->first<<"번 노드의 데이터가 "<<threshold<<"sec동안 업데이트 되지 않았습니다."<<std::endl;
        std::cout<<"따라서"<<it->first<<"번 노드의 데이터를 MEC Database에서 제거합니다."<<std::endl;
     }
  }





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


  uint16_t numberOfNodes = 8; 
  uint16_t numberOfPed = 5; 
  uint16_t simTime = 25.1; 
  double distance = 60.0; 
  double interPacketInterval = 100; 
  std::string traceFile; 
  std::string logFile;
  
  
  //MEC서버가 보행자에게 받은 데이터를 기록, 업데이트하고 
  //그 데이터를 100msec마다 읽어서 차량들에게 전송하는데 사용되는 공간
  //보행자 별 update(report) 타이밍이 다르기 때문에 자료구조에서 탐색하고 재기록하는데 노드 간 순서가 없음
  //따라서 node ID를 기준으로 빠르게 탐색할 수 있는 자료구조가 적합하다고 판단되어 Map 자료구조 사용 

  
  
  //이어서 할일 - 지금 extern 헤더파일 사용해서 모든 파일에서 map에 접근 가능
  //이젠 진짜 map 접근해서 packetsink에서는 받아온거 기록(있는지 확인하고 있으면 erase, insert)하도록 코드 추가
  //그리고 그걸 MEC udp client app은 데이터 읽어와서 string하나로 만들어 보내기 구현
  //받은 데이터로 veh에서 데이터 추출하고 알고리즘 돌리는건 추후에
  //우선 가장 중요한건 packetsink - write, udpclient - read 구현하는 것!

  

  


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
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost); //pgw --- remoteHost


  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices); 
  // interface 0: pgw addr, interface 1: remote host
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1); 
  NS_LOG_LOGIC("remote host addr: " << remoteHostAddr);
  Ipv4Address pgwAddr = internetIpIfaces.GetAddress(0);
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
  positionAlloc->Add (Vector(70.0, 180.0, 0));
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

  p2ph.EnablePcapAll("0306sat");
  
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

    pedclientApps.Add (ulClient.Install (pedNodes.Get(u))); //지금 가져온 UE node에 client app설치 UDP app

    //보행자 휴대폰은 100msec마다 GPS를 수신한다는 가정
    //변화하는 position과 velocity를 100msec마다 가져와 전송 데이터로 대기시킴 
    //report주기와는 별개로, 0.1초마다 갱신해두면 report주기에 맞게 저장되어 있는 (setfill된) 데이터를 전송
    for(uint16_t time=0; time<simTime*100; time++){
       Simulator::Schedule(Seconds(time*0.01), &ModifyPacketData_UE, pedNodes.Get(u), ulClient, pedclientApps.Get(u));
    }
  
}  

  //vehicle이 초기에 MEC에게 패킷을 보내는 과정이 필요
  //보내지 않으면 MEC쪽에 vehicle UE에 대한 정보가 없어 추후 MEC->Vehicle app 실행에 unknown UE address error occur
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
    vehClient.SetAttribute ("MaxPackets", UintegerValue(3)); //시뮬레이션 시작하고 3번만 보냄->300msec(자기 위치 알리는 용이라서)
    vehClient.SetAttribute ("PacketSize", UintegerValue(140)); 

    vehclientApps.Add (vehClient.Install (vehNodes.Get(u))); //지금 가져온 UE node에 client app설치 UDP app
    
    for(uint16_t time=0; time<100; time++){
       Simulator::Schedule(Seconds(time*0.01), &ModifyPacketData_UE, vehNodes.Get(u), vehClient, vehclientApps.Get(u));
    }
}


 for(uint32_t i=0; i< vehNodes.GetN(); ++i) {
 

    //MEC Client app - 엣지 서버가 차량들에게 broadcast하는 application
    ++udpPort_mec;
    PacketSinkHelper udpPacketSinkHelper_veh ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), udpPort_mec));
    //Ptr<Node> targetVeh;
    //targetVeh = vehNodes.Get(i); //TODO: works only for 1 eNB !

    vehserverApps.Add (udpPacketSinkHelper_veh.Install (vehNodes.Get(i))); //타겟 호스트(엣지 서버)에게 PacketSink install
    NS_LOG_LOGIC(i<<"번째 자동차에 서버 앱 설치 완료 "); //log for debugging
    
    UdpClientHelper mecClient (InetSocketAddress(ueIpIface.GetAddress (i), udpPort_mec)); 
    
    mecClient.SetAttribute ("Interval", TimeValue (MilliSeconds(100))); //100msec마다 패킷 전송
    mecClient.SetAttribute ("MaxPackets", UintegerValue(1000000)); 
    mecClient.SetAttribute ("PacketSize", UintegerValue(150)); 

    mecclientApps.Add (mecClient.Install (targetHost)); 
    
     for(uint16_t time=100; time<simTime*100; time++){
       Simulator::Schedule(Seconds(time*0.01), &ModifyPacketData_MEC, mecClient, mecclientApps.Get(i));
    }
  }

  //0~0.99sec: INITIAL STEP - vehicle nodes and pedestrian nodes (UE nodes) send their packet to MEC server
  //after Initial step - MEC server broadcast to vehicles & vehicle only receive packet not sending 

  mecclientApps.Start (Seconds (1.0)); //client only send
  mecclientApps.Stop (Seconds (simTime-0.01)); //client only sent

 
  pedclientApps.Start (Seconds (0.01)); //client only send
  pedclientApps.Stop (Seconds (simTime-0.01)); //client only sent
  
  vehclientApps.Start (Seconds (0.01)); //client only send
  vehclientApps.Stop (Seconds (0.99)); //client only sent
  
  vehserverApps.Start (Seconds (0.01)); //server only receive
  serverApps.Start (Seconds (0.01)); //server only receive

  lteHelper->EnableTraces ();
  
  for(uint16_t i=0; i<=simTime*10; i++){
     //0.1초마다 현재 MEC DB저장된 보행자 목록 출
     Simulator::Schedule(Seconds(i*0.1), &PrintDataBase);
  }
  
  for(uint16_t i=0; i<=simTime*100; i++){
     //100msec마다 MEC DB에 저장된 보행자 목록 중 만료된 보행자를 찾아 삭제
     Simulator::Schedule(Seconds(i*0.01), &CheckLastModifiedTime);
  }
  


//================================================application======================================================
  /*
  // Uncomment to enable PCAP tracing
  //p2ph.EnablePcapAll("lte-mec");
  */

  // Flow monitor
  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  flowMonitor = flowHelper.InstallAll();



  Simulator::Stop(Seconds(simTime+1));
  
  
  
  AnimationInterface anim("0306anim.xml");
  Simulator::Run();

  /*GtkConfigStore config;
  config.ConfigureAttributes();*/

  flowMonitor->SerializeToXmlFile("flow_stats_mec.xml", true, true);

  Simulator::Destroy();
  os.close (); // close log file
  return 0;

}
