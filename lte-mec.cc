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
#include <cmath>

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

#include "ns3/basic-energy-source.h"
#include "ns3/simple-device-energy-model.h"
#include "ns3/li-ion-energy-source.h"
#include "ns3/energy-source-container.h"
#include "ns3/energy-model-helper.h"
#include "ns3/energy-source.h"
#include "ns3/li-ion-energy-source-helper.h"

#include "ns3/yans-wifi-helper.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

using namespace ns3;


std::vector <int16_t> stdMeter;
std::vector <int16_t> stopFlag;
  

  
//WAVE function
void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    {
      NS_LOG_UNCOND ("Received one packet!");
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

  
  
  
/*
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
      
}*/

static double
CalculateAngle(Vector myposition)
{
  double angle;
  const double RadtoDeg = 57.2957951; //180/pi
  
  if(myposition.y==0 && myposition.x==0){ //velocity=0
    angle=0;
    }
  
  else{
    angle = atan2(myposition.y, myposition.x) * RadtoDeg; //degree
    }
    
    
  return angle;

}

//Modify packet data field - UE(pedestrian, vehicle -> current position, velocity, angle)
//This function is similar to GPS receiving
//called every 100msec
static void
ModifyPacketData_UE (Ptr<Node> node, UdpClientHelper udpclient, Ptr <Application> app )
{
   
   Ptr<MobilityModel> mymobility=node->GetObject<MobilityModel>();  //get mobility model

   uint32_t ID=node->GetId();
   std::string nodeID=std::to_string(ID);
   
   Vector myvelocity = mymobility -> GetVelocity(); //get Velocity vector
   Vector myposition = mymobility -> GetPosition(); //get Position vector
   
   double angle = CalculateAngle(myposition);
   
   //Literally, sending_data is the data to be snet in a packet
   std::string sending_data; 
   std::string pos_x, pos_y, velo_x, velo_y;
   std::string angle_str;
   
   pos_x=std::to_string(myposition.x);
   pos_y=std::to_string(myposition.y);
   velo_x=std::to_string(myvelocity.x);
   velo_y=std::to_string(myvelocity.y);
   angle_str=std::to_string(angle);
   
   
   //Each data element in the packet is separated by "/"
   sending_data.append(nodeID); sending_data.append("/");
   
   sending_data.append(pos_x); sending_data.append("/");
   sending_data.append(pos_y); sending_data.append("/");
 
   sending_data.append(velo_x); sending_data.append("/");
   sending_data.append(velo_y); sending_data.append("/");
   sending_data.append(angle_str);

   udpclient.SetFill(app, sending_data);
   Simulator::Schedule(Seconds(0.1), &ModifyPacketData_UE, node, udpclient, app);


}


//Modify packet data field (MEC server send to vehicle => ALL DATA FROM MEC DB)
//Get latest DB data
static void
ModifyPacketData_MEC (UdpClientHelper udpclient, Ptr <Application> app )
{   
   std::map<int32_t, std::vector<double>>::iterator it;
   //Literally, sending_data is the data to be sent in a packet
   std::string sending_data;
   
   std::string nodeID;
   std::string pos_x, pos_y, velo_x, velo_y, angle_str;
   
   //get all pedestrian data stored in MEC DB
   for(it=mecDb.begin(); it!=mecDb.end(); it++){
      nodeID=std::to_string(it->first);
      pos_x=std::to_string(it->second[0]); 
      pos_y=std::to_string(it->second[1]);
      velo_x=std::to_string(it->second[2]);
      velo_y=std::to_string(it->second[3]);   
      angle_str=std::to_string(it->second[4]);
      
      //Each data element in the packet is separated by "/"
      sending_data.append(nodeID); sending_data.append("/");
      sending_data.append(pos_x); sending_data.append("/");
      sending_data.append(pos_y); sending_data.append("/");
      sending_data.append(velo_x); sending_data.append("/");
      sending_data.append(velo_y); sending_data.append("/");
      sending_data.append(angle_str); sending_data.append("/");
   }
    
   udpclient.SetFill(app, sending_data);
      Simulator::Schedule(Seconds(0.1), &ModifyPacketData_MEC, udpclient, app);
}

void
PrintDataBase ()
{

//checking mecDb data
  std::map<int32_t, std::vector<double>>::iterator it;
  std::cout << "Start MEC Database print at "<< Simulator::Now()<<std::endl;
  for(it=mecDb.begin(); it!=mecDb.end(); it++){
     std::cout << "key: "<< it->first<<std::endl;
     for(uint32_t i=0; i<6; i++){
        std::cout <<"value: "<< it->second[i] <<std::endl;
     }
   }
   std::cout << "Finish MEC Database print at "<< Simulator::Now()<<std::endl;

}

/* --> 실험에선 보행자측에서 속도 확인하면서 멈추거나 느리면 자체적으로 자신의 데이터를 DB에서 지움
   --> 그러나 실제에서는 MEC server의 데이터를 보행자가 지울 수 없으므로 아래와 같은 기능이 필요할 


//이 함수는 100mesec마다 호출되어서 MEC database에 저장된 데이터 중 시스템에 참여하지 않는 보행자의 데이터를 제거
//임계값은 실험 코드 초안 작성 후, 보행자 report 주기로 계산될 수 있는 최대값으로 변경 예정 
void
CheckLastModifiedTime () 
{
  std::map<int32_t, std::vector<double>>::iterator it;
  for(it=mecDb.begin(); it!=mecDb.end(); it++){
     
     double lastUpdateTime = (Simulator::Now ().GetSeconds ())-(it->second[4]); //마지막 업데이트 시간
     
     double threshold = 5.0;
     //마지막 업데이트로부터 지금까지 임계값(default=2.0 sec)이상 지났으면
     //보행자가 이 시스템에 참여하지 않는 상태로 간주하고 map에서 해당 보행자 데이터 제거 
     if (lastUpdateTime>threshold){ 
        mecDb.erase(it->first);
        std::cout<<it->first<<"번 노드의 데이터가 "<<threshold<<"sec동안 업데이트 되지 않았습니다."<<std::endl;
        std::cout<<"따라서"<<it->first<<"번 노드의 데이터를 MEC Database에서 제거합니다."<<std::endl;
     }
  }

}
*/

//Print Remaining Energy of UE (every 1sec 너무 짧은 듯) 
static void
PrintCellInfo (Ptr<EnergySource> es)
{
  std::cout << "At " << Simulator::Now ().GetSeconds () << "sec Cell voltage: " << es->GetSupplyVoltage () << " V Remaining Capacity: " <<
  es->GetRemainingEnergy () / (3.6 * 3600) << " Ah " << es->GetNode ()->GetId() << " node ID" <<std::endl;
  Ptr<LiIonEnergySource> bat = es->GetNode ()->GetObject<LiIonEnergySource> ();
  NS_LOG_UNCOND (bat->GetRemainingEnergy ());

  if (!Simulator::IsFinished ())
    {
      Simulator::Schedule (Seconds (1),
                           &PrintCellInfo,
                           es);
    }
}

//Step 1. 보행자의 상태 확인 후 보행 중이면 알고리즘 진입
//Step 2. 보행자의 mobility model가져와서 속도 구하기 (1m/s)
//Step 3. 속도, 현재 보행 상태, 위치(PDZ or NPDZ)를 바탕으로 다음 전송 타이밍 계산
//Step 4. 계산된 시간을 노드의 interval로 설정 (지금부터 interval뒤 패킷 전송 됨)
//Step 5. 계산된 시간 뒤 다시 MyAlgorithm을 호출(한번 전송 후 다시 현재 속도를 확인하고 타이밍 계산)

static void
MyAlgorithm (Ptr<Node> node, UdpClientHelper udpclient, Ptr <Application> app , uint32_t vehNum)
{
  uint32_t nodeID = node->GetId();
  double next;
  if(stopFlag.at(nodeID - vehNum)==0){ }
  else{

  
  Ptr<MobilityModel> mymobility=node->GetObject<MobilityModel>();  //get mobility model
  
  Vector myvelocity = mymobility -> GetVelocity(); //get Velocity vector

  next = sqrt((myvelocity.x) * (myvelocity.x) + (myvelocity.y) * (myvelocity.y)); //scalar data (not vector data)
  
  
  //PDZ이면 stdMeter에 담긴 값이 1, NPDZ이면 stdMeter에 담긴 값이 3
  //보행자가 정지, 건물 내, 차량 탑승과 같은 상태이면 stopFlag에 0이 있어 report중지
  //보행자가 정상 보행(위의 상황이 아니라면) stopFlag에 1이 있어 구한 타이밍대로 report
  next = next * stdMeter.at(nodeID - vehNum) * stopFlag.at(nodeID - vehNum ); 
  
  
  //for debugging
  std::cout<<"================MY ALGORITHM================"<<std::endl;
  std::cout<<nodeID<<"번 노드의 speed: "<<next<<" 기준 거리는 "
  <<stdMeter.at(nodeID - vehNum) <<"이고 stop flag는"<<stopFlag.at(nodeID - vehNum)<<std::endl;
  std::cout<<"next ="<<next<<" Time: "<<Simulator::Now().GetSeconds()<<std::endl;

  //schedule & set interval
  Simulator::Schedule(Time(Seconds(next)), &MyAlgorithm, node, udpclient, app, vehNum);
  udpclient.SetInterval(app, Time(Seconds(next)));

  std::cout<<nodeID<<"번 노드 알고리즘 수행 완료! 새로운 주기: "<<Time(Seconds(next))<<std::endl;
  std::cout<<"알고리즘은 "<<Time(Seconds(next))<<"sec후에 다시 수행됩니다!"<<std::endl;

}
}


//Check if node is in PDZ or not
static bool
IsPDZ(Vector position)
{
  double x=position.x;
  double y=position.y;
  uint32_t check=0;
  
  if((x<=20) || ((x>=87.4) && (x<=117.4)) || (x>=184.8)){
    check++;
  }
  if((y<=-70) || ((y>=-15) && (y<=15)) || (y>=70)){
    check++;
  }
  if(check==2){
    return true; //PDZ
  }
  else{
    return false; //NPDZ
  }

}

//Determining each pedestrian's stdMeter (whether PDZ or NPDZ)
//called every 100msec
static void
CheckPedPosition (Ptr<Node> node, UdpClientHelper udpclient, Ptr <Application> app, uint32_t vehNum )
{
   Ptr<MobilityModel> mymobility=node->GetObject<MobilityModel>();  //get mobility model
   
   Vector myposition = mymobility -> GetPosition(); //get Position vector
   
   std::string pos_x, pos_y;
   uint32_t nodeID = node->GetId();
   
   if (IsPDZ(myposition)){
     //PDZ - 기준거리 1m
     std::cout<<"보행자 "<<nodeID<<"는 PDZ에 위치합니다."<<std::endl;
     
     if( stdMeter[nodeID - vehNum]==3){ //NPDZ에서 PDZ 진입
      stdMeter[nodeID - vehNum] = 1;
      
      //critical situation
      udpclient.SetInterval(app, Time(Seconds(0.01))); //당장 보낼 수 있도록 interval 설정
      Simulator::ScheduleNow(&MyAlgorithm, node, udpclient, app, vehNum); //일단 몇번 보내놓고 제대로 된 알고리즘 구하러 보내기

     }
     stdMeter[nodeID - vehNum] = 1;

   }
   else{
     //NPDZ - 기준거리 Nm(N=2,3,5,10 ...) default=3
     std::cout<<"보행자 "<<nodeID<<"는 NPDZ에 위치합니다."<<std::endl;

     if(stdMeter[nodeID - vehNum ]==1){ //PDZ에서 NPDZ진입  - not critical situation
       stdMeter[nodeID - vehNum ] = 3;
     
     }
       stdMeter[nodeID - vehNum ] = 3;
   }
  // Simulator::Schedule(Seconds(0.1), &CheckPedPosition, node, udpclient, app, vehNum);
}

static void
CheckVehActive (Ptr <Node> node, Ptr <Application> app, PacketSinkHelper helper){

  Ptr<MobilityModel> mymobility=node->GetObject<MobilityModel>();  //get mobility model
   
  Vector myvelocity = mymobility -> GetVelocity(); //get Velocity vector
  
  uint32_t nodeID = node->GetId();
  double speed;
  speed = sqrt((myvelocity.x) * (myvelocity.x) + (myvelocity.y) * (myvelocity.y)); //scalar data (not vector)
  
  if(speed == 0){
    Simulator::Schedule(Seconds(0.1), &CheckVehActive, node, app, helper);
  //  app->SetStopTime(Simulator::Now());
  //app->StopApplication();

  
  }
  else{
  

    //app->SetStartTime(Simulator::Now());
  //  app->StartApplication();
    std::cout<<nodeID <<"번 자동차 이동 시작"<<std::endl;

  }



}


//Check pedestrian velocity(speed) and change stopFlag
//Called every 100msec
static void
CheckPedVelocity (Ptr<Node> node, UdpClientHelper udpclient, Ptr <Application> app, uint32_t vehNum )
{
  
  Ptr<MobilityModel> mymobility=node->GetObject<MobilityModel>();  //get mobility model
   
  Vector myvelocity = mymobility -> GetVelocity(); //get Velocity vector
  
  uint32_t nodeID = node->GetId();
  double speed;
  speed = sqrt((myvelocity.x) * (myvelocity.x) + (myvelocity.y) * (myvelocity.y)); //scalar data (not vector)
  std::cout<<" 현재 interval"<<udpclient.GetInterval(app)<<std::endl; 
  
  
  if ((speed== 0) && stopFlag[nodeID-vehNum]== 0){ //첫 1초에 확인해보니 1초부터 시작하는 애 아닌 경우 
    /*
    udpclient.SetInterval(app, Time(Seconds(0.2)));
    MyAlgorithm(node, udpclient, app, vehNum);*/
    udpclient.SetInterval(app, Time(Seconds(0.0))); //set interval 0 (stop application)
        mecDb.erase(nodeID);
  }
  else if ((speed < 0.1) &&(stopFlag[nodeID - vehNum] == 1) ){ //속도가 기준 미만으로 떨어진 순간 
    //probably stop or indoor
    stopFlag[nodeID - vehNum ] = 0;
    std::cout<<"Stop reporting ... nodeID: "<<nodeID<<" speed: "<<speed<<"status: stop or indoor"<<std::endl;
    std::cout<<"보행자가 움직이지 않습니다. "<<nodeID<<"번 노드의 데이터를 MEC Database에서 제거합니다."<<std::endl;
    //send멈추고 db에서 erase
    mecDb.erase(nodeID);

    udpclient.SetInterval(app, Time(Seconds(0.0))); //set interval 0 (stop application)

  }
  
  else if(speed<0.1){ //계속해서 알고리즘 참여안하는 상황 
      std::cout<<"Not sending ... nodeID: "<<nodeID<<" speed: "<<speed<<"status: stop or indoor"<<std::endl;
  }
  else if (speed > 4){ //속도가 기준 이상인 상황 (차량) 
    //probably in Vehicle
    stopFlag[nodeID - vehNum ] = 0;
    std::cout<<"Stop reporting ... nodeID: "<<nodeID<<" speed: "<<speed<<"status: in Vehicle"<<std::endl;
  }
  
  //속도가 정상 범위 내인데 현재 stopFlag가 0인 상황 + interval이 0인 상황(한번 멈춰진 상황에서 복귀하는 상황) 
  else if( (stopFlag[nodeID - vehNum ] == 0) && (udpclient.GetInterval(app)==0)){ 
    stopFlag[nodeID - vehNum ] = 1;
    std::cout<<"Start reporting (복귀) ... nodeID: "<<nodeID<<" speed: "<<speed<<"status: Walk"<<std::endl;

    udpclient.SetInterval(app, Time(Seconds(0.001))); //당장 보내기, 그러고 나서 알고리즘으로 올바른 시점 계산 및 적 
    udpclient.StartAgain(app);
    MyAlgorithm(node, udpclient, app, vehNum);
    
  } 
  
  //속도가 정상 범위 내인데 현재 stopFlag가 0인 상황 + interval!=0이므로 첫 1초
  else if( stopFlag[nodeID - vehNum] == 0 ){ 

    stopFlag[nodeID - vehNum ] = 1;
    std::cout<<"Start(First time) reporting ... nodeID: "<<nodeID<<" speed: "<<speed<<"status: Walk"<<std::endl;
    udpclient.SetInterval(app, Time(Seconds(0.001)));
    MyAlgorithm(node, udpclient, app, vehNum);

    
  }   
  else {
    //stopFlag[nodeID - vehNum -3] = 1; //이전 flag도 1이고 속도도 정상 범위인 상황 -> 계속 보행 중 
    stopFlag[nodeID - vehNum] = 1;
    std::cout<<"Continuing reporting ... nodeID: "<<nodeID<<" speed: "<<speed<<"status: Walk"<<std::endl;
  }
  // Simulator::Schedule(Seconds(0.1), &CheckPedVelocity, node, udpclient, app, vehNum);
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
  //LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

  for(int i=0;i<argc;i++){
    NS_LOG_LOGIC("argv["<<i<<"] = " << argv[i]);
  }

  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint16_t numberOfPed = 500;
  uint16_t numberOfNodes = 750; 
  uint16_t numberOfVeh = 250;  
  uint16_t simTime = 3600; 
  double distance = 60.0; 
  double interPacketInterval = 100; 
  std::string traceFile1;
  std::string traceFile2; 
  std::string logFile;


  
  
  //MEC server Database는 Map<uint32_t, std::vector<double>> 사용 
  
  //MEC서버가 보행자에게 받은 데이터를 기록, 업데이트하고 
  //그 데이터를 100msec마다 읽어서 차량들에게 전송하는데 사용되는 공간
  //보행자 별 update(report) 타이밍이 다르기 때문에 자료구조에서 탐색하고 재기록하는데 노드 간 순서가 없음
  //따라서 node ID를 기준으로 빠르게 탐색할 수 있는 자료구조가 적합하다고 판단되어 Map 자료구조 사용 
  //extern 헤더파일 사용해서 헤더파일 추가한 모든 파일에서 map(MEC DB)에 접근 가능



  // Command line arguments
  CommandLine cmd;
  cmd.AddValue("traceFile1", "Ns2 movement trace file - vehicle", traceFile1);
  cmd.AddValue("traceFile2", "Ns2 movement trace file - pedestrian", traceFile2);
  cmd.AddValue("numberOfNodes", "Number of UE nodes", numberOfNodes);
  cmd.AddValue("numberOfPed", "Number of Pedestrian nodes", numberOfPed);
  cmd.AddValue("numberOfVeh", "Number of Pedestrian nodes", numberOfVeh);
  cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("logFile", "Log file", logFile);
  cmd.AddValue("distance", "Distance between eNBs [m]", distance);
  cmd.AddValue("interPacketInterval", "Inter packet interval [ms])", interPacketInterval);
  cmd.Parse(argc, argv); 



  cmd.Parse (argc,argv);

  // Check command line arguments
  if (traceFile1.empty () || traceFile2.empty() || numberOfNodes <= 0 || numberOfPed <= 0|| numberOfVeh <= 0|| simTime <= 0 || logFile.empty ())
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
    
      stdMeter.assign(numberOfPed, 1);
  stopFlag.assign(numberOfPed, 0);
  
    
// open log file for output
 // std::ofstream os;
 // os.open (logFile.c_str ());

  NodeContainer ueNodes; 
  NodeContainer enbNodes;
  ueNodes.Create(numberOfNodes);
  enbNodes.Create(1); //only one eNB 
  Ptr<Node> enb = enbNodes.Get(0);








  //ltehelper, epc로 LTE 기본환경 set
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

  // Install Mobility Model (enb node)
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector(247.2, 252.4, 0));
  positionAlloc->Add (Vector(249.2, 252.4, 0));
  
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator(positionAlloc);
  enbmobility.Install(enbNodes);



 // Create Ns2MobilityHelper with the specified trace log file as parameter
  Ns2MobilityHelper ns2veh = Ns2MobilityHelper (traceFile1);
  ns2veh.Install (); // configure movements for each node, while reading trace file
  
  Ns2MobilityHelper ns2ped = Ns2MobilityHelper (traceFile2);
  ns2ped.Install (); // configure movements for each node, while reading trace file

  // Configure callback for logging
 // Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
 //                  MakeBoundCallback (&CourseChange, &os));
 

  // Install LTE Devices to the nodes 
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  //p2ph.EnablePcapAll("0309_angle"); 
  
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
  
  
  //UE를 보행자와 차량 그룹으로 분류 
  NodeContainer pedNodes; 
  NodeContainer vehNodes;
  
  for(int u=0; u< numberOfVeh; u++) {
    Ptr<Node> vehicle = ueNodes.Get (u);
    vehNodes.Add(vehicle);
  }

  for(int u=numberOfVeh; u<numberOfNodes; u++){
    Ptr<Node> pedestrian = ueNodes.Get (u);
    pedNodes.Add(pedestrian);
  
  }

  Ptr<Node> mec = epcHelper->GetMecNode();
  enbmobility.Install(mec);
  
  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  
  
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  wifiPhy.Set("RxGain", DoubleValue(-10));
   wifiPhy.Set("TxPowerStart", DoubleValue(66));
  wifiPhy.Set("TxPowerEnd", DoubleValue(66));
  //wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
  

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer Wavedevices = wifi80211p.Install (wifiPhy, wifi80211pMac, epcHelper->GetMecNode()); //차량노드에 WAVE 설치 
  Wavedevices.Add(wifi80211p.Install (wifiPhy, wifi80211pMac, vehNodes));
  
   // Tracing
//  wifiPhy.EnablePcap ("wave-simple-80211p", Wavedevices);


  Ipv4AddressHelper ipv4Wave;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4Wave.SetBase ("20.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer iWave = ipv4Wave.Assign (Wavedevices);
  
  
  
/*
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);

*/
//=======================================application==================================================

  // Install and start applications and packet sinks

  uint16_t udpPort = 4000;

  //  uint16_t udpPort_veh = 3200;
  
  ApplicationContainer pedclientApps; //MEC server에게 보내는 pedestrian app
  ApplicationContainer mecclientApps; //Vehicle에게 보내는 MEC app
  ApplicationContainer serverApps; //pedestrian에게 받는 MEC app
  ApplicationContainer vehserverApps; //MEC에게 받는 vehicle app
  ApplicationContainer vehclientApps; //통신 초기 등록을 위해 몇번만 보내는 vehicle app

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
    ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(100)));
    ulClient.SetAttribute ("MaxPackets", UintegerValue(1000000)); 
    ulClient.SetAttribute ("PacketSize", UintegerValue(140)); 
    
    pedclientApps.Add (ulClient.Install (pedNodes.Get(u))); 



    //보행자 휴대폰은 100msec마다 GPS를 수신한다는 가정
    //변화하는 position과 velocity를 100msec마다 가져와 전송 데이터로 대기시킴 
    //report주기와는 별개로, 0.1초마다 갱신해두면 report주기에 맞게 저장되어 있는 (setfill된) 데이터를 전송
    for(uint16_t time=1; time<simTime*10; time++){ //이부분 재귀로 구현해보기(과부하)
       Simulator::Schedule(Seconds(0.9+time*0.1), &ModifyPacketData_UE, pedNodes.Get(u), ulClient, pedclientApps.Get(u));
       Simulator::Schedule(Seconds(1+time*0.1), &CheckPedPosition, pedNodes.Get(u), ulClient, pedclientApps.Get(u), numberOfVeh);
       Simulator::Schedule(Seconds(1+time*0.1), &CheckPedVelocity, pedNodes.Get(u), ulClient, pedclientApps.Get(u), numberOfVeh);
    }
     //  Simulator::Schedule(Seconds(0.9), &ModifyPacketData_UE, pedNodes.Get(u), ulClient, pedclientApps.Get(u));
     //  Simulator::Schedule(Seconds(1+0.01*u), &CheckPedPosition, pedNodes.Get(u), ulClient, pedclientApps.Get(u), numberOfVeh);
     //  Simulator::Schedule(Seconds(1+0.01*u), &CheckPedVelocity, pedNodes.Get(u), ulClient, pedclientApps.Get(u), numberOfVeh);
  
}  

  


//========================================================================================


  uint16_t udpPort_mec = 1200;

    UdpClientHelper mecClient (InetSocketAddress(Ipv4Address("20.1.1.255"), udpPort_mec)); 
    
    mecClient.SetAttribute ("Interval", TimeValue (MilliSeconds(100))); //100msec마다 패킷 전송
    mecClient.SetAttribute ("MaxPackets", UintegerValue(10000000)); 
    mecClient.SetAttribute ("PacketSize", UintegerValue(150)); 

    mecclientApps.Add (mecClient.Install (epcHelper->GetMecNode())); 
    
     for(uint16_t time=0; time<simTime*10; time++){
       Simulator::Schedule(Seconds(0.9+time*0.1), &ModifyPacketData_MEC, mecClient, mecclientApps.Get(0));

    }

 for(uint32_t i=0; i< vehNodes.GetN(); ++i) {

    //MEC Client app - 엣지 서버가 차량들에게 broadcast하는 application

    PacketSinkHelper udpPacketSinkHelper_veh ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), udpPort_mec));

    vehserverApps.Add (udpPacketSinkHelper_veh.Install (vehNodes.Get(i))); //타겟 호스트(엣지 서버)에게 PacketSink install
    
    NS_LOG_LOGIC(i<<"번째 자동차에 서버 앱 설치 완료 "); //log for debugging
    
  
    Simulator::Schedule(Seconds(1.0), &CheckVehActive, vehNodes.Get(i), vehserverApps.Get(i), udpPacketSinkHelper_veh);
  }


  mecclientApps.Start (Seconds (1.2)); //client only send
  mecclientApps.Stop (Seconds (simTime)); //client only sent

 
  pedclientApps.Start (Seconds (1.0)); //ped client only send
  pedclientApps.Stop (Seconds (simTime)); //ped client only sent

  
  vehserverApps.Start (Seconds (1.00)); //veh server only receive
  serverApps.Start (Seconds (0.00)); //mec server only receive

  lteHelper->EnableTraces ();
  
 
  
  for(uint16_t i=0; i<=simTime*10; i++){
     //0.1초마다 현재 MEC DB저장된 보행자 목록 출력  
     Simulator::Schedule(Seconds(i*0.1), &PrintDataBase);
  }
  /*
  for(uint16_t i=0; i<=simTime*100; i++){
     //100msec마다 MEC DB에 저장된 보행자 목록 중 만료된 보행자를 찾아 삭제
     Simulator::Schedule(Seconds(i*0.1), &CheckLastModifiedTime);
  }*/
 

   //배터리 기능 UE에 설치
  Ptr<EnergySourceContainer> esCont = CreateObject<EnergySourceContainer> ();
  for (uint16_t i = 0; i < numberOfPed; i++)
      {
        Ptr<SimpleDeviceEnergyModel> sem = CreateObject<SimpleDeviceEnergyModel> ();
        Ptr<LiIonEnergySource> es = CreateObject<LiIonEnergySource> ();
        esCont->Add (es);
        es->SetNode (pedNodes.Get(i));
        sem->SetEnergySource (es);
        es->AppendDeviceEnergyModel (sem);
        sem->SetNode (pedNodes.Get(i));
        pedNodes.Get(i)->AggregateObject (es);
        PrintCellInfo (esCont->Get(i));
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



  Simulator::Stop(Seconds(simTime));
  
  
  
  AnimationInterface anim("3600duration.xml");
  Simulator::Run();

  /*GtkConfigStore config;
  config.ConfigureAttributes();*/

  flowMonitor->SerializeToXmlFile("flow_stats_mec.xml", true, true);

  Simulator::Destroy();
 // os.close (); // close log file
  return 0;

}
