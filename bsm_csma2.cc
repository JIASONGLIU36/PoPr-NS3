#include "ns3/core-module.h"

#include "ns3/assert.h"

#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"

#include "ns3/udp-socket-factory.h"
#include "ns3/tcp-socket-factory.h"

#include "ns3/tcp-header.h"

#include "ns3/mobility-module.h"

#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-global-routing-helper.h"

#include "ns3/csma-module.h"

#include <iostream>
#include <random>
#include <cmath>
#include <map>

#include <typeinfo>

#include <string>

using namespace ns3;

// ./ns3 run "scratch/bsm_csma2.cc --nNodes=50 --bsmInterval=0.4" > b.txt 2> c.txt


NS_LOG_COMPONENT_DEFINE("BSMBDEXP");

// need a global map for storing the BSM received in a second.
std::map<std::string, int> BSMkeeper;

// The reopen switch, true when the verification messages are sent.
// False when new data block is created
std::map<std::string, bool> reop_sw;

// This is for scheduling the time for each group member to receive enough tcp packets.
std::map<std::string, bool> sche_sw;

// this stores the csma ip of the group members
std::map<int, std::string> Group;

// this contains the total size of every group member received new data block
std::map<std::string, uint32_t> N_DB;

// This contains the total time block received for each group member.
std::map<std::string, uint32_t> N_TB;

// TCP general ports
uint16_t dsrc_port = 50000;
uint16_t cam_port = 50001;
uint16_t radar_port = 50002;

// UDP ports
uint16_t bsm_port = 8080;
uint16_t vote_port = 9010;

// Each time block is 800 Bytes
uint16_t time_block_port = 5500; 

// TCP blockchain delivery ports
uint16_t data_chain = 9990;

// Group size need to be global
uint32_t gsize = 8;
// Different size of verification messages
uint32_t dsrcp_size = 0;
uint32_t camp_size = 0;
uint32_t radarp_size = 0;
// New data block size
uint32_t db_size = 0;

// veri time wait variable: this is the variable decide how long to wait for the verification message to cut off receiving
// default is 5 seconds
double veri_time_wait = 5.0;

/* In here, I am estimating the time of processing each type of verification messages instead of 
giving the precise estimation of what a real CAV can do. Could give a better estimation in
future if needed*/
double dsrc_proc = 0.001;
double CAM_proc = 0.02;
double radar_proc = 0.005;

// This is Time block processing time. The group member is going to compare all timeblock received and vote
double TB_proc = 0.1;

// Group leader variables
std::string gl_ip = "10.1.2.1";
Ptr<Node> gmnode = nullptr;

// Vote counter
uint32_t time_vote_counter = 0;
uint32_t data_vote_counter = 0;
// This interval decides the transfer rate of the data block. The smaller the bigger the data rate.
double transfer_interval = 0.1;

// Start time of the verification message go into the block
double BlockStartTime = 0;


std::string intToIp(uint32_t num) {
  return std::to_string((num >> 24) & 0xFF) + "." + std::to_string((num >> 16) & 0xFF) 
          + "." + std::to_string((num >> 8) & 0xFF) + "." + std::to_string(num & 0xFF);
}

// Include timestamps in created packets
Ptr<Packet> CreatePacketwithts(std::string msg, uint32_t packetSize) {
  std::string timestamp = msg;
  
  if (packetSize > msg.size()) {
        timestamp.resize(packetSize, 0); // Resize and fill with null bytes
  }
  return Create<Packet>((uint8_t *)timestamp.data(), timestamp.size());
}

// Include vote content in the packets
Ptr<Packet> CreatePacketwithvote(std::string msg, uint32_t packetSize) {
  std::string votecontent = msg;
  
  if (packetSize > msg.size()) {
        votecontent.resize(packetSize, 0); // Resize and fill with null bytes
  }
  return Create<Packet>((uint8_t *)votecontent.data(), votecontent.size());
}

void SendBsm(Ptr<Socket> socket, uint32_t bsmSize, Time interval) {
    socket->Send(CreatePacketwithts(std::to_string(Simulator::Now().GetSeconds()), bsmSize)); // Broadcast packet with timestamp
    NS_LOG_INFO("BSM sent at: " << Simulator::Now().GetSeconds());
    // Schedule the next BSM
    Simulator::Schedule(interval, &SendBsm, socket, bsmSize, interval);
}

void SendVote(Ptr<Node> senderNode, Ipv4Address serverAddress, uint16_t serverPort, std::string votemsg) {
    uint16_t votepacket_size = 80; // vote packet size
    Ptr<Socket> socket = Socket::CreateSocket(senderNode, UdpSocketFactory::GetTypeId());
  
    InetSocketAddress vote_receipt = InetSocketAddress(serverAddress, serverPort);
    socket->Bind();
    socket->Connect(vote_receipt);

    socket->Send(CreatePacketwithvote(votemsg, votepacket_size)); // Broadcast packet
    NS_LOG_INFO("Vote sent at: " << Simulator::Now().GetSeconds());
}

void StartTimeBlockCons(Ptr<Node> senderNode, std::string ip){
  uint16_t timeblockp_size = 800; 

  if (ip == gl_ip) {
    // compute total data block size
    db_size = dsrcp_size + camp_size + radarp_size;
  }
  
  for (int index = 0; index < static_cast<int>(Group.size()); ++index) {
     if (Group[index] != ip) {
       Ptr<Socket> socket = Socket::CreateSocket(senderNode, UdpSocketFactory::GetTypeId());
  
       InetSocketAddress tc_member = InetSocketAddress(Ipv4Address(Group[index].data()), time_block_port);
       socket->Bind();
       socket->Connect(tc_member);

       Ptr<Packet> packet = Create<Packet>(timeblockp_size); 
       socket->Send(packet); 
       
       NS_LOG_INFO(ip << " Time Block consensus sent to " << Group[index] << "at: " << Simulator::Now().GetSeconds());
     }
  }

}

// send the new datablock consensus to everynode, it is from gm to every group member.
void StartDataBlockCons(Ptr<Node> senderNode, uint32_t packetsz) {

  for (int index = 0; index < static_cast<int>(Group.size()); ++index) {
     if (Group[index] != gl_ip) {
       Ptr<Socket> tcpsocket = Socket::CreateSocket(senderNode, TcpSocketFactory::GetTypeId());
     
       InetSocketAddress dc_member = InetSocketAddress(Ipv4Address(Group[index].data()), data_chain);
       tcpsocket->Connect(dc_member);
       
       Ptr<Packet> packet = Create<Packet>(packetsz); 
       tcpsocket->Send(packet);
     }
  }
}


void TBreceive(Ptr<Socket> udpsocket){
    Ptr<Packet> packet;
    Address from;

    Ptr<Node> node = udpsocket->GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); 
    Ipv4InterfaceAddress iface = ipv4->GetAddress(2, 0);
    Ipv4Address ipAddr = iface.GetLocal();
    
    while ((packet = udpsocket->RecvFrom(from))) {
    	++N_TB[intToIp(ipAddr.Get())];
    	// Only send vote after all the TB are received
    	if (N_TB[intToIp(ipAddr.Get())] == (gsize-1)) {
    	  Simulator::Schedule(Seconds(TB_proc*(gsize-1)), &SendVote, node, Ipv4Address(gl_ip.data()), vote_port, std::string("Timeblock"));
    	}
    }
}

void DBreceive(Ptr<Socket> tcpSocket){
    Ptr<Packet> packet;
    Address from;
    
    Ptr<Node> node = tcpSocket->GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); 
    Ipv4InterfaceAddress iface = ipv4->GetAddress(2, 0);
    Ipv4Address ipAddr = iface.GetLocal();
    
    while ((packet = tcpSocket->RecvFrom(from))) {
      N_DB[intToIp(ipAddr.Get())] += packet->GetSize();
    }
    
    if (N_DB[intToIp(ipAddr.Get())] == db_size) {
      double Total_proc = dsrc_proc * (dsrcp_size / 1200) + CAM_proc * (camp_size / 4000) + radar_proc * (radarp_size / 1600);
      Simulator::Schedule(Seconds(Total_proc), &SendVote, node, Ipv4Address(gl_ip.data()), vote_port, std::string("Datablock"));
    }
    
}

void TcpDSRC(Ptr<Socket> tcpSocket) {
    Ptr<Packet> packet;
    Address from;
    
    Ptr<Node> node = tcpSocket->GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); 
    Ipv4InterfaceAddress iface = ipv4->GetAddress(2, 0);
    Ipv4Address ipAddr = iface.GetLocal();
    
    
    while ((packet = tcpSocket->RecvFrom(from))) {
      InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
      if (intToIp(ipAddr.Get()) == gl_ip) { // add size of received packets
        dsrcp_size += packet->GetSize(); // Should be GM only!
      }
      NS_LOG_INFO("DSRC packet received from: " << address.GetIpv4() << " at " << Simulator::Now().GetSeconds() << " with size: " << packet->GetSize());
    }
    
    if (sche_sw[intToIp(ipAddr.Get())] == false) {
      // schedule for the timeblock creation and new block entry building.
      Simulator::Schedule(Seconds(veri_time_wait), &StartTimeBlockCons, node, intToIp(ipAddr.Get()));
      sche_sw[intToIp(ipAddr.Get())] = true;
    }
    
}

void TcpCAM(Ptr<Socket> tcpSocket) {
    Ptr<Packet> packet;
    Address from;
    
    Ptr<Node> node = tcpSocket->GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); 
    Ipv4InterfaceAddress iface = ipv4->GetAddress(2, 0);
    Ipv4Address ipAddr = iface.GetLocal();
    
    while ((packet = tcpSocket->RecvFrom(from))) {
      InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
      if (intToIp(ipAddr.Get()) == gl_ip) {
        camp_size += packet->GetSize(); 
      }
      NS_LOG_INFO("CAM packet received from: " << address.GetIpv4() << " at " << Simulator::Now().GetSeconds() << " with size: " << packet->GetSize());
    } 
}

void TcpRadar(Ptr<Socket> tcpSocket) {
    Ptr<Packet> packet;
    Address from;
    
    Ptr<Node> node = tcpSocket->GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); 
    Ipv4InterfaceAddress iface = ipv4->GetAddress(2, 0);
    Ipv4Address ipAddr = iface.GetLocal();
    
    while ((packet = tcpSocket->RecvFrom(from))) {
      InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
      if (intToIp(ipAddr.Get()) == gl_ip) {
        radarp_size += packet->GetSize();
      }
      NS_LOG_INFO("Radar packet received from: " << address.GetIpv4() << " at " << Simulator::Now().GetSeconds() << " with size: " << packet->GetSize());
    } 
}


void AcceptConnectionDSRC(Ptr<Socket> socket, const Address& from) {
    NS_LOG_INFO("DSRC Accepting connection from " << InetSocketAddress::ConvertFrom(from).GetIpv4());
    socket->SetRecvCallback(MakeCallback(&TcpDSRC));
}

void AcceptConnectionCam(Ptr<Socket> socket, const Address& from) {
    NS_LOG_INFO("CAM Accepting connection from " << InetSocketAddress::ConvertFrom(from).GetIpv4());
    socket->SetRecvCallback(MakeCallback(&TcpCAM));
}

void AcceptConnectionRadar(Ptr<Socket> socket, const Address& from) {
    NS_LOG_INFO("RADAR Accepting connection from " << InetSocketAddress::ConvertFrom(from).GetIpv4());
    socket->SetRecvCallback(MakeCallback(&TcpRadar));
}

void AcceptConnectionDB(Ptr<Socket> socket, const Address& from) {
    NS_LOG_INFO("DB Accepting connection from " << InetSocketAddress::ConvertFrom(from).GetIpv4());
    socket->SetRecvCallback(MakeCallback(&DBreceive));
}

void SendDSRCVeriPacket(Ptr<Node> senderNode, Ipv4Address serverAddress, uint16_t serverPort) {
    uint16_t dsrc_size = 1200;
    Ptr<Socket> tcpSocket = Socket::CreateSocket(senderNode, TcpSocketFactory::GetTypeId());

    // Define the destination address and port
    InetSocketAddress tcpDestination(serverAddress, serverPort);
    tcpSocket->Connect(tcpDestination);

    // Send a DSRC Verification Packet
    Ptr<Packet> tcpPacket = Create<Packet>(dsrc_size); // Using ECDSA, the packet is assume to be 1200 bytes
    tcpSocket->Send(tcpPacket);

    NS_LOG_INFO("DSRC packet sent at: " << Simulator::Now().GetSeconds() << " to " << serverAddress);
}

void SendCamVeriPacket(Ptr<Node> senderNode, Ipv4Address serverAddress, uint16_t serverPort) {
    uint16_t cam_size = 4000;
    Ptr<Socket> tcpSocket = Socket::CreateSocket(senderNode, TcpSocketFactory::GetTypeId());

    // Define the destination address and port
    InetSocketAddress tcpDestination(serverAddress, serverPort);
    tcpSocket->Connect(tcpDestination);

    // Send a Camera Verification Packet
    // For Camera packet, we assume the size is 4kb for one message. Although the real 
    // message must be much bigger depends on the verification cases.
    Ptr<Packet> tcpPacket = Create<Packet>(cam_size); 
    tcpSocket->Send(tcpPacket);

    NS_LOG_INFO("CAM packet sent at: " << Simulator::Now().GetSeconds() << " to " << serverAddress);
}

void SendRadarVeriPacket(Ptr<Node> senderNode, Ipv4Address serverAddress, uint16_t serverPort) {
    uint16_t radar_size = 1600;
    Ptr<Socket> tcpSocket = Socket::CreateSocket(senderNode, TcpSocketFactory::GetTypeId());

    // Define the destination address and port
    InetSocketAddress tcpDestination(serverAddress, serverPort);
    tcpSocket->Connect(tcpDestination);

    // Send a Radar Verification Packet
    // The size of radar verification message is estimated to be at least 1600 bytes
    Ptr<Packet> tcpPacket = Create<Packet>(radar_size); 
    tcpSocket->Send(tcpPacket);

    NS_LOG_INFO("Radar packet sent at: " << Simulator::Now().GetSeconds() << " to " << serverAddress);
}

// we need a random number to decide if the camera radar messages are generated or not
int randomnumbergenerate(double input) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 9);
  return round(input * dis(gen));
}


void ReceiveBsm(Ptr<Socket> udpsocket) {
    Ptr<Packet> packet;
    Address from;
    
    Ptr<Node> node = udpsocket->GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); // Get the Ipv4 object
    Ipv4InterfaceAddress iface = ipv4->GetAddress(1, 0); // Interface 1, primary address (0)
    Ipv4Address ipAddr = iface.GetLocal(); // Get the local IP address
   
    // initialization for new ip addresses
    if (BSMkeeper.count(intToIp(ipAddr.Get())) == 0) {
       BSMkeeper[intToIp(ipAddr.Get())] = 0;
    }
    
    // initialize the reopen switch
    if (reop_sw.count(intToIp(ipAddr.Get())) == 0) {
       reop_sw[intToIp(ipAddr.Get())] = false;
    }
   
    while ((packet = udpsocket->RecvFrom(from))) {
        InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
        NS_LOG_INFO("BSM received at: " << Simulator::Now().GetSeconds() << " from " << address.GetIpv4());

        if (std::fmod(Simulator::Now().GetSeconds(), 1.0) > 0.96) {
          if (reop_sw[intToIp(ipAddr.Get())] == false) { // Send only once for BSM verification message until the new block is ready
              for (uint32_t index = 0; index < gsize; ++index) {
                // compute the proportion of the other 2 types of verification messages
                int dsrc_veri = BSMkeeper[intToIp(ipAddr.Get())];
                int Cam_veri = randomnumbergenerate(BSMkeeper[intToIp(ipAddr.Get())] * 0.1); 
                int radar_veri = randomnumbergenerate(BSMkeeper[intToIp(ipAddr.Get())] * 0.05);
                
                for (int iter = 0; iter < dsrc_veri; ++iter) {
                  Simulator::Schedule(Seconds(0.05 + iter*0.001), &SendDSRCVeriPacket, node, Ipv4Address(Group[index].data()), dsrc_port);
                }
            
                for (int iter = 0; iter < Cam_veri; ++iter) {
            	  Simulator::Schedule(Seconds(0.1 + iter*0.001), &SendCamVeriPacket, node, Ipv4Address(Group[index].data()), cam_port);
                }
            
                for (int iter = 0; iter < radar_veri; ++iter) {
            	  Simulator::Schedule(Seconds(0.2 + iter*0.001), &SendRadarVeriPacket, node, Ipv4Address(Group[index].data()), radar_port);
                }
	      }
	      
	      reop_sw[intToIp(ipAddr.Get())] = true;
	      // initialize transaction start time
              BlockStartTime = Simulator::Now().GetSeconds();
            }
            BSMkeeper[intToIp(ipAddr.Get())] = 0;
        } else if (std::fmod(Simulator::Now().GetSeconds(), 1.0) < 0.1) {
          BSMkeeper[intToIp(ipAddr.Get())]++;
        }
        
    }
}

// this is the function to reinitialize the new block
void reinitialization() {
  dsrcp_size = 0;
  camp_size = 0;
  radarp_size = 0;
  db_size = 0;
  
  time_vote_counter = 0;
  data_vote_counter = 0;
  
  for (const auto& pair : reop_sw) {
    reop_sw[pair.first] = false;
  }
  
  for (const auto& pair : N_DB) {
    N_DB[pair.first] = 0;
    N_TB[pair.first] = 0;
    
    sche_sw[pair.first] = false;
  }

}

void receiveVote(Ptr<Socket> udpsocket) {
  Ptr<Packet> packet;
  Address from;

  while ((packet = udpsocket->RecvFrom(from))) {
    uint32_t packetSize = packet->GetSize();
    uint8_t* buffer = new uint8_t[packetSize];
    packet->CopyData(buffer, packetSize);
    buffer[packetSize] = '\0'; 

    std::string blocktype(reinterpret_cast<char*>(buffer));
    if ("Timeblock" == blocktype) {
        time_vote_counter++;
    } else if ("Datablock" == blocktype) {
        data_vote_counter++;
    }

    delete[] buffer;
  }
  
  if (time_vote_counter == ceil((gsize - 1) * (2.0/3.0))) {
     // divide new block into few packets to reduce overhead and buffer size.
     int iter = 1;
     uint32_t total = 0;
     uint32_t div_psize = 100000;

     while((db_size - (div_psize * iter)) > div_psize) {
       total += div_psize;
       Simulator::Schedule(Seconds(0.25 + iter * transfer_interval), &StartDataBlockCons, gmnode, div_psize);
       iter++;
     }
     
     Simulator::Schedule(Seconds(0.25 + (iter+1) * transfer_interval), &StartDataBlockCons, gmnode, div_psize);
     total += div_psize;
     Simulator::Schedule(Seconds(0.25 + (iter+2) * transfer_interval), &StartDataBlockCons, gmnode, (db_size - total));
  } 
  
  if (data_vote_counter == ceil((gsize - 1) * (2.0/3.0))) {
    NS_LOG_INFO("New Data block creation complete! Start Time: "<< BlockStartTime << " End Time: " << Simulator::Now().GetSeconds() << " Size: " << db_size << " Bytes");
    NS_LOG_INFO("Transactions: DSRC: " << (dsrcp_size / 1200) << " CAM: " << (camp_size / 4000) << " Radar: " << (radarp_size / 1600));
    std::cout << "New Data block creation complete! Start Time: "<< BlockStartTime << " End Time: " << Simulator::Now().GetSeconds() << " Size: " << db_size << " Bytes" << std::endl;
    std::cout << "Transactions: DSRC: " << (dsrcp_size / 1200) << " CAM: " << (camp_size / 4000) << " Radar: " << (radarp_size / 1600) << std::endl;
    reinitialization();
  }
  
  // when receive vote after new block creation, the vote is cleared
  if (db_size == 0) {
    reinitialization();
  }
}

int main(int argc, char *argv[]){
  LogComponentEnable("BSMBDEXP", LOG_LEVEL_INFO);
  
  uint32_t nNodes = 10;
  uint32_t bsm_size = 100;
  double bsmInterval = 0.1;
  double Simu_time = 30.0;
  
  
  CommandLine cmd;
  cmd.AddValue("nNodes", "Number of nodes", nNodes);
  cmd.AddValue("bsmInterval", "Interval between BSMs", bsmInterval);
  cmd.AddValue("GroupSize", "Consensus group size", gsize);
  cmd.AddValue("TimeWait", "Waiting time for receiving verification msgs", veri_time_wait);
  cmd.AddValue("TI", "Data block transfer rate interval variable", transfer_interval);
  cmd.AddValue("ST", "The total simulation time", Simu_time);
  cmd.Parse(argc, argv);

  NodeContainer nodes;
  nodes.Create(nNodes);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper phy;
  phy.Set("TxPowerStart", DoubleValue(20.0));
  phy.Set("TxPowerEnd", DoubleValue(20.0));
  phy.SetChannel(channel.Create());

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  WifiHelper wifiHelper;
  wifiHelper.SetStandard(WIFI_STANDARD_80211p);

  NetDeviceContainer devices = wifiHelper.Install(phy, mac, nodes);

  phy.EnablePcap("./BSMlogs/broadcast-trace", devices);

  // C-V2N network architecture, we are using CSMA to simulate
  CsmaHelper cv2n;
  cv2n.SetChannelAttribute("DataRate", StringValue("10Gbps")); // The bandwidth are shared in CSMA
  cv2n.SetChannelAttribute("Delay", TimeValue(NanoSeconds(5000)));
  cv2n.SetDeviceAttribute("Mtu", UintegerValue(1500));
  NetDeviceContainer cv2nDevices = cv2n.Install(nodes);

  cv2n.EnablePcapAll("./BSMlogs/csma-trace");

  InternetStackHelper stack;
  stack.Install(nodes);

  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign(devices);
  
  Ipv4AddressHelper cv2nAdd;
  cv2nAdd.SetBase("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer cv2ninterfaces = cv2nAdd.Assign(cv2nDevices);

  // assign the ips of the group members & receiving switch
  for (uint32_t index = 0; index < gsize; ++index) {
       Ptr<Node> node = nodes.Get(index);
       Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); 
       Ipv4Address ipAddr = ipv4->GetAddress(2, 0).GetLocal();
       Group[index] = intToIp(ipAddr.Get());
       // initialize waiting switch; in DSRC verification.
       sche_sw[intToIp(ipAddr.Get())] = false;
       // initialize new DB size to 0
       N_DB[intToIp(ipAddr.Get())] = 0;
       // initialize TB received to 0
       N_TB[intToIp(ipAddr.Get())] = 0;
  }

  // initialization of group master node
  gmnode = nodes.Get(0);


  MobilityHelper mobility;

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  for(uint32_t i = 0; i < nodes.GetN(); ++i){
    uint16_t y_counter = 0;
  
    // switch to next line if x axis contains more than 10 nodes
    if(i > 0 && i%10 == 0){
      y_counter++;
    }
  
    positionAlloc->Add(Vector(20.0*(i%10), 20.0*y_counter, 0.0));
  }
  
  
  mobility.SetPositionAllocator(positionAlloc);
  //mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");  
  mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                          "Bounds", RectangleValue(Rectangle(0.0, 300.0, 0.0, 300.0)),
                          "Speed", StringValue("ns3::ConstantRandomVariable[Constant=10.0]"));
  mobility.Install(nodes);


  for(uint32_t i = 0; i < nodes.GetN(); ++i){
    Ptr<Node> node = nodes.Get(i);
    Ptr<Socket> socket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
  
    InetSocketAddress bdAddress = InetSocketAddress(Ipv4Address("10.1.1.255"), bsm_port);
    socket->SetAllowBroadcast(true);
    socket->Bind();
    socket->Connect(bdAddress);
    
    Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable>();
    
    Time randomStart = Seconds(uniformRandomVariable->GetValue(0.0, bsmInterval));
    
    //std::cout << randomStart << std::endl;
    Simulator::Schedule(randomStart, &SendBsm, socket, bsm_size, Seconds(bsmInterval));
  
  
    Ptr<Socket> recvSocket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), bsm_port); 
    recvSocket->Bind(local);
    recvSocket->SetRecvCallback(MakeCallback(&ReceiveBsm)); // this callback is triggered by 10.1.1.255
    
    // DSRC TCP listener setting
    Ptr<Socket> tcpSocket = Socket::CreateSocket(node, TcpSocketFactory::GetTypeId());
    InetSocketAddress bindAddress = InetSocketAddress(Ipv4Address::GetAny(), dsrc_port);
    tcpSocket->Bind(bindAddress);
    tcpSocket->Listen(); 
    tcpSocket->SetAcceptCallback(
        MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
        MakeCallback(&AcceptConnectionDSRC));
        
    // CAM listener setting
    Ptr<Socket> camSocket = Socket::CreateSocket(node, TcpSocketFactory::GetTypeId());
    InetSocketAddress camAddress = InetSocketAddress(Ipv4Address::GetAny(), cam_port);
    camSocket->Bind(camAddress);
    camSocket->Listen(); 
    camSocket->SetAcceptCallback(
        MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
        MakeCallback(&AcceptConnectionCam));
    
    // Radar listener setting
    Ptr<Socket> radarSocket = Socket::CreateSocket(node, TcpSocketFactory::GetTypeId());
    InetSocketAddress radarAddress = InetSocketAddress(Ipv4Address::GetAny(), radar_port);
    radarSocket->Bind(radarAddress);
    radarSocket->Listen(); 
    radarSocket->SetAcceptCallback(
        MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
        MakeCallback(&AcceptConnectionRadar));
        
    // UDP vote setting
    Ptr<Socket> voteSocket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
    InetSocketAddress voteAdd = InetSocketAddress(Ipv4Address::GetAny(), vote_port); 
    voteSocket->Bind(voteAdd);
    voteSocket->SetRecvCallback(MakeCallback(&receiveVote)); 
    
    // time block consensus receiver
    Ptr<Socket> tcSocket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
    InetSocketAddress tcAdd = InetSocketAddress(Ipv4Address::GetAny(), time_block_port); 
    tcSocket->Bind(tcAdd);
    tcSocket->SetRecvCallback(MakeCallback(&TBreceive)); 
    
    // data block consensus setting
    Ptr<Socket> dbSocket = Socket::CreateSocket(node, TcpSocketFactory::GetTypeId());
    InetSocketAddress dbAddress = InetSocketAddress(Ipv4Address::GetAny(), data_chain);
    dbSocket->Bind(dbAddress);
    dbSocket->Listen(); 
    dbSocket->SetAcceptCallback(
        MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
        MakeCallback(&AcceptConnectionDB));
    
  }
  
  // Setup routing table
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  
  Simulator::Stop(Seconds(Simu_time));
  Simulator::Run();
  Simulator::Destroy();
  
  NS_LOG_INFO("Simulation End");
  return 0;

}
