#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/athstats-helper.h"
#include "ns3/energy-module.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <cstdlib>

using namespace ns3;

#if 0
static bool g_verbose = true;

static void SetPosition (Ptr<Node> node, Vector position){
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
	mobility->SetPosition (position);
}

static Vector GetPosition (Ptr<Node> node){
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
	return mobility->GetPosition ();
}

static void AdvancePosition (Ptr<Node> node){
	Vector pos = GetPosition (node);
	pos.x += 5.0;
	if(pos.x >= 210.0){
		return;
	}
	SetPosition (node, pos);

	if(g_verbose){
		//std::cout << "x="<<pos.x << std::endl;
	}
	Simulator::Schedule (Seconds (1.0), &AdvancePosition, node);
}
#endif

static void PrintPosition(NodeContainer c)
{
	for (uint32_t i=0; i<c.GetN(); i++)
	{
		Ptr<MobilityModel> m = c.Get(i)->GetObject<MobilityModel> ();
		NS_LOG_UNCOND("Node" << i << " " << m->GetPosition().x << " " << m->GetPosition().y);
	}
}


static void RxDrop(Ptr<const Packet> p ){
	NS_LOG_UNCOND("RxDrop at" << Simulator::Now().GetSeconds());
}

static void StartTl(Ptr<ProposedWifiMac> src_mac, double src_rate, uint8_t alg_rate)
{
	src_mac->Start(src_rate, alg_rate);
}

void
RemainingEnergy (double oldValue, double remainingEnergy)
{
	NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
			<< "s Current remaining energy = " << remainingEnergy << "J");
}

/// Trace function for total energy consumption at node.
	void
TotalEnergy (double oldValue, double totalEnergy)
{
	NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
			<< "s Total energy consumed by radio = " << totalEnergy << "J");
}


int main(int argc, char *argv[]){

	double topology_learning_time = 300;
	double data_transmission_time = 1000;
	uint16_t alg_rate = 9;
	double src_rate = 1;
	int seed = 1;
	uint16_t nodeNum = 2;
	uint32_t packetNum = 100;//4070
	int box = 15;
	uint16_t alg_type = 2;


	CommandLine cmd;
	cmd.AddValue ("AlgorithmRate", "Algorithm Rate", alg_rate); // 0 ~ 8 : single-rate, 9 : multi-rate
	cmd.AddValue ("Seed", "Seed of simulation", seed);
	cmd.AddValue ("PacketNum", "The number of Packets", packetNum); 
	cmd.AddValue ("NodeNum", "The number of nodes", nodeNum); 
	cmd.AddValue ("Box", "length of box", box); 
	cmd.AddValue ("AlgType", "type of algorithm", alg_type); 

	cmd.Parse (argc, argv);

	SeedManager::SetRun(seed);
	WifiHelper wifi = WifiHelper::Default ();
	wifi.SetStandard(WIFI_PHY_STANDARD_80211a);

	NodeContainer nodes;
	NodeContainer non_src;
	nodes.Create(nodeNum);

	for (uint32_t i=1; i < nodes.GetN(); i++)
	{
		non_src.Add(nodes.Get(i));
	}

	NetDeviceContainer nodeDevs;

	QosWifiMacHelper wifiMac = QosWifiMacHelper::Default ();
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
	//YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

	YansWifiChannelHelper wifiChannel ;
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	// The below FixedRssLossModel will cause the rss to be fixed regardless
	// of the distance between the two stations, and the transmit power
	wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(3.5));
	wifiChannel.AddPropagationLoss ("ns3::JakesPropagationLossModel");
	//double dopplerVel=0;
	//Config::SetDefault("ns3::JakesProcess::DopplerFrequencyHz", DoubleValue(dopplerVel*5/3));

	wifiPhy.SetChannel (wifiChannel.Create ());

	Ssid ssid = Ssid ("gjlee-adhoc");
	//wifi.SetRemoteStationManager ("ns3::ArfRateWifiManager");
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
			"DataMode", StringValue ("OfdmRate6Mbps"));

	wifiMac.SetType ("ns3::ProposedWifiMac",
			"Ssid", SsidValue(ssid), "Status", UintegerValue(1), "AlgType", UintegerValue(alg_type));
	nodeDevs = wifi.Install (wifiPhy, wifiMac, nodes.Get(0));

	wifiMac.SetType ("ns3::ProposedWifiMac",
			"Ssid", SsidValue(ssid), "Status", UintegerValue(0));

	nodeDevs.Add(wifi.Install (wifiPhy, wifiMac, non_src));

	std::cout << "size of nodes: " << nodes.GetN() << std::endl;
	std::cout << "size of nodeDevs: " << nodeDevs.GetN() << std::endl;

	MobilityHelper mobility;

	Ptr<ListPositionAllocator> src_positionAlloc = CreateObject<ListPositionAllocator> ();
	//Ptr<UniformRandomVariable> Rand = CreateObject<UniformRandomVariable> ();
	src_positionAlloc->Add (Vector (0, 0.0 ,0.0));

	mobility.SetPositionAllocator (src_positionAlloc);
	//mobility.SetPositionAllocator (taPositionAlloc);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (nodes.Get(0));

	Ptr<UniformRandomVariable> xVar = CreateObject<UniformRandomVariable> ();

	//xVar->SetAttribute("Min", DoubleValue(box));
	xVar->SetAttribute("Min", DoubleValue(-1*box));
	xVar->SetAttribute("Max", DoubleValue(box));

	Ptr<UniformRandomVariable> yVar = CreateObject<UniformRandomVariable> ();
	yVar->SetAttribute("Min", DoubleValue(-1*box));
	yVar->SetAttribute("Max", DoubleValue(box));

	//yVar->SetAttribute("Min", DoubleValue(0));
	//yVar->SetAttribute("Max", DoubleValue(0));
	
	Ptr<RandomRectanglePositionAllocator> nonsrc_pos = CreateObject<RandomRectanglePositionAllocator> ();
	nonsrc_pos->SetX(xVar);
	nonsrc_pos->SetY(yVar);

	mobility.SetPositionAllocator (nonsrc_pos);
	mobility.Install(non_src);

	PrintPosition(nodes);

	Ptr<UniformRandomVariable> zVar = CreateObject<UniformRandomVariable> ();
	zVar->SetAttribute("Min", DoubleValue(50));
	zVar->SetAttribute("Max", DoubleValue(200));

	BasicEnergySourceHelper basicSourceHelper;
	EnergySourceContainer sources;
	for (uint32_t i=0; i<nodes.GetN(); i++)
	{
		basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (zVar->GetValue()));
		sources.Add(basicSourceHelper.Install (nodes.Get(i)));
	}
	// configure energy source
	// install source
	/* device energy model */
	WifiRadioEnergyModelHelper radioEnergyHelper;
	radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.3));
	radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.06));
	radioEnergyHelper.Set ("IdleCurrentA", DoubleValue (0));
	radioEnergyHelper.Set ("CcaBusyCurrentA", DoubleValue (0));
	DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (nodeDevs, sources);

	/*
		 if(nodeNum == 2) positionAlloc->Add (Vector (150, 0, 0.0));
		 else if(nodeNum == 3){
		 positionAlloc->Add (Vector (50, 0, 0.0));
		 positionAlloc->Add (Vector (100, 0, 0.0));
		 }
		 else if(nodeNum == 5){
		 positionAlloc->Add (Vector (-50, 0, 0.0));
		 positionAlloc->Add (Vector (-100, 0, 0.0));
		 positionAlloc->Add (Vector (50, 0, 0.0));
		 positionAlloc->Add (Vector (100, 0, 0.0));
		 }
		 else if(nodeNum == 10){
		 positionAlloc->Add (Vector (-38, 80, 0.0));
		 positionAlloc->Add (Vector (59, 99, 0.0));
		 positionAlloc->Add (Vector (5, -69, 0.0));
		 positionAlloc->Add (Vector (54, -78, 0.0));
		 positionAlloc->Add (Vector (34, -30, 0.0));
		 positionAlloc->Add (Vector (42, -9, 0.0));
		 positionAlloc->Add (Vector (-53, 18, 0.0));
		 positionAlloc->Add (Vector (-52, 67, 0.0));
		 positionAlloc->Add (Vector (-46, -17, 0.0));
		 positionAlloc->Add (Vector (-53, -90, 0.0));
		 }

		 else if(nodeNum == 41){
		 positionAlloc->Add (Vector (-99, -191, 0.0));
		 positionAlloc->Add (Vector (-157, -110, 0.0));
		 positionAlloc->Add (Vector (-60, -114, 0.0));
		 positionAlloc->Add (Vector (72, -170, 0.0));
		 positionAlloc->Add (Vector (-134, -135, 0.0));
		 positionAlloc->Add (Vector (66, 17, 0.0));
		 positionAlloc->Add (Vector (-11, 152, 0.0));
		 positionAlloc->Add (Vector (-126, -9, 0.0));
		 positionAlloc->Add (Vector (69, 134, 0.0));
		 positionAlloc->Add (Vector (-78, 74, 0.0));
		 positionAlloc->Add (Vector (173, 9, 0.0));
		 positionAlloc->Add (Vector (-57, 100, 0.0));
		 positionAlloc->Add (Vector (194, -174, 0.0));
		 positionAlloc->Add (Vector (-45, -129, 0.0));
		 positionAlloc->Add (Vector (52, 29, 0.0));
		 positionAlloc->Add (Vector (198, 154, 0.0));
		 positionAlloc->Add (Vector (-10, -159, 0.0));
		 positionAlloc->Add (Vector (196, 130, 0.0));
		 positionAlloc->Add (Vector (-120, 69, 0.0));
		 positionAlloc->Add (Vector (160, -54, 0.0));
		 positionAlloc->Add (Vector (86, -22, 0.0));
		 positionAlloc->Add (Vector (163, -125, 0.0));
		 positionAlloc->Add (Vector (-117, 189, 0.0));
		 positionAlloc->Add (Vector (19, 104, 0.0));
		 positionAlloc->Add (Vector (76, 93, 0.0));
		 positionAlloc->Add (Vector (-22, 49, 0.0));
		 positionAlloc->Add (Vector (-97, 73, 0.0));
		 positionAlloc->Add (Vector (-99, -151, 0.0));
		 positionAlloc->Add (Vector (100, 56, 0.0));
		 positionAlloc->Add (Vector (-128, -96, 0.0));
		 positionAlloc->Add (Vector (-162, -130, 0.0));
		 positionAlloc->Add (Vector (-142, 28, 0.0));
		 positionAlloc->Add (Vector (-137, -193, 0.0));
		 positionAlloc->Add (Vector (-41, -57, 0.0));
		 positionAlloc->Add (Vector (28, -129, 0.0));
		 positionAlloc->Add (Vector (90, -86, 0.0));
		 positionAlloc->Add (Vector (50, 5, 0.0));
		 positionAlloc->Add (Vector (-58, 85, 0.0));
		 positionAlloc->Add (Vector (-53, 161, 0.0));
		 positionAlloc->Add (Vector (-11, -177, 0.0));
		 }
		 */

	/*
		 srand(time(NULL));
		 double pos[100][2];
		 for(uint16_t i = 0; i < 100; i++){
		 for(uint16_t j = 0; j < 2; j++){
		 pos[i][j] = rand()%400 - 200;
		 }
		 }
		 std::cout << "Position of nodes" << std::endl;
		 for(uint16_t i = 0; i < nodeNum-1; i++)
		 std::cout << "positionAlloc->Add (Vector (" << pos[i][0] << ", " << pos[i][1] << ", 0.0));" << std::endl;

		 std::cout << "--------------------------MATLAB------------------------------" << std::endl;
		 std::cout << "clear all;" << std::endl;
		 std::cout << "close all;" << std::endl;
		 std::cout << "clc;" << std::endl;
		 std::cout << "\naxis([-205 205 -205 205]);" << std::endl;
		 std::cout << "\nline(0,0, 'Color', 'k', 'Marker, 's');\n" << std::endl;

		 for(uint16_t i = 0; i < nodeNum-1; i++)
		 std::cout << "line(" << pos[i][0] << ", " << pos[i][1] << ", 'Color', 'r', 'Marker', 's');" << std::endl;
		 std::cout << "\ntext(0+4,0+1, 'source');\n" << std::endl;
		 for(uint16_t i = 0; i < nodeNum-1; i++)
		 std::cout << "text(" << pos[i][0] << "+3, " << pos[i][1] << ", '" <<  i+2 << "');" << std::endl;
		 std::cout << "--------------------------MATLAB------------------------------" << std::endl;

		 for(uint16_t i = 0; i < nodeNum-1; i++)
		 positionAlloc->Add (Vector (pos[i][0], pos[i][1], 0.0));
		 */
	InternetStackHelper stack;
	stack.Install(nodes);

	Ipv4AddressHelper address;
	Ipv4InterfaceContainer interface_node;

	address.SetBase("10.20.0.0", "255.255.255.0");
	interface_node = address.Assign (nodeDevs);

	Ipv4Address multicastSource ("10.1.1.1");
	Ipv4Address multicastGroup ("224.100.100.1");

	Ipv4StaticRoutingHelper multicast;

	Ptr<Node> sender = nodes.Get(0);
	Ptr<NetDevice> senderIf = nodeDevs.Get(0);

	multicast.SetDefaultMulticastRoute(sender, senderIf);

	//std::cout <<"node1 address : " << interface_node.GetAddress(0) << '\n';
	//std::cout <<"node2 address : " << interface_node.GetAddress(1) << '\n';

	uint16_t mcast_port = 5004;
	PacketSinkHelper packetsinkHelper ("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), mcast_port));

	OnOffHelper clientHelper ("ns3::UdpSocketFactory", Address(InetSocketAddress(multicastGroup, mcast_port)));
	clientHelper.SetAttribute("OnTime",StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	clientHelper.SetAttribute("OffTime",StringValue("ns3::ConstantRandomVariable[Constant=0]"));
	clientHelper.SetAttribute("DataRate", DataRateValue(src_rate*1000000));
	clientHelper.SetAttribute("MaxBytes", UintegerValue(1316*packetNum));
	clientHelper.SetAttribute("PacketSize",UintegerValue(1316));
	clientHelper.SetAttribute("Tid",UintegerValue(5));

	ApplicationContainer video_multicast;
	ApplicationContainer sinkApps;

	video_multicast.Add(clientHelper.Install(nodes.Get(0)));

	for(uint16_t i=1; i < nodes.GetN(); i++)
		sinkApps.Add (packetsinkHelper.Install (nodes.Get(i)));

	sinkApps.Start (Seconds(0));
	sinkApps.Stop (Seconds(topology_learning_time+data_transmission_time+1));

	video_multicast.Start (Seconds(topology_learning_time));
	video_multicast.Stop (Seconds(topology_learning_time+data_transmission_time));

	Simulator::Stop (Seconds(topology_learning_time+data_transmission_time+10));

	Ptr<WifiNetDevice> src_dev = nodeDevs.Get(0)->GetObject<WifiNetDevice> ();
	Ptr<ProposedWifiMac> src_mac = src_dev->GetMac()->GetObject<ProposedWifiMac> ();
	//src_mac->SetBasicModes();
	//src_mac->DoSetRelay(12, 6);

	//Ptr<ProposedWifiMac> relay_mac;
	//relay_mac = nodeDevs.Get(1)->GetObject<WifiNetDevice> ()->GetMac()->GetObject<ProposedWifiMac> ();
	//relay_mac->SetBasicModes();
	//relay_mac->DoSetRelay(10, 4);
	//relay_mac->DoSetRelay(5, 1);

	Simulator::Schedule (Seconds (1.0), &StartTl, src_mac, src_rate, (uint8_t)alg_rate);
	//    wifiPhy.EnablePcap ("proposed-alg-run", nodeDevs.Get (8));
    //wifiPhy.EnablePcap ("proposed-alg-run", nodeDevs.Get (11));

	Simulator::Run ();

	std::ofstream fout2;
	std::ostringstream out_filename2;
	out_filename2 << "ResultNew/ee_per_seed" << seed << "_nodes" << nodeNum << "_box" << box << ".txt";
	fout2.open(out_filename2.str().c_str(), std::ostream::out);
	if(!fout2.good())
		NS_LOG_UNCOND("File open failed");

	int success_node=0;
	Ptr<OnOffApplication> onoff = video_multicast.Get(0)->GetObject<OnOffApplication> ();
	NS_LOG_UNCOND("Source node sent: " << 10*(uint32_t)(onoff->GetTotalTx()/13160) << " Time : " << Simulator::Now ().GetSeconds ());
	NS_LOG_UNCOND("===================================");
	for (uint16_t i = 1; i < nodes.GetN(); i++){
		Ptr<PacketSink> sink = nodes.Get(i)->GetApplication(0)->GetObject<PacketSink>();
		Ptr<ProposedWifiMac> mac = nodes.Get(i)->GetDevice(0)->GetObject<WifiNetDevice>()->GetMac()->GetObject<ProposedWifiMac>();
		NS_LOG_UNCOND("Node\t" << i+1 << "\t" << sink->GetTotalRx()/1316 << "\t" << (double) sink->GetTotalRx()/1316/(10*(onoff->GetTotalTx()/13160)) << "\t" << (mac->GetStatus() == 2 ? "Relay\t" : "NoRelay\t"));
		
		fout2  << i+1 << "\t" << sink->GetTotalRx()/1316 << "\t" << (double) sink->GetTotalRx()/1316/(10*(onoff->GetTotalTx()/13160)) << "\t" << (mac->GetStatus() == 2 ? "Relay\t" : "NoRelay\t") << std::endl;
		if ((double)sink->GetTotalRx()/1316/(10*(onoff->GetTotalTx()/13160)) >= 0.995)
		{
				success_node++;
		}

		//NS_LOG_UNCOND("Node " << i+2 << " received: " << sink->GetTotalRx()/1316);
	}
	/*
	for (uint32_t i=0; i<sinkApps.GetN(); i++){
		Ptr<PacketSink> sink = sinkApps.Get(i)->GetObject<PacketSink> ();
		NS_LOG_UNCOND("Node\t" << i+2 << "\t" << sink->GetTotalRx()/1316 << "\t" << (double) sink->GetTotalRx()/1316/(10*(onoff->GetTotalTx()/13160)));
		fout2 << "Node\t" << i+2 << "\t" << sink->GetTotalRx()/1316 << "\t" << (double) sink->GetTotalRx()/1316/(10*(onoff->GetTotalTx()/13160)) <<std::endl;
		//NS_LOG_UNCOND("Node " << i+2 << " received: " << sink->GetTotalRx()/1316);
	}
	*/
	fout2.close();
	//for(uint16_t i = 0; i < 100; i++)
	//std::cout << rand()%200 - 100 << std::endl;
	nodeDevs.Get(1)->TraceConnectWithoutContext("PhyRxDrop", MakeCallback(&RxDrop));
	double sum_norm_e=0;	
	double airtime =0;

	std::ofstream fout3;
	std::ostringstream out_filename3;
	out_filename3 << "ResultNew/ee_energy_seed" << seed << "_nodes" << nodeNum << "_box" << box << ".txt";
	fout3.open(out_filename3.str().c_str(), std::ostream::out);
	if(!fout3.good())
		NS_LOG_UNCOND("File open failed");


	NS_LOG_UNCOND("Node\tInitial Energy\tRemaining Energy\tTotalConsumedEnergy\n");
	for (uint32_t i=0; i < nodes.GetN(); i++)
	{
		Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (i));

		Ptr<DeviceEnergyModel> basicRadioModelPtr =
			basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
		NS_ASSERT (basicRadioModelPtr != NULL);
		
		Ptr<ProposedWifiMac> mac = nodes.Get(i)->GetDevice(0)->GetObject<WifiNetDevice>()->GetMac()->GetObject<ProposedWifiMac>();

		double norm_e = basicRadioModelPtr->GetTotalEnergyConsumption()/basicSourcePtr->GetInitialEnergy();
			
		if(mac->GetStatus()==1)
				airtime = mac->GetAirtime();

		if(mac->GetStatus() > 0 && norm_e > sum_norm_e )
			sum_norm_e = norm_e;
		
		
		NS_LOG_UNCOND( i+1 << "\t" << basicSourcePtr->GetInitialEnergy() << "\t" << basicSourcePtr->GetInitialEnergy()-basicRadioModelPtr->GetTotalEnergyConsumption() <<"\t"<<  basicRadioModelPtr->GetTotalEnergyConsumption()<<"\t" << norm_e << "\t" << mac->GetStatus() );
		fout3 << i+1 << "\t" << basicSourcePtr->GetInitialEnergy() << "\t" << basicSourcePtr->GetInitialEnergy()-basicRadioModelPtr->GetTotalEnergyConsumption() <<"\t"<<  basicRadioModelPtr->GetTotalEnergyConsumption()<<"\t" << norm_e << "\t" << mac->GetStatus() << std::endl;
	}
	fout3.close();

	std::ofstream fout;
	std::ostringstream out_filename;
	out_filename << "ResultNew/relay_energy_final_seed" << seed << "_nodes" << nodeNum << "_box" << box << ".txt";
	fout.open(out_filename.str().c_str(), std::ostream::out);
	
	if(!fout.good())
		NS_LOG_UNCOND("File open failed");
	NS_LOG_UNCOND ("\nLifetime\t" << 1/sum_norm_e);
	fout << "Lifetime\t" << 1/sum_norm_e << std::endl;
	NS_LOG_UNCOND ("Airtime\t" << airtime);
	fout << "Airtime\t" << airtime << std::endl;
	NS_LOG_UNCOND ("Success Node Ratio\t" << (double)success_node/(nodeNum-1));
	fout << "Success Node Ratio\t" << (double)success_node/(nodeNum-1) <<std::endl;

	fout.close();


	Simulator::Destroy();

	// AthstatsHelper athstats;
	// athstats.EnableAthstats ("athstats-sta", stas);
	// athstats.EnableAthstats ("athstats-ap", ap);

	return 0;
}
