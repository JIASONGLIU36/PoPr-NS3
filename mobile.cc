#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include <iostream>
#include <vector>

using namespace ns3;


int main(int argc, char *argv[]){

    NodeContainer c;
    c.Create(2);

    /*Mobility*/
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator>();
    position->Add(Vector(0.0, 0.0, 0.0));
    position->Add(Vector(2.0, 0.0, 0.0));
    mobility.SetPositionAllocator(position);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(c);


    // Print the position of each node
    for (uint32_t i = 0; i < c.GetN(); ++i) {
        Ptr<MobilityModel> mobilityModel = c.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobilityModel->GetPosition();
        std::cout << "Node " << i << " position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    return 0;
}
