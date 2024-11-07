#include "ns3/core-module.h"


#include <iostream>

using namespace ns3;

int main(int argc, char* argv[]){

    NodeContainer c;
    c.Create(2); // create 2 nodes

    return 0;
}