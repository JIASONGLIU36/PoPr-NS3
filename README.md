# PoPr-NS3
Please use NS-3.42 version to run this simulation file. The simulation file *bsm_csma2.cc* should be put into the scratch folder.

Once the simulator is ready, please use *./ns3 run "scratch/bsm_csma2.cc"* to run the simulator for PoPr.

An example of running the simulation command is *./ns3 run "scratch/bsm_csma2.cc  --TI=0.1 --nNodes=50 --GroupSize=8 --ST=200.0" > ./Result/50_G8.txt 2> c.txt*. It means run the simulator with the data block transfer rate 0.1, the participate node is 50 and the group size of the consensus is 8. The total time allows the simulator to run are 200.0 seconds.

There are following options for simulator to run:

|Option|Explaination|
|-------|-----------|
|--TI| The data block transfer rate |
|--ST| Total Simulation time running |
|--nNodes| Number of nodes participate in the simulation |
|--GroupSize| The consensus group size |
|--bsmInterval| The BSM intervals, default is set to 100ms |
