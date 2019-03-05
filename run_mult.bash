#!/bin/bash
# for i in `seq 1 10`;
# do
# 	./run "test-simulation --lossModel=5 --logFile=tworay_highway.csv --run=$i"
# done

for i in 110 120 130 140 150;
do
  echo "Running with $i nodes"
  ./run "highway-simulation --CSVfileName=tworay_highway_delay_$i.csv --nodes=$i --lossModel=5"
done

# for dt in 20 30 40 50 60;
# do
#   echo "Distance $dt m"
#   for i in `seq 1 10`;
#   do
#   	./run "intersection-simulation --logFile=int_10_$dt.csv --run=$i --dt=$dt"
#   done
#   mv ns-3-dev/int_10_$dt.csv_* output/.
# done
