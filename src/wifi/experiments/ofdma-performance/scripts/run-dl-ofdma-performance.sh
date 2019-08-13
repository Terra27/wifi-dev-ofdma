#!/bin/bash

# Use:
#   ./run-dl-ofdma-performance.sh
#   grep STA_ results_20.txt | sed 's/STA_[0-9]*: //g'
# to get the throughput values separated by a space (useful for import in a spreadsheet)

rm -f results_20.txt
rm -f results_40.txt
rm -f results_80.txt

for mcs in 11 8 4
  do
    for sta in 1 2 4 9
      do
        # NO OFDMA
        ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --enableDlOfdma=false --channelWidth=20 --guardInterval=800 --transport=Udp --radius=1 --nStations=${sta}" >> results_20.txt
        # DL OFDMA
        for ack in 1 2 3
          do
            ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --dlAckType=${ack} --channelWidth=20 --guardInterval=800 --transport=Udp --radius=1 --nStations=${sta} --maxRus=${sta}" >> results_20.txt
          done 
      done
  done

for mcs in 11 8 4
  do
    for sta in 1 2 4 8 18
      do
        # NO OFDMA
        ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --enableDlOfdma=false --channelWidth=40 --guardInterval=800 --transport=Udp --radius=1 --nStations=${sta}" >> results_40.txt
        # DL OFDMA
        for ack in 1 2 3
          do
            ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --dlAckType=${ack} --channelWidth=40 --guardInterval=800 --transport=Udp --radius=1 --nStations=${sta} --maxRus=${sta}" >> results_40.txt
          done 
      done
  done

for mcs in 11 8 4
  do
    for sta in 1 2 4 8 16 37
      do
        # NO OFDMA
        ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --enableDlOfdma=false --channelWidth=80 --guardInterval=800 --transport=Udp --radius=1 --nStations=${sta}" >> results_80.txt
        # DL OFDMA
        for ack in 1 2 3
          do
            ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --dlAckType=${ack} --channelWidth=80 --guardInterval=800 --transport=Udp --radius=1 --nStations=${sta} --maxRus=${sta}" >> results_80.txt
          done 
      done
  done
