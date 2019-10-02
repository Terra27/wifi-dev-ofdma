#!/bin/bash

# Use:
#   ./run-dl-ofdma-performance.sh
#   grep STA_ results_20.txt | sed 's/STA_[0-9]*: //g'
# to get the throughput values separated by a space (useful for import in a spreadsheet)

rm -f results_20_tcp_dl_ul.txt
rm -f results_40_tcp_dl_ul.txt
rm -f results_80_tcp_dl_ul.txt

for mcs in 11 8 4
  do
    for sta in 1 2 4 9
      do
        # NO OFDMA
        ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --enableDlOfdma=false --channelWidth=20 --guardInterval=800 --transport=Tcp --radius=1 --nStations=${sta}" >> results_20_tcp_dl_ul.txt
        # DL OFDMA
        for ack in 1 2 3
          do
            ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --dlAckType=${ack} --enableUlOfdma=true --ulPsduSize=500 --channelWidth=20 --guardInterval=800 --transport=Tcp --radius=1 --nStations=${sta} --maxRus=${sta}" >> results_20_tcp_dl_ul.txt
          done 
      done
  done

for mcs in 11 8 4
  do
    for sta in 1 2 4 8 18
      do
        # NO OFDMA
        ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --enableDlOfdma=false --channelWidth=40 --guardInterval=800 --transport=Tcp --radius=1 --nStations=${sta}" >> results_40_tcp_dl_ul.txt
        # DL OFDMA
        for ack in 1 2 3
          do
            ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --dlAckType=${ack} --enableUlOfdma=true --ulPsduSize=500 --channelWidth=40 --guardInterval=800 --transport=Tcp --radius=1 --nStations=${sta} --maxRus=${sta}" >> results_40_tcp_dl_ul.txt
          done 
      done
  done

for mcs in 11 8 4
  do
    for sta in 1 2 4 8 16 37
      do
        # NO OFDMA
        ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --enableDlOfdma=false --channelWidth=80 --guardInterval=800 --transport=Tcp --radius=1 --nStations=${sta}" >> results_80_tcp_dl_ul.txt
        # DL OFDMA
        for ack in 1 2 3
          do
            ./waf --run "wifi-dl-ofdma --simulationTime=10 --mcs=${mcs} --txopLimit=10016 --dlAckType=${ack} --enableUlOfdma=true --ulPsduSize=500 --channelWidth=80 --guardInterval=800 --transport=Tcp --radius=1 --nStations=${sta} --maxRus=${sta}" >> results_80_tcp_dl_ul.txt
          done 
      done
  done
