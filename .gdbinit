# For this example, have 'blackmagic' running already. No command line params
# are needed.
!/Users/bob/Documents/orbuculum/build/orbtrace -p vtref,3.3 -e vtref,on
file zig-out/firmware/Orbmule.elf
target extended-remote localhost:2000
set mem inaccessible-by-default off
monitor swdp_scan
attach 1
load

############### Everything below is for Tracing...remove if you don't need it ########

source ~/Documents/orbuculum/Support/gdbtrace.init
enableSTM32TRACE 4 1
# enableSTM32SWO
# prepareSWO 4000000 56000000 1 1

dwtSamplePC 1
dwtSyncTap 3
dwtPostTap 1
dwtPostInit 1
dwtPostReset 10
dwtCycEna 1

ITMId 1
ITMGTSFreq 3
ITMTSPrescale 3
ITMTXEna 1
ITMSYNCEna 1
ITMEna 1

ITMTER 0 0xFFFFFFFF
ITMTPR 0xFFFFFFFF
