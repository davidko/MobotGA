#!/bin/bash

CURTIME=`timestamp.sh`

mkdir logs_$CURTIME
mv fitnesses* logs_$CURTIME
mv gen* logs_$CURTIME
mv GAstats.txt logs_$CURTIME
