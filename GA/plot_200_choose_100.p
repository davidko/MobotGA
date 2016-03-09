#logs_201406081034
#logs_201406082250
#logs_201406090853
#logs_201406091241
#logs_201406091926
#logs_201406092222
#logs_201406100814

set multiplot layout 2, 2;
set nokey

set title "Population Diversity"
plot './logs_201406081034/GAstats.txt' using ($0):4 with lines, \
     './logs_201406082250/GAstats.txt' using ($0):4 with lines, \
     './logs_201406090853/GAstats.txt' using ($0):4 with lines, \
     './logs_201406091926/GAstats.txt' using ($0):4 with lines, \
     './logs_201406100814/GAstats.txt' using ($0):4 with lines

set title "Population Top Fitness"
plot './logs_201406081034/GAstats.txt' using ($0):1 with lines, \
     './logs_201406082250/GAstats.txt' using ($0):1 with lines, \
     './logs_201406090853/GAstats.txt' using ($0):1 with lines, \
     './logs_201406091926/GAstats.txt' using ($0):1 with lines, \
     './logs_201406100814/GAstats.txt' using ($0):1 with lines

set title "Population Average Fitness"
plot './logs_201406081034/GAstats.txt' using ($0):2 with lines, \
     './logs_201406082250/GAstats.txt' using ($0):2 with lines, \
     './logs_201406090853/GAstats.txt' using ($0):2 with lines, \
     './logs_201406091926/GAstats.txt' using ($0):2 with lines, \
     './logs_201406100814/GAstats.txt' using ($0):2 with lines

unset multiplot
