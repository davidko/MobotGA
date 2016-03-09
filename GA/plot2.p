#set ytics 10 nomirror tc lt 1
set ylabel 'Population Fitness' tc lt 1
#set y2tics 20 nomirror tc lt 2
set y2tics
set y2label 'Genepool Diversity' tc lt 3
plot 'logs_201406090853/GAstats.txt' using ($0*100):2 with lines title 'Avg Fitness', \
     'logs_201406090853/GAstats.txt' using ($0*100):1 with lines title 'Max Fitness' , \
     'logs_201406090853/GAstats.txt' using ($0*100):4 with lines axes x1y2 title 'Diversity' , \
