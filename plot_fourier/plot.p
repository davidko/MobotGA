set terminal pngcairo color dashed
set output "plot_ideal_vs_sim.png" 

set style line 1 lt 1 lc 1
set style line 2 lt 1 lc 2
set style line 3 lt 1 lc 3
set style line 4 lt 1 lc 4
set style line 5 lt 1 lc 1
set style line 6 lt 1 lc 2
set style line 7 lt 1 lc 3
set style line 8 lt 1 lc 4

set xrange [5:25]

plot "data.txt" using 1:2 with dots ls 1 notitle, \
"data.txt" using 1:3 with dots ls 2 notitle, \
"data.txt" using 1:4 with dots ls 3 notitle, \
"data.txt" using 1:5 with dots ls 4 notitle, \
"../jointdata.txt" using 1:2 with lines ls 1 title "joint 1", \
"../jointdata.txt" using 1:3 with lines ls 2 title "joint 2", \
"../jointdata.txt" using 1:4 with lines ls 3 title "joint 3", \
"../jointdata.txt" using 1:5 with lines ls 4 title "joint 4" 
#"data.txt" using 1:6 with lines , \
#"data.txt" using 1:7 with lines , \
#"data.txt" using 1:8 with lines , \
#"data.txt" using 1:9 with lines , \
#"data.txt" using 1:10 with lines , \
#"data.txt" using 1:11 with lines , \
#"data.txt" using 1:12 with lines , \
#"data.txt" using 1:13 with lines 

