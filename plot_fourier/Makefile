all:
	gcc main.c -o main -lm
	./main
	gnuplot -persist ./plot.p
	gnuplot -persist ./plot2.p
	gnuplot -persist ./plot_j1.p
	gnuplot -persist ./plot_j2.p
	gnuplot -persist ./plot_j3.p
	gnuplot -persist ./plot_j4.p
	#evince plot.png
