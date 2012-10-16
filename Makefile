all:main

main.o:main.cpp
	g++ -c `pkg-config --cflags panda3d ode` main.cpp -o main.o

main:main.o
	g++ main.o `pkg-config --libs panda3d ode` -o main
