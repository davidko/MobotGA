OBJS=main.o mobot_model.o

HEADERS=mobot_specs.h mobot_model.h

all:main

mobot_model.o:mobot_model.cpp $(HEADERS)
	g++ -c `pkg-config --cflags panda3d ode` mobot_model.cpp -o mobot_model.o

main.o:main.cpp $(HEADERS)
	g++ -c `pkg-config --cflags panda3d ode` main.cpp -o main.o

main:$(OBJS)
	g++ $(OBJS) `pkg-config --libs panda3d ode` -o main
