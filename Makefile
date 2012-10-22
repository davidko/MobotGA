OBJS=main.o mobot_model.o
HEADERS=mobot_specs.h mobot_model.h
CFLAGS=-c -g

all:main

mobot_model.o:mobot_model.cpp $(HEADERS)
	g++ $(CFLAGS) `pkg-config --cflags ode panda3d` mobot_model.cpp -o mobot_model.o

main.o:main.cpp $(HEADERS)
	g++ $(CFLAGS) `pkg-config --cflags ode panda3d` main.cpp -o main.o

main:$(OBJS)
	g++ $(OBJS) `pkg-config --libs ode panda3d` -o main

clean:
	rm -rf main $(OBJS)
