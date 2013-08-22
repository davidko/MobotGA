OBJS=main.o mobot_model.o
HEADERS=mobot_specs.h mobot_model.h
PANDA3D_CFLAGS=-I/usr/local/panda/include 
CFLAGS=-c -g -pg $(PANDA3D_CFLAGS)
LIBS=-L/usr/local/panda/lib -lpanda -lp3direct -lp3framework -lpandaexpress -lp3dtoolconfig -lp3dtool -lpython2.7

all:main

mobot_model.o:mobot_model.cpp $(HEADERS)
	g++ $(CFLAGS) `pkg-config --cflags ode python` mobot_model.cpp -o mobot_model.o

main.o:main.cpp $(HEADERS)
	g++ $(CFLAGS) `pkg-config --cflags ode python` main.cpp -o main.o

main:$(OBJS)
	g++ $(OBJS) `pkg-config --libs ode` -o main -g -pg $(LIBS)

clean:
	rm -rf main $(OBJS)
