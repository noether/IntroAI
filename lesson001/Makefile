LIBS_PATH = -I/usr/include/eigen3

CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++11 $(LIBS_PATH)
LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system

OBJECTS_TEST = lesson001.o agent.o

all: lesson001

lesson001: $(OBJECTS_TEST)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS_PATH) $(LDFLAGS)

clean:
	$(RM) *~ *.o lesson001
