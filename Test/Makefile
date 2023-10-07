TARGET = PAsound
SRC = main.cpp PAsound.cpp

INCLUDE = -I /opt/homebrew/Cellar/portaudio/19.7.0/include
LIBS = -lportaudio

# on mac/linux: libs are usually installed in /usr/lib which is likely in PATH
LIBS += -L /opt/homebrew/Cellar/portaudio/19.7.0/lib

CXX = g++
FLAGS = -std=c++11

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(INCLUDE) $(FLAGS) $(SRC) -o $(TARGET) $(LIBS)

clean:
	rm -f $(TARGET)

run:
	@./$(TARGET)
