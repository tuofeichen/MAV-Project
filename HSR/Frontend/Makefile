# compiler
CC=g++
#CFLAGS=-std=c++11 -I/usr/include/eigen3 -O3 -Wall -c
CFLAGS=-std=c++11 -mcpu=cortex-a9 -O3 -Ofast -mfloat-abi=hard -ftree-vectorize -ftree-vectorizer-verbose=9 -I/opt/opencv3/include -I/usr/include/eigen3 -I/home/odroid/OpenNI-Linux-Arm-2.2/Include -Wall -c

#linker
LDFLAGS=-L/opt/opencv3/lib -L/usr/lib -L/home/odroid/OpenNI-Linux-Arm-2.2/Redist
LIBS= -lboost_system -lboost_thread -lpthread -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_videoio -lopencv_imgproc -lopencv_imgcodecs -lOpenNI2

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH":/opt/opencv3/lib:/home/odroid/OpenNI-Linux-Arm-2.2/Redist

# sources
SOURCES=AnalyticBasedTME.cpp AsusProLive.cpp AsusProLiveOpenNI2.cpp Backend.cpp FeatureDEM.cpp Frame.cpp RANSACBasedTME.cpp TcpServer.cpp VisualOdometry.cpp UavComProtocolHandler.cpp main.cpp

OBJECTS=$(SOURCES:.cpp=.o)

EXECUTABLE=Frontend


# make all
all: $(SOURCES) $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS) 
#	$(CC) $(LDFLAGS) $(LIBS)  $(OBJECTS) -o $@
	$(CC) -o $@ $(LDFLAGS) $(OBJECTS) $(LIBS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

# clean	
clean:
	rm *o 
	rm $(EXECUTABLE)
