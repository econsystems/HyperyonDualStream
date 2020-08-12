CC = g++
CFLAGS = -ggdb -std=c++0x 
RM = rm -f
LDFLAGS = `pkg-config --cflags --libs opencv` -lpthread -ludev

default:	
	@$(CC) $(CFLAGS) HyperyonCam.cpp -o HyperyonCam $(LDFLAGS)

all:	HyperyonCam

OpenCVCam: HyperyonCam.cpp
	@$(CC) $(CFLAGS) HyperyonCam.cpp -o HyperyonCam $(LDFLAGS)

clean veryclean:
	@$(RM) HyperyonCam
