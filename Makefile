all:
	g++ -O2 -Wall  -o main main.cpp Calibration.cpp PostProcessor.cpp Scan.cpp `pkg-config --cflags --libs opencv`
clean:
	rm -rf *.o
	
