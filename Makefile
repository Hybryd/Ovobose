all:
#	g++ -O2 -Wall  -o calibration calibration.cpp `pkg-config --cflags --libs opencv`
#	g++ -O2 -Wall  -o see see.cpp `pkg-config --cflags --libs opencv`
#	g++ -O2 -Wall  -o scan scan.cpp `pkg-config --cflags --libs opencv`
#	g++ -O2 -Wall  -o calibrationMouse3 calibrationMouse3.cpp `pkg-config --cflags --libs opencv`
#	g++ -O2 -Wall  -o scan3 scan3.cpp `pkg-config --cflags --libs opencv`
#	g++ -O2 -Wall  -o calibration Calibration.cpp `pkg-config --cflags --libs opencv`
	g++ -O2 -Wall  -o main main.cpp Calibration.cpp Scan.cpp `pkg-config --cflags --libs opencv`
clean:
	rm -rf exemple
	
