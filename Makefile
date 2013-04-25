all:
	g++ -O2 -Wall  -o calibration calibration.cpp `pkg-config --cflags --libs opencv`
clean:
	rm -rf exemple
