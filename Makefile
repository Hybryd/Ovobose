all:
	g++ -O2 -Wall  -o main main.cpp `pkg-config --cflags --libs opencv`
clean:
	rm -rf exemple
