all:
	g++ -Wall -m64 -g -I "../include/" setScratch.cpp -o setScratch
	g++ -Wall -m64 -g -I "../include/" xwrite.cpp -o xwrite
	g++ -Wall -m64 -g -I "../include/" xread.cpp -o xread
	g++ -Wall -m64 -g -I "../include/" xstatus.cpp -o xstatus
	g++ -Wall -m64 -g -I "../include/" xdebug.cpp -o xdebug
	g++ -Wall -m64 -g -I "../include/" xclearopenclients.cpp -o xclearopenclients
	g++ -Wall -m64 -g -I "../include/" xcomp.cpp -o xcomp
	g++ -Wall -m64 -g -I "../include/" xreset.cpp -o xreset
	g++ -Wall -m64 -g -I "../include/" xloop.cpp -o xloop
#	g++ -Wall -m64 -g -I "../include/" xforward.cpp -o xforward
	g++ -Wall -m64 -g -I "../include/" xcntrst.cpp -o xcntrst
#	g++ -Wall -m64 -g -I "../include/" xrate.cpp -o xrate -lpthread
	g++ -Wall -m64 -g -I "../include/" xecho.cpp -o xecho -lrt -lm
	g++ -Wall -m64 -g -I "../include/" xtrans.cpp -o xtrans -lrt -lm
	g++ -Wall -m64 -g -I "../include/" xtesttx.cpp -o xtesttx -lrt -lm
	g++ -Wall -m64 -g -I "../include/" xcopy.cpp -o xcopy -lrt -lm
#	g++ -Wall -m64 -g Tests.cpp -fopenmp -o Tests -lrt
	g++ -c -Wall -m32 -I$(PWD)/../generic $(PWD)/../generic/McsRead.cpp -o McsRead.o
	g++ -c -Wall -m32 -I$(PWD)/../generic -I$(PWD) PgpCardG2Prom.cpp -o PgpCardG2Prom.o
	g++ -Wall -m32 -I "../include" McsRead.o PgpCardG2Prom.o xPromLoad.cpp -o xPromLoad -lrt -lm
	g++ -Wall -m32 -I "../include" McsRead.o PgpCardG2Prom.o xPromVerify.cpp -o xPromVerify -lrt -lm

clean:
	rm -f setScratch
	rm -f xwrite
	rm -f xread
	rm -f xstatus
	rm -f xdebug
	rm -f xcomp
	rm -f xreset
	rm -f xloop
	rm -f xforward
	rm -f xcntrst
	rm -f xrate
	rm -f xecho
	rm -f xtrans
	rm -f xtesttx
	rm -f xcopy
	rm -f Tests
	rm -f McsRead.o
	rm -f PgpCardG2Prom.o
	rm -f xPromLoad
	rm -f xPromVerify
