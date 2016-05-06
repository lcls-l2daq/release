all:
  g++ -Wall -m64 -I "../include" xStatus.cpp -o xStatus
  g++ -Wall -m64 -I "../include" xReset.cpp -o xReset
  g++ -Wall -m64 -I "../include" xDebug.cpp -o xDebug
  g++ -Wall -m64 -I "../include" xWrite.cpp -o xWrite
  g++ -Wall -m64 -I "../include" xRead.cpp -o xRead
  g++ -Wall -m64 -I "../include" xLoop.cpp -o xLoop
  g++ -Wall -m64 -I "../include" evrEnable.cpp -o evrEnable
  g++ -Wall -m64 -I "../include" evrDisable.cpp -o evrDisable
  g++ -Wall -m64 -I "../include" countReset.cpp -o countReset
  g++ -Wall -m64 -I "../include" evrRunCode.cpp -o evrRunCode
  g++ -Wall -m64 -I "../include" evrAcceptCode.cpp -o evrAcceptCode
  g++ -Wall -m64 -I "../include" evrAcceptDelay.cpp -o evrAcceptDelay
  g++ -Wall -m64 -I "../include" evrFiducial.cpp -o evrFiducial
  g++ -Wall -m64 -I "../include" evrRunDelay.cpp -o evrRunDelay
  g++ -Wall -m64 -I "../include" evrClearFrameCount.cpp -o evrClearFrameCount
  g++ -Wall -m64 -I "../include" evrConfig.cpp -o evrConfig
  g++ -Wall -m64 -I "../include" evrHdrChk.cpp -o evrHdrChk
  g++ -Wall -m64 -I "../include" evrReset.cpp -o evrReset
  g++ -Wall -m64 -I "../include" evrLaneEnable.cpp -o evrLaneEnable
  g++ -Wall -m64 -I "../include" evrLaneMode.cpp -o evrLaneMode
  g++ -c -Wall -m32 -I${PWD}/../generic -I${PWD} PgpCardG3Prom.cpp -o PgpCardG3Prom.o
  g++ -Wall -m32 -I "../include" McsRead.o PgpCardG3Prom.o xPromLoad.cpp -o xPromLoad -lrt -lm 
  g++ -Wall -m32 -I "../include" McsRead.o PgpCardG3Prom.o xPromVerify.cpp -o xPromVerify -lrt -lm 

clean:
  rm -f xStatus
  rm -f xReset
  rm -f xLoop
  rm -f xWrite
  rm -f xRead
  rm -f evrConfig
  rm -f evrEnable
  rm -f evrDisable
  rm -f evrReset
  rm -f PgpCardG3Prom.o
  rm -f xPromLoad
  rm -f xPromVerify

