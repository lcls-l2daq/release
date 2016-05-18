
#include <sys/types.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include "../include/PgpCardMod.h"

using namespace std;

int main (int argc, char **argv) {
  PgpCardTx      pgpCardTx;
  int           fd;
  uint          x;
  int           ret;
  time_t        t;
  uint          lane;
  uint          vc;
  uint          size;
  uint          *data;
  unsigned      count = 0;
  unsigned      number = 0;

      if (argc < 5) {
        printf("Usage: %s device lane vc size [number]\n", argv[0]);
        return(1);
      }

  // Get args
  lane  = atoi(argv[2]);
  vc    = atoi(argv[3]);
  size  = atoi(argv[4]);

  if (argc == 6) {
    number = atoi(argv[5]);
  }

  // Check ranges
  if ( size == 0 || lane > 7 || vc > 3 ) {
    printf("Invalid size, lane or vc value : %u, %u or %u\n", size, lane, vc);
    return(1);
  }

  if ( (fd = open(argv[1], O_RDWR)) <= 0 ) {
    cout << "Error opening " << argv[1] << endl;
    perror(argv[1]);
    return(1);
  }

  time(&t);
  srandom(t);

  data = (uint *)malloc(sizeof(uint)*size);

  pgpCardTx.model   = (sizeof(data));
  pgpCardTx.cmd     = IOCTL_Normal_Write;
  pgpCardTx.pgpVc   = vc;
  pgpCardTx.pgpLane = lane;
  pgpCardTx.size    = size;
  pgpCardTx.data    = (__u32*)data;
  cout << endl;
  do {
    // DMA Write
    cout << "Sending:";
    cout << " Lane=" << dec << lane;
    cout << ", Vc=" << dec << vc << endl;
    for (x=0; x<size; x++) {
      data[x] = random();
      cout << " 0x" << setw(8) << setfill('0') << hex << data[x];
      if ( ((x+1)%10) == 0 ) cout << endl << "   ";
    }
    cout << endl;
    ret = write(fd,&pgpCardTx,sizeof(PgpCardTx));
    cout << "Returned " << dec << ret << endl;
  } while ( count++ < number );
  free(data);

  close(fd);
  return(0);
}
