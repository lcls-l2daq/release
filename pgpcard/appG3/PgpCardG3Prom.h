//-----------------------------------------------------------------------------
// File          : PgpCardG3Prom.h
// Author        : Larry Ruckman  <ruckman@slac.stanford.edu>
// Created       : 03/19/2014
// Project       :  
//-----------------------------------------------------------------------------
// Description :
//    PgpCardG2 PROM C++ Class
//-----------------------------------------------------------------------------
// Copyright (c) 2014 by SLAC. All rights reserved.
// Proprietary and confidential to SLAC.
//-----------------------------------------------------------------------------
// Modification history :
// 03/19/2014: created
//-----------------------------------------------------------------------------

#ifndef __PGPCARDG3_PROM_H__
#define __PGPCARDG3_PROM_H__

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <linux/types.h>

#include <string.h>
#include <stdint.h>

using namespace std;

//! Class to contain generic register data.
class PgpCardG3Prom {
   public:

      //! Constructor
      PgpCardG3Prom (void volatile *mapStart, string pathToFile );

      //! Deconstructor
      ~PgpCardG3Prom ( );

      //! Check for a valid firmware version 
      bool checkFirmwareVersion ( );
      
      //! Check if file exist
      bool fileExist ( );      
      
      //! Erase the PROM
      void eraseBootProm ( );    

      //! Write the .mcs file to the PROM
      bool bufferedWriteBootProm ( );       

      //! Compare the .mcs file with the PROM
      bool verifyBootProm ( );     

      //! Print Reminder
      void rebootReminder ( );      
   
   private:
      // Local Variables
      string filePath;  
      void volatile *memVersion;
      void volatile *mapData;
      void volatile *mapAddress;
      void volatile *mapRead;      
      
      //! Erase Command
      void eraseCommand(uint32_t address);
      
      //! Program Command
      void programCommand(uint32_t address, uint16_t data);
      
      //! Buffered Program Command
      void bufferedProgramCommand(uint32_t *address, uint16_t *data, uint16_t size);

      //! Read FLASH memory Command
      uint16_t readWordCommand(uint32_t address);

      //! Generate request word 
      uint32_t genReqWord(uint16_t cmd, uint16_t data);

      //! Generic FLASH write Command
      void writeToFlash(uint32_t address, uint16_t cmd, uint16_t data);

      //! Generic FLASH read Command
      uint16_t readFlash(uint32_t address, uint16_t cmd);        
};
#endif
