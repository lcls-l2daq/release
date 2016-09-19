//-----------------------------------------------------------------------------
// File          : PgpCardG3Prom.cpp
// Author        : Larry Ruckman  <ruckman@slac.stanford.edu>
// Created       : 03/19/2014
// Project       :  
//-----------------------------------------------------------------------------
// Description :
//    PgpCardG2 PROM C++ Class
//-----------------------------------------------------------------------------
// This file is part of 'SLAC PGP Gen3 Card'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'SLAC PGP Gen3 Card', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
// Proprietary and confidential to SLAC.
//-----------------------------------------------------------------------------
// Modification history :
// 03/19/2014: created
//-----------------------------------------------------------------------------

#include <sstream>
#include <string>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <iomanip> 
#include <math.h>

#include "PgpCardG3Prom.h"
#include "McsRead.h"

using namespace std;

#define GEN3_PROM_VERSION  0xCEC83000
#define GEN3_MASK          (GEN3_PROM_VERSION >> 12)
#define PROM_BLOCK_SIZE    0x4000 // Assume the smallest block size of 16-kword/block
#define READ_MASK          0x80000000
#define PROM_SIZE          0x009481F6

// Configuration: Force default configurations
#define CONFIG_REG      0xFD4F

// Constructor
PgpCardG3Prom::PgpCardG3Prom (void volatile *mapStart, string pathToFile ) {   
   // Set the file path
   filePath = pathToFile;
   
   // Setup the register Mapping
   memVersion = mapStart;// firmware version
   mapData    = (void volatile *)((uint64_t)mapStart+0xC00);//write cmd/data bus
   mapAddress = (void volatile *)((uint64_t)mapStart+0xC04);//write/read address bus
   mapRead    = (void volatile *)((uint64_t)mapStart+0xC08);// read data bus
   
   // Setup the configuration Register
   writeToFlash(CONFIG_REG,0x60,0x03);
}

// Deconstructor
PgpCardG3Prom::~PgpCardG3Prom ( ) { 
}

//! Check for a valid firmware version  (true=valid firmware version)
bool PgpCardG3Prom::checkFirmwareVersion ( ) {
   uint32_t firmwareVersion = *((uint32_t*)memVersion);
   uint32_t PgpCardGen = firmwareVersion >> 12;

   cout << "*******************************************************************" << endl;
   cout << "Current Firmware Version on the FPGA: 0x" << hex << firmwareVersion << endl;

   if(PgpCardGen!=GEN3_MASK){
   cout << "*******************************************************************" << endl;
      cout << "Error: Not a generation 3 PGP card" << endl;
      return false;
   } else {
      return true;
   }
}

//! Check if file exist (true=exists)
bool PgpCardG3Prom::fileExist ( ) {
  ifstream ifile(filePath.c_str());
  return ifile;
}

//! Print Power Cycle Reminder
void PgpCardG3Prom::rebootReminder ( ) {
   cout << "\n\n\n\n\n";
   cout << "***************************************" << endl;
   cout << "***************************************" << endl;
   cout << "A cold reboot or power cycle is required " << endl;
   cout << "to load the new firmware." << endl;
   cout << "***************************************" << endl;
   cout << "***************************************" << endl;
   cout << "\n\n\n\n\n";
}

//! Erase the PROM
void PgpCardG3Prom::eraseBootProm ( ) {

   uint32_t address = 0;
   double size = double(PROM_SIZE);
   double percentage;
   double skim = 5.0;    

   cout << "*******************************************************************" << endl;   
   cout << "Starting Erasing ..." << endl; 
   while(address<=PROM_SIZE) {       
      eraseCommand(address);
      address += PROM_BLOCK_SIZE;
      percentage = (((double)address)/size)*100;
      if(percentage>=skim) {
         skim += 5.0;
         cout << "Erasing the PROM: " << floor(percentage) << " percent done" << endl;
      }               
   }   
   cout << "Erasing completed" << endl;
}

//! Write the .mcs file to the PROM
bool PgpCardG3Prom::bufferedWriteBootProm ( ) {
   cout << "*******************************************************************" << endl;
   cout << "Starting Writing ..." << endl; 
   McsRead mcsReader;
   McsReadData mem;
   
   uint32_t address = 0;  
   uint16_t fileData;
   uint16_t i;
   
   uint32_t bufAddr[256];  
   uint16_t bufData[256];   
   uint16_t bufSize = 0;
   
   double size = double(PROM_SIZE);
   double percentage;
   double skim = 5.0; 
   bool   toggle = false;

   //check for valid file path
   if ( !mcsReader.open(filePath) ) {
      mcsReader.close();
      cout << "mcsReader.close() = file path error" << endl;
      return false;
   }  
   
   //reset the flags
   mem.endOfFile = false;      
   
   //read the entire mcs file
   while(!mem.endOfFile) {
   
      //read a line of the mcs file
      if (mcsReader.read(&mem)<0){
         cout << "mcsReader.close() = line read error" << endl;
         mcsReader.close();
         return false;
      }
      
      // Check if this is the upper or lower byte
      if(!toggle) {
         toggle = true;
         fileData = (uint16_t)mem.data;
      } else {
         toggle = false;
         fileData |= ((uint16_t)mem.data << 8);
         
         // Latch the values
         bufAddr[bufSize] = address;
         bufData[bufSize] = fileData;
         bufSize++;
         
         // Check if we need to send the buffer
         if(bufSize==256) {
            bufferedProgramCommand(bufAddr,bufData,bufSize);
            bufSize = 0;
         }

         address++;
         percentage = (((double)address)/size)*100;
         percentage *= 2.0;//factor of two from two 8-bit reads for every write 16 bit write
         if(percentage>=skim) {
            skim += 5.0;
            cout << "Writing the PROM: " << floor(percentage) << " percent done" << endl;
         }         
      }
   }
   
   // Check if we need to send the buffer
   if(bufSize != 0) {
      // Pad the end of the block with ones
      for(i=bufSize;i<256;i++){
         bufData[bufSize] = 0xFFFF;
      }
      // Send the last block program 
      bufferedProgramCommand(bufAddr,bufData,256);  
   }     
   
   mcsReader.close();   
   cout << "Writing completed" << endl;   
   return true;
}

//! Compare the .mcs file with the PROM (true=matches)
bool PgpCardG3Prom::verifyBootProm ( ) {
   cout << "*******************************************************************" << endl;
   cout << "Starting Verification ..." << endl; 
   McsRead mcsReader;
   McsReadData mem;
   
   uint32_t address = 0;  
   uint16_t promData,fileData;
   double size = double(PROM_SIZE);
   double percentage;
   double skim = 5.0; 
   bool   toggle = false;

   //check for valid file path
   if ( !mcsReader.open(filePath) ) {
      mcsReader.close();
      cout << "mcsReader.close() = file path error" << endl;
      return(1);
   }  
   
   //reset the flags
   mem.endOfFile = false;   

   //read the entire mcs file
   while(!mem.endOfFile) {
   
      //read a line of the mcs file
      if (mcsReader.read(&mem)<0){
         cout << "mcsReader.close() = line read error" << endl;
         mcsReader.close();
         return false;
      }
      
      // Check if this is the upper or lower byte
      if(!toggle) {
         toggle = true;
         fileData = (uint16_t)mem.data;
      } else {
         toggle = false;
         fileData |= ((uint16_t)mem.data << 8);
         promData = readWordCommand(address);                
         if(fileData != promData) {
            cout << "verifyBootProm error = ";
            cout << "invalid read back" <<  endl;
            cout << hex << "\taddress: 0x"  << address << endl;
            cout << hex << "\tfileData: 0x" << fileData << endl;
            cout << hex << "\tpromData: 0x" << promData << endl;
            mcsReader.close();
            return false;
         }
         address++;
         percentage = (((double)address)/size)*100;
         percentage *= 2.0;//factore of two from two 8-bit reads for every write 16 bit write
         if(percentage>=skim) {
            skim += 5.0;
            cout << "Verifying the PROM: " << floor(percentage) << " percent done" << endl;
         }         
      }
   }
   
   mcsReader.close();  
   cout << "Verification completed" << endl;
   cout << "*******************************************************************" << endl;   
   return true;
}

//! Erase Command
void PgpCardG3Prom::eraseCommand(uint32_t address) {
   uint16_t status = 0;
   
   // Unlock the Block
   writeToFlash(address,0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address,0x50,0x50);   
   
   // Send the erase command
   writeToFlash(address,0x20,0xD0);
   
   while(1) {
      // Get the status register
      status = readFlash(address,0x70);
      
      // Check for erasing failure
      if ( (status&0x20) != 0 ) {
      
         // Unlock the Block
         writeToFlash(address,0x60,0xD0);
         
         // Reset the status register
         writeToFlash(address,0x50,0x50);   
         
         // Send the erase command
         writeToFlash(address,0x20,0xD0);      
      
      // Check for FLASH not busy
      } else if ( (status&0x80) != 0 ) {
         break;
      }
   } 

   // Lock the Block
   writeToFlash(address,0x60,0x01);   
}

//! Program Command
void PgpCardG3Prom::programCommand(uint32_t address, uint16_t data) {
   uint16_t status = 0;
   
   // Unlock the Block
   writeToFlash(address,0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address,0x50,0x50);   
   
   // Send the program command
   writeToFlash(address,0x40,data);   
   
   while(1) {
      // Get the status register
      status = readFlash(address,0x70);
      
      // Check for programming failure
      if ( (status&0x10) != 0 ) {
      
         // Unlock the Block
         writeToFlash(address,0x60,0xD0);
         
         // Reset the status register
         writeToFlash(address,0x50,0x50);   
         
         // Send the program command
         writeToFlash(address,0x40,data);     
      
      // Check for FLASH not busy
      } else if ( (status&0x80) != 0 ) {
         break;
      }
   } 

   // Lock the Block
   writeToFlash(address,0x60,0x01);   
}

//! Buffered Program Command
void PgpCardG3Prom::bufferedProgramCommand(uint32_t *address, uint16_t *data, uint16_t size) {
   uint16_t status = 0;
   uint16_t i;
   
   // Unlock the Block
   writeToFlash(address[0],0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address[0],0x50,0x50);

   // Send the buffer program command and size
   writeToFlash(address[0],0xE8,(size-1));   
   
   // Load the buffer
   for(i=0;i<size;i++) {
      readFlash(address[i],data[i]);
   }
  
   // Confirm buffer programming
   readFlash(address[0],0xD0);  
   
   while(1) {
      // Get the status register
      status = readFlash(address[0],0x70);
      
      // Check for programming failure
      if ( (status&0x10) != 0 ) {
      
         // Unlock the Block
         writeToFlash(address[0],0x60,0xD0);
         
         // Reset the status register
         writeToFlash(address[0],0x50,0x50);   
         
         // Send the buffer program command and size
         writeToFlash(address[0],0xE8,(size-1));   
         
         // Load the buffer
         for(i=0;i<size;i++) {
            readFlash(address[i],data[i]);
         }
        
         // Confirm buffer programming
         readFlash(address[0],0xD0);                    
      
      // Check for FLASH not busy
      } else if ( (status&0x80) != 0 ) {
         break;
      }
   } 

   // Lock the Block
   writeToFlash(address[0],0x60,0x01);   
}

//! Read FLASH memory Command
uint16_t PgpCardG3Prom::readWordCommand(uint32_t address) {
   return readFlash(address,0xFF);
}

//! Generate request word 
uint32_t PgpCardG3Prom::genReqWord(uint16_t cmd, uint16_t data) {
   uint32_t readReq;
   readReq = ( ((uint32_t)cmd << 16) | ((uint32_t)data) );
   return readReq;
}

//! Generic FLASH write Command 
void PgpCardG3Prom::writeToFlash(uint32_t address, uint16_t cmd, uint16_t data) {
   
   asm("nop");//no operation function      
         
   // Set the data bus
   *((uint32_t*)mapData) = genReqWord(cmd,data);
   
   asm("nop");//no operation function   
   
   // Set the address bus and initiate the transfer
   *((uint32_t*)mapAddress) = (~READ_MASK & address);
}

//! Generic FLASH read Command
uint16_t PgpCardG3Prom::readFlash(uint32_t address, uint16_t cmd) {
   uint32_t readReg;
   
   asm("nop");//no operation function      
      
   // Set the data bus
   *((uint32_t*)mapData) = genReqWord(cmd,0xFF);
   
   asm("nop");//no operation function   
   
   // Set the address bus and initiate the transfer
   *((uint32_t*)mapAddress) = (READ_MASK | address);   
   
   asm("nop");//no operation function   
   
   // Read the data register
   readReg = *((uint32_t*)mapRead);
   
   // return the readout data
   return (uint16_t)(readReg&0xFFFF);
}
