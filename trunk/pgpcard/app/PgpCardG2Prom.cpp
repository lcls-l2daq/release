//-----------------------------------------------------------------------------
// File          : PgpCardG2Prom.cpp
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

#include <sstream>
#include <string>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <iomanip> 

#include "PgpCardG2Prom.h"
#include "McsRead.h"

using namespace std;

#define GEN2_PROM_VERSION  0xCEC82100
#define GEN2_MASK          (GEN2_PROM_VERSION >> 12)
#define PROM_BLOCK_SIZE    0x10000
#define READ_MASK          0x80000000
#define PROM_SIZE          0x001ACD7F

// Configuration: Asynchronous Read and default for everything else
#define CONFIG_REG      0xBDDF

// Constructor
PgpCardG2Prom::PgpCardG2Prom (void volatile *mapStart, string pathToFile ) {   
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
PgpCardG2Prom::~PgpCardG2Prom ( ) { 
}

//! Check for a valid firmware version  (true=valid firmware version)
bool PgpCardG2Prom::checkFirmwareVersion ( ) {
   uint32_t firmwareVersion = *((uint64_t*)memVersion);
   uint32_t PgpCardGen = firmwareVersion >> 12;

   cout << "*******************************************************************" << endl;
   cout << "Current Firmware Version on the FPGA: 0x" << hex << firmwareVersion << endl;

   if(PgpCardGen!=GEN2_MASK){
   cout << "*******************************************************************" << endl;
      cout << "Error: Not a genration 2 PGP card" << endl;
      return false;
   } else if(firmwareVersion<GEN2_PROM_VERSION) {
   cout << "*******************************************************************" << endl;
      cout << "Error: This firmware doesn't have PROM support. ";
      cout << "Please manually program (via USB JTAG programmer) ";;
      cout << "firmware version 0x" << GEN2_PROM_VERSION << " (or later) to the PROM. ";
      cout << "Don't forget to power cycle the computer after manually programming the PROM. " << endl;
      return false;
   } else {
      return true;
   }
}

//! Check if file exist (true=exists)
bool PgpCardG2Prom::fileExist ( ) {
  ifstream ifile(filePath.c_str());
  return ifile;
}

//! Print Power Cycle Reminder
void PgpCardG2Prom::powerCycleReminder ( ) {
   cout << "\n\n\n\n\n";
   cout << "*******************************************************************" << endl;
   cout << "*******************************************************************" << endl;
   cout << "A power cycle of the computer is required to load the new firmware!" << endl;
   cout << "*******************************************************************" << endl;
   cout << "*******************************************************************" << endl;
   cout << "\n\n\n\n\n";
}

//! Erase the PROM
void PgpCardG2Prom::eraseBootProm ( ) {

   uint32_t address = 0;
   double size = double(PROM_SIZE);

   cout << "*******************************************************************" << endl;   
   cout << "Starting Erasing ..." << endl; 
   while(address<=PROM_SIZE) {       
      // Print the status to screen
      cout << hex << "Erasing PROM from 0x" << address << " to 0x" << (address+PROM_BLOCK_SIZE-1);
      cout << setprecision(3) << " ( " << ((double(address))/size)*100 << " percent done )" << endl;      
      
      // execute the erase command
      eraseCommand(address);
      
      //increment the address pointer
      address += PROM_BLOCK_SIZE;
   }   
   cout << "Erasing completed" << endl;
}

//! Write the .mcs file to the PROM
bool PgpCardG2Prom::writeBootProm ( ) {
   cout << "*******************************************************************" << endl;
   cout << "Starting Writing ..." << endl; 
   McsRead mcsReader;
   McsReadData mem;
   
   uint32_t address = 0;  
   uint16_t fileData;
   double size = double(PROM_SIZE);
   double percentage;
   double skim = 0.0; 
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
         programCommand(address,fileData);
         address++;
         percentage = (((double)address)/size)*100;
         percentage *= 2.0;//factor of two from two 8-bit reads for every write 16 bit write
         if(percentage>=skim) {
            skim += 5.0;
            cout << "Writing the PROM: " << percentage << " percent done" << endl;
         }         
      }
   }
   
   mcsReader.close();   
   cout << "Writing completed" << endl;   
   return true;
}

//! Compare the .mcs file with the PROM (true=matches)
bool PgpCardG2Prom::verifyBootProm ( ) {
   cout << "*******************************************************************" << endl;
   cout << "Starting Verification ..." << endl; 
   McsRead mcsReader;
   McsReadData mem;
   
   uint32_t address = 0;  
   uint16_t promData,fileData;
   double size = double(PROM_SIZE);
   double percentage;
   double skim = 0.0; 
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
            cout << "Verifying the PROM: " << percentage << " percent done" << endl;
         }         
      }
   }
   
   mcsReader.close();  
   cout << "Verification completed" << endl;
   cout << "*******************************************************************" << endl;   
   return true;
}

//! Erase Command
void PgpCardG2Prom::eraseCommand(uint32_t address) {
   uint16_t status = 0;
   
   // Unlock the Block
   writeToFlash(address,0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address,0x50,0x50);   
   
   // Send the erase command
   writeToFlash(address,0x20,0xD0);
   
   while( (status&0x80) == 0 ){
      status = readFlash(address,0x70);
   } 

   // Lock the Block
   writeToFlash(address,0x60,0x01);   
}

//! Program Command
void PgpCardG2Prom::programCommand(uint32_t address, uint16_t data) {
   uint16_t status = 0;
   
   // Unlock the Block
   writeToFlash(address,0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address,0x50,0x50);   
   
   // Send the program command
   writeToFlash(address,0x40,data);   
   
   while( (status&0x80) == 0 ){
      status = readFlash(address,0x70);
   } 

   // Lock the Block
   writeToFlash(address,0x60,0x01);   
}

//! Read FLASH memory Command
uint16_t PgpCardG2Prom::readWordCommand(uint32_t address) {
   return readFlash(address,0xFF);
}

//! Generate request word 
uint32_t PgpCardG2Prom::genReqWord(uint16_t cmd, uint16_t data) {
   uint32_t readReq;
   readReq = ( ((uint32_t)cmd << 16) | ((uint32_t)data) );
   return readReq;
}

//! Generic FLASH write Command 
void PgpCardG2Prom::writeToFlash(uint32_t address, uint16_t cmd, uint16_t data) {
   // Set the data bus
   *((uint32_t*)mapData) = genReqWord(cmd,data);
   
   // Set the address bus and initiate the transfer
   *((uint32_t*)mapAddress) = (~READ_MASK & address);
}

//! Generic FLASH read Command
uint16_t PgpCardG2Prom::readFlash(uint32_t address, uint16_t cmd) {
   uint32_t readReg;
      
   // Set the data bus
   *((uint32_t*)mapData) = genReqWord(cmd,0xFF);
   
   // Set the address bus and initiate the transfer
   *((uint32_t*)mapAddress) = (READ_MASK | address);   
   
   // Read the data register
   readReg = *((uint32_t*)mapRead);
   
   // return the readout data
   return (uint16_t)(readReg&0xFFFF);
}
