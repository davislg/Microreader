/******************************************************************************
 
                Microchip Memory Disk Drive File System
 
 *****************************************************************************
  FileName:        Internal Flash.c
  Dependencies:    See includes section.
  Processor:       PIC18/PIC24/dsPIC33/PIC32 microcontrollers
  Compiler:        MPLAB(R) C18/C30/C32 compilers are supported by this code
  Company:         Microchip Technology, Inc.
 
  Software License Agreement
 
  The software supplied herewith by Microchip Technology Incorporated
  (the “Company”) for its PICmicro® Microcontroller is intended and
  supplied to you, the Company’s customer, for use solely and
  exclusively on Microchip PICmicro Microcontroller products. The
  software is owned by the Company and/or its supplier, and is
  protected under applicable copyright laws. All rights are reserved.
  Any use in violation of the foregoing restrictions may subject the
  user to criminal sanctions under applicable laws, as well as to
  civil liability for the breach of the terms and conditions of this
  license.
 
  THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
  WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
  TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
  IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
  CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
*****************************************************************************/
//DOM-IGNORE-BEGIN
/********************************************************************
 Change History:
  Rev    Description
  -----  -----------
  1.2.5  Fixed bug where sector address was calculated incorrectly
  1.2.7  Re-implemented sector read/write functions for PIC24
         devices.  This removes the 32kB PSV size restriction.
         Also added some additional error checking.
  1.3.0  Expanded flash memory pointer size on PIC18 to 24-bits.  This
         allows the MSD volume to exceed 64kB on large flash memory devices.
  1.3.6   Modified "FSConfig.h" to "FSconfig.h" in '#include' directive.
  1.3.8   Modified "MDD_IntFlash_SectorRead" to write the correct word data
          in 'buffer' pointer
********************************************************************/
//DOM-IGNORE-END

#include "Compiler.h"
#include "FSIO.h"
#include "FSDefs.h"
#include "string.h"
#include "Internal Flash.h"
#include "HardwareProfile.h"
#include "FSconfig.h"
#include "iic.h"

/*************************************************************************/
/*  Note:  This file is included as a template of a C file for           */
/*         a new physical layer. It is designed to go with               */
/*         "TEMPLATEFILE.h"                                              */
/*************************************************************************/

/******************************************************************************
 * Global Variables
 *****************************************************************************/

#ifdef USE_PIC18
	#pragma udata
	#pragma code
#endif

static MEDIA_INFORMATION mediaInformation;

/******************************************************************************
 * Prototypes
 *****************************************************************************/
void EraseBlock(ROM BYTE* dest);
void WriteRow(void);
void WriteByte(unsigned char);
BYTE DISKmount( DISK *dsk);
BYTE LoadMBR(DISK *dsk);
BYTE LoadBootSector(DISK *dsk);
extern void Delayms(BYTE milliseconds);
MEDIA_INFORMATION * MediaInitialize(void);
void UnlockAndActivate(BYTE);

//Arbitray, but "uncommon" value.  Used by UnlockAndActivateWR() to enhance robustness.
#define NVM_UNLOCK_KEY  (BYTE)0xB5   

/******************************************************************************
 * Function:        BYTE MediaDetect(void)
 *
 * PreCondition:    InitIO() function has been executed.
 *
 * Input:           void
 *
 * Output:          TRUE   - Card detected
 *                  FALSE   - No card detected
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
BYTE MDD_IntFlash_MediaDetect()
{
	return TRUE;
}//end MediaDetect

/******************************************************************************
 * Function:        WORD ReadSectorSize(void)
 *
 * PreCondition:    MediaInitialize() is complete
 *
 * Input:           void
 *
 * Output:          WORD - size of the sectors for this physical media.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
WORD MDD_IntFlash_ReadSectorSize(void)
{
    return MEDIA_SECTOR_SIZE;
}

/******************************************************************************
 * Function:        DWORD ReadCapacity(void)
 *
 * PreCondition:    MediaInitialize() is complete
 *
 * Input:           void
 *
 * Output:          DWORD - size of the "disk" - 1 (in terms of sector count).  
 *                  Ex: In other words, this function returns the last valid 
 *                  LBA address that may be read/written to.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
DWORD MDD_IntFlash_ReadCapacity(void)
{
    //The SCSI READ_CAPACITY command wants to know the last valid LBA address 
    //that the host is allowed to read or write to.  Since LBA addresses start
    //at and include 0, a return value of 0 from this function would mean the 
    //host is allowed to read and write the LBA == 0x00000000, which would be 
    //1 sector worth of capacity.
    //Therefore, the last valid LBA that the host may access is 
    //MDD_INTERNAL_FLASH_TOTAL_DISK_SIZE - 1.
        
    return (MDD_INTERNAL_FLASH_TOTAL_DISK_SIZE - 1); 
}

/******************************************************************************
 * Function:        BYTE InitIO(void)
 *
 * PreCondition:    None
 *
 * Input:           void
 *
 * Output:          TRUE   - Card initialized
 *                  FALSE   - Card not initialized
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
BYTE MDD_IntFlash_InitIO (void)
{
    return  TRUE;
}

/******************************************************************************
 * Function:        BYTE MediaInitialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Returns a pointer to a MEDIA_INFORMATION structure
 *
 * Overview:        MediaInitialize initializes the media card and supporting variables.
 *
 * Note:            None
 *****************************************************************************/
MEDIA_INFORMATION * MDD_IntFlash_MediaInitialize(void)
{
    mediaInformation.validityFlags.bits.sectorSize = TRUE;
    mediaInformation.sectorSize = MEDIA_SECTOR_SIZE;
    
	mediaInformation.errorCode = MEDIA_NO_ERROR;
	return &mediaInformation;
}//end MediaInitialize


/******************************************************************************
 * Function:        BYTE SectorRead(DWORD sector_addr, BYTE *buffer)
 *
 * PreCondition:    None
 *
 * Input:           sector_addr - Sector address, each sector contains 512-byte
 *                  buffer      - Buffer where data will be stored, see
 *                                'ram_acs.h' for 'block' definition.
 *                                'Block' is dependent on whether internal or
 *                                external memory is used
 *
 * Output:          Returns TRUE if read successful, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        SectorRead reads 512 bytes of data from the card starting
 *                  at the sector address specified by sector_addr and stores
 *                  them in the location pointed to by 'buffer'.
 *
 * Note:            The device expects the address field in the command packet
 *                  to be byte address. Therefore the sector_addr must first
 *                  be converted to byte address. This is accomplished by
 *                  shifting the address left 9 times.
 *****************************************************************************/
//The flash memory is organized differently on the different microcontroller
//families.  Therefore, multiple versions of this function are implemented.

BYTE MDD_IntFlash_SectorRead(DWORD sector_addr, BYTE* buffer)
{
	ee_read_512(sector_addr,buffer);
    return TRUE;
}    



/******************************************************************************
 * Function:        BYTE SectorWrite(DWORD sector_addr, BYTE *buffer, BYTE allowWriteToZero)
 *
 * PreCondition:    None
 *
 * Input:           sector_addr - Sector address, each sector contains 512-byte
 *                  buffer      - Buffer where data will be read
 *                  allowWriteToZero - If true, writes to the MBR will be valid
 *
 * Output:          Returns TRUE if write successful, FALSE otherwise
 *
 * Side Effects:    None
 *
 * Overview:        SectorWrite sends 512 bytes of data from the location
 *                  pointed to by 'buffer' to the card starting
 *                  at the sector address specified by sector_addr.
 *
 * Note:            The sample device expects the address field in the command packet
 *                  to be byte address. Therefore the sector_addr must first
 *                  be converted to byte address. This is accomplished by
 *                  shifting the address left 9 times.
 *****************************************************************************/

BYTE MDD_IntFlash_SectorWrite(DWORD sector_addr, BYTE* buffer, BYTE allowWriteToZero)
{
	ee_write_512(sector_addr,buffer);
    return TRUE;
}


/******************************************************************************
 * Function:        BYTE WriteProtectState(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          BYTE    - Returns the status of the "write enabled" pin
 *
 * Side Effects:    None
 *
 * Overview:        Determines if the card is write-protected
 *
 * Note:            None
 *****************************************************************************/

BYTE MDD_IntFlash_WriteProtectState(void)
{
    #if defined(INTERNAL_FLASH_WRITE_PROTECT)
        return TRUE;
    #else
	    return FALSE;
    #endif
}

