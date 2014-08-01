/*******************************************************************************
  MRF24W Config Data

  File Name: 
    drv_wifi_config_data.c  
  
  Summary:
    Module for the Microchip TCP/IP stack using WiFi MAC

  Description:
    - Stores and retrieves WiFi configuration information to non-volatile memory
      (NVM)
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/


#include <string.h>

#include "tcpip/src/tcpip_private.h"

#if defined(TCPIP_IF_MRF24W)


DRV_WIFI_CONFIG_DATA *p_wifi_ConfigData;


/****************************************************************************
  Section:
    Stack-Level Functions
  ***************************************************************************/
#if defined(WF_SAVE_CONFIG_TO_MEMORY)
	
	#if defined(WF_MEMORY_EXTERN_EEPROM)
			/*	********************************************* 
		#define WIFI_STORAGET_PARTITION_SIZE    (140)
			This setting "WIFI_STORAGET_PARTITION_SIZE" is the memory size for WiFi setting. 
			***************************************************/
			void WiFi_WriteConfigToMemory(void)
			{
	        #if defined(MEDIA_STORAGE_EEPROM)
				XEEBeginWrite(0);
				XEEWriteArray((uint8_t*)p_wifi_ConfigData, sizeof(DRV_WIFI_CONFIG_DATA));
			#endif
			}
			void WiFi_ReadConfigFromMemory(void)
			{
		    #if defined(MEDIA_STORAGE_EEPROM)
				XEEReadArray(0,(uint8_t*)p_wifi_ConfigData, sizeof(DRV_WIFI_CONFIG_DATA));
			#endif
			}
			void WiFi_EraseConfigFromMemory(void)
			{
				DRV_WIFI_CONFIG_DATA tmp_wifi_ConfigData;
				memset(&tmp_wifi_ConfigData,0x00,sizeof(DRV_WIFI_CONFIG_DATA));
				
			#if defined(MEDIA_STORAGE_EEPROM)
				XEEBeginWrite(0);
				XEEWriteArray((uint8_t*)&tmp_wifi_ConfigData, sizeof(DRV_WIFI_CONFIG_DATA));
			#endif
			}
	
    #elif defined(WF_MEMORY_INSIDE_FLASH)
#if 0
	    #define NVM_PROGRAM_PAGE 0xbd070000
			uint8_t buf_flash[100];
	
	
			//NVMErasePage((void*)NVM_PROGRAM_PAGE);
			//NVMWriteRow((void*)NVM_PROGRAM_PAGE, (void*)buf_flash);
			//p_data = (void*)NVM_PROGRAM_PAGE;
	
		
			void WiFi_WriteConfigToMemory(void)
			{
				if(sizeof(DRV_WIFI_CONFIG_DATA) > 512) 
					SYS_ASSERT(true, "p_wifi_ConfigData > 512");
				
				NVMErasePage((void*)NVM_PROGRAM_PAGE);
				NVMWriteRow((void*)NVM_PROGRAM_PAGE, (void*)p_wifi_ConfigData);
			}
			void WiFi_ReadConfigFromMemory(void)
			{
				int i;
				uint8_t *p_data1, *p_data2;
				p_data1 = (void*)NVM_PROGRAM_PAGE;
				p_data2 = (void*)p_wifi_ConfigData; 
				for(i=0;i<sizeof(DRV_WIFI_CONFIG_DATA);i++)
				{
					p_data2[i] = p_data1[i];
				}
			}
			void WiFi_EraseConfigFromMemory(void)
			{
				NVMErasePage((void*)NVM_PROGRAM_PAGE);
			}
#else
                        void wifi_nvm_write(uint8_t *data_config, int size);
                        void wifi_nvm_read(uint8_t *data_comfig, int size);
                        void wifi_nvm_erase(int size);
                        void WiFi_WriteConfigToMemory(void)
                        {
                        wifi_nvm_write((uint8_t*)p_wifi_ConfigData, sizeof(DRV_WIFI_CONFIG_DATA));
                        }
                        void WiFi_ReadConfigFromMemory(void)
                        {
                        wifi_nvm_read((uint8_t*)p_wifi_ConfigData,sizeof(DRV_WIFI_CONFIG_DATA));
                        }
                        void WiFi_EraseConfigFromMemory(void)
                        {
                        wifi_nvm_erase(sizeof(DRV_WIFI_CONFIG_DATA));
                        }
#endif
    #endif     //WF_MEMORY_INSIDE_FLASH
	
#else   //WF_SAVE_CONFIG_TO_MEMORY
    DRV_WIFI_CONFIG_DATA SaveWiFiConf_in_memory={0};
    void WiFi_WriteConfigToMemory(void)
    {
        int i;
        uint8_t *p_data1, *p_data2;
        p_data1 = (void*)p_wifi_ConfigData;
        p_data2 = (void*)&SaveWiFiConf_in_memory; 
        for(i=0;i<sizeof(DRV_WIFI_CONFIG_DATA);i++)
        {
            p_data2[i] = p_data1[i];
        }
    }
    void WiFi_ReadConfigFromMemory(void)
    {
        int i;
        uint8_t *p_data1, *p_data2;
        p_data1 = (void*)&SaveWiFiConf_in_memory;
        p_data2 = (void*)p_wifi_ConfigData; 
        for(i=0;i<sizeof(DRV_WIFI_CONFIG_DATA);i++)
        {
            p_data2[i] = p_data1[i];
        }        
    }
	void WiFi_EraseConfigFromMemory(void)
    {
        memset(&SaveWiFiConf_in_memory, 0, sizeof(SaveWiFiConf_in_memory));
    }

#endif  //WF_SAVE_CONFIG_TO_MEMORY
	
	
DRV_WIFI_CONFIG_DATA wifi_ConfigData;
DRV_WIFI_CONFIG_DATA *p_wifi_ConfigData = &wifi_ConfigData;


/*******************************************************************************
  Function:
    void DRV_WIFI_ConfigDataPrint(void);

  Summary:
    Outputs to console the configuration data from the board EEPROM.

  Description:
    This function outputs configuration data from the board EEPROM.

  Precondition:
    TCPIP stack should be initialized.

  Parameters:
    None

  Returns:
    None

  Remarks:
    None.
  *****************************************************************************/
void DRV_WIFI_ConfigDataPrint(void)
{
    int i; 
    char buf[10];

    SYS_CONSOLE_PRINT("Size of Configdata:  %lu\r\n", sizeof(wifi_ConfigData));
    SYS_CONSOLE_PRINT("networkType:         %d\r\n",  wifi_ConfigData.networkType);
    SYS_CONSOLE_PRINT("netSSID:             %s\r\n",  wifi_ConfigData.netSSID);
    SYS_CONSOLE_PRINT("SsidLength:          %d\r\n",  wifi_ConfigData.SsidLength);
    SYS_CONSOLE_PRINT("SecurityMode:        %d\r\n",  wifi_ConfigData.SecurityMode);
    //SYS_CONSOLE_PRINT("SecurityKey:         %s\r\n",  wifi_ConfigData.SecurityKey);
    {
        for ( i=0; i < wifi_ConfigData.SecurityKeyLength ; i++ )
        {
            sprintf(buf,"%.2x", wifi_ConfigData.SecurityKey[i]);
            SYS_CONSOLE_MESSAGE(buf);
        }
        SYS_CONSOLE_MESSAGE("\r\n");
    }
    SYS_CONSOLE_PRINT("SecurityKeyLength:   %d\r\n",  wifi_ConfigData.SecurityKeyLength);
}


/*******************************************************************************
  Function:
    bool DRV_WIFI_ConfigDataErase(void);

  Summary:
    Erases configuration data from the board EEPROM.

  Description:
    This function erases configuration data from the board EEPROM.

 Parameters:
    None

  Returns:
    None
 *****************************************************************************/
bool DRV_WIFI_ConfigDataErase(void)
{
    WiFi_EraseConfigFromMemory();
    return true;
}

#if 1

#include <sys/attribs.h>
#include "driver/nvm/drv_nvm.h"
#if defined(__32MX795F512L__)
#define     NVM_WF_CONFIG__ADDRESS           0x9d07e000
#define     NVM_CODE_BEGIN_ADDR        0x9d070000
#define     NVM_CODE_END_ADDR          0x9d080000
#elif defined(__32MZ2048ECH144__) || defined (__32MZ2048ECM144__)
#define     NVM_WF_CONFIG__ADDRESS           0x9d1fe000
#define     NVM_CODE_BEGIN_ADDR        0x9d000000
#define     NVM_CODE_END_ADDR          0x9d1fffff
#else
#error "please reference datasheet, to assign the correct space"
#endif

const uint8_t NVM_DATA_FLASH_RESERVE[DRV_NVM_PAGE_SIZE*2] __attribute__((address(NVM_WF_CONFIG__ADDRESS))) = {0};

typedef struct
{
    DRV_HANDLE              nvmHandle;
    DRV_NVM_BUFFER_HANDLE nvmbufferHandle;
} WIFI_NVM_HANDLE;
WIFI_NVM_HANDLE wifi_nvm_handle={0};

void wifi_nvm_read(uint8_t *data_comfig, int size)
{
    if(wifi_nvm_handle.nvmHandle == 0)
     {
         wifi_nvm_handle.nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READWRITE);
         if(DRV_HANDLE_INVALID == wifi_nvm_handle.nvmHandle){ SYS_ASSERT(DRV_HANDLE_INVALID != wifi_nvm_handle.nvmHandle,"Error Opening Driver");}
     }

     wifi_nvm_handle.nvmbufferHandle = DRV_NVM_Read(wifi_nvm_handle.nvmHandle, data_comfig, (uint8_t*)NVM_WF_CONFIG__ADDRESS, size);
     if(DRV_NVM_BUFFER_HANDLE_INVALID == wifi_nvm_handle.nvmbufferHandle){ SYS_ASSERT(false, "Driver Read Failed");}
     while(DRV_NVM_BUFFER_COMPLETED != DRV_NVM_BufferStatus(wifi_nvm_handle.nvmHandle, wifi_nvm_handle.nvmbufferHandle)){;}
}
void wifi_nvm_write(uint8_t *data_config, int size)
{
    if(wifi_nvm_handle.nvmHandle == 0)
    {
        wifi_nvm_handle.nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READWRITE);
        if(DRV_HANDLE_INVALID == wifi_nvm_handle.nvmHandle){ SYS_ASSERT(DRV_HANDLE_INVALID != wifi_nvm_handle.nvmHandle,"Error Opening Driver");}
    }
    wifi_nvm_handle.nvmbufferHandle = DRV_NVM_Erase(wifi_nvm_handle.nvmHandle,(uint8_t*)NVM_WF_CONFIG__ADDRESS, DRV_NVM_PAGE_SIZE);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == wifi_nvm_handle.nvmbufferHandle){SYS_ASSERT(false, "Driver Erase Failed");}
    while(DRV_NVM_BUFFER_COMPLETED != DRV_NVM_BufferStatus(wifi_nvm_handle.nvmHandle, wifi_nvm_handle.nvmbufferHandle))  { ;}

    wifi_nvm_handle.nvmbufferHandle = DRV_NVM_Write(wifi_nvm_handle.nvmHandle, (uint8_t *)(NVM_WF_CONFIG__ADDRESS), (uint8_t *)data_config, size);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == wifi_nvm_handle.nvmbufferHandle){SYS_ASSERT(false, "Driver Writing Failed");}
    while(DRV_NVM_BUFFER_COMPLETED != DRV_NVM_BufferStatus(wifi_nvm_handle.nvmHandle, wifi_nvm_handle.nvmbufferHandle)){;}


}
void wifi_nvm_erase(int size)
{
    if(wifi_nvm_handle.nvmHandle == 0)
    {
        wifi_nvm_handle.nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READWRITE);
        if(DRV_HANDLE_INVALID == wifi_nvm_handle.nvmHandle){ SYS_ASSERT(DRV_HANDLE_INVALID != wifi_nvm_handle.nvmHandle,"Error Opening Driver");}
    }
    wifi_nvm_handle.nvmbufferHandle = DRV_NVM_Erase(wifi_nvm_handle.nvmHandle,(uint8_t*)NVM_WF_CONFIG__ADDRESS, DRV_NVM_PAGE_SIZE);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == wifi_nvm_handle.nvmbufferHandle){SYS_ASSERT(false, "Driver Erase Failed");}
    while(DRV_NVM_BUFFER_COMPLETED != DRV_NVM_BufferStatus(wifi_nvm_handle.nvmHandle, wifi_nvm_handle.nvmbufferHandle))  { ;}
}
#if 0 // for test
uint8_t NVM_DATA_TEST_BUFF[DRV_NVM_PAGE_SIZE] = {0};

void APP_NVM_Info(void)
{
    const uint8_t  *addr1, *addr2, *addr3;
    char buf_t[30];
    addr1 = &NVM_DATA_FLASH_RESERVE[0];
    addr2 = &NVM_DATA_FLASH_RESERVE[DRV_NVM_PAGE_SIZE*2 - 1];
    addr3 = (uint8_t*)NVM_WF_CONFIG__ADDRESS;

    SYS_CONSOLE_MESSAGE("\r\nadd is: ");
    sprintf(buf_t,"addr1 = 0x%x\r\n",addr1); SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t,"addr2 = 0x%x\r\n",addr2); SYS_CONSOLE_MESSAGE(buf_t);
    sprintf(buf_t,"addr3 = 0x%x\r\n",addr3); SYS_CONSOLE_MESSAGE(buf_t);
}


int jian_flag = 0;

void APP_NVM_READ(uint32_t flag)
{
    int i;
    uint8_t buf_t[30];
    uint32_t Source_Addr;
    switch(flag)
        {
        case 0:
            Source_Addr = NVM_WF_CONFIG__ADDRESS - DRV_NVM_PAGE_SIZE;
            break;
        case 1:
            Source_Addr = NVM_WF_CONFIG__ADDRESS ;
            break;
        case 2:
            Source_Addr = NVM_WF_CONFIG__ADDRESS + DRV_NVM_PAGE_SIZE;
            break;
        case 3:
            Source_Addr = NVM_WF_CONFIG__ADDRESS + 2*DRV_NVM_PAGE_SIZE;
            break;
        case 4:
            Source_Addr = NVM_WF_CONFIG__ADDRESS + 3*DRV_NVM_PAGE_SIZE;
            break;
        case 5:
            Source_Addr = NVM_WF_CONFIG__ADDRESS + 4*DRV_NVM_PAGE_SIZE;
            break;
        default:
            Source_Addr = NVM_WF_CONFIG__ADDRESS;
            break;
        }
    if(wifi_nvm_handle.nvmHandle == 0)
    {
        wifi_nvm_handle.nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READWRITE);
        if(DRV_HANDLE_INVALID == wifi_nvm_handle.nvmHandle){ SYS_ASSERT(DRV_HANDLE_INVALID != wifi_nvm_handle.nvmHandle,"Error Opening Driver");}
    }

    wifi_nvm_handle.nvmbufferHandle = DRV_NVM_Read(wifi_nvm_handle.nvmHandle, NVM_DATA_TEST_BUFF, (uint8_t*)Source_Addr, DRV_NVM_PAGE_SIZE);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == wifi_nvm_handle.nvmbufferHandle){ SYS_ASSERT(false, "Driver Read Failed");}
    while(DRV_NVM_BUFFER_COMPLETED != DRV_NVM_BufferStatus(wifi_nvm_handle.nvmHandle, wifi_nvm_handle.nvmbufferHandle)){;}

    SYS_CONSOLE_MESSAGE("\r\n---------");
    for(i=0;i<DRV_NVM_PAGE_SIZE;i++)
    {
        if(i%64 == 0) {sprintf(buf_t,"\r\n[%08x]:",i+Source_Addr);SYS_CONSOLE_MESSAGE(buf_t);}
        sprintf(buf_t,"%02x ",NVM_DATA_TEST_BUFF[i]);
        SYS_CONSOLE_MESSAGE(buf_t);
    }

    SYS_CONSOLE_MESSAGE("\r\n---------");
}
void APP_NVM_Write1(void)
{
    int i;
    uint8_t buf_t[10];

    char c_tmp = 0;
    for(i = 0;i<DRV_NVM_PAGE_SIZE;i++)
    {
        NVM_DATA_TEST_BUFF[i] = 0x55;
    }
    SYS_CONSOLE_MESSAGE("1. Open\r\n");
    if(wifi_nvm_handle.nvmHandle == 0)
    {
        wifi_nvm_handle.nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READWRITE);
        if(DRV_HANDLE_INVALID == wifi_nvm_handle.nvmHandle){ SYS_ASSERT(DRV_HANDLE_INVALID != wifi_nvm_handle.nvmHandle,"Error Opening Driver");}
    }
    SYS_CONSOLE_MESSAGE("2. erase\r\n");
    wifi_nvm_handle.nvmbufferHandle = DRV_NVM_Erase(wifi_nvm_handle.nvmHandle,(uint8_t*)NVM_WF_CONFIG__ADDRESS, DRV_NVM_PAGE_SIZE);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == wifi_nvm_handle.nvmbufferHandle){SYS_ASSERT(false, "Driver Erase Failed");}
    while(DRV_NVM_BUFFER_COMPLETED != DRV_NVM_BufferStatus(wifi_nvm_handle.nvmHandle, wifi_nvm_handle.nvmbufferHandle))  { ;}
    SYS_CONSOLE_MESSAGE("3. Write\r\n");
    wifi_nvm_handle.nvmbufferHandle = DRV_NVM_Write(wifi_nvm_handle.nvmHandle, (uint8_t *)(NVM_WF_CONFIG__ADDRESS), (uint8_t *)NVM_DATA_TEST_BUFF, DRV_NVM_ROW_SIZE);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == wifi_nvm_handle.nvmbufferHandle){SYS_ASSERT(false, "Driver Writing Failed");}
    while(DRV_NVM_BUFFER_COMPLETED != DRV_NVM_BufferStatus(wifi_nvm_handle.nvmHandle, wifi_nvm_handle.nvmbufferHandle)){;}
    SYS_CONSOLE_MESSAGE("4. Done\r\n");
}

void APP_NVM_Write2(void)
{
    int i;
    uint8_t buf_t[10];

    char c_tmp = 0;
    for(i = 0;i<DRV_NVM_PAGE_SIZE;i++)
    {
        NVM_DATA_TEST_BUFF[i] = 0x55;
    }
    
		SYS_CONSOLE_MESSAGE("2. erase\r\n");		
		NVMErasePage((void*)NVM_WF_CONFIG__ADDRESS);
        SYS_CONSOLE_MESSAGE("3. Write\r\n");
		NVMWriteRow((void*)NVM_WF_CONFIG__ADDRESS, (void*)NVM_DATA_TEST_BUFF);
		SYS_CONSOLE_MESSAGE("4. Done\r\n");	
}





 int Command_nvm_info(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    APP_NVM_Info();
    return true;
}


 int Command_nvm_read(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    int flag_nvm = 0;
    SYS_CONSOLE_MESSAGE("nvm_read1\r\n");
    if(argc <2) flag_nvm = 0;
    else
    {
        SYS_CONSOLE_MESSAGE(argv[1]);
        sscanf(argv[1],"%x",&flag_nvm);
    }
    {
        char buf_t[30];
        sprintf(buf_t,"get flag=0x%x\r\n",flag_nvm);
        SYS_CONSOLE_MESSAGE(buf_t);
    }

    APP_NVM_READ(flag_nvm);
    return true;
}

 int Command_nvm_write1(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Command_nvm_write1\r\n");
    APP_NVM_Write1();
    return true;
}
 int Command_nvm_write2(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Command_nvm_write2\r\n");
    APP_NVM_Write2();
    return true;
}


#endif
void APP_NVM_ReadALL(void)
{
 
	uint32_t i;
    char buf_t[30];
	uint8_t *p_data1;
    for(i=0x9d070000;i<0x9D080000;i++)
    {
        p_data1 = (void*)i;
        if(i%64 == 0) {sprintf(buf_t,"\r\n[%08x]:",i);SYS_CONSOLE_MESSAGE(buf_t);}
        sprintf(buf_t,"%02x ",*p_data1);
        SYS_CONSOLE_MESSAGE(buf_t);
    }
}

 int Command_nvm_readall(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{

    APP_NVM_ReadALL();
    return true;
}


#endif
#endif // TCPIP_IF_MRF24W

//DOM-IGNORE-END


