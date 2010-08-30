#ifndef _MDDI_CLIENT_FPGA_NV_H_INCLUDED
#define _MDDI_CLIENT_FPGA_NV_H_INCLUDED

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                      FPGA_NV MDDI registers                  */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

//6.1.1 Instruction Code Table for Display
// Software Reset
#define     SWRESET                 0x010       
// Sleep In Mode      
#define     SLEPCTRL              0x110       
// Display On mode        
#define     DISPON                0x290       
// Window Horizontal RAM low Address Start     
#define     WHRAMLOWST             0x2A0       
// Window Horizontal RAM high Address Start      
#define     WHRAMHIGHST            0x2A1       
// Window Horizontal RAM low Address End
#define     WHRAMLOWEND          0x2A2       
// Window Horizontal RAM high Address End      
#define     WHRAMHIGHEND         0x2A3       
//    Window Vertical RAM low Address Start          
#define     WVRAMLOWST            0x2B0       
//    Window Vertical RAM high Address Start         
#define     WVRAMHIGHST            0x2B1       
//    Window Vertical RAM low Address End         
#define     WVRAMLOWEND            0x2B2       
//    Window Vertical RAM high Address End         
#define     WVRAMHIGHEND            0x2B3       
// Write Data to GRAM/Read Data from GRAM
#define     R_WRAM                 0x2C0       
// RAM low Address Set (Horizontal Address)
#define     RAMHSETL                0x2D0         
// RAM high Address Set (Horizontal Address)
#define     RAMHSETH                 0x2D1       
// RAM low Address Set (Vertical Address)         
#define     RAMVSETL                 0x2D2       
// RAM high Address Set (Vertical Address) 
#define     RAMVSETH              0x2D3       
// Defines the partial display area (300h~303h) 
#define     SET_PARTIAL_AREA     0x300         
// Frame Tearing Effect Position      
#define     FPOSITION             0x350        
//Frame Tearing Effect Position and Output cycle      
#define     FCYCLE                 0x351             
//Entry Mode1      
#define     ENTRYMODE1           0x360             
//Entry Mode 2     
#define     ENTRYMODE2          0x361                 
//i80 Interface Endian Control  
#define     I80CTRL             0x362             
// Resizing Control    
#define     RESCTRL                0x370             
//Driver Output Control 1 
#define     DRV_OUT_CTRL1        0x371             
//Driver Output Control 2 
#define     DRV_OUT_CTRL2        0x372             
// Interface Pixel Format 
#define     COLMOD                0x3A0             
//FPV_LDO Enable/Disable
#define     FPV_LDO             0x400             
// High pulse width of FIFO PWM
#define     FPWM_H                0x410             
// Low pulse width of FIFO PWM           
#define     FPWM_L                0x411                
// Numbers of PWM pulses for FIFO PWM            
#define     FPWM_N                0x412             
// The total duration of PWM event for FIFO PWM
#define     FPWM_TO                0x413             

//6.1.1 Instruction Code Table for Content Adaptive Backlight Control (CABC)
//Write Display Brightness             
#define     WRDISBV                0x510         
//Write CTRL Display                       
#define     WRCTRLD                0x530 
//Write Content Adaptive Brightness Control        
#define     WRCABC                 0x550     
//Write CABC Minimum Brightness             
#define     WRCABCMB               0x5E0 
//Read CABC Brightness                  
#define     RDPWM                  0x6A0                 
// PWM duty offset control
#define     PWMDOC                 0x6B0
//Write PWM Frequency                  
#define     WRPWMF                 0x6B1

#define     RDAPL           0x6C0
#define     CABCDUTY0        0x6D0
#define     CABCDUTY1        0x6D1
#define     CABCDUTY2        0x6D2
#define     CABCDUTY3        0x6D3
#define     CABCDUTY4        0x6D4
#define     CABCDUTY5        0x6D5
#define     CABCDUTY6        0x6D6
#define     CABCDUTY7        0x6D7
#define     CABCDUTY8        0x6D8


/*
             6C0h          RDAPL              0            0           0           APL[5]      APL[4]      APL[3]        APL[2]        APL[1]              APL[0]   003Fh
             6D0h          CABCDUTY             0                                                CABC_PWM0[7 : 0]                                                     00E6h
             6D1h          CABCDUTY             0                                                CABC_PWM1[7 : 0]                                                     00D9h
             6D2h          CABCDUTY             0                                                CABC_PWM2[7 : 0]                                                     00CCh
             6D3h          CABCDUTY             0                                                CABC_PWM3[7 : 0]                                                     00C0h
             6D4h          CABCDUTY             0                                                CABC_PWM4[7 : 0]                                                     00B3h
             6D5h          CABCDUTY             0                                                CABC_PWM5[7 : 0]                                                     00A6h
             6D6h          CABCDUTY             0                                                CABC_PWM6[7 : 0]                                                     0099h
             6D7h          CABCDUTY             0                                                CABC_PWM7[7 : 0]                                                     0099h
             6D8h          CABCDUTY             0                                                CABC_PWM8[7 : 0]                                                     0099h
*/
 
//Set CABC UI Mode Condition                 
#define     CABC_UI1               0x6DD                  
//Set CABC UI Mode Condition
#define     CABC_UI2               0x6DE                  


#define CABC_MOV 0x6e0
#define CABC_ADJ 0x6e1

/*
             6E0h         CABC_MOV              0            0           0             0           0                  MOV_MODE_GAMMA[3 : 0]                           0002h
             6E1h          CABC_ADJ             0            0                                            APL_TH[6 : 0]                                               0018h
*/


// Force CABC PWM in Some Conditions
#define     CABC_FORCE1            0x6E2
// Force CABC PWM in Some Conditions                    
#define     CABC_FORCE2            0x6E3  

#define MOVDET 0x6E4
#define MOVSC 0x6E5

/*
             6E4h           MOVDET              0            0                                            MOVDET[6 : 0]                                               0000h
             6E5h           MOVSC               0            0           0             0                              MOVSC[4 : 0]                                    0000h
*/

//CABCDMT: Set Dimming Time Length for CABC                  
#define     CABCDMT              0x6E6                  

//6.1.1 Instruction Code Table for Panel Setting
//Driver Output Control 
#define     DRVCTRL                0xB00       
// LCD Driving Wave Control
#define     LCDCTRL                0xB10      
//Display Control 
#define     DISPCTRL0               0xB20       

#define    PANELCTRL1             0xB30
#define    PANELCTRL2             0xB40
#define    PANELCTRL3             0xB41
#define    PANELCTRL4             0xB50
#define    PANELCTRL5             0xB51
#define    PANELCTRL6             0xB60
#define    PANELCTRL7             0xB70
#define    PANELCTRL8             0xB80
#define    PANELCTRL9             0xB90
#define    PUMPCTRL1              0xBA0
#define    PUMPCTRL2              0xBA1
#define    PANELCTRL10            0xBB0
#define    PANELCTRL11            0xBB1
#define    PANELCTRL12            0xBB2
#define    PANELCTRL13            0xBB3
#define    PANELCTRL14            0xBC0
#define    PANELCTRL15            0xBC1
#define    PANELCTRL16            0xBC2
#define    PANELCTRL17            0xBC3
#define    SOUTCTRL1              0xBE0
#define    SOUTCTRL2              0xBE1
#define    PWRCTRL1               0xBF0
#define    SPECTRL                0xC10

/*
             B30h       PANELCTRL1            0           DIV        RTN[6]         RTN[5]    RTN[4]     RTN[3]       RTN[2]      RTN[1]      RTN[0]       0046h
             B40h       PANELCTRL2            0            0         FS[6]          FS[5]     FS[4]       FS[3]        FS[2]       FS[1]       FS[0]       0007h
             B41h       PANELCTRL3            0          FW[7]       FW[6]          FW[5]     FW[4]       FW[3]       FW[2]        FW[1]       FW[0]      001Ch
             B50h       PANELCTRL4            0            0         STI[6]         STI[5]    STI[4]      STI[3]      STI[2]       STI[1]      STI[0]      000Fh
             B51h       PANELCTRL5            0         SWI[7]       SWI[6]         SWI[5]    SWI[4]      SWI[3]      SWI[2]      SWI[1]       SWI[0]     007Ah
             B60h       PANELCTRL6            0            0       MUXS [6]     MUXS [5]     MUXS [4]   MUXS [3]     MUXS [2]    MUXS [1]    MUXS [0]      0016h
             B70h       PANELCTRL7            0            0           0              0      MUXW [4]   MUXW [3]    MUXW [2]     MUXW [1]    MUXW [0]     000Dh
             B80h       PANELCTRL8            0            0           0              0      MUXG [4]   MUXG [3]     MUXG [2]    MUXG [1]    MUXG [0]      0004h
             B90h       PANELCTRL9            0            0           0            MCP[5]    MCP[4]     MCP [3]     MCP [2]      MCP [1]     MCP [0]      0007h
             BA0h        PUMPCTRL1             0        RCKT [7]     RCKT [6]    RCKT [5]     RCKT [4]    RCKT [3]    RCKT [2]    RCKT [1]     RCKT [0]     0004h
             BA1h        PUMPCTRL2             0        RCKW [7]    RCKW [6]     RCKW [5]     RCKW [4]   RCKW [3]     RCKW [2]    RCKW [1]    RCKW [0]      0086h
             BB0h        PANELCTRL10            0         SOE [8]     SOE [7]      SOE [6]     SOE [5]     SOE [4]      SOE [3]     SOE [2]     SOE [1]      00FFh
             BB1h        PANELCTRL11            0            0           0              0         0           0           0            0        SOE [9]      0001h
             BB2h        PANELCTRL12            0        SOE [17]     SOE [16]    SOE [15]     SOE [14]    SOE [13]    SOE [12]    SOE [11]     SOE [10]     00FFh
             BB3h        PANELCTRL13            0            0           0              0         0           0           0            0        SOE [18]     0001h
             BC0h        PANELCTRL14            0         SOP [8]     SOP [7]      SOP [6]     SOP [5]     SOP [4]      SOP [3]     SOP [2]     SOP [1]      0000h
             BC1h        PANELCTRL15            0            0           0              0         0           0           0            0        SOP [9]      0000h
             BC2h        PANELCTRL16            0         SOP[17]     SOP [16]    SOP [15]     SOP [14]    SOP [13]    SOP [12]    SOP [11]     SOP [10]     0000h
             BC3h        PANELCTRL17            0            0           0              0         0           0           0            0        SOP[18]      0000h
             BE0h        SOUTCTRL1             0            0           0              0         0           0           0         FSW[1]      FSW[0]       0000h
             BE1h        SOUTCTRL2             0            0           0         MOD[5]      MOD[4]      MOD[3]       MOD[2]      MOD[1]      MOD[0]       0000h
             BF0h        PWRCTRL1             0            0           0              0         0           0         AP [2]       AP [1]      AP [0]      0003h
             C10h        SPECTRL              0         nVCMS          0              0         0        VCMS[1]     VCMS[0]     VCMM [1]    VCMM [0]      0006h
*/

//Display Control 
////#define     DISPCTRL1               0xB21
//// RGB Interface Blanking Porch setting        
//#define     RGBPRCTR0               0xB22       
//#define     RGBPRCTR1               0xB23       
//#define     RGBPRCTR2               0xB24       
//#define     RGBPRCTR3               0xB25       
//// RGB Interface setting
//#define     RGBIF_SET              0xB26       
//// Panel Interface Control 0
//#define     PANELCTRL0             0xB30 
//// PANELCTRL10: Panel Interface Control 10       
//#define     PANELCTRL10            0xBB0     
//#define     PANELCTRL11            0xBB1         
// PANELCTRL12: Panel Interface Control 12
//#define     PANELCTRL12            0xBC0
//PANELCTRL13: Panel Interface Control 13         
//#define     PANELCTRL13            0xBC1
//Power Control 1         


//#define     PWRCTRL1               0xBF0   

//#define     SPECTRL              0xC10

    
//Power Control 2
#define     PWRCTRL2               0xC20       
//Power Control 3
#define     PWRCTRL3               0xC30       
//Power Control 4
#define     PWRCTRL4               0xC31       
//Power Control 5
#define     PWRCTRL5               0xC40        
//Power Control 6
#define     PWRCTRL6               0xC41        
//Positive VCOM Voltage Control 
#define     VCOMHCTRL              0xC50        
//VCOM Voltage Control
#define     VCOMLCTRL              0xC51 
//Negative VCOM Voltage Control        
#define     VCOMDCCTRL             0xC52
//VR Voltage Control         
//#define     VRCTRL                 0xC60      
//VCI2 Voltage Control 
#define     VCI2CTRL               0xC70 
// VGHOFF Voltage Control      
//#define     VGHOFFCTRL             0xC80         
// Device Code Read
#define     RDDID1_0               0xD00      
// Device Code Read
#define     RDDID1_1               0xD01      
//User ID Code Control 
#define     USERID                 0xD10      
//Revision ID Code
#define     REVISIONID             0xD20        
//Read NV Memory Flag Status1 
#define     RDVNT1                 0xD30      
//Read NV Memory Flag Status2
#define     RDVNT2                 0xD31
//NV Memory Write Command 1      
#define     EPWRITE1               0xDE0       
//NV Memory Write Command 2
#define     EPWRITE2               0xDE1       
// NV Memory Write Command 3
#define     EPWRITE3               0xDE2       
// MTP Write Function Enable
#define     MTPWR                  0xDe3      
//Red Gamma Control 1~16 (E00h~E0Fh) 
#define     GMACTRL1_0             0xE00        
#define     GMACTRL1_1             0xE01                
#define     GMACTRL1_2             0xE02                
#define     GMACTRL1_3             0xE03                
#define     GMACTRL1_4             0xE04                
#define     GMACTRL1_5             0xE05                
#define     GMACTRL1_6             0xE06                
#define     GMACTRL1_7             0xE07                
#define     GMACTRL1_8             0xE08                
#define     GMACTRL1_9             0xE09                
#define     GMACTRL1_A             0xE0A                
#define     GMACTRL1_B             0xE0B                
#define     GMACTRL1_C             0xE0C                
#define     GMACTRL1_D             0xE0D                
#define     GMACTRL1_E             0xE0E                
#define     GMACTRL1_F             0xE0F 
//Green Gamma Control 1~16 (E10h~E1Fh)                
#define     GMACTRL2_0             0xE10         
#define     GMACTRL2_1             0xE11                 
#define     GMACTRL2_2             0xE12                 
#define     GMACTRL2_3             0xE13                 
#define     GMACTRL2_4             0xE14                 
#define     GMACTRL2_5             0xE15                 
#define     GMACTRL2_6             0xE16                 
#define     GMACTRL2_7             0xE17                 
#define     GMACTRL2_8             0xE18                 
#define     GMACTRL2_9             0xE19                 
#define     GMACTRL2_A             0xE1A                 
#define     GMACTRL2_B             0xE1B                 
#define     GMACTRL2_C             0xE1C                 
#define     GMACTRL2_D             0xE1D                 
#define     GMACTRL2_E             0xE1E                 
#define     GMACTRL2_F             0xE1F  
//Green Gamma Control 1~16 (E10h~E1Fh)                
#define     GMACTRL3_0             0xE20         
#define     GMACTRL3_1             0xE21                 
#define     GMACTRL3_2             0xE22                 
#define     GMACTRL3_3             0xE23                 
#define     GMACTRL3_4             0xE24                 
#define     GMACTRL3_5             0xE25                 
#define     GMACTRL3_6             0xE26                 
#define     GMACTRL3_7             0xE27                 
#define     GMACTRL3_8             0xE28                 
#define     GMACTRL3_9             0xE29                 
#define     GMACTRL3_A             0xE2A                 
#define     GMACTRL3_B             0xE2B                 
#define     GMACTRL3_C             0xE2C                 
#define     GMACTRL3_D             0xE2D                 
#define     GMACTRL3_E             0xE2E                 
#define     GMACTRL3_F             0xE2F  
//Green Gamma Control 1~16 (E10h~E1Fh)                
#define     GMACTRL4_0             0xE30         
#define     GMACTRL4_1             0xE31                 
#define     GMACTRL4_2             0xE32                 
#define     GMACTRL4_3             0xE33                 
#define     GMACTRL4_4             0xE34                 
#define     GMACTRL4_5             0xE35                 
#define     GMACTRL4_6             0xE36                 
#define     GMACTRL4_7             0xE37                 
#define     GMACTRL4_8             0xE38                 
#define     GMACTRL4_9             0xE39                 
#define     GMACTRL4_A             0xE3A                 
#define     GMACTRL4_B             0xE3B                 
#define     GMACTRL4_C             0xE3C                 
#define     GMACTRL4_D             0xE3D                 
#define     GMACTRL4_E             0xE3E                 
#define     GMACTRL4_F             0xE3F  
// Blue Gamma Control 1~16 (E40h~E4Fh)                
#define     GMACTRL5_0             0xE40         
#define     GMACTRL5_1             0xE41                 
#define     GMACTRL5_2             0xE42                 
#define     GMACTRL5_3             0xE43                 
#define     GMACTRL5_4             0xE44                 
#define     GMACTRL5_5             0xE45                 
#define     GMACTRL5_6             0xE46                 
#define     GMACTRL5_7             0xE47                 
#define     GMACTRL5_8             0xE48                 
#define     GMACTRL5_9             0xE49                 
#define     GMACTRL5_A             0xE4A                 
#define     GMACTRL5_B             0xE4B                 
#define     GMACTRL5_C             0xE4C                 
#define     GMACTRL5_D             0xE4D                 
#define     GMACTRL5_E             0xE4E                 
#define     GMACTRL5_F             0xE4F

#define     GMACTRL6_0             0xE50         
#define     GMACTRL6_1             0xE51                 
#define     GMACTRL6_2             0xE52                 
#define     GMACTRL6_3             0xE53                 
#define     GMACTRL6_4             0xE54                 
#define     GMACTRL6_5             0xE55                 
#define     GMACTRL6_6             0xE56                 
#define     GMACTRL6_7             0xE57                 
#define     GMACTRL6_8             0xE58                 
#define     GMACTRL6_9             0xE59                 
#define     GMACTRL6_A             0xE5A                 
#define     GMACTRL6_B             0xE5B                 
#define     GMACTRL6_C             0xE5C                 
#define     GMACTRL6_D             0xE5D                 
#define     GMACTRL6_E             0xE5E                 
#define     GMACTRL6_F             0xE5F

#endif /* _MDDI_CLIENT_FPGA_NV_H_INCLUDED */

