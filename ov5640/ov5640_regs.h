#ifndef __OV5640_REGS_H__
#define __OV5640_REGS_H__

#define OV5640_REG_SYSCTL0 0x3008
#define OV5640_REG_CHIPID_MSB 0x300A
#define     OV5640_CHIPID_MSB    0x56
#define OV5640_REG_CHIPID_LSB 0x300B
#define     OV5640_CHIPID_LSB    0x40
#define     OV5640_CHIPID (OV5640_CHIPID_MSB << 16 | OV5640_CHIPID_LSB)



#define OV5640_PIXEL_ARRAY_WIDTH            2624
#define OV5640_PIXEL_ARRAY_HEIGHT           1964

#define OV5640_PIXEL_ARRAY_ACTIVE_WIDTH     2592
#define OV5640_PIXEL_ARRAY_ACTIVE_HEIGHT    1944

#define OV5640_PIXEL_ARRAY_ACTIVE_LEFT      16
#define OV5640_PIXEL_ARRAY_ACTIVE_TOP       14

#define OV5640_SYSCLK_FREQ_DEF              26600000

#define OV5640_REG_TIMING_TC_REG21  0x3821  // default 0x00 RW Bit[0]: Horizontal binning enable
#define OV5640_REG_CURRENT_TRANSIENT_RESPONSE_CONTROL   0x3602 // Bit[3:0]: current transient response control



//enable/disable bits, ENABLE = 1 and DISABLE = 0.
//system and IO pad control [0x3000 ~ 0x3052]
//address register name	default value	R/W	description
// 0x3000 SYSTEM RESET00 0x30 RW Reset for Individual Block (0: enable block; 1: reset block)
// Bit[7]: Reset BIST
// Bit[6]:Reset MCU program memory
// Bit[5]:Reset MCU
// Bit[4]:Reset OTP
// Bit[3]:Reset STB
// Bit[2]:Reset d5060
// Bit[1]:Reset timing control
// Bit[0]:Reset array control
#define OV5640_SYS_RST_00 0x3000

// 0x3001 SYSTEM RESET01 0x08 RW Reset for Individual Block (0: enable block; 1: reset block)
// Bit[7]:Reset AWB registers
// Bit[6]:Reset AFC
// Bit[5]:Reset ISP
// Bit[4]:Reset FC
// Bit[3]:Reset S2P
// Bit[2]:Reset BLC
// Bit[1]:Reset AEC registers
// Bit[0]:Reset AEC
#define OV5640_SYS_RST_01 0x3001

// 0x3002 SYSTEM RESET02 0x1C RW Reset for Individual Block  (0: enable block; 1: reset block)
// Bit[7]:Reset VFIFO
// Bit[6]:Debug mode
// Bit[5]:Reset format
// Bit[4]:Reset JFIFO
// Bit[3]:Reset SFIFO
// Bit[2]:Reset JPG
// Bit[1]:Reset format MUX
// Bit[0]:Reset average
#define OV5640_SYS_RST_02 0x3002

struct ov5640_reg
{
    u16 addr;
    u8 val;
};

static struct ov5640_reg init_table[] =
{
};


#endif//__OV5640_REGS_H__

