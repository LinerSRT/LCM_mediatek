/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
   BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h> 
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>
#endif
#include <cust_gpio_usage.h>
#ifndef FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif
#ifdef BUILD_LK
#include <debug.h>
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

static const unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    


#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define HX8395 0

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0XFFFE
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE									0

#define LCM_ID_OTM1284 0x40

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/


	//must use 0x39 for init setting for all register.

	
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0x12,0x87,0x01}},	//EXTC=1
	{0x00,	1,	{0x80}},			//Orise mode enable
	{0xff,	2,	{0x12,0x87}},

	{0x00,	1,	{0x92}},
	{0xff,	2,	{0x20,0x02}},		//MIPI 4 Lane  20=3 lane

	{0x00,	1,	{0x80}},			 //TCON Setting
	{0xC0,	9,	{0x00,0x64,0x00, 0x10,0x10,0x00,0x64,0x10,0x10}}, 

	{0x00,	1,	{0x90}},			 //Panel Timing Setting
	{0xc0, 6, {0x00,0x5c,0x00,0x01,0x00,0x04}},
	
		{0x00,	1,	{0xb3}},			 //Interval Scan Frame: 0 frame, column inversion
	{0xc0,2,{0x00,0x55}},
	
		{0x00,	1,	{0x81}},			 //frame rate:60Hz
	{0xc1, 1, {0x66}},

//-------------------- power setting --------------------//
	{0x00,	1,	{0xa0}},			 //dcdc setting
	{0xC4,	14, {0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}}, 

	{0x00,	1,	{0xb0}},			 //clamp voltage setting
	{0xc4,	2,	{0x00,0x00}},

	{0x00,	1,	{0x91}},			 //VGH=16V, VGL=-12V, pump ratio:VGH=8x, VGL=-5x
	{0xc5, 2, {0x69,0xd2}},

	{0x00,	1,	{0x00}},			 //GVDD=4.900V, NGVDD=-4.900V
	{0xd8,2,{0xc7,0xc7}},     /////////////////////////////////////////////////////////////

	{0x00, 1, {0x00}},
	{0xd9, 1, {0x5d}},

	{0x00,	1,	{0xb3}},			 //VDD_18V=1.7V, LVDSVDD=1.6V
	{0xc5,	1,	{0x84}},

	{0x00,	1,	{0xbb}},			 //LVD voltage level setting
	{0xc5,	1,	{0x8a}},
	
	{0x00, 1, {0x80}},
	{0xc4, 1, {0x01}},

	{0x00, 1, {0x88}},
	{0xc4, 1, {0x01}},

	{0x00,	1,	{0xc2}},			 //LVD voltage level setting
	{0xf5, 1, {0xc0}},

	{0x00,	1,	{0x82}},		//chopper
	{0xC4,	1,	{0x0a}},

	{0x00,	1,	{0xc6}},		//debounce
	{0xb0,	1,	{0x03}},

//-------------------- control setting --------------------//
	{0x00,	1,	{0x00}},			 //ID1
	{0xd0,	1,	{0x40}},

	{0x00,	1,	{0x00}},			 //ID2, ID3
	{0xd1,	2,	{0x00,0x00}},

//-------------------- power on setting --------------------//
	{0x00,	1,	{0xb2}},			 //VGLO1
	{0xf5,	2,	{0x00,0x00}},

	{0x00,	1,	{0xb6}},			 //VGLO2
	{0xf5,	2,	{0x00,0x00}},

	{0x00,	1,	{0x94}},			 //VCL pump dis
	{0xf5,	2,	{0x00,0x00}},

	{0x00,	1,	{0xd2}},			 //VCL reg. en
	{0xf5,	2,	{0x06,0x15}},

	{0x00,	1,	{0xb4}},			 //VGLO1/2 Pull low setting
	{0xc5,	1,	{0xcc}},		   //d[7] vglo1 d[6] vglo2 => 0: pull vss, 1: pull vgl

//------------------ for Power IC ---------------------------------
	{0x00,	1,	{0x90}},			 //Mode-3
	{0xf5,	4,	{0x02,0x11,0x02,0x15}},

	{0x00,	1,	{0x90}},			 //2xVPNL, 1.5*=00, 2*=50, 3*=a0
	{0xc5,	1,	{0x50}},

	{0x00,	1,	{0x94}},			 //Frequency
	{0xc5,	1,	{0x66}},

//-------------------- panel timing state control --------------------//
	{0x00,	1,	{0x80}},			 //panel timing state control
	{0xCB,	11, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,	1,	{0x90}},			 //panel timing state control
	{0xCB,	15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,	1,	{0xA0}},			 //panel timing state control
	{0xCB,	15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,	1,	{0xB0}},			 //panel timing state control
	{0xCB,	15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,	1,	{0xC0}},			 //panel timing state control
	{0xCB,	15, {0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x05,0x00,0x00,0x00,0x00}},

	{0x00,	1,	{0xD0}},			 //panel timing state control
	{0xCB,	15, {0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05}},

	{0x00,	1,	{0xE0}},			 //panel timing state control
	{0xCB,	14, {0x05,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00}},

	{0x00,	1,	{0xF0}},			 //panel timing state control
	{0xCB,	11, {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

//-------------------- panel pad mapping control --------------------//
	{0x00,	1,	{0x80}},			 //panel pad mapping control
	{0xCC,	15, {0x29,0x2a,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x06,0x00,0x08,0x00,0x00,0x00,0x00}},

	{0x00,	1,	{0x90}},			 //panel pad mapping control
	{0xCC,	15, {0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x29,0x2a,0x09,0x0b,0x0d,0x0f,0x11,0x13}},

	{0x00,	1,	{0xA0}},			 //panel pad mapping control
	{0xCC,	14, {0x05,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00}},

	{0x00,	1,	{0xB0}},			 //panel pad mapping control
	{0xCC,	15, {0x29,0x2a,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x01,0x00,0x07,0x00,0x00,0x00,0x00}},

	{0x00,	1,	{0xC0}},			 //panel pad mapping control
	{0xCC,	15, {0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x29,0x2a,0x14,0x12,0x10,0x0e,0x0c,0x0a}},

	{0x00,	1,	{0xD0}},			 //panel pad mapping control
	{0xCC,	14, {0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00}},

//-------------------- panel timing setting --------------------//
	{0x00,1,{0x80}},            //panel VST setting
	{0xce, 12, {0x89,0x05,0x00,0x88,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x90}},             //panel VEND setting
	{0xce, 14, {0x54,0xfc,0x00,0x54,0xfd,0x00,0x55,0x00,0x00,0x55,0x01,0x00,0x00,0x00}},

	{0x00,1,{0xa0}},            //panel CLKA1/2 setting
	{0xce, 14, {0x58,0x07,0x05,0x08,0x00,0x00,0x00,0x58,0x06,0x05,0x09,0x00,0x00,0x00}},

	{0x00,1,{0xb0}},             //panel CLKA3/4 setting
	{0xce, 14, {0x58,0x05,0x05,0x0a,0x00,0x00,0x00,0x58,0x04,0x05,0x0b,0x00,0x00,0x00}},

	{0x00,1,{0xc0}},            //panel CLKb1/2 setting
	{0xce, 14, {0x58,0x03,0x05,0x0c,0x00,0x00,0x00,0x58,0x02,0x05,0x0d,0x00,0x00,0x00}},

	{0x00,1,{0xd0}},             //panel CLKb3/4 setting
	{0xce, 14, {0x58,0x01,0x05,0x0e,0x00,0x00,0x00,0x58,0x00,0x05,0x0f,0x00,0x00,0x00}},

	{0x00,1,{0x80}},            //panel CLKc1/2 setting
	{0xcf, 14, {0x50,0x00,0x05,0x10,0x00,0x00,0x00,0x50,0x01,0x05,0x11,0x00,0x00,0x00}},

	{0x00,1,{0x90}},            //panel CLKc3/4 setting
	{0xcf, 14, {0x50,0x02,0x05,0x12,0x00,0x00,0x00,0x50,0x03,0x05,0x13,0x00,0x00,0x00}},

	{0x00,1,{0xa0}},            //panel CLKd1/2 setting
	{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xb0}},            //panel CLKd3/4 setting
	{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xc0}},             //panel ECLK setting
	{0xcf, 11, {0x3d,0x3d,0x20,0x20,0x00,0x00,0x01,0x01,0x20,0x00,0x00}},

//---------------------------------------------------------------------------------//
	{0x00,	1,	{0xb5}},
	{0xc5,	6,	{0x0b,0x95,0xff,0x0b,0x95,0xff}},

	{0x00, 1, {0x00}},
	{0xe1, 20, {0x11,0x2f,0x3e,0x4e,0x5e,0x6d,0x70,0x99,0x8a,0xa2,0x64,0x4f,0x64,0x44,0x44,0x38,0x2b,0x1d,0x11,0x11}},

	{0x00,	1,	{0x00}},
	{0xe2, 20, {0x05,0x2f,0x3e,0x4d,0x5d,0x6c,0x70,0x99,0x8a,0xa2,0x64,0x4f,0x64,0x44,0x44,0x38,0x2b,0x1d,0x11,0x07}},

	{0x00, 1, {0x92}},
	{0xb3, 1, {0x02}},

	{0x00, 1, {0x90}},
	{0xb6, 1, {0xb6}},

	{0x00, 1, {0xa0}},
	{0xc1, 1, {0x02}},

	{0x00,	1,	{0x00}}, //Orise mode disable
	{0xff,	3,	{0xff,0xff,0xff}},

	{0x11,	0,	{}},
	{REGFLAG_DELAY, 120, {}},
	
	{0x29,	0,	{}},
	{REGFLAG_DELAY, 20, {}},
 
	{REGFLAG_END_OF_TABLE, 0x00, {}},

};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 150, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_backlight_level_setting[] = {
{0x51, 1, {0xFF}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

static void lcm_init_registers()
{	
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_THREE_LANE;
        //The following defined the fomat for data coming from LCD engine.
        params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
        params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
        params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
        params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

        // Highly depends on LCD driver capability.
         params->dsi.packet_size=256;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 12;
		params->dsi.vertical_frontporch					= 18;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		params->dsi.horizontal_sync_active				= 2;
		params->dsi.horizontal_backporch				= 42;
		params->dsi.horizontal_frontporch				= 44;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		
        params->dsi.PLL_CLOCK = 250; //this value must be in MTK suggested table
        
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd			= 0x0a;
		params->dsi.lcm_esd_check_table[0].count		= 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id1 = 0;
	unsigned int id2 = 0;
	unsigned int id = 0;
	
	unsigned char buffer[5];
	unsigned int array[16];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50);

	//push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xda, buffer, 1);

	id1 = buffer[0]; //we only need ID
	id = id1;
	
#ifdef BUILD_LK
	dprintf(CRITICAL,"%s,  id otm1284A= 0x%08x\n", __func__, id);
#endif

	return (LCM_ID_OTM1284 == id)?1:0;


}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);//5
    SET_RESET_PIN(0);
	MDELAY(120);//50
    SET_RESET_PIN(1);
	MDELAY(10);//100

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
} 


static void lcm_suspend(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);//5
    SET_RESET_PIN(0);
	MDELAY(20);//50
    SET_RESET_PIN(1);
	MDELAY(100);

	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(0);
	MDELAY(50);//50
}



static void lcm_resume(void)
{

	lcm_init();
//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);	
}

#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif

static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];
	int ret = 0;
	
	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);
	if(buffer[0]!=0x9C)
	{
		printk("[LCM ERROR] [0x9c]=0x%02x\n", buffer[0]);
		return TRUE;
	}

	return FALSE;
#else
	return FALSE;
 #endif
}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	return TRUE;
}


static void lcm_setbacklight(unsigned int level)
{

}

LCM_DRIVER otm1287a_hd720_dsi_vdo_dijing_lcm_drv = 
{
    .name			= "otm1287a_dsi_vdo_dijing",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power		= lcm_init_power,
//  .resume_power = lcm_resume_power,
//  .suspend_power = lcm_suspend_power,
	.set_backlight	= lcm_setbacklight,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
