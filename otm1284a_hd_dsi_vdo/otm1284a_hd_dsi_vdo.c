#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

#if 0
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <linux/xlog.h>
#include <mach/mt_pm_ldo.h>
#endif
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  			(720)
#define FRAME_HEIGHT 			(1280)
#define LCM_OTM1283_ID			(0x1283)

#define REGFLAG_DELAY          		(0XFEE)
#define REGFLAG_END_OF_TABLE      	(0xFFFF)	// END OF REGISTERS MARKER

#ifndef FALSE
#define FALSE 0
#endif
//#define GPIO_LCD_RST_EN      (GPIO112)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    		(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update)		(lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd)						(lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums)			(lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg						(lcm_util.dsi_read_reg())
#define read_reg_v2(cmd, buffer, buffer_size)   		(lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

static struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static void lcm_suspend(void);

static struct LCM_setting_table lcm_initialization_setting[] = {
                  
//qicai otm1284a auo hd                                                                                               

	{0x00,1,{0x00}},
	{0xff,3,{0x12,0x84,0x01}},	//EXTC=1
	{0x00,1,{0x80}},	        //Orise mode enable
	{0xff,2,{0x12,0x84}},

	{0x00,1,{0xa6}},             //zigzag setting
	{0xb3,1,{0x0f}},

//-------------------- panel setting --------------------//
	{0x00,1,{0x80}},             //TCON Setting
	{0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},

	{0x00,1,{0x90}},             //Panel Timing Setting
  {0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},

	{0x00,1,{0xa2}},
	{0xC0,3,{0x01,0x00,0x00}},

	{0x00,1,{0xb3}},             //Interval Scan Frame: 0 frame, column inversion
	{0xc0,2,{0x00,0x55}},


        {0x00,1,{0xb5}},             
	{0xc0,1,{0x18}},

	{0x00,1,{0x81}},             //frame rate:60Hz
	{0xc1,1,{0x55}},

//-------------------- power setting --------------------//
	{0x00,1,{0xa0}},             //dcdc setting
	{0xc4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},

	{0x00,1,{0xb0}},             //clamp voltage setting
	{0xc4,2,{0x00,0x00}},

	{0x00,1,{0x91}},             //VGH=13V, VGL=-12V, pump ratio:VGH=6x, VGL=-5x
	{0xc5,2,{0x29,0x52}},

	{0x00,1,{0x00}},             //GVDD=4.9V, NGVDD=-4.9V
	{0xd8,2,{0xbe,0xbe}},             

	{0x00,1,{0x00}}, 		//VCOM=-1.15V           
	{0xd9,1,{0x59}},             

	{0x00,1,{0xb3}},             //VDD_18V=1.7V, LVDSVDD=1.6V
	{0xc5,1,{0x84}},

	{0x00,1,{0xbb}},             //LVD voltage level setting
	{0xc5,1,{0x8a}},

	{0x00,1,{0x82}},		//chopper
	{0xC4,1,{0x0a}},

	{0x00,1,{0xc6}},		//debounce
	{0xb0,1,{0x03}},

	{0x00,1,{0xc2}},             //precharge disable
	{0xf5,1,{0x40}},

	{0x00,1,{0xc3}},             //sample hold gvdd
	{0xf5,1,{0x85}},

//-------------------- control setting --------------------//
	{0x00,1,{0x00}},             //ID1
	{0xd0,1,{0x40}},

	{0x00,1,{0x00}},             //ID2, ID3
	{0xd1,2,{0x00,0x00}},

//-------------------- power on setting --------------------//
	{0x00,1,{0x80}},             //source blanking frame = black, defacult='30'
	{0xc4,1,{0x00}},

	{0x00,1,{0x98}},             //vcom discharge=gnd:'10', '00'=disable
	{0xc5,1,{0x10}},

	{0x00,1,{0x81}},
	{0xf5,1,{0x15}},  // ibias off
	{0x00,1,{0x83}}, 
	{0xf5,1,{0x15}},  // lvd off
	{0x00,1,{0x85}},
	{0xf5,1,{0x15}},  // gvdd off
	{0x00,1,{0x87}}, 
	{0xf5,1,{0x15}},  // lvdsvdd off
	{0x00,1,{0x89}},
	{0xf5,1,{0x15}},  // nvdd_18 off
	{0x00,1,{0x8b}}, 
	{0xf5,1,{0x15}},  // en_vcom off

	{0x00,1,{0x95}},
	{0xf5,1,{0x15}},  // pump3 off
	{0x00,1,{0x97}}, 
	{0xf5,1,{0x15}},  // pump4 off
	{0x00,1,{0x99}},
	{0xf5,1,{0x15}},  // pump5 off

	{0x00,1,{0xa1}}, 
	{0xf5,1,{0x15}},  // gamma off
	{0x00,1,{0xa3}},
	{0xf5,1,{0x15}},  // sd ibias off
	{0x00,1,{0xa5}}, 
	{0xf5,1,{0x15}},  // sdpch off
	{0x00,1,{0xa7}}, 
	{0xf5,1,{0x15}},  // sdpch bias off
	{0x00,1,{0xab}},
	{0xf5,1,{0x18}},  // ddc osc off

	{0x00,1,{0x94}},             //VCL pump dis
	{0xf5,2,{0x00,0x00}},

	{0x00,1,{0xd2}},             //VCL reg. en
	{0xf5,2,{0x06,0x15}},

	{0x00,1,{0xb1}},             //VGLO, VGHO setting
	{0xf5,13,{0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x15,0x08,0x15}},

	{0x00,1,{0xb4}},             //VGLO1/2 Pull low setting
	{0xc5,1,{0xcc}},		//d[7] vglo1 d[6] vglo2 => 0: pull vss, 1: pull vgl

//-------------------- for Power IC ---------------------------------
	{0x00,1,{0x90}},             //Mode-3
	{0xf5,4,{0x02,0x11,0x02,0x15}},

	{0x00,1,{0x90}},             //2xVPNL, 1.5*=00, 2*=50, 3*=a0
	{0xc5,1,{0x50}},

	{0x00,1,{0x94}},             //Frequency
	{0xc5,1,{0x66}},

//-------------------- panel timing state control --------------------//
	{0x00,1,{0x80}},             //panel timing state control
	{0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x90}},             //panel timing state control
	{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff,0x00}},

	{0x00,1,{0xa0}},             //panel timing state control
	{0xcb,15,{0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xb0}},             //panel timing state control
	{0xcb,15,{0x00,0x00,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xc0}},             //panel timing state control
	{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x05,0x05,0x05,0x05,0x05}},

	{0x00,1,{0xd0}},             //panel timing state control
	{0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05}},

	{0x00,1,{0xe0}},             //panel timing state control
	{0xcb,14,{0x05,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00}},

	{0x00,1,{0xf0}},             //panel timing state control
	{0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

//-------------------- panel pad mapping control --------------------//
	{0x00,1,{0x80}},             //panel pad mapping control
	{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x07,0x00,0x0d,0x09,0x0f,0x0b,0x11}},

	{0x00,1,{0x90}},             //panel pad mapping control
	{0xcc,15,{0x15,0x13,0x17,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06}},

	{0x00,1,{0xa0}},             //panel pad mapping control
	{0xcc,14,{0x08,0x00,0x0e,0x0a,0x10,0x0c,0x12,0x16,0x14,0x18,0x02,0x04,0x00,0x00}},

	{0x00,1,{0xb0}},             //panel pad mapping control
	{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x02,0x00,0x0c,0x10,0x0a,0x0e,0x14}},

	{0x00,1,{0xc0}},             //panel pad mapping control
	{0xcc,15,{0x18,0x12,0x16,0x08,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03}},

	{0x00,1,{0xd0}},             //panel pad mapping control
	{0xcc,14,{0x01,0x00,0x0b,0x0f,0x09,0x0d,0x13,0x17,0x11,0x15,0x07,0x05,0x00,0x00}},

//-------------------- panel timing setting --------------------//
	{0x00,1,{0x80}},             //panel VST setting
	{0xce,12,{0x87,0x03,0x28,0x86,0x03,0x28,0x85,0x03,0x28,0x84,0x03,0x28}},

	{0x00,1,{0x90}},             //panel VEND setting
	{0xce,14,{0x34,0xfc,0x28,0x34,0xfd,0x28,0x34,0xfe,0x28,0x34,0xff,0x28,0x00,0x00}},

	{0x00,1,{0xa0}},             //panel CLKA1/2 setting
	{0xce,14,{0x38,0x07,0x05,0x00,0x00,0x28,0x00,0x38,0x06,0x05,0x01,0x00,0x28,0x00}},

	{0x00,1,{0xb0}},             //panel CLKA3/4 setting
	{0xce,14,{0x38,0x05,0x05,0x02,0x00,0x28,0x00,0x38,0x04,0x05,0x03,0x00,0x28,0x00}},

	{0x00,1,{0xc0}},             //panel CLKb1/2 setting
	{0xce,14,{0x38,0x03,0x05,0x04,0x00,0x28,0x00,0x38,0x02,0x05,0x05,0x00,0x28,0x00}},

	{0x00,1,{0xd0}},             //panel CLKb3/4 setting
	{0xce,14,{0x38,0x01,0x05,0x06,0x00,0x28,0x00,0x38,0x00,0x05,0x07,0x00,0x28,0x00}},

	{0x00,1,{0x80}},             //panel CLKc1/2 setting
	{0xcf,14,{0x38,0x07,0x05,0x00,0x00,0x18,0x25,0x38,0x06,0x05,0x01,0x00,0x18,0x25}},

	{0x00,1,{0x90}},             //panel CLKc3/4 setting
	{0xcf,14,{0x38,0x05,0x05,0x02,0x00,0x18,0x25,0x38,0x04,0x05,0x03,0x00,0x18,0x25}},

	{0x00,1,{0xa0}},             //panel CLKd1/2 setting
	{0xcf,14,{0x38,0x03,0x05,0x04,0x00,0x18,0x25,0x38,0x02,0x05,0x05,0x00,0x18,0x25}},

	{0x00,1,{0xb0}},             //panel CLKd3/4 setting
	{0xcf,14,{0x38,0x01,0x05,0x06,0x00,0x18,0x25,0x38,0x00,0x05,0x07,0x00,0x18,0x25}},

	{0x00,1,{0xc0}},             //panel ECLK setting
	{0xcf,11,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x81,0x00,0x03,0x08}},

//-------------------- gamma --------------------//
	{0x00,1,{0x00}},                                                                                                                                                               
	{0xE1,20,{0x08,0x13,0x1d,0x2b,0x3b,0x4b,0x4e,0x7e,0x75,0x91,0x72,0x5b,0x6a,0x47,0x46,0x3d,0x31,0x24,0x1a,0x12}}, 
                                                                                        
	{0x00,1,{0x00}},                                                                                                                                                           
	{0xE2,20,{0x08,0x13,0x1d,0x2b,0x3b,0x4b,0x4e,0x7e,0x75,0x91,0x72,0x5b,0x6a,0x47,0x46,0x3d,0x31,0x24,0x1a,0x12}}, 


  	{0x00,1,{0x00}},             //Orise mode disable
  	{0xff,3,{0xff,0xff,0xff}},


	{0x11,0,{0x00}},//SLEEP OUT                                                                                             
	{REGFLAG_DELAY,120,{}},                                                                                                 
	                               				                                                                                
	{0x29,0,{0x00}},//Display ON                                                                                            
	{REGFLAG_DELAY,50,{}},  
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 100, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {

		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}

	}

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

		params->dsi.mode   = BURST_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
            params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
            params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
            params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
                 params->dsi.packet_size=256;
	//params->dsi.intermediat_buffer_num = 2;
    // Video mode setting		
    params->dsi.vertical_sync_active = 6;
    params->dsi.vertical_backporch = 24;
    params->dsi.vertical_frontporch = 26;
    params->dsi.vertical_active_line = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active = 8;
    params->dsi.horizontal_backporch = 42;
    params->dsi.horizontal_frontporch = 44;	
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;	
#ifndef FPGA_EARLY_PORTING
    params->dsi.PLL_CLOCK = 200; //this value must be in MTK suggested table
#else
    params->dsi.pll_div1 = 1;
    params->dsi.pll_div2 = 1;
    params->dsi.fbk_div = 30;	//26;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
    params->dsi.fbk_sel = 0;
#endif
}

static void lcm_init(void)
{
	unsigned int data_array[16];
#if 0
#if defined(BUILD_LK)
	printf("%s, otm1283a lcm_init \n", __func__);
#else
	printk("%s, otm1283a lcm_init  \n", __func__);
#endif

#ifdef BUILD_LK
	upmu_set_rg_vgp2_en(0);
	upmu_set_rg_vgp2_vosel(6);
	upmu_set_rg_vgp2_en(1);
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP6, VOL_3300, "LCM");
#endif
	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	MDELAY(50);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(150);
#endif

	SET_RESET_PIN(1);
	MDELAY(10); 
	SET_RESET_PIN(0);
	MDELAY(50); 
	SET_RESET_PIN(1);
	MDELAY(150);  

/*
	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) /
		   sizeof(struct LCM_setting_table), 1);
*/	
	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) /
		   sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00280500;	// Display Off
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00100500;	// Sleep In
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);

#if 0
#ifdef BUILD_LK
	upmu_set_rg_vgp2_en(0);
#else
	hwPowerDown(MT65XX_POWER_LDO_VGP6, "LCM");
#endif
#endif

	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(1); // 1ms
	
	SET_RESET_PIN(1);
	MDELAY(120);     

}


static void lcm_resume(void)
{
//#ifndef BUILD_LK
	lcm_init();
//#endif
}


static unsigned int lcm_compare_id(void)
{
	unsigned int id0, id1, id2, id3, id4;
	unsigned char buffer[5];
	unsigned int array[5];
return 1;

#if 0
#ifdef BUILD_LK
	upmu_set_rg_vgp2_vosel(6);
	upmu_set_rg_vgp2_en(1);
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP6, VOL_3300, "LCM");
#endif

	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(100);
#endif

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(100); 

	// Set Maximum return byte = 1
	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 5);
	id0 = buffer[0];
	id1 = buffer[1];
	id2 = buffer[2];
	id3 = buffer[3];
	id4 = buffer[4];

#if defined(BUILD_LK)
	printf("%s, otm1283a Module ID = {%x, %x, %x, %x, %x} \n", __func__, id0,
	       id1, id2, id3, id4);
#else
	printk("%s, otm1283a Module ID = {%x, %x, %x, %x,%x} \n", __func__, id0,
	       id1, id2, id3, id4);
#endif
	id0 = (id2 << 8) | id3;
	//return (LCM_OTM1283_ID == ((id2 << 8) | id3)) ? 1 : 0;

#if defined(BUILD_LK)
	printf("otm1283a Module ID = %x \n", id0);
#else
	printk("otm1283a Module ID = %x \n", id0);
#endif

    if(id0 == LCM_OTM1283_ID)
    	return 1;
    else
        return 0;
}

//

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
//LCM_DRIVER otm1283a_bft_ivo_hd_dsi_vdo_lcm_drv = 
LCM_DRIVER otm1284a_hd_dsi_vdo_qc_lcm_drv = 
{
       .name			= "otm1284a_hd_dsi_vdo_qc",
	.set_util_funcs = lcm_set_util_funcs,
	.compare_id     = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
    #if defined(LCM_DSI_CMD_MODE)
        .update         = lcm_update,
    #endif
};
