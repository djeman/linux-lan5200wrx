/*
 * LG LAN5200WRx board support.
 */

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/init.h>
#include <linux/input.h>	/* KEY_* codes */
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/ata_platform.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>

#include <asm/bootinfo.h>
#include <asm/idle.h>
#include <asm/reboot.h>
#include <asm/fw/fw.h>

#include <asm/mach-au1x00/au1000.h>
#include <asm/mach-au1x00/gpio-au1300.h>
#include <asm/mach-au1x00/au1100_mmc.h>
#include <asm/mach-au1x00/au1200fb.h>
#include <asm/mach-au1x00/au1xxx_dbdma.h>
#include <asm/mach-au1x00/au1xxx_psc.h>
#include <asm/mach-au1x00/prom.h>

const char *get_system_type(void)
{
	return "LAN5200WRx";
}

void __init prom_init(void)
{
	unsigned char *memsize_str;
	uint64_t memsize;

	prom_argc = (int)fw_arg0;
	prom_argv = (char **)fw_arg1;
	prom_envp = (char **)fw_arg2;

	prom_init_cmdline();
	memsize_str = prom_getenv("memsize");
	if (!memsize_str || kstrtoull(memsize_str, 0, &memsize))
		memsize = 256 << 20; /* all boards have at least 256MB RAM */

	if (memsize > 0x10000000) {
		add_memory_region(0, 0x10000000, BOOT_MEM_RAM);
#ifdef CONFIG_HIGHMEM
		add_memory_region(0x30000000, (memsize - 0x10000000), BOOT_MEM_RAM);
#endif
	} else {
		add_memory_region(0, memsize, BOOT_MEM_RAM);
	}
}

void prom_putchar(unsigned char c)
{
	alchemy_uart_putchar(AU1300_UART2_PHYS_ADDR, c);
}

static struct platform_device lan5200wrx_rtc_dev = {
	.name	= "rtc-au1xxx",
	.id	= -1,
};

static void lan5200wrx_power_off(void)
{
	while (1)
		asm volatile (
		"	.set	mips32					\n"
		"	wait						\n"
		"	.set	mips0					\n");
}

static void lan5200wrx_reset(char *c)
{
	/* Jump to the reset vector */
	__asm__ __volatile__("jr\t%0" : : "r"(0xbfc00000));
}

static int __init lan5200wrx_late_setup(void)
{
	if (!pm_power_off)
		pm_power_off = lan5200wrx_power_off;
	if (!_machine_halt)
		_machine_halt = lan5200wrx_power_off;
	if (!_machine_restart)
		_machine_restart = lan5200wrx_reset;

	platform_device_register(&lan5200wrx_rtc_dev);

	return 0;
}
device_initcall(lan5200wrx_late_setup);

/******************************************************************************/

static struct i2c_board_info lan5200wrx_i2c_devs[] __initdata = {
	{ I2C_BOARD_INFO("wm8731", 0x1b), },	/* I2S audio codec */
	{ I2C_BOARD_INFO("ne1619", 0x2d), },	/* adm1025-compat hwmon */
};

/* multifunction pins to assign to GPIO controller */
static int lan5200wrx_gpio_pins[] __initdata = {
	AU1300_PIN_LCDPWM0, AU1300_PIN_PSC2SYNC1, AU1300_PIN_WAKE1,
	AU1300_PIN_WAKE2, AU1300_PIN_WAKE3, AU1300_PIN_FG3AUX,
	AU1300_PIN_EXTCLK1,
	/* UART1 pins (without tx/rx): 11-16 */
	AU1300_PIN_U1RI, AU1300_PIN_U1DCD, AU1300_PIN_U1DSR,
	AU1300_PIN_U1CTS, AU1300_PIN_U1RTS, AU1300_PIN_U1DTR,
	/* UART0 pins: 19-24 */
	AU1300_PIN_U0RI, AU1300_PIN_U0DCD, AU1300_PIN_U0DSR,
	AU1300_PIN_U0CTS, AU1300_PIN_U0RTS, AU1300_PIN_U0DTR,
	/* SD1 interface: 32-37 */
	AU1300_PIN_SD1DAT0, AU1300_PIN_SD1DAT1, AU1300_PIN_SD1DAT2,
	AU1300_PIN_SD1DAT3, AU1300_PIN_SD1CMD, AU1300_PIN_SD1CLK,
	/* SD2 interface: 38-43 */
	AU1300_PIN_SD2DAT0, AU1300_PIN_SD2DAT1, AU1300_PIN_SD2DAT2,
	AU1300_PIN_SD2DAT3, AU1300_PIN_SD2CMD, AU1300_PIN_SD2CLK,
	/* PCMCIA interface: 62-70 */
	AU1300_PIN_PCE2, AU1300_PIN_PCE1, AU1300_PIN_PIOS16,
	AU1300_PIN_PIOR, AU1300_PIN_PWE, AU1300_PIN_PWAIT,
	AU1300_PIN_PREG, AU1300_PIN_POE, AU1300_PIN_PIOW,
	-1,	/* terminator */
};

/* multifunction pins to assign to device functions */
static int lan5200wrx_dev_pins[] __initdata = {
	/* wake-from-str pins 0-3 */
	AU1300_PIN_WAKE0,
	/* external clock sources for PSC0 */
	AU1300_PIN_EXTCLK0,
	/* 8bit MMC interface on SD0: 6-9 */
	AU1300_PIN_SD0DAT4, AU1300_PIN_SD0DAT5, AU1300_PIN_SD0DAT6,
	AU1300_PIN_SD0DAT7,
	/* UART1 pins (tx/rx): 17-18 */
	AU1300_PIN_U1RX, AU1300_PIN_U1TX,
	/* UART2: 25-26 */
	AU1300_PIN_U2RX, AU1300_PIN_U2TX,
	/* UART3: 27-28 */
	AU1300_PIN_U3RX, AU1300_PIN_U3TX,
	/* LCD controller PWMs, ext pixclock: 30-31 */
	AU1300_PIN_LCDPWM1, AU1300_PIN_LCDCLKIN,
	/* PSC0/1 clocks: 44-45 */
	AU1300_PIN_PSC0CLK, AU1300_PIN_PSC1CLK,
	/* PSCs: 46-49/50-53/54-57/58-61 */
	AU1300_PIN_PSC0SYNC0, AU1300_PIN_PSC0SYNC1, AU1300_PIN_PSC0D0,
	AU1300_PIN_PSC0D1,
	AU1300_PIN_PSC1SYNC0, AU1300_PIN_PSC1SYNC1, AU1300_PIN_PSC1D0,
	AU1300_PIN_PSC1D1,
	AU1300_PIN_PSC2SYNC0,			    AU1300_PIN_PSC2D0,
	AU1300_PIN_PSC2D1,
	AU1300_PIN_PSC3SYNC0, AU1300_PIN_PSC3SYNC1, AU1300_PIN_PSC3D0,
	AU1300_PIN_PSC3D1,
	/* camera interface H/V sync inputs: 71-72 */
	AU1300_PIN_CIMLS, AU1300_PIN_CIMFS,
	/* PSC2/3 clocks: 73-74 */
	AU1300_PIN_PSC2CLK, AU1300_PIN_PSC3CLK,
	-1,	/* terminator */
};

static void __init lan5200wrx_gpio_config(void)
{
	int *i;

	i = &lan5200wrx_dev_pins[0];
	while (*i != -1)
		au1300_pinfunc_to_dev(*i++);

	i = &lan5200wrx_gpio_pins[0];
	while (*i != -1)
		au1300_gpio_direction_input(*i++);/* implies pin_to_gpio */

	au1300_gpio_direction_output(AU1300_PIN_PIOW, 1);/* usb power */

	au1300_set_dbdma_gpio(1, AU1300_PIN_FG3AUX);
}

/**********************************************************************/

/*
 * NOR FLASH
 */

static struct mtd_partition lan5200wrx_mtd_partitions[] = {
	{
		.name	= "boot",
		.size	= 0x00300000,
		.offset = MTDPART_OFS_APPEND,
		.mask_flags = MTD_WRITEABLE,
	},
	{
		.name	= "splash",
		.size	= 0x00100000,
		.offset = MTDPART_OFS_APPEND,
		.mask_flags = MTD_WRITEABLE,
	},
};

static struct physmap_flash_data lan5200wrx_flash_data = {
	.width		= 2,
	.nr_parts	= ARRAY_SIZE(lan5200wrx_mtd_partitions),
	.parts		= lan5200wrx_mtd_partitions,
};

static struct resource lan5200wrx_mtd_resource = {
	.start	= 0x1fc00000,
	.end	= 0x1fffffff,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device lan5200wrx_mtd_device = {
	.name		= "physmap-flash",
	.dev		= {
		.platform_data	= &lan5200wrx_flash_data,
	},
	.num_resources	= 1,
	.resource	= &lan5200wrx_mtd_resource,
};

/******************************************************************************/

static struct resource au1300_psc3_res[] = {
	[0] = {
		.start	= AU1300_PSC3_PHYS_ADDR,
		.end	= AU1300_PSC3_PHYS_ADDR + 0x0fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AU1300_PSC3_INT,
		.end	= AU1300_PSC3_INT,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= AU1300_DSCR_CMD0_PSC3_TX,
		.end	= AU1300_DSCR_CMD0_PSC3_TX,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= AU1300_DSCR_CMD0_PSC3_RX,
		.end	= AU1300_DSCR_CMD0_PSC3_RX,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device lan5200wrx_i2c_dev = {
	.name		= "au1xpsc_smbus",
	.id		= 0,	/* bus number */
	.num_resources	= ARRAY_SIZE(au1300_psc3_res),
	.resource	= au1300_psc3_res,
};

/******************************************************************************/

static int lan5200wrx_sd0_inserted(void *mmc_host)
{
	return 1; /* disable for now, it doesn't work yet */
}

static int lan5200wrx_sd0_readonly(void *mmc_host)
{
	return 0;
}

struct au1xmmc_platform_data lan5200wrx_sd0_platdata = {
	.card_inserted		= lan5200wrx_sd0_inserted,
	.card_readonly		= lan5200wrx_sd0_readonly,
	.mask_host_caps		= MMC_CAP_NEEDS_POLL,
	.non_removable 		= true,
};

static struct resource au1300_sd0_res[] = {
	[0] = {
		.start	= AU1100_SD0_PHYS_ADDR,
		.end	= AU1100_SD0_PHYS_ADDR,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AU1300_SD0_INT,
		.end	= AU1300_SD0_INT,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= AU1300_DSCR_CMD0_SDMS_TX0,
		.end	= AU1300_DSCR_CMD0_SDMS_TX0,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= AU1300_DSCR_CMD0_SDMS_RX0,
		.end	= AU1300_DSCR_CMD0_SDMS_RX0,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device lan5200wrx_sd0_dev = {
	.dev = {
		.platform_data	= &lan5200wrx_sd0_platdata,
	},
	.name		= "au1xxx-mmc",
	.id		= 0,
	.resource	= au1300_sd0_res,
	.num_resources	= ARRAY_SIZE(au1300_sd0_res),
};

/**********************************************************************/

static int lan5200wrxfb_panel_index(void)
{
	return 10;	/* LGE_800x480_TFT_36MHz */
}

static struct au1200fb_platdata lan5200wrxfb_pd = {
	.panel_index	= lan5200wrxfb_panel_index,
};

static struct resource au1300_lcd_res[] = {
	[0] = {
		.start	= AU1200_LCD_PHYS_ADDR,
		.end	= AU1200_LCD_PHYS_ADDR + 0x800 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AU1300_LCD_INT,
		.end	= AU1300_LCD_INT,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 au1300_lcd_dmamask = DMA_BIT_MASK(32);

static struct platform_device lan5200wrx_lcd_dev = {
	.name		= "au1200-lcd",
	.id		= 0,
	.dev = {
		.dma_mask		= &au1300_lcd_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &lan5200wrxfb_pd,
	},
	.num_resources	= ARRAY_SIZE(au1300_lcd_res),
	.resource	= au1300_lcd_res,
};

/**********************************************************************/

static struct resource au1300_psc2_res[] = {
	[0] = {
		.start	= AU1300_PSC2_PHYS_ADDR,
		.end	= AU1300_PSC2_PHYS_ADDR + 0x0fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AU1300_PSC2_INT,
		.end	= AU1300_PSC2_INT,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= AU1300_DSCR_CMD0_PSC2_TX,
		.end	= AU1300_DSCR_CMD0_PSC2_TX,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= AU1300_DSCR_CMD0_PSC2_RX,
		.end	= AU1300_DSCR_CMD0_PSC2_RX,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device lan5200wrx_i2s_dev = {
	.name		= "au1xpsc_i2s",
	.id		= 2,	/* PSC ID */
	.num_resources	= ARRAY_SIZE(au1300_psc2_res),
	.resource	= au1300_psc2_res,
};

/**********************************************************************/

static struct resource au1300_psc0_res[] = {
	[0] = {
		.start	= AU1300_PSC0_PHYS_ADDR,
		.end	= AU1300_PSC0_PHYS_ADDR + 0x0fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AU1300_PSC0_INT,
		.end	= AU1300_PSC0_INT,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= AU1300_DSCR_CMD0_PSC0_TX,
		.end	= AU1300_DSCR_CMD0_PSC0_TX,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.start	= AU1300_DSCR_CMD0_PSC0_RX,
		.end	= AU1300_DSCR_CMD0_PSC0_RX,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device lan5200wrx_i2s2_dev = {
	.name		= "au1xpsc_i2s2",
	.id		= 0,	/* PSC ID */
	.num_resources	= ARRAY_SIZE(au1300_psc0_res),
	.resource	= au1300_psc0_res,
};

/**********************************************************************/

static struct platform_device lan5200wrx_i2sdma_dev = {
	.name		= "au1xpsc-pcm",
	.id		= 2,	/* PSC ID */
};

static struct platform_device lan5200wrx_i2s2dma_dev = {
	.name		= "au1xpsc-pcm",
	.id		= 0,	/* PSC ID */
};

static struct platform_device lan5200wrx_sndi2s_dev = {
	.name		= "au1xpsc-i2s",
};

/**********************************************************************/

static struct platform_device *lan5200wrx_dev[] __initdata = {
	&lan5200wrx_mtd_device,
	&lan5200wrx_i2c_dev,
	&lan5200wrx_sd0_dev,
	&lan5200wrx_lcd_dev,
	&lan5200wrx_i2s_dev,
	&lan5200wrx_i2s2_dev,
	&lan5200wrx_i2sdma_dev,
	&lan5200wrx_i2s2dma_dev,
	&lan5200wrx_sndi2s_dev,
};

static int __init lan5200wrx_dev_init(void)
{
	int cpldirq;
	struct clk *c;

	/* setup CPLD IRQ muxer */
	cpldirq = au1300_gpio_to_irq(AU1300_PIN_EXTCLK1);
	irq_set_irq_type(cpldirq, IRQ_TYPE_LEVEL_HIGH);

	/*
	 * setup board
	 */
	i2c_register_board_info(0, lan5200wrx_i2c_devs,
				ARRAY_SIZE(lan5200wrx_i2c_devs));

	/* Audio PSC clock for bt is supplied by codecs (PSC0) */
	__raw_writel(PSC_SEL_CLK_SERCLK,
	    (void __iomem *)KSEG1ADDR(AU1300_PSC0_PHYS_ADDR) + PSC_SEL_OFFSET);
	wmb();
	/* SPI DAB (PSC1) */
	__raw_writel(PSC_SEL_CLK_INTCLK,
	    (void __iomem *)KSEG1ADDR(AU1300_PSC1_PHYS_ADDR) + PSC_SEL_OFFSET);
	wmb();
	/* Audio PSC clock for media is supplied by external clock (PSC2) */
	__raw_writel(PSC_SEL_CLK_EXTCLK,
	    (void __iomem *)KSEG1ADDR(AU1300_PSC2_PHYS_ADDR) + PSC_SEL_OFFSET);
	wmb();
	/* I2C driver wants 50MHz, get as close as possible */
	c = clk_get(NULL, "psc3_intclk");
	if (!IS_ERR(c)) {
		clk_set_rate(c, 50000000);
		clk_prepare_enable(c);
		clk_put(c);
	}
	__raw_writel(PSC_SEL_CLK_INTCLK,
	    (void __iomem *)KSEG1ADDR(AU1300_PSC3_PHYS_ADDR) + PSC_SEL_OFFSET);
	wmb();

	return platform_add_devices(lan5200wrx_dev, ARRAY_SIZE(lan5200wrx_dev));
}
device_initcall(lan5200wrx_dev_init);

void __init board_setup(void)
{
	lan5200wrx_gpio_config();

	printk(KERN_INFO "**** LG LAN5200WRx Platform ****\n");

	/* enable UARTs, Codekit Bl only enables #2 */
	alchemy_uart_enable(AU1300_UART0_PHYS_ADDR);
	alchemy_uart_enable(AU1300_UART1_PHYS_ADDR);
	alchemy_uart_enable(AU1300_UART3_PHYS_ADDR);
}
