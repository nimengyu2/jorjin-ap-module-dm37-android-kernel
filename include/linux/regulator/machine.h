/*
 * machine.h -- SoC Regulator support, machine/board driver API.
 *
 * Copyright (C) 2007, 2008 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood <lrg@slimlogic.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Regulator Machine/Board Interface.
 */

#ifndef __LINUX_REGULATOR_MACHINE_H_
#define __LINUX_REGULATOR_MACHINE_H_

#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

struct regulator;

/*
 * Regulator operation constraint flags. These flags are used to enable
 * certain regulator operations and can be OR'ed together.
 * 调整器参数标志，这些标志将会使用来使能确定的调整器操作
 * VOLTAGE:  Regulator output voltage can be changed by software on this
 *           board/machine.
 *           调整器输出电压能够被软件改变，在板子和机器上
 * CURRENT:  Regulator output current can be changed by software on this
 *           board/machine.
 *           调整器输出电流能够被软件改变
 * MODE:     Regulator operating mode can be changed by software on this
 *           board/machine.
 *           调整器操作模式能够被软件改变
 * STATUS:   Regulator can be enabled and disabled.
 *           调整器能够被禁止和使能
 * DRMS:     Dynamic Regulator Mode Switching is enabled for this regulator.
 *           动态调整器模式能够被使能
 */

#define REGULATOR_CHANGE_VOLTAGE	0x1  
#define REGULATOR_CHANGE_CURRENT	0x2
#define REGULATOR_CHANGE_MODE		0x4
#define REGULATOR_CHANGE_STATUS		0x8
#define REGULATOR_CHANGE_DRMS		0x10

/**
 * struct regulator_state - regulator state during low power system states
 *
 * This describes a regulators state during a system wide low power
 * state.  One of enabled or disabled must be set for the
 * configuration to be applied.
 *
 * @uV: Operating voltage during suspend.
 * @mode: Operating mode during suspend.
 * @enabled: Enabled during suspend.
 * @disabled: Disabled during suspend.
 */
struct regulator_state {
	int uV;	/* suspend voltage */
	unsigned int mode; /* suspend regulator operating mode */
	int enabled; /* is regulator enabled in this suspend state */
	int disabled; /* is the regulator disbled in this suspend state */
};

/**
 * struct regulation_constraints - regulator operating constraints.
 *
 * This struct describes regulator and board/machine specific constraints.
 *
 * @name: Descriptive name for the constraints, used for display purposes.
 *
 * @min_uV: Smallest voltage consumers may set.
 * @max_uV: Largest voltage consumers may set.
 *
 * @min_uA: Smallest consumers consumers may set.
 * @max_uA: Largest current consumers may set.
 *
 * @valid_modes_mask: Mask of modes which may be configured by consumers.
 * @valid_ops_mask: Operations which may be performed by consumers.
 *
 * @always_on: Set if the regulator should never be disabled.
 * @boot_on: Set if the regulator is enabled when the system is initially
 *           started.  If the regulator is not enabled by the hardware or
 *           bootloader then it will be enabled when the constraints are
 *           applied.
 * @apply_uV: Apply the voltage constraint when initialising.
 *
 * @input_uV: Input voltage for regulator when supplied by another regulator.
 *
 * @state_disk: State for regulator when system is suspended in disk mode.
 * @state_mem: State for regulator when system is suspended in mem mode.
 * @state_standby: State for regulator when system is suspended in standby
 *                 mode.
 * @initial_state: Suspend state to set by default.
 * @initial_mode: Mode to set at startup.
 */
struct regulation_constraints {

	char *name;  // 名字

	/* voltage output range (inclusive) - for voltage control */
	int min_uV;  // 输出电压 用于电压控制  最低电压
	int max_uV;  // 最大电压

	/* current output range (inclusive) - for current control */
	int min_uA;  // 输出电流 用于电流控制  最低电流
	int max_uA; // 最高电流

	/* valid regulator operating modes for this machine */
	unsigned int valid_modes_mask;  // 调整器操作模式

	/* valid operations for regulator on this machine */
	unsigned int valid_ops_mask;  // 有效的调整器操作模式

	/* regulator input voltage - only if supply is another regulator */
	int input_uV;  // 调整期输入电压

	/* regulator suspend states for global PMIC STANDBY/HIBERNATE */
	// 调整器挂起的状态 针对全局的pmic 标准/深度休眠
	struct regulator_state state_disk;
	struct regulator_state state_mem;
	struct regulator_state state_standby;
	// 初始状态是否休眠
	suspend_state_t initial_state; /* suspend state to set at init */

	/* mode to set on startup */
	// 启动的时候设定的模式
	unsigned int initial_mode;

	/* constraint flags */
	// 包含的标志位
	// 调整期始终开启
	unsigned always_on:1;	/* regulator never off when system is on */
	// 在启动到时候开启
	unsigned boot_on:1;	/* bootloader/firmware enabled regulator */
	// 如果min=max，则使用uv参数
	unsigned apply_uV:1;	/* apply uV constraint if min == max */
};

/**
 * struct regulator_consumer_supply - supply -> device mapping
 * 结构体 regulator_consumer_supply 调整器用电者映射   供电者-->用电者映射 
 * This maps a supply name to a device.  Only one of dev or dev_name
 * can be specified.  Use of dev_name allows support for buses which
 * make struct device available late such as I2C and is the preferred
 * form.
 * 这些映射一个供电者名称到一个设备结构体，只能有一个设备或者设备名称
 * 使用dev_name允许支持总线
 *
 * @dev: Device structure for the consumer.
 * @dev_name: Result of dev_name() for the consumer.
 * @supply: Name for the supply.
 */
struct regulator_consumer_supply {
	struct device *dev;	/* consumer 用电者设备结构体*/
	const char *dev_name;   /* dev_name() for consumer 用电者设备名称*/
	const char *supply;	/* consumer supply - e.g. "vcc"供电者 */
};

// 初始化调整器用电者结构体
/* Initialize struct regulator_consumer_supply */
#define REGULATOR_SUPPLY(_name, _dev_name)			\
{	// 供电者							\
	.supply		= _name,				\
	.dev_name	= _dev_name,				\
	// 用电者设备名称
}

/**
 * struct regulator_init_data - regulator platform initialisation data.
 * 调整期平台初始化数据
 * Initialisation constraints, our supply and consumers supplies.
 * 初始化结构体，我们的供电
 * @supply_regulator: Parent regulator.  Specified using the regulator name
 *                    as it appears in the name field in sysfs, which can
 *                    be explicitly set using the constraints field 'name'.
 *                    父调整器，指定使用调整器的名字，这里将会显示在sysfs系统中
 *                    可以使用结构体name
 * @supply_regulator_dev: Parent regulator (if any) - DEPRECATED in favour
 *                        of supply_regulator.
 *
 * @constraints: Constraints.  These must be specified for the regulator to
 *               be usable.  
 * @num_consumer_supplies: Number of consumer device supplies. 设备供电的耗电设备个数
 * @consumer_supplies: Consumer device supply configuration. 用电设备配置参数
 *
 * @regulator_init: Callback invoked when the regulator has been registered.调整器初始化
 * @driver_data: Data passed to regulator_init.
 */
struct regulator_init_data {
	const char *supply_regulator;        /* or NULL for system supply */
	struct device *supply_regulator_dev; /* or NULL for system supply */

	// 调整器 系统规定参数
	struct regulation_constraints constraints;

	int num_consumer_supplies;
	struct regulator_consumer_supply *consumer_supplies;

	/* optional regulator machine specific init */
	// 可选的调整器机器指定初始化
	int (*regulator_init)(void *driver_data);
	// 内核不需要触摸这个
	void *driver_data;	/* core does not touch this */
};

int regulator_suspend_prepare(suspend_state_t state);

#ifdef CONFIG_REGULATOR
void regulator_has_full_constraints(void);
void regulator_use_dummy_regulator(void);
#else
static inline void regulator_has_full_constraints(void)
{
}

static inline void regulator_use_dummy_regulator(void)
{
}
#endif

#endif
