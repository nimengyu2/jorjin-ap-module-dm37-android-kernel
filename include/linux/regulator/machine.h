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
 * ������������־����Щ��־����ʹ����ʹ��ȷ���ĵ���������
 * VOLTAGE:  Regulator output voltage can be changed by software on this
 *           board/machine.
 *           �����������ѹ�ܹ�������ı䣬�ڰ��Ӻͻ�����
 * CURRENT:  Regulator output current can be changed by software on this
 *           board/machine.
 *           ��������������ܹ�������ı�
 * MODE:     Regulator operating mode can be changed by software on this
 *           board/machine.
 *           ����������ģʽ�ܹ�������ı�
 * STATUS:   Regulator can be enabled and disabled.
 *           �������ܹ�����ֹ��ʹ��
 * DRMS:     Dynamic Regulator Mode Switching is enabled for this regulator.
 *           ��̬������ģʽ�ܹ���ʹ��
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

	char *name;  // ����

	/* voltage output range (inclusive) - for voltage control */
	int min_uV;  // �����ѹ ���ڵ�ѹ����  ��͵�ѹ
	int max_uV;  // ����ѹ

	/* current output range (inclusive) - for current control */
	int min_uA;  // ������� ���ڵ�������  ��͵���
	int max_uA; // ��ߵ���

	/* valid regulator operating modes for this machine */
	unsigned int valid_modes_mask;  // ����������ģʽ

	/* valid operations for regulator on this machine */
	unsigned int valid_ops_mask;  // ��Ч�ĵ���������ģʽ

	/* regulator input voltage - only if supply is another regulator */
	int input_uV;  // �����������ѹ

	/* regulator suspend states for global PMIC STANDBY/HIBERNATE */
	// �����������״̬ ���ȫ�ֵ�pmic ��׼/�������
	struct regulator_state state_disk;
	struct regulator_state state_mem;
	struct regulator_state state_standby;
	// ��ʼ״̬�Ƿ�����
	suspend_state_t initial_state; /* suspend state to set at init */

	/* mode to set on startup */
	// ������ʱ���趨��ģʽ
	unsigned int initial_mode;

	/* constraint flags */
	// �����ı�־λ
	// ������ʼ�տ���
	unsigned always_on:1;	/* regulator never off when system is on */
	// ��������ʱ����
	unsigned boot_on:1;	/* bootloader/firmware enabled regulator */
	// ���min=max����ʹ��uv����
	unsigned apply_uV:1;	/* apply uV constraint if min == max */
};

/**
 * struct regulator_consumer_supply - supply -> device mapping
 * �ṹ�� regulator_consumer_supply �������õ���ӳ��   ������-->�õ���ӳ�� 
 * This maps a supply name to a device.  Only one of dev or dev_name
 * can be specified.  Use of dev_name allows support for buses which
 * make struct device available late such as I2C and is the preferred
 * form.
 * ��Щӳ��һ�����������Ƶ�һ���豸�ṹ�壬ֻ����һ���豸�����豸����
 * ʹ��dev_name����֧������
 *
 * @dev: Device structure for the consumer.
 * @dev_name: Result of dev_name() for the consumer.
 * @supply: Name for the supply.
 */
struct regulator_consumer_supply {
	struct device *dev;	/* consumer �õ����豸�ṹ��*/
	const char *dev_name;   /* dev_name() for consumer �õ����豸����*/
	const char *supply;	/* consumer supply - e.g. "vcc"������ */
};

// ��ʼ���������õ��߽ṹ��
/* Initialize struct regulator_consumer_supply */
#define REGULATOR_SUPPLY(_name, _dev_name)			\
{	// ������							\
	.supply		= _name,				\
	.dev_name	= _dev_name,				\
	// �õ����豸����
}

/**
 * struct regulator_init_data - regulator platform initialisation data.
 * ������ƽ̨��ʼ������
 * Initialisation constraints, our supply and consumers supplies.
 * ��ʼ���ṹ�壬���ǵĹ���
 * @supply_regulator: Parent regulator.  Specified using the regulator name
 *                    as it appears in the name field in sysfs, which can
 *                    be explicitly set using the constraints field 'name'.
 *                    ����������ָ��ʹ�õ����������֣����ｫ����ʾ��sysfsϵͳ��
 *                    ����ʹ�ýṹ��name
 * @supply_regulator_dev: Parent regulator (if any) - DEPRECATED in favour
 *                        of supply_regulator.
 *
 * @constraints: Constraints.  These must be specified for the regulator to
 *               be usable.  
 * @num_consumer_supplies: Number of consumer device supplies. �豸����ĺĵ��豸����
 * @consumer_supplies: Consumer device supply configuration. �õ��豸���ò���
 *
 * @regulator_init: Callback invoked when the regulator has been registered.��������ʼ��
 * @driver_data: Data passed to regulator_init.
 */
struct regulator_init_data {
	const char *supply_regulator;        /* or NULL for system supply */
	struct device *supply_regulator_dev; /* or NULL for system supply */

	// ������ ϵͳ�涨����
	struct regulation_constraints constraints;

	int num_consumer_supplies;
	struct regulator_consumer_supply *consumer_supplies;

	/* optional regulator machine specific init */
	// ��ѡ�ĵ���������ָ����ʼ��
	int (*regulator_init)(void *driver_data);
	// �ں˲���Ҫ�������
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
