/*
 * Command for accessing DataFlash.
 *
 * Copyright (C) 2008 Atmel Corporation
 */
#include <common.h>

extern int usb_charger_detect(void);
extern int get_battery_voltage(void);

static int do_charger(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	printf("Current charger is %d\n", usb_charger_detect());
	printf("Current voltage is %d\n", get_battery_voltage());

	return 0;
}

U_BOOT_CMD(
	charger,	1,	0,	do_charger,
	"Check if charger is connected",
	"charger - returns current charger status");
