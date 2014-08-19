#ifndef OBREEY_H
#define OBREEY_H

struct system_led_platform_data {
	int (*init)(void);
	int (*control)(bool);
	int (*state)(void);
};

#endif
