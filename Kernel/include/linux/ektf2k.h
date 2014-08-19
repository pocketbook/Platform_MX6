#ifndef _LINUX_ELAN_KTF2K_H
#define _LINUX_ELAN_KTF2K_H


//#define ELAN_X_MAX 2496
//#define ELAN_Y_MAX 1856

#define ELAN_X_MAX 1200
#define ELAN_Y_MAX 1600


#define ELAN_KTF2K_NAME "ekt2xxx"

struct elan_ktf2k_i2c_platform_data {
   uint16_t version;
   int abs_x_min;
   int abs_x_max;
   int abs_y_min;
   int abs_y_max;
   int intr_gpio;
   int (*power)(int on);
   int rst_gpio;
};

#endif /*_LINUX_ELAN_KTF2K_H */