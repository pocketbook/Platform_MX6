/*
 * The size of share region is 4KB.
 */
#define IS_UPDKEY_IN_MMC 1

#ifdef IS_UPDKEY_IN_MMC

#define UPD_MMC_NUM 1
#define UPD_BLOCK_NUM 1290
#define UPD_BLOCK_COUNT 1
#define SHARE_REGION_SIZE ((512 * UPD_BLOCK_COUNT))
#define SHARE_REGION_BASE NULL

#define UPD_MMC "/dev/mmcblk0"

#else
#ifndef PAGE_SIZE
#define PAGE_SIZE	0x1000
#endif
#define	SHARE_REGION_SIZE	PAGE_SIZE
#define SHARE_REGION_BASE	(0xA0000000 - SHARE_REGION_SIZE)
#endif

#define RESERVED_SIZE	(SHARE_REGION_SIZE - sizeof(struct share_region_flags) - sizeof(unsigned long))
#define CRC32_RESIDUE	0xdebb20e3UL /* ANSI X3.66 residue*/

struct share_region_flags
{
	unsigned int uboot_flag;
	unsigned int kernel_flag;
};

struct share_region_t
{
	struct share_region_flags flags;
	unsigned char reserved[RESERVED_SIZE];
	unsigned long checksum;
};
//typedef struct share_region_t share_region_t;

/* Values of boot flag*/
enum {
	BOOT_NORMAL = 0x0,
	BOOT_SYSTEM_UPGRADE,
	BOOT_OTA,
};

struct flag_cb {
	char *path;
	unsigned long (*flag_get)(void);
	void (*flag_set)(unsigned long);
};

