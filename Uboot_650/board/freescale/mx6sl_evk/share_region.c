#define DEBUG

#include <common.h>
#include <share_region.h>
#include <asm/arch/iomux.h>

int tmp = 0;
static share_region_t *const share_region = (share_region_t *)SHARE_REGION_BASE;

extern int check_update_keys(void);

static unsigned bg_crc32(unsigned char *buf, int len)
{
	return crc32(0L, buf, len);
}

int read_share_region()
{
	return share_region->checksum;
}

/* CRC right return 1, else return 0 */
static int check_share_region(void)
{
	int result = 0;

	if (share_region) {
		unsigned long computed_residue, checksum;

		checksum = share_region->checksum;

		computed_residue = ~bg_crc32((unsigned char*)share_region, SHARE_REGION_SIZE);
		debug("the computed_residue = %lx\n", computed_residue);
		result = CRC32_RESIDUE == computed_residue;
	}

	debug("the result is %x\n", result);
	return result;

}

static void save_share_region(void)
{
	if (share_region) {
		share_region->checksum = bg_crc32((unsigned char*)share_region, SHARE_REGION_SIZE - sizeof(unsigned long ));
		debug("the checksum addr = %p\n", &(share_region->checksum));
		debug("share region checksum = %lx\n", share_region->checksum);
	}
}

static void clear_flags(void)
{
	share_region->flags.uboot_flag = 0;
	share_region->flags.kernel_flag = 0;
}

/***************************************
* update_flag=0, BOOT_NORMAL
* update_flag=1, BOOT_SYSTEM_UPGRADE
* update_flag=2, BOOT_OTA
****************************************/
static int recovery_handle(void)
{
	unsigned int val = 0;
	unsigned int reg1 = 0;
	unsigned int reg2 = 0;
	struct spi_slave *slave;

	if (share_region->flags.uboot_flag != BOOT_NORMAL)
		goto out;

	if (check_update_keys() == 1) {
		debug("update keys were pressed\n");
		share_region->flags.uboot_flag = BOOT_SYSTEM_UPGRADE;
	}

out:
	save_share_region();
	debug("share_region = %d\n", share_region->flags.uboot_flag);

	return share_region->flags.uboot_flag;
}

int get_update_flag(void)
{
	return share_region->flags.uboot_flag;
}

int share_region_handle(void)
{
	if (!check_share_region())
		memset(share_region, 0x0, 0x1000);

	tmp = share_region->flags.kernel_flag;  //this can clear the flag
	memset(share_region, 0x0, 0x1000);
	share_region->flags.uboot_flag = tmp;

	return recovery_handle();
}

void set_share_region(int flag)
{
	clear_flags();
	share_region->flags.kernel_flag = flag;
	save_share_region();
}
