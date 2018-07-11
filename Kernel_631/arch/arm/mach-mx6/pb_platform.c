/*
 * pb_platform.c
 *
 *  Created on: Sep 1, 2017
 *      Author: dushes
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/crc32.h>
#include <linux/proc_fs.h>

//offset where is stored hwconfig
#define HWCONFIG_BLOCK_OFFSET 1291 * 512
#define HWCONFIG_BLOCK_SIZE 512
#define HWCONFIG_STORAGE "/dev/mmcblk0"

#define CRC32_RESIDUE	0xdebb20e3UL /* ANSI X3.66 residue*/

struct hwconfig_region_s {
	char magic[8];
	unsigned long long hwconfig;
	unsigned char reserved[512 - /*config size*/sizeof(unsigned long long) - /*checksum size*/sizeof(unsigned long) - /*magis size*/8];
	unsigned long checksum;
} hwconfig_region;

static int hwconfig_init();
DECLARE_DELAYED_WORK(hwc, hwconfig_init);
static int count = 100;

static unsigned int crc32_(unsigned char *buf, int len) {
	return crc32_le(~0L, buf, len);
}

static int proc_hwconfig_read(char *page, char **start, off_t off, int count, int *eof, void *data) {
	*eof = 1;
	int size = sizeof(hwconfig_region.hwconfig);

	pr_debug("[%s] page = %p; off = %i; count = %i; size = %i\n",__func__,page,off,count,size);

	if (off > 0)
		return 0;


	memcpy(page, &(hwconfig_region.hwconfig), size);
	return size;//snprintf(page, PAGE_SIZE, "%llu", hwconfig_region.hwconfig);
}

#ifdef DEBUG
static void dump_hwconfig_region(void) {
	int i = 0;
	printk("-----[%s]-----\n",__func__);
	for (i = 0; i < HWCONFIG_BLOCK_SIZE; i++) {
		printk("<c> %hhx ",((char *)&hwconfig_region)[i]);
	}
	printk("\n-----[END]-----\n");
}
#else
static void dump_hwconfig_region(void) {};
#endif



static int hwconfig_init() {
	struct proc_dir_entry *file;

    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;
    int ret = 0;
    unsigned long computed_residue;
	unsigned long computed_crc;
    loff_t offset = HWCONFIG_BLOCK_OFFSET;

    pr_debug("[%s] started\n",__func__);

    //printk("[%s] sh=%li offset = %lli\n",__func__, share_region, offset);
    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(HWCONFIG_STORAGE, O_RDONLY, 0);
    if(IS_ERR(filp)) {
    	err = PTR_ERR(filp);
    	printk("[%s] Error while opening %s (%i); looks like VFS is not mounted; retry in 50ms; count = %i\n",__func__, HWCONFIG_STORAGE, err, count);
    	if (count--)
    		schedule_delayed_work(&hwc, msecs_to_jiffies(50));

    	return err;
    }
    ret = vfs_read(filp, (char *)&hwconfig_region, HWCONFIG_BLOCK_SIZE, &offset);
    printk ("[%s] vfs_read returned %i\n",__func__,ret);

    filp_close(filp, NULL);
    set_fs(oldfs);

    //check hwconfig block
    computed_residue = crc32_((unsigned char *)&hwconfig_region, HWCONFIG_BLOCK_SIZE);
//    computed_crc = ~crc32_((unsigned char *)&hwconfig_region, HWCONFIG_BLOCK_SIZE - sizeof(hwconfig_region.checksum));
//    pr_debug("[%s] crc = %lx; crc_int = %lx; computed_residue = %lx",__func__, computed_crc, hwconfig_region.checksum, computed_residue);
    dump_hwconfig_region();

    if (computed_residue != CRC32_RESIDUE) {
    	printk("[%s] HWCONFIG is not found!\n",__func__);
    	return 0;
    }

	file = create_proc_entry("hwconfig", S_IRUGO, NULL);
	if (!file) {
		printk("could not create /proc/hwconfig\n");
		return -1;
	}

	file->read_proc = proc_hwconfig_read;
	pr_debug("[%s] registered\n",__func__);
	return 0;
}

int hwc_init(void) {

	schedule_delayed_work(&hwc,msecs_to_jiffies(100));
}

late_initcall(hwc_init);
