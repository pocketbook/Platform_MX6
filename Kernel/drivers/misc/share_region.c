#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/crc32.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include "linux/share_region.h"

static struct share_region_t *share_region = NULL;

static unsigned bg_crc32(unsigned char *buf, int len)
{
	return ~crc32_le(~0L, buf, len);
}

void clear_flags(void)
{
	share_region->flags.uboot_flag = 0;
	share_region->flags.kernel_flag = 0;
}

static void save_share_region(void)
{
	if (share_region) {
		share_region->checksum = bg_crc32((u8 *)share_region, SHARE_REGION_SIZE - sizeof(unsigned long));
	}
}

static int check_share_region (void)
{
	int result = 0;
	
	if (share_region) {
		unsigned long computed_residue, checksum;

		checksum = share_region->checksum;
		printk("++++++++++++++++++++++++++++++share_region->flags.uboot_flag = %lx\n", *(unsigned long *)share_region);
//		printk("++++++++++++++++++++++++++++++share_region->checksum = %lx\n", share_region->checksum);

		computed_residue = ~bg_crc32((unsigned char*)share_region, SHARE_REGION_SIZE);
//		printk("++++++++++++++++++computed_residue = %lx", computed_residue);
		result = CRC32_RESIDUE == computed_residue;
	}
	
	if (result) {
		printk("share region: passed\n");
	} else {
		printk("share region: failed\n");
	}
	
	return result;
}

static unsigned long get_kernel_update_flag(void)
{
	printk("%d\n", share_region->flags.uboot_flag);
	return (share_region->flags.uboot_flag);
} 

static void set_kernel_update_flag(unsigned long flag)
{
	if (flag > 2) 
		return;

//	printk("flag = %lx\n", flag);
	share_region->flags.kernel_flag = flag;
	share_region->flags.uboot_flag = flag;
	save_share_region();
}

static struct flag_cb flag_proc[] = {
	{"update_flag", get_kernel_update_flag, set_kernel_update_flag},	
};

static int proc_flag_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	const struct flag_cb *p = data;

	*eof = 1;
	
	if (p && p->flag_get) {
		return snprintf(page, PAGE_SIZE, "%lu\n", p->flag_get());
	} else {
		return -EIO;
	}	
}

static int proc_flag_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	const struct flag_cb *p = data;
	char buf[16];
	if (count > sizeof(buf) -1 )
		return -EINVAL;
	if (!count)
		return 0;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	buf[count] = '\0';
	
	if (p && p->flag_set) {
		switch (buf[0]) {
		case '0':
				p->flag_set(0x0);
				break;
		case '1':
				p->flag_set(0x1);
				break;
		case '2':
				p->flag_set(0x2);
				break;
		default:
				break;
		}
		
		return count;
	} else {
		return -EIO;
	}
}

int __init share_region_init(void)
{
	unsigned int i;
	struct proc_dir_entry *dir;
	
	share_region = ioremap(SHARE_REGION_BASE, SHARE_REGION_SIZE);

	printk("share_region checksum = %lx, and flag = %x\n", share_region->checksum, share_region->flags.uboot_flag);
	if (!share_region)
		printk("++++++++++++++++++++++++++++++++++fail to reserve!\n");
#if 0
	for (i = 0; i < 10; i++) {
		printk("%d:[%0x] = %x\n", i, share_region + 4*i, *(unsigned int *)(share_region + 4*i));

	}
#endif 
	if (!check_share_region()) {
		printk("+++++++++++++++++++++++++++++++++++share_region invalid\n");
		clear_flags();
	}

	dir = proc_mkdir("share_region", NULL);
	if (!dir) {
		printk("could not create /proc/share_region\n");
	}
	
	for (i =0 ; i < (sizeof(flag_proc) / sizeof(*flag_proc)); i++) {
		struct proc_dir_entry *tmp;
		mode_t mode;

		mode = 0;
		if (flag_proc[i].flag_get) {
			mode |= S_IRUGO;
		}
		if (flag_proc[i].flag_set) {
			mode |= S_IWUGO;
		}
		
		tmp = create_proc_entry(flag_proc[i].path, mode, dir);
		if (tmp) {
			tmp->data = (void *)&flag_proc[i];
			tmp->read_proc = proc_flag_read;
			tmp->write_proc = proc_flag_write;
		}
		else {
			printk("could not create /proc/share_region/%s\n", flag_proc[i].path);
		}

	}
	return i;
}

late_initcall(share_region_init);
