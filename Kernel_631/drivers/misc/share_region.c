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
#include <asm/uaccess.h>
#include <asm/fcntl.h>
#include <linux/delay.h>

//#define DEBUG

static struct share_region_t *share_region = NULL;

#ifdef DEBUG
static void dump_share_region(void) {
	int i = 0;
	if (!share_region) {
		printk("[%s] No share reg addr\n",__func__);
		return;
	}
	printk("-----[%s]-----\n",__func__);
	for (i = 0; i < SHARE_REGION_SIZE; i++) {
		printk("<c> %hhx ",((char *)share_region)[i]);
	}
	printk("\n-----[END]-----\n");
}
#else
static void dump_share_region(void) {};
#endif

static int read_share_region(void) {
#ifdef IS_UPDKEY_IN_MMC
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;
    int ret = 0;

    loff_t offset = UPD_BLOCK_NUM * 512;

    if (!share_region) {
    	printk("[%s] No share region addr\n",__func__);
    	return -1;
    }

    //printk("[%s] sh=%li offset = %lli\n",__func__, share_region, offset);
    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(UPD_MMC, O_RDONLY, 0);
    if(IS_ERR(filp)) {
    	err = PTR_ERR(filp);
    	printk("[%s] Error while opening %s (%i)\n",__func__, UPD_MMC, err);
    	return err;
    }
    ret = vfs_read(filp, (char *)share_region, SHARE_REGION_SIZE, &offset);
    filp_close(filp, NULL);
    set_fs(oldfs);
    dump_share_region();
    return !(ret == SHARE_REGION_SIZE);
#else
    return 0;
#endif
}

static int write_share_region(void) {
#ifdef IS_UPDKEY_IN_MMC
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;
    int ret = 0;

    loff_t offset = UPD_BLOCK_NUM * 512;

    if (!share_region) {
    	printk("[%s] No share region addr\n",__func__);
    	return -1;
    }

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(UPD_MMC, O_WRONLY, 0);
    if(IS_ERR(filp)) {
    	err = PTR_ERR(filp);
    	printk("[%s] Error while opening %s (%i)\n",__func__, UPD_MMC, err);
    	return err;
    }
    ret = vfs_write(filp, (const char *)share_region, SHARE_REGION_SIZE, &offset);
    filp_close(filp, NULL);
    set_fs(oldfs);

    dump_share_region();
    return !(ret == SHARE_REGION_SIZE);
#else
    return 0;
#endif
}

static unsigned bg_crc32(unsigned char *buf, int len)
{
	return ~crc32_le(~0L, buf, len);
}

void clear_flags(void)
{
	if (!share_region) return;

	share_region->flags.uboot_flag = 0;
	share_region->flags.kernel_flag = 0;
}

static void save_share_region(void)
{
	if (share_region) {
		share_region->checksum = bg_crc32((u8 *)share_region, SHARE_REGION_SIZE - sizeof(unsigned long));
		write_share_region();
	}
}

static int check_share_region (void)
{
	int result = 0;
	if (read_share_region()) {
		printk("[%s] Read error\n",__func__);
		return result;
	}

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
	read_share_region();
	if (!share_region) return -1;

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

int share_region_init(void)
{
	unsigned int i;
	int *sh;
	struct proc_dir_entry *dir;
	
#ifdef IS_UPDKEY_IN_MMC
    if (share_region == NULL) {
    	share_region = kmalloc(SHARE_REGION_SIZE,GFP_KERNEL);
    	if (share_region == NULL) return -1;
    	memset(share_region,0,SHARE_REGION_SIZE);
    }
#else
	share_region = ioremap(SHARE_REGION_BASE, SHARE_REGION_SIZE);
	sh = phys_to_virt(share_region);
	printk("sh=%p; SHARE_REGION_BASE = %p; share_region = %p; \nshare_region checksum = %lx, and flag = %x\n", sh,SHARE_REGION_BASE,
			share_region, share_region->checksum, share_region->flags.uboot_flag);
	if (!share_region)
		printk("++++++++++++++++++++++++++++++++++fail to reserve!\n");
#if 0
	for (i = 0; i < 10; i++) {
		//printk("%d:[%0x] = %x\n", i, share_region + 4*i, *(unsigned int *)(share_region + 4*i));
		printk("%d:[%0x] = %x\n", i, sh + 4*i, *(unsigned int *)(sh + 4*i));

	}
#endif 
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

#ifdef IS_UPDKEY_IN_MMC
DECLARE_DELAYED_WORK(sri,share_region_init);
int share_region_init_(void) {
	schedule_delayed_work(&sri,msecs_to_jiffies(500));
}
late_initcall(share_region_init_);
#else
late_initcall(share_region_init);
#endif
