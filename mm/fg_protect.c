#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/swap.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/vmstat.h>
#include <linux/file.h>
#include <linux/writeback.h>
#include <linux/buffer_head.h>
#include <linux/mm_inline.h>
#include <linux/rmap.h>
#include <linux/compaction.h>
#include <linux/memcontrol.h>
#include <linux/sysctl.h>
#include <linux/printk.h>
#include <linux/proc_fs.h>

#define MAX_BUF_LEN 10

unsigned long fg_mapped_file = 0;

static atomic_long_t fg_mapcount_debug_1[21] = {
	ATOMIC_INIT(0)
};

static atomic_t fg_mapcount = ATOMIC_INIT(0);
static atomic_t fg_mapcount_enable = ATOMIC_INIT(0);
unsigned long  memavail_noprotected = 0;
static bool fg_protect_setup = false;

extern s64 get_mem_cgroup_app_uid(struct mem_cgroup *memcg);
extern bool is_fg(int uid);
extern struct mem_cgroup *get_next_memcg(struct mem_cgroup *prev);
extern void get_next_memcg_break(struct mem_cgroup *memcg);


static bool mem_available_is_low(void)
{
	long available = si_mem_available();

	if (available < memavail_noprotected)
		return true;

	return false;
}


static void update_fg_mapcount(long fg_mapped_file) {
	unsigned long fg_mapped_file_mb = fg_mapped_file >> 8;
	if (fg_mapped_file_mb  < 100)
		atomic_set(&fg_mapcount, 0);
	else if (fg_mapped_file_mb  < 150)
		atomic_set(&fg_mapcount, 1);
	else if (fg_mapped_file_mb  < 200)
		atomic_set(&fg_mapcount, 2);
	else if (fg_mapped_file_mb  < 250)
		atomic_set(&fg_mapcount, 3);
	else if (fg_mapped_file_mb  < 300)
		atomic_set(&fg_mapcount, 4);
	else if (fg_mapped_file_mb  < 350)
		atomic_set(&fg_mapcount, 5);
	else if (fg_mapped_file_mb  < 400)
		atomic_set(&fg_mapcount, 6);
	else if (fg_mapped_file_mb  < 450)
		atomic_set(&fg_mapcount, 7);
	else if (fg_mapped_file_mb  < 500)
		atomic_set(&fg_mapcount, 8);
	else if (fg_mapped_file_mb  < 550)
		atomic_set(&fg_mapcount, 9);
	else if (fg_mapped_file_mb  < 600)
		atomic_set(&fg_mapcount, 10);
	else
		atomic_set(&fg_mapcount, 20);
}

unsigned long page_should_fg_protect(struct page *page)
{
	int file;

	struct mem_cgroup *memcg = NULL;
	int uid;
	long fg_mapped_file = 0;

	file = is_file_lru(page_lru(page));

	if (unlikely(!fg_protect_setup))
		return 0;

	if (unlikely(!page_evictable(page) || PageUnevictable(page)))
		return 0;

	if (unlikely(mem_available_is_low()))
		return 0;

	if (atomic_read(&fg_mapcount_enable) && file && page_memcg(page)) {
		memcg = page_memcg(page);
		uid = (int)get_mem_cgroup_app_uid(memcg);
		if (is_fg(uid)) {
			fg_mapped_file = atomic_long_read(&memcg->stat[NR_FILE_MAPPED]);
			update_fg_mapcount(fg_mapped_file);
			if (page_mapcount(page) > atomic_read(&fg_mapcount)) {
				return hpage_nr_pages(page);
			}
		}
	}
    return 0;
}

static void do_traversal_fg_memcg()
{
	struct page *page;
	struct lruvec* lruvec;
	int lru, i;
	struct mem_cgroup *memcg = NULL;
	pg_data_t *pgdat = NODE_DATA(0);

	while ((memcg = get_next_memcg(memcg))) {
		if (is_fg(get_mem_cgroup_app_uid(memcg))) {
			lruvec = mem_cgroup_lruvec(pgdat, memcg);
			fg_mapped_file = atomic_long_read(&memcg->stat[NR_FILE_MAPPED]);
			spin_lock_irq(&pgdat->lru_lock);
			for_each_evictable_lru(lru) {
				int file = is_file_lru(lru);
				if (file) {
					list_for_each_entry(page, &lruvec->lists[lru], lru) {
						if (!page)
							continue;
						if (PageHead(page)) {
							for (i = 0; i < compound_nr(page); i++) {
								if (page_mapcount(page) < 20)
									atomic_long_add(1, &fg_mapcount_debug_1[page_mapcount(page)]);
								else
									atomic_long_add(1, &fg_mapcount_debug_1[20]);
							}
							continue;
						}
						if (page_mapcount(page) < 20)
							atomic_long_add(1, &fg_mapcount_debug_1[page_mapcount(page)]);
						else
							atomic_long_add(1, &fg_mapcount_debug_1[20]);
					}
				}
			}
			spin_unlock_irq(&pgdat->lru_lock);
			get_next_memcg_break(memcg);
			break;
		}
	}
}

static int fg_protect_show(struct seq_file *m, void *arg)
{
	int i = 0;
	memset(fg_mapcount_debug_1, 0, sizeof(fg_mapcount_debug_1));
	do_traversal_fg_memcg();

	seq_printf(m,
		   "fg_mapcount:     %d\n",
		   fg_mapcount);
	seq_printf(m,
		   "fg_mapped_file:     %lu\n",
		   fg_mapped_file);

	for (i = 0; i < 21; i++) {
		seq_printf(m,
			   "fg_mapcount_debug_%d:     %lu\n",
			   i, fg_mapcount_debug_1[i]);
	}

	seq_putc(m, '\n');

	return 0;
}

static int fg_protect_open(struct inode *inode, struct file *file)
{
    return single_open(file, fg_protect_show, NULL);
}

static const struct file_operations fg_protect_file_operations = {
	.open		= fg_protect_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static ssize_t fg_mapcount_enable_ops_write(struct file *file,
			const char __user *buff, size_t len, loff_t *ppos)
{
	int ret;
	char kbuf[MAX_BUF_LEN] = {'\0'};
	char *str;
	int val;

	if (len > MAX_BUF_LEN - 1) {
		return -EINVAL;
	}

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	str = strstrip(kbuf);
	if (!str) {
		return -EINVAL;
	}

	ret = kstrtoint(str, 10, &val);
	if (ret) {
		return -EINVAL;
	}

	if (val < 0 || val > INT_MAX) {
		return -EINVAL;
	}

	printk("fg_mapcount_ops_write is %d\n", val);
	atomic_set(&fg_mapcount_enable, val);

	return len;
}


static ssize_t fg_mapcount_enable_ops_read(struct file *file,
			char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[MAX_BUF_LEN] = {'\0'};
	int len;

	len = snprintf(kbuf, MAX_BUF_LEN, "%d\n", fg_mapcount_enable);

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct file_operations fg_mapcount_enable_ops = {
	.write = fg_mapcount_enable_ops_write,
	.read = fg_mapcount_enable_ops_read,
};

static int __init fg_protect_init(void)
{
    struct proc_dir_entry *pentry;
	pg_data_t *pgdat;
	static struct proc_dir_entry *enable_entry;
	memavail_noprotected = totalram_pages / 10;

	fg_protect_setup = true;
	proc_create("fg_protect_show", 0400, NULL, &fg_protect_file_operations);
	enable_entry = proc_create("fg_protect_enable", 0666, NULL, &fg_mapcount_enable_ops);

	if (!enable_entry)
		return -ENOMEM;

	return 0;
}

module_init(fg_protect_init);
