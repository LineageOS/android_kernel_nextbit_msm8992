#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/setup.h>

#define VERIFIED_BOOTSTATE_EQUALS "androidboot.verifiedbootstate="
#define SANITIZED_VERIFIED_BOOTSTATE VERIFIED_BOOTSTATE_EQUALS "green"

static char cleaned_cmdline[COMMAND_LINE_SIZE];
static bool sanitize_verifiedbootstate = false;

module_param(sanitize_verifiedbootstate, bool, S_IRUGO | S_IWUSR);

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	if (sanitize_verifiedbootstate)
		seq_printf(m, "%s %s\n", cleaned_cmdline, SANITIZED_VERIFIED_BOOTSTATE);
	else
		seq_printf(m, "%s\n", saved_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void save_cleaned_cmdline(const char *cmdline)
{
	const char *vbs = strstr(cmdline, VERIFIED_BOOTSTATE_EQUALS);
	if (vbs) {
		ssize_t len = vbs-cmdline;
		const char *post = strchr(vbs, ' ');
		strncpy(cleaned_cmdline, cmdline, len);
		if (post) {
		    strcpy(&cleaned_cmdline[len], post+1);
		} else {
		    cleaned_cmdline[len] = '\0';
		}
	} else {
		strcpy(cleaned_cmdline, cmdline);
	}
}

static int __init proc_cmdline_init(void)
{
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	save_cleaned_cmdline(saved_command_line);
	return 0;
}
module_init(proc_cmdline_init);
