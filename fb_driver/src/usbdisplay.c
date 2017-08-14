#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/file.h> 
#include <linux/syscalls.h> 
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

/* some IO methods
   since accessing filesystem from kernel space
   could be painful */
struct file *cfopen(const char *path) 
{
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path,  O_CREAT|O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
    set_fs(oldfs);
    if (IS_ERR(filp)) {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}
void cfclose(struct file *file) 
{
    filp_close(file, NULL);
}
int cfwrite(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

static void write_file(char *filename, uint8_t* data, long len)
{
  struct file *file = cfopen(filename);
  if (file == NULL) {
  	printk(KERN_ERR "write_file error");
  }
  //printk(KERN_INFO "Seems like OPENED");

  cfwrite(file, 0, data, len);

  cfclose(file);
  //printk(KERN_INFO "Seems like OK");
}

#define FB_NAME             "uart_arduino_"

//adjust for your needs
#define WIDTH               320
#define HEIGHT              240
#define BITS_PER_PIXEL      16
#define DUMP_PATH           "/home/pi/dump.bin"

#define VIDEOMEMSIZE        153600

//here all the magic is stored
static struct fb_info fb_info;

//fixed params
static struct fb_fix_screeninfo sfb_fix __initdata = {
    .id         =   FB_NAME,
    .smem_len   =   VIDEOMEMSIZE,
    .type       =   FB_TYPE_PACKED_PIXELS,
    .visual     =   FB_VISUAL_TRUECOLOR,
    .xpanstep   =   0,
    .ypanstep   =   0,
    .ywrapstep  =   0, 
    .accel      =   FB_ACCEL_NONE,
    .line_length =	WIDTH  * BITS_PER_PIXEL / 8,
};

//variable params
//actually I don't know why this is called this way
static struct fb_var_screeninfo sfb_default __initdata = {
    .xres           = WIDTH,
    .yres           = HEIGHT,
    .xres_virtual   = WIDTH,
    .yres_virtual   = HEIGHT,
    .width          = WIDTH,      
    .height         = HEIGHT,
    .bits_per_pixel = BITS_PER_PIXEL,
    //NOTE: here we have BGR565, NOT RGB565/RGB888/etc
    //You will likely have to adjust this
    //bits for color= offset, count, something
    .red            = {11, 5, 0},
    .green          = {5, 6, 0},
    .blue           = {0, 5, 0},
    .transp			= {0, 0, 0},
    .grayscale      = 0,
    .activate       = FB_ACTIVATE_NOW,
    .pixclock       = 30060,
    .vmode          = FB_VMODE_NONINTERLACED,
};

//this will be launched on update
static void sfb_deferred_io(struct fb_info *info, struct list_head *pagelist) {
	//printk(KERN_INFO "Seems like deferred io is launched");
	//volatile uint16_t curcolor = *(uint16_t*)(fb_info.screen_base + 320 * 2 + 4);
	//printk(KERN_INFO "Seems like we havent crashed!");
	//printk(KERN_INFO "The address is %p", fb_info.screen_base);
	//printk(KERN_INFO "Color at (2;2) is %x!", curcolor);

    //here we will only dump a framebuffer to file
  	write_file(DUMP_PATH, fb_info.screen_base, VIDEOMEMSIZE);
	//printk(KERN_INFO "Seems like we STILL havent crashed!");
}

//available operations
//we do not have to implement all of them
static struct fb_ops sfb_ops = {
    .owner          = THIS_MODULE,
    //.fb_open        = xxxfb_open,
    //.fb_read        = fb_sys_read,
    //.fb_write       = fb_sys_write,
    //.fb_release     = xxxfb_release,
    //.fb_check_var   = xxxfb_check_var,
    //.fb_set_par     = xxxfb_set_par,
    //.fb_setcolreg = xxxfb_setcolreg,
    //.fb_blank       = xxxfb_blank,
    //.fb_pan_display = xxxfb_pan_display,
    .fb_fillrect    = cfb_fillrect,   /* Needed !!! */
    .fb_copyarea    = cfb_copyarea,   /* Needed !!! */
    .fb_imageblit   = cfb_imageblit,  /* Needed !!! */
    //.fb_cursor      = xxxfb_cursor,     
    //.fb_sync        = xxxfb_sync,
    //.fb_ioctl       = xxxfb_ioctl,
    //.fb_mmap        = xxxfb_mmap,
};

static struct fb_deferred_io sfb_defio = {
	.delay		=  10, /*or HZ for one second*/
	.deferred_io	= sfb_deferred_io,
};


int xxxfb_init(void) {

    //NOTE: vzalloc gives pointer, there is int conversion because compiler is mad
	sfb_fix.smem_start = (int)(vzalloc(sfb_fix.smem_len));
	fb_info.screen_base = (uint8_t __force __iomem* )sfb_fix.smem_start;
	printk(KERN_INFO "fbX: The smem address is %p", (void*)sfb_fix.smem_start);

    fb_info.fbops = &sfb_ops;
    fb_info.var = sfb_default;
    fb_info.fix = sfb_fix;
    fb_info.flags = FBINFO_FLAG_DEFAULT;
    fb_info.fbdefio = &sfb_defio;
 
    fb_alloc_cmap(&fb_info.cmap, 256, 0);

	if (register_framebuffer(&fb_info) < 0) {
        return -EINVAL;
    }

   	fb_deferred_io_init(&fb_info);
 
    printk(KERN_INFO "fb%d: Sample frame buffer device initialized \n", fb_info.node);
	return 0;
}

//TODO: fix being able to unmount this
void xxxfb_exit(void) {
	fb_deferred_io_cleanup(&fb_info);
	vfree(fb_info.screen_base);
	unregister_framebuffer(&fb_info);
	return;
}

module_init(xxxfb_init);
module_exit(xxxfb_exit);
MODULE_AUTHOR("Nikita Tomilov <nikita.tomilov@emlid.com>");
MODULE_LICENSE("GPL v2");