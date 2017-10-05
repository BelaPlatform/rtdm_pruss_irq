#include <linux/module.h>
#include <rtdm/driver.h>
MODULE_LICENSE("GPL");

#define CM_PER 0x44e00000
#define CM_PER_SIZE 0x3fff

// ARM interrupt number for PRU event EVTOUT2
// this is Host 4 in the PRU intc, which maps to EVTOUT2
#define irq_number 22

#define PRU_SYSTEM_EVENT 20
#define PRU_INTC_CHANNEL 4
#define PRU_INTC_HOST PRU_INTC_CHANNEL

#define PRU_INTC_HMR1_REG    0x804 // this is PRU_INTC_HMR2_REG in __prussdrv.h
#define AM33XX_INTC_PHYS_BASE 0x4a320000
#define PRU_INTC_SIZE 0x2000
#define PRU_INTC_SIPR1_REG   0xD00
#define PRU_INTC_SIPR2_REG   0xD04

#define PRU_INTC_ESR0_REG    0x300 // this is PRU_INTC_ESR1_REG in __prussdrv.h

#define PRU_INTC_SITR1_REG   0xD80
#define PRU_INTC_SITR2_REG   0xD84

#define PRU_INTC_CMR5_REG    0x414 // note: __prussdrv.h calls this PRU_INTC_CMR6_REG
#define PRU_INTC_SECR1_REG   0x280
#define PRU_INTC_SECR2_REG   0x284
#define PRU_INTC_HIER_REG    0x1500
#define PRU_INTC_GER_REG     0x010
#define PRU_INTC_EISR_REG    0x028
#define PRU_INTC_HIEISR_REG  0x034


struct rtdm_pruss_irq_context {
	int* buf;
	int size;
	void* gpio1_addr;
	rtdm_irq_t irq_n;
	void* pruintc_io;
	unsigned int linux_irq;
	rtdm_event_t event;
	nanosecs_abs_t irq_start;
	nanosecs_abs_t irq_stop;
};

static int irq_handler(rtdm_irq_t *irq_handle){
	struct rtdm_pruss_irq_context *ctx;
	int status;

 	ctx = ((struct rtdm_pruss_irq_context*)irq_handle->cookie);
	// 4.4.2.3.5 Interrupt status clearing
	// check the pending enabled status (is it enabled AND has it been triggered?)
	status = ioread32(ctx->pruintc_io + PRU_INTC_SECR1_REG) & (1 << PRU_SYSTEM_EVENT);
	if(status)
	{
		rtdm_event_signal(&ctx->event);
		// clear the event
		iowrite32((1 << PRU_SYSTEM_EVENT), ctx->pruintc_io + PRU_INTC_SECR1_REG);
		return RTDM_IRQ_HANDLED;
	} else {
		return RTDM_IRQ_NONE;
	}
}

void init_pru(struct rtdm_pruss_irq_context *ctx){
	unsigned int value;
	ctx->pruintc_io = ioremap(AM33XX_INTC_PHYS_BASE, PRU_INTC_SIZE);
	// Set polarity of system events
	iowrite32(0xFFFFFFFF, ctx->pruintc_io + PRU_INTC_SIPR1_REG);
	iowrite32(0xFFFFFFFF,  ctx->pruintc_io + PRU_INTC_SIPR2_REG);
	// Set type of system events
	iowrite32(0x0, ctx->pruintc_io + PRU_INTC_SITR1_REG);
	iowrite32(0x0, ctx->pruintc_io + PRU_INTC_SITR2_REG);

	// we are triggering event 4 from the PRU by doing:
	// MOV R31.b0, (1 << 5) | 4
	// which is system event 20 of the PRU INTC controller

	// 4.4.2.5 INTC Basic Programming Model
	// map system event 20 to interrupt controller channel 4
	// we are using CMR5 because that is system event 20
	iowrite32(PRU_INTC_CHANNEL, ctx->pruintc_io + PRU_INTC_CMR5_REG);

	// map PRU channel interrupt to host 
	// TODO: possible conflict
	iowrite32(PRU_INTC_HOST, ctx->pruintc_io + PRU_INTC_HMR1_REG); //TODO: write only 8 bits

	//clear system events
	iowrite32(0xFFFFFFFF, ctx->pruintc_io + PRU_INTC_SECR1_REG);
	iowrite32(0xFFFFFFF, ctx->pruintc_io + PRU_INTC_SECR2_REG);
	
	//enable host interrupt
	//iowrite32((1 << PRU_INTC_HOST), ctx->pruintc_io + PRU_INTC_HIER_REG);
	//value = ioread32(ctx->pruintc_io + PRU_INTC_HIER_REG);
	//printk(KERN_WARNING "PRU_INTC_HIER_REG: %#x\n", value);

	// 4.4.3.2.1 INTC methodology > Interrupt Processing > Interrupt Enabling
	iowrite32(PRU_SYSTEM_EVENT, ctx->pruintc_io + PRU_INTC_EISR_REG);

	// enable host interrupt output
	iowrite32(PRU_INTC_CHANNEL, ctx->pruintc_io + PRU_INTC_HIEISR_REG);

	// not written in the manual
	iowrite32((1 << PRU_SYSTEM_EVENT), ctx->pruintc_io + PRU_INTC_ESR0_REG);

	// enable Global Enable Register
	iowrite32(1, ctx->pruintc_io + PRU_INTC_GER_REG);
	value = ioread32(ctx->pruintc_io + PRU_INTC_HIER_REG);
}

void init_intc(struct rtdm_pruss_irq_context *ctx){
	struct device_node *of_node = of_find_node_by_name(NULL, "interrupt-controller");
	struct irq_domain *intc_domain = irq_find_matching_fwnode(&of_node->fwnode, DOMAIN_BUS_ANY); 
	ctx->linux_irq = irq_create_mapping(intc_domain, irq_number);
	int res = rtdm_irq_request(&ctx->irq_n, ctx->linux_irq, irq_handler, 0, "rtdm_pruss_irq_irq", (void*)ctx);
	printk(KERN_ALERT "rtdm_pruss_irq linux_irq: %i\n", ctx->linux_irq);
	if(res != 0)
		printk(KERN_ALERT "rtdm interrupt registered: %i FAILED\n", res);
}

static int rtdm_pruss_irq_open(struct rtdm_fd *fd, int oflags){
	struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);
	
	printk(KERN_WARNING "rtdm_pruss_irq_open\n");

	init_intc(ctx);
	init_pru(ctx);
	rtdm_event_init(&ctx->event, 0);
	ctx->buf = (int*)rtdm_malloc(4096);
	return 0;
}

static void rtdm_pruss_irq_close(struct rtdm_fd *fd){
	struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);
	// disable the Global Enable Register of the PRU INTC
	iowrite32(0, ctx->pruintc_io + PRU_INTC_GER_REG);

	rtdm_irq_free(&ctx->irq_n);
	rtdm_free(ctx->buf);
	rtdm_event_pulse(&ctx->event);
	rtdm_event_destroy(&ctx->event);
	//irq_dispose_mapping(ctx->linux_irq); // calling this causes a stack trace in dmesg
	rtdm_printk("rtdm_pruss_irq closed\n");
}

static ssize_t rtdm_pruss_irq_write(struct rtdm_fd *fd, const void __user *buf, size_t size){
	// struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);
	// res = rtdm_copy_from_user(fd, ctx->buf, buf, size);
	// ctx->size = size;

	rtdm_printk(KERN_WARNING "rtdm_pruss_irq:write() Nothing happening\n");
	return size;
}

static ssize_t rtdm_pruss_irq_read_nrt(struct rtdm_fd *fd, void __user *buf, size_t size){
	printk(KERN_ALERT "Trying to read non-realtime. \n");
	return size;
}

static ssize_t rtdm_pruss_irq_read(struct rtdm_fd *fd, void __user *buf, size_t size){
	int ret;
	int value = 0;
	struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);

	nanosecs_rel_t timeout = 100000000;
	// wait till event gets signalled or timeout occurs
	// if the event has been signalled already, will return immediately
	ret = rtdm_event_timedwait(&ctx->event, timeout, NULL);
	//rtdm_printk(KERN_WARNING "rtdm_pruss_irq event got released\n");

	//rtdm_copy_to_user(fd, buf, &ret, size);
	// ret will be 0 on success or -ETIMEDOUT if the PRU interrupt did not come through
	if(ret)
		return -ETIMEDOUT;
	else
		return 0;
}

static struct rtdm_driver rtdm_pruss_irq_driver = {
	.profile_info		= RTDM_PROFILE_INFO(rtdm_test_rtdm_pruss_irq,
						    RTDM_CLASS_EXPERIMENTAL,
						    RTDM_SUBCLASS_GENERIC,
						    0),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 1,
	.context_size		= sizeof(struct rtdm_pruss_irq_context),
	.ops = {
		.open		= rtdm_pruss_irq_open,
		.close		= rtdm_pruss_irq_close,
		//.ioctl_rt	= rtdm_basic_ioctl_rt,
		//.ioctl_nrt	= rtdm_basic_ioctl_nrt,
		.read_nrt	= rtdm_pruss_irq_read_nrt,
		.write_nrt	= rtdm_pruss_irq_write,
		.read_rt	= rtdm_pruss_irq_read,
		.write_rt	= rtdm_pruss_irq_write,
	},
};


static struct rtdm_device device = {
	.driver = &rtdm_pruss_irq_driver,
	.label = "rtdm_pruss_irq_%d",
};

static int __init rtdm_pruss_irq_init(void){
	printk(KERN_WARNING "rtdm_pruss_irq loaded\n");
	return rtdm_dev_register(&device);
}

static void __exit rtdm_pruss_irq_exit(void){
	printk(KERN_WARNING "rtdm_pruss_irq unloaded\n");
	return rtdm_dev_unregister(&device);
}

module_init(rtdm_pruss_irq_init);
module_exit(rtdm_pruss_irq_exit);
