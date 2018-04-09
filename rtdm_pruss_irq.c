#include <linux/module.h>
#include <rtdm/driver.h>
MODULE_LICENSE("GPL");

#define CM_PER 0x44e00000
#define CM_PER_SIZE 0x3fff

// ARM interrupt number for PRU event EVTOUT2
// this is Host 4 in the PRU intc, which maps to EVTOUT2
#define irq_number 23

#define PRU_SYSTEM_EVENT 21 // done
#define PRU_INTC_CHANNEL 5 // done
#define PRU_INTC_HOST PRU_INTC_CHANNEL

u32 PRU_INTC_HMR_REGS[] = {
	0x800,
	0x804,
	0x808,
};
#define AM33XX_INTC_PHYS_BASE 0x4a320000
#define PRU_INTC_SIZE 0x2000
#define PRU_INTC_SIPR0_REG   0xD00
#define PRU_INTC_SIPR1_REG   0xD04

#define PRU_INTC_ESR0_REG    0x300
#define PRU_INTC_ESR1_REG    0x304

#define PRU_INTC_SITR0_REG   0xD80
#define PRU_INTC_SITR1_REG   0xD84

static u32 PRU_INTC_CMR_REGS[] = {
	0x400,
	0x404,
	0x408,
	0x40C,
	0x410,
	0x414,
	0x418,
	0x41C,
	0x420,
	0x424,
	0x42C,
	0x430,
	0x434,
	0x438,
	0x43C,
};
#define PRU_INTC_SECR0_REG   0x280
#define PRU_INTC_SECR1_REG   0x284
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
	u32 pru_system_event = PRU_SYSTEM_EVENT;
	u32 pru_intc_secr_reg = PRU_INTC_SECR0_REG;

 	ctx = ((struct rtdm_pruss_irq_context*)irq_handle->cookie);
	// 4.4.2.3.6 Interrupt status clearing
	// check the pending enabled status (is it enabled AND has it been triggered?)
	status = ioread32(ctx->pruintc_io + pru_intc_secr_reg) & (1 << pru_system_event);
	if(status)
	{
		rtdm_event_signal(&ctx->event);
		// clear the event
		iowrite32((1 << pru_system_event), ctx->pruintc_io + pru_intc_secr_reg);
		return RTDM_IRQ_HANDLED;
	} else {
		return RTDM_IRQ_NONE;
	}
}

void init_pru(struct rtdm_pruss_irq_context *ctx){
	unsigned int value;
	unsigned int pru_intc_channel = PRU_INTC_CHANNEL;
	unsigned int pru_intc_host = PRU_INTC_HOST;
	unsigned int pru_system_event = PRU_SYSTEM_EVENT;
	ctx->pruintc_io = ioremap(AM33XX_INTC_PHYS_BASE, PRU_INTC_SIZE);
	// Set polarity of system events
	iowrite32(0xFFFFFFFF, ctx->pruintc_io + PRU_INTC_SIPR0_REG);
	iowrite32(0xFFFFFFFF,  ctx->pruintc_io + PRU_INTC_SIPR1_REG);
	// Set type of system events
	iowrite32(0x0, ctx->pruintc_io + PRU_INTC_SITR0_REG);
	iowrite32(0x0, ctx->pruintc_io + PRU_INTC_SITR1_REG);

	// PRU has 64 system events that are internally mapped
	// to 10 channels of the PRU's INTC.
	// These are in turn mapped to 10 host channels
	// which are exported to ARM's INTC.

	// for 16 <= pru_system_event < 32,
	// we can trigger event pru_system_event from the PRU by doing:
	// MOV R31.b0, (1 << 5) | (pru_system_event - 16)

	// 4.4.2.5 INTC Basic Programming Model
	// map system event pru_system_event to interrupt controller channel pru_intc_channel
	iowrite8(pru_intc_channel, ctx->pruintc_io + PRU_INTC_CMR_REGS[pru_system_event >> 2] + (pru_system_event & 3));

	// map PRU channel interrupt to host 
	iowrite8(pru_intc_host, ctx->pruintc_io + PRU_INTC_HMR_REGS[pru_intc_channel >> 2] + (pru_intc_channel & 3));

	//clear system events
	iowrite32(0xFFFFFFFF, ctx->pruintc_io + PRU_INTC_SECR0_REG);
	iowrite32(0xFFFFFFF, ctx->pruintc_io + PRU_INTC_SECR1_REG);
	
	//enable host interrupt
	//iowrite32((1 << PRU_INTC_HOST), ctx->pruintc_io + PRU_INTC_HIER_REG);
	//value = ioread32(ctx->pruintc_io + PRU_INTC_HIER_REG);
	//printk(KERN_WARNING "PRU_INTC_HIER_REG: %#x\n", value);

	// 4.4.3.2.1 INTC methodology > Interrupt Processing > Interrupt Enabling
	iowrite32(pru_system_event, ctx->pruintc_io + PRU_INTC_EISR_REG);

	// enable host interrupt output
	iowrite32(pru_intc_host, ctx->pruintc_io + PRU_INTC_HIEISR_REG);

	// Enable system event to trigger the output (not clearly written in the manual)
	if(pru_system_event < 32)
		iowrite32((1 << pru_system_event), ctx->pruintc_io + PRU_INTC_ESR0_REG); // TODO: disable this in _close()
	else
		iowrite32((1 << (pru_system_event - 32)), ctx->pruintc_io + PRU_INTC_ESR1_REG); // TODO: disable this in _close()

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
	iowrite32(0, ctx->pruintc_io + PRU_INTC_GER_REG); // TODO: only clear the enabled IRQ and do not disable the global

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
