#include <linux/module.h>
#include <rtdm/driver.h>
#include "rtdm_pruss_irq.h"
MODULE_LICENSE("GPL");

#define PRU_ICSS_EVTOUT0 20 // TRM 6.3, table 6-1

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
	0x428,
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
	int size;
	void* gpio1_addr;
	rtdm_irq_t irq_handle;
	int irq_handle_inited;
	void* pruintc_io;
	unsigned int linux_irq;
	rtdm_event_t event;
	int event_inited;
	nanosecs_abs_t irq_start;
	nanosecs_abs_t irq_stop;
	u32 pru_intc_channel;
	u32 pru_intc_host;
	s32 arm_irq_handle_number;
	u8* pru_system_events;
	u8 pru_system_events_count;
};

static int irq_handler(rtdm_irq_t *irq_handle){
	struct rtdm_pruss_irq_context *ctx;
	int status;
	u32 pru_system_event;
	u32 pru_system_event_bit;
	u32 pru_intc_secr_reg;

 	ctx = ((struct rtdm_pruss_irq_context*)irq_handle->cookie);
	pru_system_event = ctx->pru_system_events[0];
	pru_intc_secr_reg = pru_system_event < 32 ? PRU_INTC_SECR0_REG : PRU_INTC_SECR1_REG;
	pru_system_event_bit = pru_system_event & 31;
	// 4.4.2.3.6 Interrupt status clearing
	// check the pending enabled status (is it enabled AND has it been triggered?)
	status = ioread32(ctx->pruintc_io + pru_intc_secr_reg) & (1 << pru_system_event_bit);
	if(status)
	{
		rtdm_event_signal(&ctx->event);
		// clear the event
		iowrite32((1 << pru_system_event_bit), ctx->pruintc_io + pru_intc_secr_reg);
		return RTDM_IRQ_HANDLED;
	} else {
		return RTDM_IRQ_NONE;
	}
}

static void* pOffset;
static void iowrite8p(u8 val, void* dest)
{
	printk(KERN_WARNING ": %#X = %#X [u8]\n", dest - pOffset, val);
	iowrite8(val, dest);
}
static void iowrite32p(u32 val, void* dest)
{
	printk(KERN_WARNING ": %#X = %#X [u32]\n", dest - pOffset, val);
	iowrite32(val, dest);
}
static void init_pru(struct rtdm_pruss_irq_context *ctx){
	u32 value;
	u32 pru_intc_channel = ctx->pru_intc_channel;
	u32 pru_intc_host = ctx->pru_intc_host;
	u32 n;
	for(n = 0; n < ctx->pru_system_events_count; ++n)
		printk(KERN_WARNING "INITING_PRU: system_event: %u, intc_channel: %u, intc_host: %u\n", ctx->pru_system_events[n], pru_intc_channel, pru_intc_host);
	ctx->pruintc_io = ioremap(AM33XX_INTC_PHYS_BASE, PRU_INTC_SIZE);
	pOffset = ctx->pruintc_io;
	// 4.4.2.5 INTC Basic Programming Model
	// 4.4.2.5 (1.)
	// Set polarity of system events: HIGH is active
	// Polarity all system events is always high.
	iowrite32p(0xFFFFFFFF, ctx->pruintc_io + PRU_INTC_SIPR0_REG);
	iowrite32p(0xFFFFFFFF,  ctx->pruintc_io + PRU_INTC_SIPR1_REG);
	// 4.4.2.5 (1.)
	// Set type of system events: PULSE
	// Type of all system events is always pulse.
	iowrite32p(0x0, ctx->pruintc_io + PRU_INTC_SITR0_REG);
	iowrite32p(0x0, ctx->pruintc_io + PRU_INTC_SITR1_REG);

	// PRU has 64 system events that are internally mapped
	// to 10 channels of the PRU's INTC.
	// These are in turn mapped to 10 host channels, of which
	// 0 and 1 are exported to PRUs' R31 bit 30 and 31
	// while 2 to 9 are exported to ARM's INTC.

	// 4.4.2.5 (2.)
	// map system event pru_system_event to interrupt controller channel pru_intc_channel
        printk(KERN_WARNING "cmr_reg");
	for(n = 0; n < ctx->pru_system_events_count; ++n)
	{
		u8 pru_system_event = ctx->pru_system_events[n];
		iowrite8p(pru_intc_channel, ctx->pruintc_io + PRU_INTC_CMR_REGS[pru_system_event >> 2] + (pru_system_event & 3));
	}
	// 4.4.2.5 (3.)
	// map PRU channel interrupt to host 
        printk(KERN_WARNING "hmr_reg");
	iowrite8p(pru_intc_host, ctx->pruintc_io + PRU_INTC_HMR_REGS[pru_intc_channel >> 2] + (pru_intc_channel & 3));

	// 4.4.2.5 (4.)
	//clear system events
        printk(KERN_WARNING "secr0");
	iowrite32p(0xFFFFFFFF, ctx->pruintc_io + PRU_INTC_SECR0_REG);
        printk(KERN_WARNING "secr1");
	iowrite32p(0xFFFFFFF, ctx->pruintc_io + PRU_INTC_SECR1_REG);
	
	// 4.4.2.5 (5.)
	//enable host interrupt

	// 4.4.3.2.1 INTC methodology > Interrupt Processing > Interrupt Enabling
	// this register is write-only
	printk(KERN_WARNING "eisr_reg");
	for(n = 0; n < ctx->pru_system_events_count; ++n)
	{
		u8 pru_system_event = ctx->pru_system_events[n];
		iowrite32p(pru_system_event, ctx->pruintc_io + PRU_INTC_EISR_REG); // TODO: disable this in _close() (EICR)
	}

	// enable host interrupt output
	printk(KERN_WARNING "hieisr_reg");
	iowrite32p(pru_intc_host, ctx->pruintc_io + PRU_INTC_HIEISR_REG); // TODO: disable this in _close() (HIDISR)

	// HIER: The Host Interrupt Enable Registers enable or disable individual host interrupts. These work separately from the global enables. There is one bit per host interrupt. These bits are updated when writing to the Host Interrupt Enable Index Set and Host Interrupt Enable Index Clear registers.
        printk(KERN_WARNING "hier");
	//iowrite32p((1 << pru_intc_host), ctx->pruintc_io + PRU_INTC_HIER_REG);
	value = ioread32(ctx->pruintc_io + PRU_INTC_HIER_REG);
        printk(KERN_WARNING "hier read: %x\n", value);

	// Enable system event to trigger the output (not clearly exlplained in the manual)
	for(n = 0; n < ctx->pru_system_events_count; ++n)
	{
		u8 pru_system_event = ctx->pru_system_events[n];
		if(pru_system_event < 32)
		{
			printk(KERN_WARNING "esr0");
			iowrite32p((1 << pru_system_event), ctx->pruintc_io + PRU_INTC_ESR0_REG); // TODO: disable this in _close() (ECR0)
		}
		else
		{
			printk(KERN_WARNING "esr1");
			iowrite32p((1 << (pru_system_event - 32)), ctx->pruintc_io + PRU_INTC_ESR1_REG); // TODO: disable this in _close() (ECR1)
		}
	}

	// 4.4.2.5 (7.)
	// enable Global Enable Register
	printk(KERN_WARNING "intc_ger");
	iowrite32p(1, ctx->pruintc_io + PRU_INTC_GER_REG);
}




static int init_arm_intc(struct rtdm_pruss_irq_context *ctx){
	int res;
	struct device_node *of_node = of_find_node_by_name(NULL, "interrupt-controller");
	struct irq_domain *intc_domain = irq_find_matching_fwnode(&of_node->fwnode, DOMAIN_BUS_ANY); 
	ctx->linux_irq = irq_create_mapping(intc_domain, ctx->arm_irq_handle_number);
	res = rtdm_irq_request(&ctx->irq_handle, ctx->linux_irq, irq_handler, 0, "rtdm_pruss_irq_irq", (void*)ctx);
	printk(KERN_INFO "rtdm_pruss_irq linux_irq: %i for arm_irq_handle_number: %i\n", ctx->linux_irq, ctx->arm_irq_handle_number);
	if(res != 0)
	{
		printk(KERN_WARNING "rtdm interrupt registered: %i FAILED\n", res);
		return -1;
	}
	ctx->irq_handle_inited = 1;
	return 0;
}

static int refcount = 0;
static int allocate_pru_system_events(struct rtdm_pruss_irq_context* ctx)
{
	ctx->pru_system_events = (u8*)rtdm_malloc(sizeof(ctx->pru_system_events[0]) * ctx->pru_system_events_count);
	if(!ctx->pru_system_events)
		return 1;
	return 0;
}
static int rtdm_pruss_ioctl(struct rtdm_fd* fd, unsigned int request, void __user* arg){
	int err;
	u32 n;
	struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);
	struct rtdm_pruss_irq_registration reg;

	if(_IOC_TYPE(request) != RTDM_PRUSS_IRQ_IOC_MAGIC)
	{
		printk(KERN_WARNING "Unknown ioctl");
		return -ENOTTY;
	}
	// if user wants to write, check that we can read
	// from user the appropriate size
	if(_IOC_DIR(request) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, arg, _IOC_SIZE(request));
	if(err)
	{
		printk(KERN_WARNING "ioctl: unable to access argument memory");
		return -EFAULT;
	}

	// TODO: guard against device removal while executing this ioctl.
	// (using spinlocks)
	switch(request)
	{
		case RTDM_PRUSS_IRQ_REGISTER_FULL:
			err = rtdm_copy_from_user(fd, &reg, arg, sizeof(struct rtdm_pruss_irq_registration));
			if(err)
			{
				return -1;
			}
			ctx->pru_system_events_count = reg.pru_system_events_count;
			allocate_pru_system_events(ctx);
			err = !access_ok(VERIFY_READ, reg.pru_system_events, _IOC_SIZE(reg.pru_system_events_count * sizeof(reg.pru_system_events[0])));
			if(err)
			{
				printk(KERN_WARNING "ioctl: unable to access pru_system_events");
				return -EFAULT;
			}

			err = rtdm_copy_from_user(fd,
					ctx->pru_system_events,
					(u8 __user*)reg.pru_system_events,
					sizeof(ctx->pru_system_events[0]) * ctx->pru_system_events_count);
			for(n = 0; n < ctx->pru_system_events_count; ++n)
				printk(KERN_WARNING "events: %u\n", ctx->pru_system_events[n]);
			ctx->pru_intc_channel = reg.pru_intc_channel;
			ctx->pru_intc_host = reg.pru_intc_host;
			break;
		case RTDM_PRUSS_IRQ_REGISTER:
			ctx->pru_system_events_count = 1;
			allocate_pru_system_events(ctx);
			ctx->pru_system_events[0] = (u8)(u32)arg;
			// apply "default" mapping
			// (ranges checked below)
			ctx->pru_intc_channel = ctx->pru_system_events[0] - 16;
			ctx->pru_intc_host = ctx->pru_intc_channel;
			break;
		default:
			break;
	}
	if(
		ctx->pru_system_events[0] > 64 || // TODO: check other pru_sytem_events as well
		ctx->pru_intc_channel > 9 ||
		ctx->pru_intc_host > 9
	)
	{
		return -EINVAL;
	}
	// -2 because PRUSS INTC 2:9 are mapped to PRU_ICSS_EVTOUT0:PRU_ICSS_EVTOUT7
	// (channels 0 and 1 cannot be routed to ARM)
	ctx->arm_irq_handle_number = ctx->pru_intc_host >= 2 ? ctx->pru_intc_channel - 2 + PRU_ICSS_EVTOUT0 : -1;
	printk(KERN_INFO
		"Registering PRU interrupt:\n"
		"  PRU ICSS system event: %u\n"
		"  PRU INTC channel: %u\n"
		"  PRU INTC host interrupt: %u\n"
		"  ARM IRQ number:  %d\n",
		ctx->pru_system_events[0], // TODO: check other pru_system_events_as_well
		ctx->pru_intc_channel,
		ctx->pru_intc_host,
		ctx->arm_irq_handle_number
	);
	if(ctx->arm_irq_handle_number >= 0)
	{
		if((err = init_arm_intc(ctx)))
		{
			printk(KERN_WARNING "Unable to register interrupt\n");
			return err;
		}
		rtdm_event_init(&ctx->event, 0);
		ctx->event_inited = 1;
	}
	init_pru(ctx);
	refcount++;
	return 0;
}

static int rtdm_pruss_irq_open(struct rtdm_fd *fd, int oflags){
	struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);
	printk(KERN_WARNING "rtdm_pruss_irq_open\n");
	memset(ctx, sizeof(*ctx),  0);
	return 0;
}

static void rtdm_pruss_irq_close(struct rtdm_fd *fd){
	struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);
	refcount--;
	printk(KERN_WARNING "rtdm_pruss_irq_close\n");
	rtdm_free(ctx->pru_system_events);
	// disable the Global Enable Register of the PRU INTC
	if(ctx->pruintc_io)
		iowrite32(0, ctx->pruintc_io + PRU_INTC_GER_REG); // TODO: only clear the enabled IRQ and do not disable the global (unless it's the last one)

	if(ctx->irq_handle_inited)
	{
		printk(KERN_WARNING "rtdm_irq_free\n");
		rtdm_irq_free(&ctx->irq_handle);
	}
	if(ctx->event_inited)
	{
		printk(KERN_WARNING "rtdm_event_pulse\n");
		rtdm_event_pulse(&ctx->event);
		printk(KERN_WARNING "rtdm_event_destroy\n");
		rtdm_event_destroy(&ctx->event);
	}
	//irq_dispose_mapping(ctx->linux_irq); // calling this causes a stack trace in dmesg
}

static ssize_t rtdm_pruss_irq_read(struct rtdm_fd *fd, void __user *buf, size_t size){
	int ret;
	nanosecs_rel_t timeout = 100000000;
	struct rtdm_pruss_irq_context *ctx = rtdm_fd_to_private(fd);
	if(ctx->arm_irq_handle_number < 0)
	{
		// the ARM interrupt has not been registered
		return -ECANCELED;
	}

	// wait till event gets signalled or timeout occurs
	// if the event has been signalled already, will return immediately
	ret = rtdm_event_timedwait(&ctx->event, timeout, NULL);
	//rtdm_printk(KERN_WARNING "rtdm_pruss_irq event got released\n");

	// ret will be 0 on success or -ETIMEDOUT if the PRU interrupt did not come through on time
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
	.device_flags		= RTDM_NAMED_DEVICE,
	.device_count		= 1,
	.context_size		= sizeof(struct rtdm_pruss_irq_context),
	.ops = {
		.open		= rtdm_pruss_irq_open,
		.close		= rtdm_pruss_irq_close,
		.ioctl_rt	= rtdm_pruss_ioctl,
		.ioctl_nrt	= rtdm_pruss_ioctl,
		.read_rt	= rtdm_pruss_irq_read,
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
