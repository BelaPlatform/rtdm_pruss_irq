#ifndef RTDM_PRUSS_IRQ
#define RTDM_PRUSS_IRQ
/**
 * rtdm driver to register interrupts from the PRU on TI am335x.
 * References to "TRM" refer to sections, tables and figures from the 
 * AM335x Technical Reference Manual (SPRUH73P, revised March 2017)
 *
 * The PRUSS INTC is described in (TRM 4.4.2), see e.g.: Figure 4-17
 */
#include <linux/types.h>
#include <linux/ioctl.h>

#define RTDM_PRUSS_IRQ_VERSION 2

#define RTDM_PRUSS_IRQ_IOC_MAGIC 'p'
struct rtdm_pruss_irq_registration {
	/**
	 * A pointer to an array of pru system_events
	 * For each element, valid range: 0:63 (TRM table 4-22)
	 */
	const __u8* pru_system_events;
	/**
	 * How many elements in the pru_system_events array
	 */
	__u8 pru_system_events_count;
	/**
	 * valid range: 0:9 (TRM 4.4.2.3.4)
	 */
	__u32 pru_intc_channel;
	/**
	 * valid range: 0:9 (TRM 4.4.2.1).
	 * Only 2:9 can be exported to ARM
	 * (mapped to IRQ 20:27 (TRM 6.3))
	 */
	__u32 pru_intc_host;
};

/**
 * Map the requested PRUSS system_event to the default ARM IRQ
 * and then request it from Linux.
 * Valid input range: 18:25
 * PRUSS system_events 18:25 get mapped to PRUSS INTC channels 2:9,
 * which get mapped to ARM interrupts 20:27 (PRU_ICSS_EVTOUT0:PRU_ICSS_EVTOUT7)
 *
 * for 16 <= pru_system_event < 32,
 * we can trigger event pru_system_event from the PRU by doing:
 * MOV R31.b0, (1 << 5) | (pru_system_event - 16)
 */
#define RTDM_PRUSS_IRQ_REGISTER _IOW(RTDM_PRUSS_IRQ_IOC_MAGIC, 0, __u32)

/**
 * Map an arbitrary PRUSS system event to an arbitrary PRUSS INTC chanenl
 * to an arbitrary host channel (which is bound to an ARM IRQ line (TRM 6.3))
 * and then request it from Linux.
 */
#define RTDM_PRUSS_IRQ_REGISTER_FULL _IOW(RTDM_PRUSS_IRQ_IOC_MAGIC, 1, struct rtdm_pruss_irq_registration*)

/**
 * Set the verbose level (0, 1 or 2)
 */
#define RTDM_PRUSS_IRQ_VERBOSE _IOW(RTDM_PRUSS_IRQ_IOC_MAGIC, 2, __u32)
#endif /* RTDM_PRUSS_IRQ */
