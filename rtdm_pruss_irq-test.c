#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <error.h>
#include <string.h>
#include <rtdm/rtdm.h>
#include <time.h>
#include <inttypes.h>
#include "rtdm_pruss_irq-test_bin.h"

#define USE_PRUSSDRV
//#define USE_MEMMAP_PRU

#ifdef USE_PRUSSDRV
#include <prussdrv.h>
#endif

#ifdef XENOMAI_SKIN_posix
#include <pthread.h>
pthread_t demo_task;
#endif

#ifdef XENOMAI_SKIN_native
#include <rtdk.h>
#include <native/task.h>
RT_TASK demo_task;
#endif

char demo_task_name[] = "real-time-task";


#define VERBOSE

int fd;
int pin = 69;
int timingpin = 0;
int tpretcode;
int gShouldStop = 0;

char *rtdm_driver = "/dev/rtdm/rtdm_pruss_irq_0";
#define EVERY 1000


#if 0
static inline int toggle_timing_pin(void)
{
    if (!timingpin)
	return 0;
#ifdef VERBOSE
    printf("requesting GPIO_IRQ_PIN_TOGGLE %d on the timing pin %d\n", GPIO_IRQ_PIN_TOGGLE, timingpin);
#endif
    tpretcode = ioctl(fd, GPIO_IRQ_PIN_TOGGLE, &timingpin);
    if(tpretcode < 0)
    {
        printf("ioctl returned %d\n", tpretcode);
    }
    return tpretcode;
}
#endif

static const uint32_t GPIO_SIZE =  0x198;
static const uint32_t GPIO_DATAIN = (0x138 / 4);
static const uint32_t GPIO_CLEARDATAOUT = (0x190 / 4);
static const uint32_t GPIO_SETDATAOUT = (0x194 / 4);
static const uint32_t GPIO_ADDRESSES[4] = {
	0x44E07000,
	0x4804C000,
	0x481AC000,
	0x481AE000,
};

unsigned int pinMask = 1 << 18;
unsigned int* gpio;

int init_timing_gpio()
{
	fd = open("/dev/mem", O_RDWR);
	int bank = 0;
	pin = 22;
	pinMask = 1 << pin;
	uint32_t gpioBase = GPIO_ADDRESSES[bank];
	gpio = (uint32_t *)mmap(0, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpioBase);
	if(gpio == MAP_FAILED){
		fprintf(stderr, "Unable to map GPIO pin %u\n", pin);
		return -2;
	}

	return 0;
}
void* demo(void *arg) {
	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = 100000000;
	while(!gShouldStop)
	{
		int dest;
		rt_printf("Reading from userspace\n");
		gpio[GPIO_CLEARDATAOUT] = pinMask;
		read(fd, &dest, sizeof(dest));
		gpio[GPIO_SETDATAOUT] = pinMask;
		rt_printf("Successfully read from userspace\n");
#ifdef XENOMAI_SKIN_native
		//rt_task_sleep(req.tv_nsec);
#endif
#ifdef XENOMAI_SKIN_posix
       		nanosleep(&req, NULL);
#endif
	}
	return NULL;
}
#if 0
{
    int irqs = EVERY;
    struct gpio_irq_data rd = {
	.pin = pin,
	.falling =  0
    };
    int rc;

    pthread_setname_np(pthread_self(), "trivial");
    struct sched_param  param = { .sched_priority = 99 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

#ifdef VERBOSE
    printf("requesting GPIO_IRQ_BIND %d on the pin %d\n", GPIO_IRQ_BIND, pin);
#endif
    if ((rc = ioctl(fd,  GPIO_IRQ_BIND, &rd)) < 0) {
	    perror("ioctl GPIO_IRQ_BIND");
	    return (void*)-2;
    }

    while (!gShouldStop) {
#ifdef VERBOSE
        printf("waiting %d for pin %d\n", GPIO_IRQ_PIN_WAIT, pin);
#endif
	if ((rc = ioctl (fd,  GPIO_IRQ_PIN_WAIT, 0)) < 0) {
            printf("ioctl error! rc=%d %s\n", rc, strerror(-rc));
            break;
        }
        if(toggle_timing_pin() < 0)
            return (void*)-1;
        if(toggle_timing_pin() < 0)
            return (void*)-1;
#ifdef VERBOSE
	printf("resuming\n");
	irqs--;
	if (!irqs)  {
	    irqs = EVERY;
	    printf("%d IRQs, tpretcode=%d\n",EVERY, tpretcode);
	}  
#endif
    }
    return (void*)0;
}
#endif

void catch_signal(int sig)
{
    fprintf (stderr, "catch_signal sig=%d\n", sig);
    signal(SIGTERM,  SIG_DFL);
    signal(SIGINT, SIG_DFL);
    gShouldStop = 1;
}

static int devMemWrite(off_t target, uint32_t* value)
{
	const unsigned long MAP_SIZE = 4096UL;
	const unsigned long MAP_MASK = (MAP_SIZE - 1);
	int fd;
	void *map_base, *virt_addr;
	uint32_t writeval = *value;

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	{
		fprintf(stderr, "Unable to open /dev/mem: %d %s\n", fd, strerror(-fd));
		return -1;
	}
	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
	if(map_base == (void *) -1)
	{
		fprintf(stderr, "Unable to mmap %ld\n", target);
		return -1;
	}

	virt_addr = ((char*)map_base) + (target & MAP_MASK);
	// writes 4-byte word (unsigned long)
	*((unsigned long *) virt_addr) = writeval;
	uint32_t read_result = *((unsigned long *) virt_addr);
	*value = read_result;
	if(munmap(map_base, MAP_SIZE) == -1)
	{
		fprintf(stderr, "Error while unmapping memory\n");
	}
	close(fd);
	return 0;
}

int main(int argc, char* argv[])
{
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
#ifdef USE_PRUSSDRV
	printf("Init'ing prussdrv and starting PRU\n");
        prussdrv_init();
        if(prussdrv_open(PRU_EVTOUT_0)) {
                fprintf(stderr, "Failed to open PRU driver\n");
                return 1;
        }
        // if(prussdrv_exec_program(1, "pru_irq_test.bin"))
	if(prussdrv_exec_code(1, PRUcode, sizeof(PRUcode)))
        {
                fprintf(stderr, "Error while loading PRU program\n");
                return -1;
        }
#endif
#ifdef USE_MEMMAP_PRU
	printf("Init'ing PRU via memmap\n");
	//memmap the PRU memory
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	{
		fprintf(stderr, "Unable to open /dev/mem: %d %s\n", fd, strerror(-fd));
		return -1;
	}
	int size = 0x7ffff;
	int target = 0x4a300000;
	const unsigned long MAP_SIZE = 4096UL;
	const unsigned long MAP_MASK = (MAP_SIZE - 1);
	void* map_base = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
	void* virt_addr = ((char*)map_base) + (target & MAP_MASK);
	uint32_t value;

	uint32_t ctrl_register_offset = 0x24000;
	uint32_t instruction_memory_offset = 0x38000;
	memcpy(virt_addr, &value, sizeof(value));
	printf("Read: %#x\n", value);
	// disable PRU (will reset the INTC registers, so run before open()'ing the driver )
	value = 1;
	memcpy(virt_addr + ctrl_register_offset, &value, sizeof(value));
	// load code to PRU instruction RAM
	memcpy(virt_addr + instruction_memory_offset, PRUcode, sizeof(PRUcode));

	// enable PRU
	value = 2;
	memcpy(virt_addr + ctrl_register_offset, &value, sizeof(value));
#endif
	

    if(init_timing_gpio())
        exit(1);
    printf("Pointer: %#p\n", gpio);
    if (argc > 1)
	pin = atoi(argv[1]);
    if (argc > 2)
	timingpin = atoi(argv[2]);


    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Open RTDM driver
    if ((fd = open(rtdm_driver, O_RDWR)) < 0) {
	perror("rt_open");
	exit(-1);
    }
    printf("Returned fd: %d\n", fd);

#ifdef XENOMAI_SKIN_posix
    pthread_create(&demo_task, NULL, demo, NULL);
    pthread_setname_np(demo_task, demo_task_name);

    void* taskReturn;
    pthread_join(demo_task, &taskReturn);
    int iret = (int)taskReturn;
    printf("Returned: %d\n", iret);
#endif
#ifdef XENOMAI_SKIN_native
	int ret;
	if(ret = rt_task_create(&demo_task, demo_task_name, 16384 * 8, 90, T_JOINABLE | T_FPU))
	{
		fprintf(stderr, "Impossible to create task: %d %s\n", -ret, strerror(-ret));
	}
	if(ret = rt_task_start(&demo_task, &demo, 0))
	{
		fprintf(stderr, "Impossible to start task: %d %s\n", -ret, strerror(-ret));
	}
	while(!gShouldStop)
		usleep(1000000);
	rt_task_delete(&demo_task);
	rt_task_join(&demo_task);
#endif

#ifdef USE_PRUSSDRV
        prussdrv_exit();
#endif
#ifdef USE_MEMMAP_PRU
#define PRU_INTC_SECR1_REG   0x280
#define PRU_INTC_SECR2_REG   0x284
	value = 0xFFFFFFFF;
	memcpy(virt_addr + PRU_INTC_SECR1_REG, &value, sizeof(value));
	memcpy(virt_addr + PRU_INTC_SECR2_REG, &value, sizeof(value));
	// disable PRU
	value = 1;
	memcpy(virt_addr + ctrl_register_offset, &value, sizeof(value));
#endif 
    close(fd);
    printf("Closed\n");
    return 0;
}
