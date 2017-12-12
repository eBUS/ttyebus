//===============================================================================================================
//
// ttyebus - real time linux kernel module for the ebusd using the PL011 UART on a Rasperry Pi
//
// Copyright (C) 2017 Galileo53 <galileo53@gmx.at>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This is a LINUX kernel module exclusively for the ebusd using the PL011 UART running on Raspberry Pi 3.
// The latency between receiving and transmitting of a character is nearly zero. This is achieved by
// disabling the hardware FIFO at the UART completely and using a ring-buffer managed at the interrupt
// handler of the "Receiver Holding Register" interrupt.
//
//===============================================================================================================

#include <linux/fs.h> 	            // file stuff
#include <linux/kernel.h>           // printk()
#include <linux/errno.h>            // error codes
#include <linux/module.h>           // THIS_MODULE
#include <linux/delay.h>            // udelay
#include <linux/interrupt.h>        // request_irq
#include <linux/miscdevice.h>       // misc_register
#include <linux/io.h>               // ioremap
#include <linux/spinlock.h>         // spinlocks
#include <asm/uaccess.h>            // copy_to_user

#include <linux/init.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>



// #define DEBUG 1                  // if uncommented, will write some debug messages to /var/log/kern.log
// #define IRQDEBUG 1               // if uncommented, writes messages from the interrupt handler too (there are a lot of messages!)
// #define LOOPBACK 1               // if uncommented, connects the Tx output to the Rx input of the UART. For testing only.

// prototypes
static int ttyebus_open(struct inode* inode, struct file* file);
static int ttyebus_close(struct inode* inode, struct file* file);
static ssize_t ttyebus_read(struct file* file_ptr, char __user* user_buffer, size_t count, loff_t* offset);
static ssize_t ttyebus_write(struct file* file_ptr, const char __user* user_buffer, size_t count, loff_t* offset);
static long ttyebus_ioctl(struct file* fp, unsigned int cmd, unsigned long arg);

#define DEVICE_NAME         "ttyebus"           // The device will appear at /dev/ttyebus

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Galileo53");
MODULE_DESCRIPTION("Kernel module for the ebusd directly connected through the PL011 UART to the eBus adapter");
MODULE_VERSION("1.1");

// file operations with this kernel module
static struct file_operations ttyebus_fops =
    {
    .owner   = THIS_MODULE,
    .open    = ttyebus_open,
    .release = ttyebus_close,
    .read    = ttyebus_read,
    .write   = ttyebus_write,
    .unlocked_ioctl = ttyebus_ioctl
    };

static struct miscdevice misc =
    {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &ttyebus_fops,
    .mode = S_IRUSR |   // User read
            S_IWUSR |   // User write
            S_IRGRP |   // Group read
            S_IWGRP |   // Group write
            S_IROTH |   // Other read
            S_IWOTH     // Other write
    };

static unsigned int RaspiModel;
static unsigned int MajorNumber;
static void* GpioAddr;
static void* UartAddr;
static unsigned int DeviceOpen;
static wait_queue_head_t WaitQueue;
static spinlock_t SpinLock;

// ring buffer used for receiving data
enum { RX_BUFF_SIZE = 64 };
static volatile unsigned int RxTail = 0;
static volatile unsigned int RxHead = 0;
static unsigned int RxBuff[RX_BUFF_SIZE];

// linear buffer used for transmitting data
enum { TX_BUFF_SIZE = 64 };
static volatile unsigned int TxTail = TX_BUFF_SIZE;
static volatile unsigned int TxHead = TX_BUFF_SIZE;
static unsigned char TxBuff[TX_BUFF_SIZE];

#ifdef IRQDEBUG
static int IrqCounter = 0;
#endif


// RASPI I/O Base address
// ======================
// #define BCM2708_PERI_BASE       0x20000000      // RASPI 1
#define BCM2708_PERI_BASE 0x3F000000               // RASPI 2 and 3

// BCM2835 base address
// ====================
#define SYST_BASE           (BCM2708_PERI_BASE + 0x00003000)
#define DMA_BASE            (BCM2708_PERI_BASE + 0x00007000)
#define IRQ_BASE            (BCM2708_PERI_BASE + 0x0000B000)
#define CLK_BASE            (BCM2708_PERI_BASE + 0x00101000)
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x00200000)
#define UART0_BASE          (BCM2708_PERI_BASE + 0x00201000)
#define PCM_BASE            (BCM2708_PERI_BASE + 0x00203000)
#define SPI0_BASE           (BCM2708_PERI_BASE + 0x00204000)
#define I2C0_BASE           (BCM2708_PERI_BASE + 0x00205000)
#define PWM_BASE            (BCM2708_PERI_BASE + 0x0020C000)
#define UART1_BASE          (BCM2708_PERI_BASE + 0x00215000)
#define I2C1_BASE           (BCM2708_PERI_BASE + 0x00804000)
#define I2C2_BASE           (BCM2708_PERI_BASE + 0x00805000)
#define DMA15_BASE          (BCM2708_PERI_BASE + 0x00E05000)

// GPIO register
// =============
#define GPIO_INPUT          0
#define GPIO_OUTPUT         1
#define GPIO_ALT_0          4
#define GPIO_ALT_1          5
#define GPIO_ALT_2          6
#define GPIO_ALT_3          7
#define GPIO_ALT_4          3
#define GPIO_ALT_5          2
 
#define GPIO_FSEL0          (GpioAddr+0x00)
#define GPIO_FSEL1          (GpioAddr+0x04)
#define GPIO_FSEL2          (GpioAddr+0x08)
#define GPIO_FSEL3          (GpioAddr+0x0C)
#define GPIO_FSEL4          (GpioAddr+0x10)
#define GPIO_FSEL5          (GpioAddr+0x14)

#define GPIO_PULL           (GpioAddr+0x94)                       // Pull up/pull down
#define GPIO_PULLCLK0       (GpioAddr+0x98)                       // Pull up/pull down clock
#define GPIO_PULLCLK1       (GpioAddr+0x9C)                       // Pull up/pull down clock
#define GPIO_BANK           (Gpio >> 5)
#define GPIO_BIT            (1 << (Gpio & 0x1F))


#define GPIO_PULL_OFF       0
#define GPIO_PULL_DOWN      1
#define GPIO_PULL_UP        2

// The UART interrupt is interrupt 57 according to the BCM2835 ARM Peripherals manual.
// For some reason it is allocated to 87 by RASPIAN
#define RASPI_UART_IRQ      87


// PL011 UART register (16C650 type)
// =================================
#define UART_DATA         (UartAddr+0x00)
#define UART_RX_ERR       (UartAddr+0x04)
#define UART_FLAG         (UartAddr+0x18)
#define UART_ILPR         (UartAddr+0x20)
#define UART_INT_BAUD     (UartAddr+0x24)
#define UART_FRAC_BAUD    (UartAddr+0x28)
#define UART_LINE_CTRL    (UartAddr+0x2C)
#define UART_CTRL         (UartAddr+0x30)
#define UART_FIFO_LEVEL   (UartAddr+0x34)
#define UART_INT_MASK     (UartAddr+0x38)
#define UART_RAW_INT      (UartAddr+0x3C)
#define UART_INT_STAT     (UartAddr+0x40)
#define UART_INT_CLR      (UartAddr+0x44)
#define UART_DMA_CTRL     (UartAddr+0x48)
#define UART_TEST_CTRL    (UartAddr+0x80)
#define UART_TEST_IN      (UartAddr+0x84)
#define UART_IEST_OUT     (UartAddr+0x88)
#define UART_TEST_DATA    (UartAddr+0x8C)
#define UART_MEM_SIZE     0xC0

// UART_FLAG register
// ==================
#define UART_RX_FIFO_EMPTY (1 << 4)
#define UART_TX_FIFO_FULL  (1 << 5)

// UART Line Control Register
// ==========================
#define UART_LCR_BREAK          (1 << 0)
#define UART_LCR_PARITY_EN      (1 << 1)
#define UART_LCR_EVEN_PARITY    (1 << 2)
#define UART_LCR_2_STOP         (1 << 3)
#define UART_LCR_FIFO_EN        (1 << 4)
#define UART_LCR_8_BITS         (3 << 5)
#define UART_LCR_STICK_PARITY   (1 << 7)

// UART Control Register
// ======================
#define UARTCR_UART_ENABLE      (1 << 0)
#define UARTCR_LOOPBACK         (1 << 7)
#define UARTCR_TX_ENABLE        (1 << 8)
#define UARTCR_RX_ENABLE        (1 << 9)
#define UARTCR_RTS              (1 << 11)

// UART Interrupt masks
// ====================
#define INT_CTS                 (1 << 1)
#define INT_RX                  (1 << 4)
#define INT_TX                  (1 << 5)
#define INT_RX_TIMEOUT          (1 << 6)
#define INT_FRAMING_ERR         (1 << 7)
#define INT_PARITY_ERR          (1 << 8)
#define INT_BREAK_ERR           (1 << 9)
#define INT_OVER_ERR            (1 << 10)



// ===============================================================================================
//
//                                    delay
//
// ===============================================================================================
static inline void delay(int32_t count)
    {
	asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
	    : "=r"(count): [count]"0"(count) : "cc");
    }

// ===============================================================================================
//
//                                    ttyebus_irq_handler
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Fired on interrupt. If data is in the receiver holding register, transfer it to the ring
//      buffer. If transmitter holding register has become empty, fill it with another data from
//      the linear buffer.
//
// ===============================================================================================
static irqreturn_t ttyebus_irq_handler(int irq, void* dev_id)
    {
    unsigned int IntStatus;
    unsigned int DataWord;
    unsigned int IntMask;
    unsigned int RxNext;

#ifdef IRQDEBUG
    printk(KERN_NOTICE "ttyebus: IRQ %d called", IrqCounter);
#endif

    IntStatus = ioread32(UART_INT_STAT);

    if (IntStatus & INT_RX)
        {
        // clear the interrupt
        // ===================
        iowrite32(INT_RX, UART_INT_CLR);

        // see if the buffer will be full after this interrupt
        // ===================================================
        spin_lock(&SpinLock);
        RxNext = RxHead + 1;
        if (RxNext >= RX_BUFF_SIZE)
            RxNext = 0;

        if (RxNext != RxTail)
            {
            // data was received and is available in the receiver holding register
            // ===================================================================
            RxBuff[RxHead] = ioread32(UART_DATA);
            RxHead = RxNext;
            }

        // else
            // buffer overrun. do nothing. just discard the data.
            // eventually todo: if someone needs to know, we can throw an error here
            // =====================================================================
        spin_unlock(&SpinLock);

        // clear any receiver error
        // ========================
        iowrite32(0, UART_RX_ERR);

        // if the calling task is waiting, wake him up. If there is no task at all, this is a NOP
        // ======================================================================================
        wake_up(&WaitQueue);
        }

    // Transmitter
    // ===========
    if (IntStatus & INT_TX)
        {
        // clear the interrupt
        // ===================
        iowrite32(INT_TX, UART_INT_CLR);

        // The transmitter holding register has become empty.
        // see if some more data available
        // ==================================================
        spin_lock(&SpinLock);
        if (TxTail < TxHead)
            {
            // fill the transmitter holding register with new data
            // ===================================================
            DataWord = TxBuff[TxTail++];
            iowrite32(DataWord, UART_DATA);
            // (do nothing with the interrupt line - keep the INT_TX active)
            }
        else
            {
            // no more data in the transmit buffer. disable the TX interrupt
            // =============================================================
            IntMask = ioread32(UART_INT_MASK);
            iowrite32(IntMask & ~INT_TX, UART_INT_MASK);
            }
        spin_unlock(&SpinLock);
        }

#ifdef IRQDEBUG
    printk(KERN_NOTICE "ttyebus: IRQ %d exit", IrqCounter);
    IrqCounter++;
#endif

    return IRQ_HANDLED;
    }


// ===============================================================================================
//
//                                    ttyebus_set_gpio_mode
//
// ===============================================================================================
//
// Parameter:
//      Gpio                Number of the GPIO port
//      Function            one of GPIO_INPUT, GPIO_OUTPUT, GPIO_ALT_0, etc.
//
// Returns:
//
// Description:
//      Set the mode for the GPIO port. Especially in this program GPIO_ALT_0 at port 14, 15 will
//      connect the ports to the UART Rx and Tx
//
// ===============================================================================================
static void ttyebus_set_gpio_mode(unsigned int Gpio, unsigned int Function)
    {
    unsigned int RegOffset = (Gpio / 10) << 2;
    unsigned int Bit = (Gpio % 10) * 3;
    volatile unsigned int Value = ioread32(GpioAddr + RegOffset);
    iowrite32((Value & ~(0x7 << Bit)) | ((Function & 0x7) << Bit), GpioAddr + RegOffset);
    }


// ===============================================================================================
//
//                                    ttyebus_gpio_pullupdown
//
// ===============================================================================================
//
// Parameter:
//      Gpio                Number of the GPIO port
//      pud                 one of GPIO_PULL_OFF, GPIO_PULL_DOWN or GPIO_PULL_UP
//
// Returns:
//
// Description:
//      Set the pull-up or pull-down at the specified GPIO port
//
// ===============================================================================================
void ttyebus_gpio_pullupdown(unsigned int Gpio, unsigned int pud)
    {
    // fill the new value for pull up or down
    // ======================================
    iowrite32(pud, GPIO_PULL);
    delay(150);     // provide hold time for the control signal

    // transfer the new value to the GPIO pin
    // ======================================
    iowrite32(GPIO_BIT, GPIO_PULLCLK0 + GPIO_BANK);
    delay(150);     // provide hold time for the control signal

    // remove the control signal to make it happen
    // ===========================================
    iowrite32(0, GPIO_PULL);
    iowrite32(0, GPIO_PULLCLK0 + GPIO_BANK);
    }


// ===============================================================================================
//
//                                    ttyebus_read
//
// ===============================================================================================
//
// Parameter:
//      file_ptr
//      user_buffer         Buffer in user space where to receive the data
//      count               Number of bytes to read
//      offset              Pointer to a counter that can hold an offset when reading chunks
//
// Returns:
//      Number of bytes read
//
// Description:
//      Called when a process, which already opened the dev file, attempts to read from it, like
//      "cat /dev/ttyebus"
//
// ===============================================================================================
static ssize_t ttyebus_read(struct file* file_ptr, char __user* user_buffer, size_t Count, loff_t* offset)
    {
    unsigned int NumBytes;
    unsigned int result;
    unsigned long Flags;
    enum { BUFFER_SIZE = 512 };
    char buffer[BUFFER_SIZE];

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Read request with offset = %d and count = %u", (int)*offset, (unsigned int)Count);
#endif

    // wait until a character is received or if timeout (100ms) occurs.
    // note that wait_event_timeout is a macro that will already avoid the race condition that may
    // happen where new data arrives between testing (RxTail != RxHead) and effective sleeping of this task.
    // =====================================================================================================
    result = wait_event_timeout(WaitQueue, RxTail != RxHead, msecs_to_jiffies(100));
    if (result == 0)
		return -EBUSY; // timeout

    // collect all bytes received so far from the receive buffer
    // we must convert from a ring buffer to a linear buffer
    // =========================================================
    NumBytes = 0;
    spin_lock_irqsave(&SpinLock, Flags);
    while (RxTail != RxHead && NumBytes < Count)
        {
        buffer[NumBytes++] = RxBuff[RxTail++];
        if (RxTail >= RX_BUFF_SIZE)
            RxTail = 0;
        }
    spin_unlock_irqrestore(&SpinLock, Flags);

    // copying data to user space requires a special function to be called
    // ===================================================================
    if (copy_to_user(user_buffer, buffer, Count) != 0)
        return -EFAULT;

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Read exit with %d bytes read", NumBytes);
#endif

    return NumBytes;        // the number of bytes actually received
    }
    
    
// ===============================================================================================
//
//                                    ttyebus_write
//
// ===============================================================================================
//
// Parameter:
//      file_ptr
//      user_buffer         Buffer in user space where to receive the data
//      count               Number of bytes to write
//      offset              Pointer to a counter that can hold an offset when writing chunks
//
// Returns:
//      Number of bytes written
//
// Description:
//      Called when a process, which already opened the dev file, attempts to write to it, like
//      "echo "hello" > /dev/ttyebus"
//
// ===============================================================================================
static ssize_t ttyebus_write(struct file* file_ptr, const char __user* user_buffer, size_t Count, loff_t* offset)
    {
    int result;
    int Timeout;
    unsigned long Flags;
    unsigned int DataWord;
    unsigned int IntMask;

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Write request with offset = %d and count = %u", (int)*offset, (unsigned int)Count);
#endif

    // if transmission is still in progress, wait until done
    // =====================================================
    Timeout = 500;
    while (TxTail < TxHead)
        {
        if (--Timeout < 0)
            return -EBUSY;
        udelay(500);
        }

    // copying data from user space requires a special function to be called
    // =====================================================================
    if (Count > TX_BUFF_SIZE)
        Count = TX_BUFF_SIZE;
    result = copy_from_user(TxBuff, user_buffer, Count);
    if (result > 0)             // not all requested bytes copied
        Count = result;         // nuber of bytes copied
    else if (result != 0)
        return -EFAULT;

    // Fill the first character directly to the hardware, the rest will be
    // fetched by the interrupt handler upon handling the TX interrupt
    // ===================================================================
    spin_lock_irqsave(&SpinLock, Flags);
    DataWord = TxBuff[0];
    TxTail = 1;
    TxHead = Count;
    iowrite32(DataWord, UART_DATA);

    // enable the TX interrupt. will be asserted when the transmitter holding becomes empty
    // ====================================================================================
    IntMask = ioread32(UART_INT_MASK);
    iowrite32(IntMask | INT_TX, UART_INT_MASK);
    spin_unlock_irqrestore(&SpinLock, Flags);

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Write exit with %d bytes written", Count);
#endif

    return Count;        // the number of bytes actually transmitted
    }
    
    

// ===============================================================================================
//
//                                    ttyebus_open
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Called when a process tries to open the device file, like "cat /dev/ttyebus"
//
// ===============================================================================================
static int ttyebus_open(struct inode* inode, struct file* file)
    {
    unsigned int UartCtrl;

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Open at at major %d  minor %d", imajor(inode), iminor(inode));
#endif

    // do not allow another open if already open
    // =========================================
	if (DeviceOpen)
		return -EBUSY;
	DeviceOpen++;

	// Disable UART0
    // =============
	iowrite32(0, UART_CTRL);

    // reset the ring buffer and the linear buffer
    // ===========================================
    RxTail = RxHead = 0;
    TxTail = TxHead = TX_BUFF_SIZE;

	// Setup the GPIO pin 14 && 15 to ALTERNATE 0 (connect to UART)
    // ============================================================
    ttyebus_set_gpio_mode(15, GPIO_ALT_0);      // GPIO15 connected to RxD
    ttyebus_set_gpio_mode(14, GPIO_ALT_0);      // GPIO14 connected to TxD

	// Set pull-down for the GPIO pin
    // ==============================
    ttyebus_gpio_pullupdown(14, GPIO_PULL_UP);
    ttyebus_gpio_pullupdown(15, GPIO_PULL_UP);

	// Clear pending interrupts
    // ========================
	iowrite32(0x7FF, UART_INT_CLR);

    // Set integer & fractional part of baud rate to 2400 Baud fixed. Divider = 3.000.000 / 2400
    // Fractional part = 0. In contrary to the documentation, there is no additional divider by 16
    // (RASPI 3). May be different at RASPI 1/2 ??
    // ===========================================================================================
	iowrite32(3000000 / 2400, UART_INT_BAUD);
	iowrite32(0, UART_FRAC_BAUD);
 
	// Disable FIFO & 8 bit (1 stop bit, no parity)
    // ============================================
	iowrite32(UART_LCR_8_BITS, UART_LINE_CTRL);

    // Enable receiver interrupt
    // =========================
    iowrite32(INT_RX, UART_INT_MASK);

	// Enable UART0, receive & transfer part of UART
    // =============================================
	UartCtrl = UARTCR_UART_ENABLE | UARTCR_TX_ENABLE | UARTCR_RX_ENABLE | UARTCR_RTS;
#ifdef LOOPBACK
    UartCtrl |= UARTCR_LOOPBACK;
#endif
	iowrite32(UartCtrl, UART_CTRL);

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: Open exit");
#endif

	return 0;
    }


// ===============================================================================================
//
//                                    ttyebus_close
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//
// Description:
//      Called when a process closes the device file.
//
// ===============================================================================================
static int ttyebus_close(struct inode *inode, struct file *file)
    {
    printk(KERN_NOTICE "ttyebus: Close at at major %d  minor %d", imajor(inode), iminor(inode));

	DeviceOpen--;

	// Disable UART0
    // =============
	iowrite32(0, UART_CTRL);

    printk(KERN_NOTICE "ttyebus: Close exit");

	return 0;
    }


// ===============================================================================================
//
//                                    ttyebus_ioctl
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//      OK
//
// Description:
//      I/O control. Currently this does nothing. ebusd just calls it to see if the device
//      is working. So only return an OK status
//
// ===============================================================================================
static long ttyebus_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
    {
	return 0;
    }


// ===============================================================================================
//
//                                      ttyebus_raspi_model
//
// ===============================================================================================
// Description:
//      Get the Rasperry Pi model number from /sys/firmware/devicetree/base/model. The string
//      has usualle the form "Raspberry Pi 3 Model B Rev 1.2"
//      Extract the number and return it.
//
// ===============================================================================================
unsigned int ttyebus_raspi_model(void)
    {
    struct file* filp = NULL;
    char buf[32];
    unsigned int NumBytes = 0;

    // get current segment descriptor, set segment descriptor
    // associated to kernel space
    // ======================================================
    mm_segment_t old_fs = get_fs();
    set_fs(get_ds());

    // read the file
    // =============
    filp = filp_open("/sys/firmware/devicetree/base/model", O_RDONLY, 0);
    if (filp == NULL)
        {
        set_fs(old_fs);
        return 0;
        }
    NumBytes = filp->f_op->read(filp, buf, sizeof(buf), &filp->f_pos);
    set_fs(old_fs);

    // restore the segment descriptor
    // ==============================
    filp_close(filp, NULL);

    // interpret the data from the file
    // ================================
    if (NumBytes < 14)
        return 0;

    //todo: interpret this correctly for other RASPIs

    return buf[13] - '0';
    }


// ===============================================================================================
//
//                                    ttyebus_register
//
// ===============================================================================================
//
// Parameter:
//
// Returns:
//      Major Number of the driver
//
// Description:
//      Register the device to the kernel by use of the register-chrdev(3) call. Since the first
//      parameter to this call is 0, the system will assign a Major Number by itself. A
//      device name is given and the file_operations structure is also passed to the kernel.
//
// ===============================================================================================
int ttyebus_register(void)
    {
    int result;

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: register_device() is called");
#endif

    // Get the RASPI model
    // ===================
    RaspiModel = ttyebus_raspi_model();
    if (RaspiModel < 2 || RaspiModel > 3)
        {
        printk(KERN_NOTICE "ttyebus: Unknown RASPI model %d\n", RaspiModel);
//        return -EFAULT;
        }

    // Dynamically allocate a major number for the device
    // ==================================================
    MajorNumber = register_chrdev(0, DEVICE_NAME, &ttyebus_fops);
    if (MajorNumber < 0)
        {
        printk(KERN_WARNING "ttyebus: can\'t register character device with errorcode = %i", MajorNumber);
        return MajorNumber;
        }

#ifdef DEBUG
    printk(KERN_NOTICE "ttyebus: registered character device with major number = %i and minor numbers 0...255", MajorNumber);
#endif

    // Register the device driver. We are using misc_register instead of
    // device_create so we are able to set the attributes to rw for everybody
    // ======================================================================
    result = misc_register(&misc);
    if (result)
        {
        unregister_chrdev(MajorNumber, DEVICE_NAME);
        printk(KERN_ALERT "ttyebus: Failed to create the device");
        return result;
        }

    // remap the I/O registers to some memory we can access later on
    // =============================================================
    GpioAddr = ioremap(GPIO_BASE, SZ_4K);
    UartAddr = ioremap(UART0_BASE, SZ_4K);

    // set up a queue for waiting
    // ==========================
    init_waitqueue_head(&WaitQueue);

    // initialize the spinlock
    // =======================
    spin_lock_init(&SpinLock);

    // Install Interrupt Handler
    // =========================
    result = request_irq(RASPI_UART_IRQ, ttyebus_irq_handler, 0, "ttyebus_irq_handler", NULL);
    if (result)
        {
        unregister_chrdev(MajorNumber, DEVICE_NAME);
        printk(KERN_ALERT "ttyebus: Failed to request IRQ %d", RASPI_UART_IRQ);
        return result;
        }

    DeviceOpen = 0;

#ifdef DEBUG
    printk(KERN_INFO "ttyebus: device created correctly");
#endif

    return result;
    }
    
    
// ===============================================================================================
//
//                                    ttyebus_unregister
//
// ===============================================================================================
// Parameter:
//
// Returns:
//
// Description:
//      Unmap the I/O, free the IRQ and unregister the device
//
// ===============================================================================================
void ttyebus_unregister(void)
    {
    printk(KERN_NOTICE "ttyebus: unregister_device()");

    // release the mapping
    if (GpioAddr)
        iounmap(GpioAddr);
    if (UartAddr)
        iounmap(UartAddr);

    GpioAddr = 0;
    UartAddr = 0;

    free_irq(RASPI_UART_IRQ, NULL);

    misc_deregister(&misc);
    unregister_chrdev(MajorNumber, DEVICE_NAME);

    MajorNumber = 0;
    }


// ===============================================================================================
//
//                                module_init()      module_exit()
//
// ===============================================================================================
// Description:
//        Before Linux 2.4, the init and cleanup functions have to be named init_module() and
//        cleanup_module() exactly.
//        As of Linux 2.4, there are two macros, module_init() and module_exit() defined in
//        linux/init.h that allow us to use any name for the init and cleanup we want.
//        Note that the functions must be defined *before* calling the macros, otherwise you'll
//        get compilation errors.
//
// ===============================================================================================
module_init(ttyebus_register);
module_exit(ttyebus_unregister);



