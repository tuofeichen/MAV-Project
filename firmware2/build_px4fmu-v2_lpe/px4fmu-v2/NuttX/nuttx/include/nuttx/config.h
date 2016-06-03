/* config.h -- Autogenerated! Do not edit. */

#ifndef __INCLUDE_NUTTX_CONFIG_H
#define __INCLUDE_NUTTX_CONFIG_H

/* Architecture-specific options *************************/

#define CONFIG_NUTTX_NEWCONFIG 1
#define CONFIG_HOST_OSX 1
#define CONFIG_RAW_BINARY 1
#define CONFIG_ARCH_MATH_H 1
#undef CONFIG_DEBUG
#undef CONFIG_DEBUG_VERBOSE
#define CONFIG_DEBUG_FS 1
#define CONFIG_DEBUG_DMA 1
#define CONFIG_DEBUG_SYMBOLS 1
#define CONFIG_ARCH_ARM 1
#define CONFIG_ARCH "arm"
#define CONFIG_ARCH_CHIP_STM32 1
#define CONFIG_ARCH_CORTEXM4 1
#define CONFIG_ARCH_FAMILY "armv7-m"
#define CONFIG_ARCH_CHIP "stm32"
#define CONFIG_ARMV7M_USEBASEPRI 1
#define CONFIG_ARCH_HAVE_CMNVECTOR 1
#define CONFIG_ARMV7M_CMNVECTOR 1
#define CONFIG_ARCH_HAVE_FPU 1
#define CONFIG_ARCH_FPU 1
#define CONFIG_ARCH_HAVE_MPU 1
#define CONFIG_ARMV7M_TOOLCHAIN_GNU_EABI 1
#undef CONFIG_ARMV7M_STACKCHECK
#define CONFIG_SERIAL_TERMIOS 1
#define CONFIG_SDIO_DMA 1
#define CONFIG_SDIO_DMAPRIO 0x00010000
#define CONFIG_ARCH_CHIP_STM32F427V 1
#define CONFIG_STM32_STM32F40XX 1
#define CONFIG_STM32_STM32F427 1
#define CONFIG_STM32_ADC1 1
#define CONFIG_STM32_BKPSRAM 1
#define CONFIG_STM32_CCMDATARAM 1
#define CONFIG_STM32_DMA1 1
#define CONFIG_STM32_DMA2 1
#define CONFIG_STM32_I2C1 1
#define CONFIG_STM32_I2C2 1
#define CONFIG_STM32_OTGFS 1
#define CONFIG_STM32_PWR 1
#define CONFIG_STM32_SDIO 1
#define CONFIG_STM32_SPI1 1
#define CONFIG_STM32_SPI2 1
#define CONFIG_STM32_SPI4 1
#define CONFIG_STM32_SYSCFG 1
#define CONFIG_STM32_TIM1 1
#define CONFIG_STM32_TIM3 1
#define CONFIG_STM32_TIM4 1
#define CONFIG_STM32_TIM9 1
#define CONFIG_STM32_TIM10 1
#define CONFIG_STM32_TIM11 1
#define CONFIG_STM32_USART1 1
#define CONFIG_STM32_USART2 1
#define CONFIG_STM32_USART3 1
#define CONFIG_STM32_UART4 1
#define CONFIG_STM32_USART6 1
#define CONFIG_STM32_UART7 1
#define CONFIG_STM32_UART8 1
#define CONFIG_STM32_WWDG 1
#define CONFIG_STM32_ADC 1
#define CONFIG_STM32_SPI 1
#define CONFIG_STM32_I2C 1
#define CONFIG_STM32_FLASH_PREFETCH 1
#define CONFIG_STM32_JTAG_SW_ENABLE 1
#define CONFIG_STM32_DISABLE_IDLE_SLEEP_DURING_DEBUG 1
#define CONFIG_STM32_DMACAPABLE 1
#define CONFIG_STM32_USART 1
#define CONFIG_STM32_RXDMA_BUFFER_SIZE_OVERRIDE 256
#define CONFIG_USART1_RXDMA 1
#define CONFIG_USART2_RXDMA 1
#define CONFIG_USART3_RXDMA 1
#define CONFIG_UART4_RXDMA 1
#define CONFIG_UART5_RXDMA 1
#define CONFIG_USART6_RXDMA 1
#define CONFIG_UART7_RXDMA 1
#define CONFIG_UART8_RXDMA 1
#define CONFIG_SERIAL_DISABLE_REORDERING 1
#define CONFIG_STM32_USART_SINGLEWIRE 1
#define CONFIG_STM32_I2CTIMEOSEC 0
#define CONFIG_STM32_I2CTIMEOMS 10
#define CONFIG_SDIO_PRI 128
#define CONFIG_ARCH_DMA 1
#define CONFIG_ARCH_HAVE_VFORK 1
#define CONFIG_ARCH_STACKDUMP 1
#define CONFIG_ARCH_HAVE_RAMVECTORS 1
#define CONFIG_BOARD_LOOPSPERMSEC 16717
#define CONFIG_DRAM_START 0x20000000
#define CONFIG_DRAM_SIZE 262144
#define CONFIG_ARCH_HAVE_INTERRUPTSTACK 1
#define CONFIG_ARCH_INTERRUPTSTACK 750
#define CONFIG_BOOT_RUNFROMFLASH 1
#define CONFIG_ARCH_BOARD_PX4FMU_V2 1
#define CONFIG_ARCH_BOARD_CUSTOM 1
#define CONFIG_ARCH_BOARD ""
#define CONFIG_NSH_MMCSDMINOR 0
#define CONFIG_NSH_MMCSDSLOTNO 0
#define CONFIG_MMCSD_HAVE_SDIOWAIT_WRCOMPLETE 1
#define CONFIG_MSEC_PER_TICK 1
#define CONFIG_RR_INTERVAL 0
#define CONFIG_SCHED_INSTRUMENTATION 1
#define CONFIG_TASK_NAME_SIZE 24
#define CONFIG_START_YEAR 1970
#define CONFIG_START_MONTH 1
#define CONFIG_START_DAY 1
#define CONFIG_DEV_CONSOLE 1
#define CONFIG_PRIORITY_INHERITANCE 1
#define CONFIG_SEM_PREALLOCHOLDERS 0
#define CONFIG_SEM_NNESTPRIO 8
#define CONFIG_FDCLONE_STDIO 1
#define CONFIG_SDCLONE_DISABLE 1
#define CONFIG_SCHED_WAITPID 1
#define CONFIG_SCHED_ATEXIT 1
#define CONFIG_SCHED_ATEXIT_MAX 1
#define CONFIG_USER_ENTRYPOINT nsh_main
#define CONFIG_SIG_SIGUSR1 1
#define CONFIG_SIG_SIGUSR2 2
#define CONFIG_SIG_SIGALARM 3
#define CONFIG_SIG_SIGCONDTIMEDOUT 16
#define CONFIG_SIG_SIGWORK 4
#define CONFIG_MAX_TASKS 32
#define CONFIG_MAX_TASK_ARGS 10
#define CONFIG_NPTHREAD_KEYS 4
#define CONFIG_NFILE_DESCRIPTORS 42
#define CONFIG_NFILE_STREAMS 8
#define CONFIG_NAME_MAX 32
#define CONFIG_PREALLOC_MQ_MSGS 4
#define CONFIG_MQ_MAXMSGSIZE 32
#define CONFIG_MAX_WDOGPARMS 2
#define CONFIG_PREALLOC_WDOGS 50
#define CONFIG_PREALLOC_TIMERS 50
#define CONFIG_IDLETHREAD_STACKSIZE 1000
#define CONFIG_USERMAIN_STACKSIZE 2500
#define CONFIG_PTHREAD_STACK_MIN 512
#define CONFIG_PTHREAD_STACK_DEFAULT 2048
#define CONFIG_DEV_NULL 1
#define CONFIG_I2C 1
#define CONFIG_I2C_TRANSFER 1
#define CONFIG_ARCH_HAVE_I2CRESET 1
#define CONFIG_I2C_RESET 1
#define CONFIG_SPI 1
#define CONFIG_SPI_EXCHANGE 1
#define CONFIG_RTC 1
#define CONFIG_RTC_DATETIME 1
#define CONFIG_RTC_HSECLOCK 1
#define CONFIG_WATCHDOG 1
#define CONFIG_MMCSD 1
#define CONFIG_MMCSD_NSLOTS 1
#define CONFIG_MMCSD_MULTIBLOCK_DISABLE 1
#define CONFIG_MMCSD_SDIO 1
#define CONFIG_MTD 1
#define CONFIG_MTD_PARTITION 1
#define CONFIG_MTD_BYTE_WRITE 1
#define CONFIG_MTD_RAMTRON 1
#define CONFIG_PIPES 1
#define CONFIG_SERIAL 1
#define CONFIG_SERIAL_REMOVABLE 1
#define CONFIG_ARCH_HAVE_UART4 1
#define CONFIG_ARCH_HAVE_UART7 1
#define CONFIG_ARCH_HAVE_UART8 1
#define CONFIG_ARCH_HAVE_USART1 1
#define CONFIG_ARCH_HAVE_USART2 1
#define CONFIG_ARCH_HAVE_USART3 1
#define CONFIG_ARCH_HAVE_USART6 1
#define CONFIG_MCU_SERIAL 1
#define CONFIG_STANDARD_SERIAL 1
#define CONFIG_SERIAL_NPOLLWAITERS 2
#define CONFIG_UART7_SERIAL_CONSOLE 1
#define CONFIG_USART1_RXBUFSIZE 128
#define CONFIG_USART1_TXBUFSIZE 32
#define CONFIG_USART1_BAUD 115200
#define CONFIG_USART1_BITS 8
#define CONFIG_USART1_PARITY 0
#define CONFIG_USART1_2STOP 0
#define CONFIG_USART2_RXBUFSIZE 600
#define CONFIG_USART2_TXBUFSIZE 1100
#define CONFIG_USART2_BAUD 57600
#define CONFIG_USART2_BITS 8
#define CONFIG_USART2_PARITY 0
#define CONFIG_USART2_2STOP 0
#define CONFIG_USART2_IFLOWCONTROL 1
#define CONFIG_USART2_OFLOWCONTROL 1
#define CONFIG_USART3_RXBUFSIZE 300
#define CONFIG_USART3_TXBUFSIZE 300
#define CONFIG_USART3_BAUD 57600
#define CONFIG_USART3_BITS 8
#define CONFIG_USART3_PARITY 0
#define CONFIG_USART3_2STOP 0
#define CONFIG_USART3_IFLOWCONTROL 1
#define CONFIG_USART3_OFLOWCONTROL 1
#define CONFIG_UART4_RXBUFSIZE 300
#define CONFIG_UART4_TXBUFSIZE 300
#define CONFIG_UART4_BAUD 57600
#define CONFIG_UART4_BITS 8
#define CONFIG_UART4_PARITY 0
#define CONFIG_UART4_2STOP 0
#define CONFIG_USART6_RXBUFSIZE 300
#define CONFIG_USART6_TXBUFSIZE 300
#define CONFIG_USART6_BAUD 57600
#define CONFIG_USART6_BITS 8
#define CONFIG_USART6_PARITY 0
#define CONFIG_USART6_2STOP 0
#define CONFIG_UART7_RXBUFSIZE 300
#define CONFIG_UART7_TXBUFSIZE 300
#define CONFIG_UART7_BAUD 57600
#define CONFIG_UART7_BITS 8
#define CONFIG_UART7_PARITY 0
#define CONFIG_UART7_2STOP 0
#define CONFIG_UART8_RXBUFSIZE 300
#define CONFIG_UART8_TXBUFSIZE 300
#define CONFIG_UART8_BAUD 57600
#define CONFIG_UART8_BITS 8
#define CONFIG_UART8_PARITY 0
#define CONFIG_UART8_2STOP 0
#define CONFIG_SERIAL_IFLOWCONTROL 1
#define CONFIG_SERIAL_OFLOWCONTROL 1
#define CONFIG_USBDEV 1
#define CONFIG_USBDEV_BUSPOWERED 1
#define CONFIG_USBDEV_MAXPOWER 500
#define CONFIG_CDCACM 1
#define CONFIG_CDCACM_EP0MAXPACKET 64
#define CONFIG_CDCACM_EPINTIN 1
#define CONFIG_CDCACM_EPINTIN_FSSIZE 64
#define CONFIG_CDCACM_EPINTIN_HSSIZE 64
#define CONFIG_CDCACM_EPBULKOUT 3
#define CONFIG_CDCACM_EPBULKOUT_FSSIZE 64
#define CONFIG_CDCACM_EPBULKOUT_HSSIZE 512
#define CONFIG_CDCACM_EPBULKIN 2
#define CONFIG_CDCACM_EPBULKIN_FSSIZE 64
#define CONFIG_CDCACM_EPBULKIN_HSSIZE 512
#define CONFIG_CDCACM_NWRREQS 4
#define CONFIG_CDCACM_NRDREQS 4
#define CONFIG_CDCACM_BULKIN_REQLEN 96
#define CONFIG_CDCACM_RXBUFSIZE 600
#define CONFIG_CDCACM_TXBUFSIZE 8000
#define CONFIG_CDCACM_VENDORID 0x26ac
#define CONFIG_CDCACM_PRODUCTID 0x0011
#define CONFIG_CDCACM_VENDORSTR "3D Robotics"
#define CONFIG_CDCACM_PRODUCTSTR "PX4 FMU v2.x"
#define CONFIG_FS_FAT 1
#define CONFIG_FAT_LCNAMES 1
#define CONFIG_FAT_LFN 1
#define CONFIG_FAT_MAXFNAME 32
#define CONFIG_FS_FATTIME 1
#define CONFIG_FAT_DMAMEMORY 1
#define CONFIG_FS_NXFFS 1
#define CONFIG_NXFFS_PREALLOCATED 1
#define CONFIG_NXFFS_ERASEDSTATE 0xff
#define CONFIG_NXFFS_PACKTHRESHOLD 32
#define CONFIG_NXFFS_MAXNAMLEN 32
#define CONFIG_NXFFS_TAILTHRESHOLD 2048
#define CONFIG_FS_ROMFS 1
#define CONFIG_FS_BINFS 1
#define CONFIG_MM_REGIONS 2
#define CONFIG_GRAN 1
#define CONFIG_BUILTIN 1
#define CONFIG_STDIO_BUFFER_SIZE 32
#define CONFIG_STDIO_LINEBUFFER 1
#define CONFIG_NUNGET_CHARS 2
#define CONFIG_LIB_HOMEDIR "/"
#define CONFIG_LIBC_FLOATINGPOINT 1
#define CONFIG_LIB_RAND_ORDER 1
#define CONFIG_EOL_IS_EITHER_CRLF 1
#define CONFIG_POSIX_SPAWN_PROXY_STACKSIZE 1024
#define CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE 2048
#define CONFIG_LIBC_STRERROR 1
#define CONFIG_ARCH_LOWPUTC 1
#define CONFIG_LIB_SENDFILE_BUFSIZE 512
#define CONFIG_ARCH_OPTIMIZED_FUNCTIONS 1
#define CONFIG_ARCH_MEMCPY 1
#define CONFIG_SCHED_WORKQUEUE 1
#define CONFIG_SCHED_HPWORK 1
#define CONFIG_SCHED_WORKPRIORITY 192
#define CONFIG_SCHED_WORKPERIOD 5000
#define CONFIG_SCHED_WORKSTACKSIZE 1600
#define CONFIG_SCHED_LPWORK 1
#define CONFIG_SCHED_LPWORKPRIORITY 50
#define CONFIG_SCHED_LPWORKPERIOD 50000
#define CONFIG_SCHED_LPWORKSTACKSIZE 1600
#define CONFIG_C99_BOOL8 1
#define CONFIG_HAVE_CXX 1
#define CONFIG_HAVE_CXXINITIALIZE 1
#define CONFIG_BUILTIN_PROXY_STACKSIZE 1024
#define CONFIG_EXAMPLES_CDCACM 1
#define CONFIG_EXAMPLES_MOUNT 1
#define CONFIG_EXAMPLES_NSH 1
#define CONFIG_NSH_LIBRARY 1
#define CONFIG_NSH_BUILTIN_APPS 1
#define CONFIG_NSH_CODECS_BUFSIZE 128
#define CONFIG_NSH_FILEIOSIZE 512
#define CONFIG_NSH_STRERROR 1
#define CONFIG_NSH_LINELEN 128
#define CONFIG_NSH_MAXARGUMENTS 12
#define CONFIG_NSH_NESTDEPTH 8
#define CONFIG_NSH_ROMFSETC 1
#define CONFIG_NSH_ROMFSMOUNTPT "/etc"
#define CONFIG_NSH_INITSCRIPT "init.d/rcS"
#define CONFIG_NSH_ROMFSDEVNO 0
#define CONFIG_NSH_ROMFSSECTSIZE 128
#define CONFIG_NSH_ARCHROMFS 1
#define CONFIG_NSH_FATDEVNO 1
#define CONFIG_NSH_FATSECTSIZE 512
#define CONFIG_NSH_FATNSECTORS 1024
#define CONFIG_NSH_FATMOUNTPT "/tmp"
#define CONFIG_NSH_CONSOLE 1
#define CONFIG_NSH_ARCHINIT 1
#define CONFIG_SYSTEM_READLINE 1
#define CONFIG_READLINE_ECHO 1
#define CONFIG_SYSTEM_SYSINFO 1
#define CONFIG_NSOCKET_DESCRIPTORS 0
#define CONFIG_APPS_DIR "../apps"

/* Sanity Checks *****************************************/

/* If this is an NXFLAT, external build, then make sure that
 * NXFLAT support is enabled in the base code.
 */

#if defined(__NXFLAT__) && !defined(CONFIG_NXFLAT)
# error "NXFLAT support not enabled in this configuration"
#endif

/* NXFLAT requires PIC support in the TCBs. */

#if defined(CONFIG_NXFLAT)
# undef CONFIG_PIC
# define CONFIG_PIC 1
#endif

/* Binary format support is disabled if no binary formats are
 * configured (at present, NXFLAT is the only supported binary.
 * format).
 */

#if !defined(CONFIG_NXFLAT) && !defined(CONFIG_ELF) && !defined(CONFIG_BUILTIN)
# undef CONFIG_BINFMT_DISABLE
# define CONFIG_BINFMT_DISABLE 1
#endif

/* The correct way to disable RR scheduling is to set the
 * timeslice to zero.
 */

#ifndef CONFIG_RR_INTERVAL
# define CONFIG_RR_INTERVAL 0
#endif

/* The correct way to disable filesystem supuport is to set the number of
 * file descriptors to zero.
 */

#ifndef CONFIG_NFILE_DESCRIPTORS
# define CONFIG_NFILE_DESCRIPTORS 0
#endif

/* If a console is selected, then make sure that there are resources for
 * three file descriptors and, if any streams are selected, also for three
 * file streams.
 *
 * CONFIG_DEV_CONSOLE means that a builtin console device exists at /dev/console
 * and can be opened during boot-up.  Other consoles, such as USB consoles, may
 * not exist at boot-upand have to be handled in a different way.  Three file
 * descriptors and three file streams are still needed.
 */

#if defined(CONFIG_DEV_CONSOLE) || defined(CONFIG_CDCACM_CONSOLE) || \
    defined(CONFIG_PL2303_CONSOLE)
# if CONFIG_NFILE_DESCRIPTORS < 3
#   undef CONFIG_NFILE_DESCRIPTORS
#   define CONFIG_NFILE_DESCRIPTORS 3
# endif

# if CONFIG_NFILE_STREAMS > 0 && CONFIG_NFILE_STREAMS < 3
#  undef CONFIG_NFILE_STREAMS
#  define CONFIG_NFILE_STREAMS 3
# endif

/* If no console is selected, then disable all builtin console devices */

#else
#  undef CONFIG_DEV_LOWCONSOLE
#  undef CONFIG_RAMLOG_CONSOLE
#endif

/* If priority inheritance is disabled, then do not allocate any
 * associated resources.
 */

#if !defined(CONFIG_PRIORITY_INHERITANCE) || !defined(CONFIG_SEM_PREALLOCHOLDERS)
# undef CONFIG_SEM_PREALLOCHOLDERS
# define CONFIG_SEM_PREALLOCHOLDERS 0
#endif

#if !defined(CONFIG_PRIORITY_INHERITANCE) || !defined(CONFIG_SEM_NNESTPRIO)
# undef CONFIG_SEM_NNESTPRIO
# define CONFIG_SEM_NNESTPRIO 0
#endif

/* If no file descriptors are configured, then make certain no
 * streams are configured either.
 */

#if CONFIG_NFILE_DESCRIPTORS == 0
# undef CONFIG_NFILE_STREAMS
# define CONFIG_NFILE_STREAMS 0
#endif

/* There must be at least one memory region. */

#ifndef CONFIG_MM_REGIONS
# define CONFIG_MM_REGIONS 1
#endif

/* If the end of RAM is not specified then it is assumed to be the beginning
 * of RAM plus the RAM size.
 */

#ifndef CONFIG_DRAM_END
# define CONFIG_DRAM_END (CONFIG_DRAM_START+CONFIG_DRAM_SIZE)
#endif

/* If no file streams are configured, then make certain that buffered I/O
 * support is disabled
 */

#if CONFIG_NFILE_STREAMS == 0
# undef CONFIG_STDIO_BUFFER_SIZE
# define CONFIG_STDIO_BUFFER_SIZE 0
#endif

/* If no standard C buffered I/O is not supported, then line-oriented buffering
 * cannot be supported.
 */

#if CONFIG_STDIO_BUFFER_SIZE == 0
# undef CONFIG_STDIO_LINEBUFFER
#endif

/* If the maximum message size is zero, then we assume that message queues
 * support should be disabled
 */

#if CONFIG_MQ_MAXMSGSIZE <= 0 && !defined(CONFIG_DISABLE_MQUEUE)
# define CONFIG_DISABLE_MQUEUE 1
#endif

/* If mountpoint support in not included, then no filesystem can be supported */

#ifdef CONFIG_DISABLE_MOUNTPOINT
# undef CONFIG_FS_FAT
# undef CONFIG_FS_ROMFS
# undef CONFIG_FS_NXFFS
# undef CONFIG_FS_SMARTFS
# undef CONFIG_FS_BINFS
# undef CONFIG_NFS
#endif

/* Check if any readable and writable filesystem (OR USB storage) is supported */

#undef CONFIG_FS_READABLE
#undef CONFIG_FS_WRITABLE
#if defined(CONFIG_FS_FAT) || defined(CONFIG_FS_ROMFS) || defined(CONFIG_USBMSC) || \
    defined(CONFIG_FS_NXFFS) || defined(CONFIG_FS_SMARTFS) || defined(CONFIG_FS_BINFS) || \
    defined(CONFIG_NFS)
# define CONFIG_FS_READABLE 1
#endif

#if defined(CONFIG_FS_FAT) || defined(CONFIG_USBMSC) || defined(CONFIG_FS_NXFFS) || \
    defined(CONFIG_NFS)
# define CONFIG_FS_WRITABLE 1
#endif

/* There can be no network support with no socket descriptors */

#if CONFIG_NSOCKET_DESCRIPTORS <= 0
# undef CONFIG_NET
#endif

/* Conversely, if there is no network support, there is no need for
 * socket descriptors
 */

#ifndef CONFIG_NET
# undef CONFIG_NSOCKET_DESCRIPTORS
# define CONFIG_NSOCKET_DESCRIPTORS 0
#endif

/* Protocol support can only be provided on top of basic network support */

#ifndef CONFIG_NET
# undef CONFIG_NET_TCP
# undef CONFIG_NET_UDP
# undef CONFIG_NET_ICMP
#endif

/* NFS client can only be provided on top of UDP network support */

#if !defined(CONFIG_NET) || !defined(CONFIG_NET_UDP)
# undef CONFIG_NFS
#endif

/* Verbose debug and sub-system debug only make sense if debug is enabled */

#ifndef CONFIG_DEBUG
# undef CONFIG_DEBUG_VERBOSE
# undef CONFIG_DEBUG_SCHED
# undef CONFIG_DEBUG_MM
# undef CONFIG_DEBUG_PAGING
# undef CONFIG_DEBUG_DMA
# undef CONFIG_DEBUG_FS
# undef CONFIG_DEBUG_LIB
# undef CONFIG_DEBUG_BINFMT
# undef CONFIG_DEBUG_NET
# undef CONFIG_DEBUG_USB
# undef CONFIG_DEBUG_GRAPHICS
# undef CONFIG_DEBUG_GPIO
# undef CONFIG_DEBUG_SPI
# undef CONFIG_DEBUG_STACK
#endif

/* User entry point. This is provided as a fall-back to keep compatibility
 * with existing code, for builds which do not define CONFIG_USER_ENTRYPOINT.
 */

#ifndef CONFIG_USER_ENTRYPOINT
# define CONFIG_USER_ENTRYPOINT user_start
#endif

#endif /* __INCLUDE_NUTTX_CONFIG_H */
