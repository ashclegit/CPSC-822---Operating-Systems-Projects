// Course:  CPSC 8220 - Spring 2017
// Project: #1 - Device Driver
// Team Members:
//    Ashwin Kumar Vajantri [aashwin@g.clemson.edu]
//    Kunwar Deep Singh Toor [ktoor@g.clemson.edu]
//    Matthew Pfister [mpfiste@g.clemson.edu]
//    Rohith Raju [rraju@g.clemson.edu]
//    Saroj Kumar Dash [sdash@g.clemson.edu]
//    Vishnuprabhu Thirugnanasambandam [vthirug@g.clemson.edu]

// Description:
//    Shared header for the Kyouko3 PCIe graphics card driver.

#include <linux/ioctl.h>

// ~~ Only Kernel/User Shared Definitions Appear Here
// ioctl commands
#define VMODE      _IOW (0xCC, 0, unsigned long)
#define BIND_DMA   _IOW (0xCC, 1, unsigned long)
#define START_DMA  _IOWR(0xCC, 2, unsigned long)
#define FIFO_QUEUE _IOWR(0xCC, 3, unsigned long)
#define FIFO_FLUSH _IO  (0xCC, 4)
#define UNBIND_DMA _IOW (0xCC, 5, unsigned long)

// graphics modes
#define GRAPHICS_OFF 0
#define GRAPHICS_ON 1

// fifo commands
#define VertexColor 0x5010
#define VertexCoordinate 0x5000
#define CommandPrimitive 0x3000
#define VertexEmit 0x3004
#define RasterFlush 0x3FFC
// ~~
