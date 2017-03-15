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
//    User code for issuing commands to the Kyouko3 PCIe graphics card
//    in order to draw triangles.

// Usage:
//    To draw a triangle with commands issued through the FIFO,
//    run the program with a command-line argument of 0 (or no argument).
//
//    To draw 50,000 triangles with commands issued through DMA buffers,
//    run the program with a command-line argument of 1. The commands are
//    separated across 50 DMA buffer requests, each buffer holding the commands
//    to draw 1,000 triangles.

#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "mymod.h"

#define U_NUM_TRIANGLES 1000 // # of triangle written to each DMA buffer
#define U_NUM_BUFFS 50       // # of DMA buffers written to

void die_with_message(char *message) {
   fprintf(stderr, "%s\n", message);
   exit(1);
}

void u_fifo_queue(int fd, unsigned int command, unsigned int value) {
   unsigned int fifo_entry[] = { command, value };
   if(ioctl(fd, FIFO_QUEUE, &fifo_entry)) die_with_message("FIFO_QUEUE call to ioctl failed."); 
}

void fifo_commands(int fd) {
   int i, j;
   static float coordinates[3][4] = {
      -0.7f, -0.7f,  0.0f,  1.0f, // upper-left
       0.7f, -0.7f,  0.0f,  1.0f, // upper-right
       0.7f,  0.7f,  0.0f,  1.0f  // lower-right
   };
   static float colors[3][4] = {
       0.0f,  0.0f,  1.0f,  0.0f, // red
       0.0f,  1.0f,  0.0f,  0.0f, // green
       1.0f,  0.0f,  0.0f,  0.0f  // blue
   };

   // Fill the queue with vertex information
   u_fifo_queue(fd, CommandPrimitive, 1);
   for(i = 0; i < 3; ++i) {
      for(j = 0; j < 4; ++j) u_fifo_queue(fd, VertexColor + j * 4, *(unsigned int*)&colors[i][j]);
      for(j = 0; j < 4; ++j) u_fifo_queue(fd, VertexCoordinate + j * 4, *(unsigned int*)&coordinates[i][j]);
      u_fifo_queue(fd, VertexEmit, 0);      
   }
   u_fifo_queue(fd, CommandPrimitive, 0);

   u_fifo_queue(fd, RasterFlush, 0);
   ioctl(fd, FIFO_FLUSH, 0);
}

void generate_coordinates(float *coords) {
   int i;
   for(i = 0; i < 3; ++i) {
      coords[i*3 + 0] = 2.0f * drand48() - 1.0f;
      coords[i*3 + 1] = 2.0f * drand48() - 1.0f;
      coords[i*3 + 2] = 0.0f;
   }
}

void generate_colors(float *colors) {
   int i;
   for(i = 0; i < 3; ++i) {
      colors[i*3 + 0] = drand48();
      colors[i*3 + 1] = drand48();
      colors[i*3 + 2] = drand48();
   }
}

struct u_dma_header {
   uint32_t address: 14;
   uint32_t count: 10;
   uint32_t opcode: 8;
};

void dma_commands(int fd) {
   unsigned int i, j, k, *u_buff_base, *u_buff_curr;
   unsigned long byte_count;
   float colors[9], coords[9]; 
   struct u_dma_header dma_header = {
      .address = 0x1045,
      .count = 0,
      .opcode = 0x14
   };

   srand(time(NULL));
   if(ioctl(fd, BIND_DMA, &u_buff_base)) die_with_message("BIND_DMA call to ioctl failed.");

   for(i = 0; i < U_NUM_BUFFS; ++i) {
      u_buff_curr = u_buff_base + 1;
      dma_header.count = byte_count = 0;

      for(j = 0; j < U_NUM_TRIANGLES; ++j) {
         generate_colors(colors);
         generate_coordinates(coords);

         for(k = 0; k < 3; ++k) {
            *(u_buff_curr++) = *(unsigned int*)&colors[k*3 + 0];
            *(u_buff_curr++) = *(unsigned int*)&colors[k*3 + 1];
            *(u_buff_curr++) = *(unsigned int*)&colors[k*3 + 2];
         }
         for(k = 0; k < 3; ++k) {
            *(u_buff_curr++) = *(unsigned int*)&coords[k*3 + 0];
            *(u_buff_curr++) = *(unsigned int*)&coords[k*3 + 1];
            *(u_buff_curr++) = *(unsigned int*)&coords[k*3 + 2];
         }

         dma_header.count += 3;
         byte_count += 72;
      }

      *u_buff_base = *(unsigned int *)&dma_header; // Prepend the header.
      u_buff_base = (unsigned int*)byte_count;     // Pull double-duty as the buffer byte size.

      u_fifo_queue(fd, RasterFlush, 0);
      ioctl(fd, FIFO_FLUSH, 0);
      if(ioctl(fd, START_DMA, &u_buff_base)) die_with_message("START_DMA call to ioctl failed.");
   }
   u_fifo_queue(fd, RasterFlush, 0);
   ioctl(fd, FIFO_FLUSH, 0);
   ioctl(fd, UNBIND_DMA, 0);
}

int main(int argc, char **argv) {
   int fd, mode = 0;

   // Parse potential command-line argument.
   if(argc > 2) die_with_message("Invalid number of command-line arguments.");
   if(argc == 2) mode = atoi(argv[1]);

   // Open the character device.
   fd = open("/dev/kyouko3", O_RDWR);
   if(fd < 0) die_with_message("Character device was unable to be opened.");

   // Run graphics commands.
   ioctl(fd, VMODE, GRAPHICS_ON);
   if(mode == 0) fifo_commands(fd);
   else          dma_commands(fd);
   sleep(3); 
   ioctl(fd, VMODE, GRAPHICS_OFF);

   close(fd);
}
