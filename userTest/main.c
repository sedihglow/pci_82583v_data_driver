/******************************************************************************
 * filename: main.c
 *
 * Used to test the ethernet driver, blink the Led
 *
 * Written by: James Ross
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <inttypes.h>

#define B_RATE 5
int main(void)
{
    int fd;
    uint32_t ret;
    uint16_t head;
    uint16_t tail;

    /* open driver */
    fd = open("/dev/pciDev", O_RDWR);
    if(fd == -1){
        printf("open error\n");
        exit(EXIT_FAILURE);
    }
   
    read(fd, &ret, sizeof(uint32_t));

    head = (ret >> 16) & 0xFFFF;
    tail = ret;
   
    printf("Head: %"PRIu16"\n", head);
    printf("Tail: %"PRIu16"\n", tail);
    close(fd);
    exit(EXIT_SUCCESS);
}
