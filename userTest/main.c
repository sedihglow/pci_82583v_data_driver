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

#define B_RATE 5
int main(void)
{
    int retBytes = 0;
    int fd;
    int readBuff = 0;
    int toWrite = B_RATE;

    /* open driver */
    fd = open("/dev/pciLED", O_RDWR);
    if(fd == -1){
        printf("open error\n");
        exit(EXIT_FAILURE);
    }
   
    read(fd, &readBuff, sizeof(int));

    printf("pre-write: blink rate: %d\n", readBuff);

    sleep(5); /* watch LED blink at module default/param rate */

    printf("post first sleep\n");

    write(fd, &toWrite, sizeof(int));

    printf("post-write, pre-read\n");

    read(fd, &readBuff, sizeof(int));

    printf("post-write: blink rate: %d\n", readBuff);

    sleep(15); /* watch led blink at B_RATE */

    close(fd);
    exit(EXIT_SUCCESS);
}
