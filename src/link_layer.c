// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include "serial_port.h"
// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define MAX_RETRIES 3
#define FLAG 0x7E
#define A 0x03
#define C_SET 0x03
#define C_UA 0x07


int alarmEnabled = FALSE;
int alarmCount = 0;

uint8_t areceived;
uint8_t creceived;

State state = START;
void stateMachine (uint8_t byte){
    switch (state){
        case START:
            if (byte == FLAG){
                state = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (byte == A){
                areceived = byte;
                state = A_RCV;
            } else if (byte != FLAG){
                state = START;
            }
            break;
        case A_RCV:
            if (byte == C_SET || byte == C_UA){
                state = C_RCV;
                creceived = byte;
            } else if (byte == FLAG){
                state = FLAG_RCV;
            } else {
                state = START;
            }
            break;    
        case C_RCV:
            if (byte == (areceived ^ creceived)){
                state = BCC_OK;
            } else if (byte == FLAG){
                state = FLAG_RCV;
            } else {
                state = START;
            }
            break;
        case BCC_OK:
            if (byte == FLAG){
                state = STOP_STATE;
            } else {
                state = START;
            }
            break;
        case STOP_STATE:
            break;
    }
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO: Implement this function

    // Open Serial Port
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0){
            perror("openSerialPort");
            return -1;
    }
    printf("Serial port %s opened\n", connectionParameters.serialPort);


    // If transmitter
    if (connectionParameters.role == LlTx){
        state = START;
        int nBytesBuf = 0;

        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            exit(1);
        }

        while (alarmCount < MAX_RETRIES && state != STOP_STATE)
        {
            // Read one byte from serial port.
            // NOTE: You must check how many bytes were actually read by reading the return value.
            // In this example, we assume that the byte is always read, which may not be true.
            if (!alarmEnabled)
            {
                unsigned char SET_frame[] = {FLAG, A, C_SET, A^C_SET, FLAG};
                writeBytesSerialPort(SET_frame, sizeof(SET_frame));
                printf("SET sent (try #%d)\n", alarmCount + 1);
                
                alarm(3); 
                alarmEnabled = TRUE;
            }


            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            if (bytes > 0)
            {
                nBytesBuf += bytes;
                stateMachine(byte);

                printf("Byte received: %c\n", byte);
                if (state == STOP_STATE){ 
                    printf("Received STOP state. Ending reading.\n");
                    alarm(0);
                }
            }
        }

    }
    else if (connectionParameters.role == LlRx){
        state = START;
        int nBytesBuf = 0;

        // Set up the alarm handler in case you want timeouts
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1) {
            perror("sigaction");
            exit(1);
        }

        while (state != STOP_STATE) {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            if (bytes > 0) {
                nBytesBuf += bytes;
                stateMachine(byte);

                printf("Byte received: %02X\n", byte);

                if (state == STOP_STATE) {
                    // SET frame received correctly, send UA
                    unsigned char UA_frame[] = {FLAG, A, C_UA, A^C_UA, FLAG};
                    writeBytesSerialPort(UA_frame, sizeof(UA_frame));
                    printf("UA sent to transmitter\n");
                }
            }
        }
    }
    else return -1;

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}


