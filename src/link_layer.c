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

// MISC



#define _POSIX_SOURCE 1 // POSIX compliant source
#define FLAG 0x7E
#define A 0x03
#define C_SET 0x03
#define C_UA 0x07

#define ESC 0x7D
#define C_I(seq) ((seq) ? 0x80 : 0x00)
#define C_RR(nr) (0x05 | ((nr & 0x01) << 7)) 
#define C_REJ(nr) (0x01 | ((nr & 0x01) << 7))

#define C_DISC 0x0B


static int gTimeout = 0;
static int gRetries = 0;
static LinkLayer currentLayer;


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

void alarmInit(){
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
}

static unsigned char computeBCC1(unsigned char a, unsigned char c) {
    return a ^ c;
}

static unsigned char computeBCC2(const unsigned char *data, int len) {
    unsigned char bcc = 0;
    for (int i = 0; i < len; i++)
        bcc ^= data[i];
    return bcc;
}

static void sendSupervisionFrame(uint8_t control){
    uint8_t frame[5];
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = control;
    frame[3] = computeBCC1(A, control);
    frame[4] = FLAG;
    writeBytesSerialPort(frame, sizeof(frame));
}

static void sendRR(int nr){
    uint8_t c = C_RR(nr);
    sendSupervisionFrame(c);
}

static void sendREJ(int nr){
    uint8_t c = C_REJ(nr);
    sendSupervisionFrame(c);
}

static int stuffData(const unsigned char *input, int inputSize, unsigned char *output){
    int index = 0;
    for (int i = 0; i < inputSize; i++){
        if (input[i] == FLAG){
            output[index++] = ESC;
            output[index++] = 0x5E;
        } else if (input[i] == ESC){
            output[index++] = ESC;
            output[index++] = 0x5D;
        } else {
            output[index++] = input[i];
        }
    }
    return index;
}

static int destuffData(const unsigned char *input, int inputSize, unsigned char *output){
    int index = 0;
    for (int i = 0; i < inputSize; i++){
        if (input[i] == ESC){
            i++;
            if (i >= inputSize) break;
            output[index++] = input[i] ^ 0x20;
        } else {
            output[index++] = input[i];
        }
    }
    return index;
}

LlWriteState lw_state;
uint8_t lw_areceived;
uint8_t lw_creceived;

void lw_reset_state() {
    lw_state = WAIT_FLAG;
    lw_areceived = 0;
    lw_creceived = 0;
}

int lw_process_byte(uint8_t byte, int expected_seq)
{
    switch (lw_state) {
        case WAIT_FLAG:
            if (byte == FLAG) 
                lw_state = WAIT_A;
            break;
        case WAIT_A:
            if (byte == A) { 
                lw_areceived = byte; 
                lw_state = WAIT_C; 
            }
            else if (byte != FLAG) 
                lw_state = WAIT_FLAG;
            break;
        case WAIT_C:
            if (byte == C_RR(expected_seq) || byte == C_RR((expected_seq+1)%2)) { 
                lw_creceived = byte; 
                lw_state = WAIT_BCC; 
            }
            else if (byte == C_REJ(expected_seq) || byte == C_REJ((expected_seq+1)%2)) {
                lw_creceived = byte; 
                lw_state = WAIT_BCC;
            }
            else if (byte == FLAG) 
                lw_state = WAIT_A;
            else 
                lw_state = WAIT_FLAG;
            break;
        case WAIT_BCC:
            if (byte == (lw_areceived ^ lw_creceived)) 
                lw_state = WAIT_STOP;
            else if (byte == FLAG) 
                lw_state = WAIT_A;
            else 
                lw_state = WAIT_FLAG;
            break;
        case WAIT_STOP:
            if (byte == FLAG) {
                if (lw_creceived == C_RR((expected_seq + 0) % 2) || lw_creceived == C_RR((expected_seq + 1) % 2))
                    return 1; // RR
                else if (lw_creceived == C_REJ((expected_seq + 0) % 2) || lw_creceived == C_REJ((expected_seq + 1) % 2))
                    return 2; // REJ
                lw_reset_state();
            } else lw_state = WAIT_FLAG;
            break;
    }
    return 0;
}


// LLOPEN
int llopen(LinkLayer connectionParameters)
{
    currentLayer = connectionParameters;
    gTimeout = connectionParameters.timeout;
    gRetries = connectionParameters.nRetransmissions;

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

        alarmInit();

        while (alarmCount < connectionParameters.nRetransmissions && state != STOP_STATE)
        {
            // Read one byte from serial port.
            // NOTE: You must check how many bytes were actually read by reading the return value.
            // In this example, we assume that the byte is always read, which may not be true.
            if (!alarmEnabled)
            {
                sendSupervisionFrame(C_SET);
                printf("SET sent (try #%d)\n", alarmCount + 1);
                
                alarm(connectionParameters.timeout); 
                alarmEnabled = TRUE;
            }


            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            if (bytes > 0)
            {
                nBytesBuf += bytes;
                stateMachine(byte);

                printf("Byte received: 0x%02X\n", byte);
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
        alarmInit();

        while (state != STOP_STATE) {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            if (bytes > 0) {
                nBytesBuf += bytes;
                stateMachine(byte);

                printf("Byte received: %02X\n", byte);

                if (state == STOP_STATE) {
                    // SET frame received correctly, send UA
                    sendSupervisionFrame(C_UA);
                    printf("UA sent to transmitter\n");
                }
            }
        }
    }
    else return -1;

    return 0;
}


// LLWRITE
int llwrite(const unsigned char *buf, int bufSize)
{
    static int sequenceNumber = 0;

    unsigned char BCC2 = computeBCC2(buf, bufSize);

    // Build I Frame
    unsigned char frame[2 * (bufSize + 6)];
    int index = 0;
    frame[index++] = FLAG;
    frame[index++] = A;
    frame[index++] = C_I(sequenceNumber);
    frame[index++] = computeBCC1(A, frame[2]);

    // Byte stuffing for DATA + BCC2
    unsigned char dataWithBCC2[bufSize + 1];
    memcpy(dataWithBCC2, buf, bufSize);
    dataWithBCC2[bufSize] = BCC2;

    int stuffedLen = stuffData(dataWithBCC2, bufSize + 1, frame + index);
    index += stuffedLen;

    frame[index++] = FLAG;

    // Transmission variables
    alarmEnabled = FALSE;
    alarmCount = 0;
    lw_reset_state();

    alarmInit();


    while (alarmCount < gRetries){
        if (!alarmEnabled){
            writeBytesSerialPort(frame, index);
            printf("I-frame (seq=%d) sent, try #%d\n", sequenceNumber, alarmCount + 1);
            alarm(gTimeout);
            alarmEnabled = TRUE;
        }

        unsigned char byte;
        int bytes = readByteSerialPort(&byte);

        if (bytes > 0) {
            int result = lw_process_byte(byte, (sequenceNumber + 1) % 2);

            if (result == 1){
                printf("RR received. Frame accepted.\n");
                alarm(0);
                sequenceNumber = (sequenceNumber + 1) % 2;
                return bufSize;
            } else if (result == 2){
                printf("REJ received. Retransmitting.\n");
                alarm(0);
                alarmEnabled = FALSE;
            }
        }
    }

    printf("Transmission failed after retries.\n");
    return -1;
}




// LLREAD
int llread(unsigned char *packet)
{
    static int expected_seq = 0;
    unsigned char frame[2048];
    int index = 0;
    unsigned char byte;
    int res;

    do {
        res = readByteSerialPort(&byte);
    } while (res > 0 && byte != FLAG);

    while (1){
        res = readByteSerialPort(&byte);
        if (res <= 0) continue;

        frame[index++] = byte;
        if (byte == FLAG) break;
        if (index >= sizeof(frame)) {
            fprintf(stderr, "[llread] Frame too large\n");
            break;
        }
    }


    unsigned char A_r = frame[0];
    unsigned char C_r = frame[1];
    unsigned char BCC1 = frame[2];

    if (BCC1 != computeBCC1(A_r, C_r)) {
        fprintf(stderr, "[llread] Header BCC1 error\n");
        sendREJ(expected_seq);
        return -1;
    }

    if (C_r != 0x00 && C_r != 0x80) {
        fprintf(stderr, "[llread] Not an I frame (C=0x%02X)\n", C_r);
        return -1;
    }

    int Ns = (C_r >> 7) & 0x01;


    unsigned char destuffed[2048];
    int destuffIndex = destuffData(frame + 3, index - 4, destuffed); // skip A, C, BCC1 and final FLAG
    if (destuffIndex < 0) {
        fprintf(stderr, "[llread] Destuffing error\n");
        sendREJ(expected_seq);
        return -1;
    }

    if (destuffIndex < 1) {
        fprintf(stderr, "[llread] Empty frame after destuff\n");
        return -1;
    }

    unsigned char receivedBCC2 = destuffed[destuffIndex - 1];
    unsigned char computedBCC2 = computeBCC2(destuffed, destuffIndex - 1);

    if (receivedBCC2 != computedBCC2) {
        fprintf(stderr, "[llread] Data BCC2 error (expected 0x%02X, got 0x%02X)\n",
                computedBCC2, receivedBCC2);
        sendREJ(expected_seq);
        return -1;
    }

    if (Ns == expected_seq) {
        memcpy(packet, destuffed, destuffIndex - 1);
    
        sendRR((expected_seq + 1) % 2);

        printf("[llread] I-frame (Ns=%d) accepted, RR(%d) sent\n", Ns, (expected_seq + 1) % 2);
        expected_seq = (expected_seq + 1) % 2;
        
        return destuffIndex - 1;
    } else {
        sendRR(expected_seq);
        printf("[llread] Duplicate I-frame (Ns=%d). RR resent.\n", Ns);
        return 0;
    }



    return 0;
}







// LLCLOSE
int llclose()
{

    int res;
    LlCloseState state = CLOSE_WAIT_FLAG;
    unsigned char byte;

    alarmInit();

    alarmEnabled = FALSE;
    alarmCount = 0;

    if (currentLayer.role == LlTx){
        printf("Transmitter: Sending DISC...\n");

        while (alarmCount < gRetries){
            if(!alarmEnabled){
                sendSupervisionFrame(C_DISC);
                printf("DISC sent (try #%d)\n", alarmCount + 1);
                alarm(gTimeout);
                alarmEnabled = TRUE;
            }

            while (1) {
                res = readByteSerialPort(&byte);
                if (res < 0) {
                    perror("readByteSerialPort"); 
                    return -1; }
                if (res == 0) continue;

                if(llclose_process_byte(&state, byte) > 0){
                    printf("Transmitter: DISC frame received. Sending UA...\n");
                    sendSupervisionFrame(C_UA);
                    printf("UA sent. Closing connection.\n");
                    alarm(0);
                    closeSerialPort();
                    return 0;
                }
            }
        }

        printf("Transmitter: Failed to receive DISC after retries. Closing connection.\n");
        return -1;

    }
    else if (currentLayer.role == LlRx){
        printf("Receiver: Waiting for DISC...\n");
        state = CLOSE_WAIT_FLAG;

        while (1) {
            res = readByteSerialPort(&byte);
            if (res < 0) { 
                perror("readByteSerialPort"); 
                closeSerialPort(); 
                return -1; 
            }
            if (res == 0) continue;

            if(llclose_process_byte(&state, byte))
            {
                printf("Receiver: DISC frame received. Sending DISC...\n");
                sendSupervisionFrame(C_DISC);
                break;
            }
        }

        alarmEnabled = FALSE;
        alarmCount = 0;
        state = CLOSE_WAIT_FLAG;
        printf("Receiver: Waiting for UA...\n");


        while (alarmCount < gRetries) {
            if (!alarmEnabled) {
                alarm(gTimeout);
                alarmEnabled = TRUE;
            }

            res = readByteSerialPort(&byte);
            if (res < 0) { 
                perror("readByteSerialPort"); 
                closeSerialPort(); 
                return -1; 
            }
            if (res == 0) continue;

            /* Now llclose_process_byte returns 2 for UA */
            int rc = llclose_process_byte(&state, byte);
            if (rc == 2) {
                printf("Receiver: UA frame received. Closing connection.\n");
                alarm(0);
                closeSerialPort();
                return 0;
            }
        }

        printf("Receiver: Failed to receive UA. Closing connection.\n");
        closeSerialPort();
        return -1;
    }
    return 0;
}



int llclose_process_byte(LlCloseState *state, uint8_t byte) {
    // return: 0 if not complete, 1 if DISC received, 2 if UA received

    static uint8_t matched_control = 0;

    switch (*state){
        case CLOSE_WAIT_FLAG:
            if (byte == FLAG){
                *state = CLOSE_WAIT_A;
            }
            break;
        case CLOSE_WAIT_A:
            if (byte == A){
                *state = CLOSE_WAIT_C;
            } else if (byte != FLAG){
                *state = CLOSE_WAIT_FLAG;
            }
            break;
        case CLOSE_WAIT_C:
            if (byte == C_DISC || byte == C_UA){
                matched_control = byte;
                *state = CLOSE_WAIT_BCC;
            } else if (byte == FLAG){
                *state = CLOSE_WAIT_A;
            } else {
                *state = CLOSE_WAIT_FLAG;
            }
            break;
        case CLOSE_WAIT_BCC:
            if (matched_control!=0 && byte == computeBCC1(A, matched_control)){
                *state = CLOSE_WAIT_STOP;
            } else if (byte == FLAG){
                *state = CLOSE_WAIT_A;
            } else {
                *state = CLOSE_WAIT_FLAG;
            }
            break;
        case CLOSE_WAIT_STOP:
            if (byte == FLAG){
                if (matched_control == C_DISC)
                    return 1;
                else if (matched_control == C_UA)
                    return 2;
            } else {
                *state = CLOSE_WAIT_FLAG;
            }
            break;
    }
    return 0;
}