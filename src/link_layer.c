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
#define C_DISC 0x0B
#define C_I(seq) ((seq) ? 0x80 : 0x00)
#define C_RR(nr) (0x05 | ((nr & 0x01) << 7)) 
#define C_REJ(nr) (0x01 | ((nr & 0x01) << 7))

#define ESC 0x7D
#define ESC_FLAG 0x5E
#define ESC_ESC 0x5D

#define MIN_SUPERVISION_FRAME_SIZE 5
#define MAX_FRAME_SIZE 2048
#define FRAME_HEADER_SIZE 3
#define FRAME_OVERHEAD 4

#define MAX_STUFFED_SIZE(data_len) (2* ((data_len) + 6))
static int gTimeout = 0;
static int gRetries = 0;
static LinkLayer currentLayer;


int alarmEnabled = FALSE;
int alarmCount = 0;


// ============================================================================
//  BCC HELPERS
// ============================================================================

static unsigned char computeBCC1(unsigned char a, unsigned char c) {
    return a ^ c;
}

static unsigned char computeBCC2(const unsigned char *data, int len) {
    unsigned char bcc = 0;
    for (int i = 0; i < len; i++)
        bcc ^= data[i];
    return bcc;
}


// ============================================================================
// PARSER STATE MACHINE
// ============================================================================

static void resetParser(FrameParser *parser) {
    parser->state = PARSE_WAIT_FLAG;
    parser->receivedA = 0;
    parser->receivedC = 0;
}

static int parseSupervisionFrame(FrameParser *parser, unsigned char byte){
    switch (parser->state){
        case PARSE_WAIT_FLAG:
            if (byte == FLAG)
                parser->state = PARSE_WAIT_A;
            break;
        case PARSE_WAIT_A:
            if (byte == A){
                parser->receivedA = byte;
                parser->state = PARSE_WAIT_C;
            } else if (byte != FLAG)
                parser->state = PARSE_WAIT_FLAG;
            break;
        case PARSE_WAIT_C:
            parser->receivedC = byte;
            parser->state = PARSE_WAIT_BCC;
            break;    
        case PARSE_WAIT_BCC:
            if (byte == computeBCC1(parser->receivedA, parser->receivedC))
                parser->state = PARSE_WAIT_STOP;
            else if (byte == FLAG)
                parser->state = PARSE_WAIT_A;
            else 
                parser->state = PARSE_WAIT_FLAG;
            break;
        case PARSE_WAIT_STOP:
            if (byte == FLAG){
                return 1; // Frame received successfully
            } else
                parser->state = PARSE_WAIT_FLAG;
            break;
    }
    return 0;
}


// ============================================================================
// ALARM HANDLER AND RETRY LOGIC
// ============================================================================

void alarmHandler(int signal){
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

static void startRetryTimer(int timeout) {
    alarm(timeout);
    alarmEnabled = TRUE;
}

static void stopRetryTimer(void) {
    alarm(0);
    alarmEnabled = FALSE;
}

static void resetRetryState(void) {
    alarmEnabled = FALSE;
    alarmCount = 0;
}

static int canRetry(void) {
    return alarmCount < gRetries;
}

// ============================================================================
// FRAME HELPERS
// ============================================================================

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


// ============================================================================
// DATA STUFFING / DESTUFFING
// ============================================================================

static int stuffData(const unsigned char *input, int inputSize, unsigned char *output){
    int index = 0;
    for (int i = 0; i < inputSize; i++){
        if (input[i] == FLAG){
            output[index++] = ESC;
            output[index++] = ESC_FLAG;
        } else if (input[i] == ESC){
            output[index++] = ESC;
            output[index++] = ESC_ESC;
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
            if (i >= inputSize) return -1; // Error: ESC at end of input
            output[index++] = input[i] ^ 0x20;
        } else {
            output[index++] = input[i];
        }
    }
    return index;
}


// ============================================================================
// FRAME READING AND VALIDATION
// ============================================================================

static int readCompleteFrame(unsigned char *frame, int maxLen) {
    unsigned char byte;
    int res;
    int index = 0;

    // Wait for starting FLAG
    do {
        res = readByteSerialPort(&byte);
        if (res < 0) return -1;
    } while (res > 0 && byte != FLAG);

    // Read until ending FLAG
    while (1) {
        res = readByteSerialPort(&byte);
        if (res < 0) return -1;
        if (res == 0) continue;

        if (index < maxLen)
            frame[index++] = byte;
        else {
            fprintf(stderr, "[readCompleteFrame] Frame too large\n");
            return -1;
        }

        if (byte == FLAG) break;
    }
    return index;
}

static int validateIFrameHeader(unsigned char A_r, unsigned char C_r, unsigned char BCC1, int expected_seq) {
    // Check BCC1
    if (BCC1 != computeBCC1(A_r, C_r)) {
        fprintf(stderr, "[llread] Header BCC1 error\n");
        sendREJ(expected_seq);
        return -1;
    }

    // Check if it's an I-frame (C must be 0x00 or 0x80)
    if (C_r != 0x00 && C_r != 0x80) {
        fprintf(stderr, "[llread] Not an I-frame (C=0x%02X)\n", C_r);
        return -1;
    }

    // Extract and return sequence number
    return (C_r >> 7) & 0x01;
}

static int validateBCC2(const unsigned char *data, int dataLen, unsigned char receivedBCC2) {
    unsigned char computedBCC2 = computeBCC2(data, dataLen);
    
    if (receivedBCC2 != computedBCC2) {
        fprintf(stderr, "[llread] BCC2 error: computed=0x%02X, received=0x%02X\n",
                computedBCC2, receivedBCC2);
        return -1;
    }
    return 0;
}


// ============================================================================
// LLOPEN
// ============================================================================

int llopen(LinkLayer connectionParameters)
{
    currentLayer = connectionParameters;
    gTimeout = connectionParameters.timeout;
    gRetries = connectionParameters.nRetransmissions;

    
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0){
            perror("openSerialPort");
            return -1;
    }
    printf("Serial port %s opened\n", connectionParameters.serialPort);


    // If transmitter
    if (connectionParameters.role == LlTx){
        FrameParser parser;
        resetParser(&parser);

        alarmInit();

        while (canRetry())
        {
            if (!alarmEnabled)
            {
                sendSupervisionFrame(C_SET);
                printf("SET sent (try #%d)\n", alarmCount + 1);
                startRetryTimer(gTimeout);
            }

            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            if (bytes > 0)
            {
                
                if(parseSupervisionFrame(&parser, byte) && parser.receivedC == C_UA){
                    printf("UA frame received successfully.\n");
                    stopRetryTimer();
                    return 0;
                }
            }
        }

        printf("llopen Tx failed after retries.\n");
        closeSerialPort();
        return -1;
    } else if (connectionParameters.role == LlRx){
        FrameParser parser;
        resetParser(&parser);

        // Set up the alarm handler 
        alarmInit();

        while (1) {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            if (bytes > 0) {

                printf("Byte received: %02X\n", byte);

                if (parseSupervisionFrame(&parser, byte)){
                    if (parser.receivedC == C_SET){
                        sendSupervisionFrame(C_UA);
                        printf("SET received, UA sent to transmitter\n");
                        return 0;
                    }
                    resetParser(&parser);
                }
            }
        }
    }
    return -1;
}


// ============================================================================
// LLWRITE
// ============================================================================

int llwrite(const unsigned char *buf, int bufSize)
{
    static int sequenceNumber = 0;

    unsigned char BCC2 = computeBCC2(buf, bufSize);

    // Build I Frame
    unsigned char frame[MAX_STUFFED_SIZE(bufSize)];
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

    resetRetryState();

    FrameParser parser;
    resetParser(&parser);
    alarmInit();


    while (canRetry()){
        if (!alarmEnabled){
            writeBytesSerialPort(frame, index);
            printf("I-frame (seq=%d) sent, try #%d\n", sequenceNumber, alarmCount + 1);
            startRetryTimer(gTimeout);
        }

        unsigned char byte;
        int bytes = readByteSerialPort(&byte);

        if (bytes > 0) {
            if (parseSupervisionFrame(&parser, byte)){
                int expected = (sequenceNumber + 1) % 2;
                if (parser.receivedC == C_RR(expected)){
                    printf("RR received. Frame accepted.\n");
                    stopRetryTimer();
                    sequenceNumber = expected;
                    return bufSize;
                } else if (parser.receivedC == C_REJ(sequenceNumber % 2)){
                    printf("REJ received. Retransmitting.\n");
                    stopRetryTimer();
                    resetParser(&parser);
                }
            } 
        }
    }

    printf("Transmission failed after retries.\n");
    return -1;
}


// ============================================================================
// LLREAD
// ============================================================================

int llread(unsigned char *packet)
{
    static int expected_seq = 0;
    unsigned char frame[MAX_FRAME_SIZE];
    

    int index = readCompleteFrame(frame, sizeof(frame));
    if (index < MIN_SUPERVISION_FRAME_SIZE) {
        fprintf(stderr, "[llread] Incomplete frame received\n");
        return -1;
    }


    unsigned char A_r = frame[0];
    unsigned char C_r = frame[1];
    unsigned char BCC1 = frame[2];

    int Ns = validateIFrameHeader(A_r, C_r, BCC1, expected_seq);
    if (Ns < 0) return -1;


    unsigned char destuffed[MAX_FRAME_SIZE];
    int destuffIndex = destuffData(frame + FRAME_HEADER_SIZE, index - FRAME_OVERHEAD, destuffed); // skip A, C, BCC1 and final FLAG
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
    
    if (validateBCC2(destuffed, destuffIndex - 1, receivedBCC2) < 0) {
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

}


// ============================================================================
// LLCLOSE
// ============================================================================

int llclose()
{
    int res;
    unsigned char byte;

    alarmInit();
    resetRetryState();

    if (currentLayer.role == LlTx){
        printf("Transmitter: Sending DISC...\n");
        FrameParser parser;
        resetParser(&parser);

        while (canRetry()){
            if(!alarmEnabled){
                sendSupervisionFrame(C_DISC);
                printf("DISC sent (try #%d)\n", alarmCount + 1);
                startRetryTimer(gTimeout);
            }

            while (1) {
                res = readByteSerialPort(&byte);
                if (res < 0) {
                    perror("readByteSerialPort"); 
                    closeSerialPort();
                    return -1; }
                if (res == 0) continue;

                if(parseSupervisionFrame(&parser, byte) && parser.receivedC == C_DISC){
                    printf("Transmitter: DISC frame received. Sending UA...\n");
                    sendSupervisionFrame(C_UA);
                    printf("UA sent. Closing connection.\n");
                    stopRetryTimer();
                    closeSerialPort();
                    return 0;
                }
            }
        }

        printf("Transmitter: Failed to receive DISC after retries. Closing connection.\n");
        closeSerialPort();
        return -1;

    }
    else if (currentLayer.role == LlRx){
        printf("Receiver: Waiting for DISC...\n");
        FrameParser parser;
        resetParser(&parser);

        while (1) {
            res = readByteSerialPort(&byte);
            if (res < 0) { 
                perror("readByteSerialPort"); 
                closeSerialPort(); 
                return -1; 
            }
            if (res == 0) continue;

            if(parseSupervisionFrame(&parser, byte) && parser.receivedC == C_DISC)
            {
                printf("Receiver: DISC frame received. Sending DISC...\n");
                sendSupervisionFrame(C_DISC);
                break;
            }
        }

        resetRetryState();
        resetParser(&parser);
        printf("Receiver: Waiting for UA...\n");


        while (canRetry()) {
            if (!alarmEnabled) {
                startRetryTimer(gTimeout);
            }

            res = readByteSerialPort(&byte);
            if (res < 0) { 
                perror("readByteSerialPort"); 
                closeSerialPort(); 
                return -1; 
            }
            if (res == 0) continue;


            if (parseSupervisionFrame(&parser, byte) && parser.receivedC == C_UA) {
                printf("Receiver: UA frame received. Closing connection.\n");
                stopRetryTimer();
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

