// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

#define MAX_SIZE 256

// ============================================================================
// CONTROL CONSTANTS
// ============================================================================
#define CONTROL_DATA 1
#define CONTROL_START 2
#define CONTROL_END 3


// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static int createControlPacket(unsigned char *packet, unsigned char controlType) {
    packet[0] = controlType;
    return 1;
}

static int createDataPacket(unsigned char *packet, const unsigned char *data, size_t dataLen) {
    packet[0] = CONTROL_DATA;
    packet[1] = dataLen & 0xFF;         // L1 (LSB)
    packet[2] = (dataLen >> 8) & 0xFF;  // L2 (MSB)
    memcpy(packet + 3, data, dataLen);
    return 3 + dataLen;
}

static int parseDataPacket(const unsigned char *packet, int packetLen, unsigned char *data) {
    if (packetLen < 3) {
        fprintf(stderr, "[APP] Data packet too short\n");
        return -1;
    }
    
    int dataLen = packet[1] | (packet[2] << 8); // L1 | (L2 << 8)
    
    if (dataLen != packetLen - 3) {
        fprintf(stderr, "[APP] Length mismatch: expected %d, got %d\n", 
                dataLen, packetLen - 3);
        return -1;
    }
    
    memcpy(data, packet + 3, dataLen);
    return dataLen;
}

// After parseDataPacket:

// ============================================================================
// TRANSMITTER
// ============================================================================
static int transmitFile(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("[APP] Failed to open file");
        return -1;
    }
    
    // Send START packet
    unsigned char startPacket[MAX_SIZE];
    int startLen = createControlPacket(startPacket, CONTROL_START);
    if (llwrite(startPacket, startLen) < 0) {
        fprintf(stderr, "[APP] Failed to send START packet\n");
        fclose(file);
        return -1;
    }
    printf("[APP] START packet sent\n");
    
    // Send DATA packets
    unsigned char buffer[MAX_SIZE];
    unsigned char dataPacket[MAX_SIZE + 3];
    size_t bytesRead;
    int totalSent = 0;
    
    while ((bytesRead = fread(buffer, 1, MAX_SIZE, file)) > 0) {
        int packetLen = createDataPacket(dataPacket, buffer, bytesRead);
        
        if (llwrite(dataPacket, packetLen) < 0) {
            fprintf(stderr, "[APP] llwrite() failed\n");
            fclose(file);
            return -1;
        }
        
        totalSent += bytesRead;
        printf("[APP] DATA packet sent (%zu bytes, total: %d)\n", bytesRead, totalSent);
    }
    
    fclose(file);
    
    // Send END packet
    unsigned char endPacket[MAX_SIZE];
    int endLen = createControlPacket(endPacket, CONTROL_END);
    if (llwrite(endPacket, endLen) < 0) {
        fprintf(stderr, "[APP] Failed to send END packet\n");
        return -1;
    }
    printf("[APP] END packet sent\n");
    
    printf("[APP] File transmission complete: %d bytes sent\n", totalSent);
    return 0;
}


// ============================================================================
// RECEIVER
// ============================================================================
static int receiveFile(const char *filename) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        perror("[APP] Failed to create file");
        return -1;
    }
    
    printf("[APP] Receiving file: %s\n", filename);
    
    unsigned char packet[MAX_SIZE + 3];
    unsigned char data[MAX_SIZE];
    int totalReceived = 0;
    int startReceived = 0;
    
    while (1) {
        int received = llread(packet);
        
        if (received < 0) {
            fprintf(stderr, "[APP] llread() failed\n");
            break;
        }
        
        if (received == 0) continue;
        
        unsigned char control = packet[0];
        
        switch (control) {
            case CONTROL_START:
                printf("[APP] START packet received\n");
                startReceived = 1;
                break;
                
            case CONTROL_DATA:
                if (!startReceived) {
                    fprintf(stderr, "[APP] DATA before START!\n");
                    fclose(file);
                    return -1;
                }
                
                int dataLen = parseDataPacket(packet, received, data);
                if (dataLen < 0) {
                    fprintf(stderr, "[APP] Failed to parse DATA packet\n");
                    continue;
                }
                
                fwrite(data, 1, dataLen, file);
                totalReceived += dataLen;
                printf("[APP] DATA packet received (%d bytes, total: %d)\n", 
                       dataLen, totalReceived);
                break;
                
            case CONTROL_END:
                printf("[APP] END packet received\n");
                fclose(file);
                printf("[APP] File reception complete: %d bytes received\n", totalReceived);
                return 0;
                
            default:
                fprintf(stderr, "[APP] Unknown control: 0x%02X\n", control);
                break;
        }
    }
    
    fclose(file);
    return -1;
}


// ============================================================================
// APPLICATION LAYER MAIN FUNCTION
// ============================================================================

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;

    // Configure link layer parameters
    snprintf(ll.serialPort, sizeof(ll.serialPort), "%s", serialPort);

    // Map role string to enum
    if (strcmp(role, "tx") == 0)
        ll.role = LlTx;
    else
        ll.role = LlRx;

    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    // Open connection
    printf("[APP] Opening connection...\n");
    if (llopen(ll)<0){
        printf("[APP] llopen() failed\n");
        return;
    }

    printf("[APP] Connection established\n");

    int result;
    if (ll.role ==LlTx){
        result = transmitFile(filename);
    } else {
        result = receiveFile(filename);
    }

    
    printf("[APP] Closing connection...\n");
    
    if (llclose() == 0){
        printf("[APP] Connection closed\n");
    } else {
        printf("[APP] llclose() failed\n");
    }

    if (result == 0){
        printf("[APP] File transfer completed successfully\n");
    } else {
        printf("[APP] File transfer failed\n");
    }
}
