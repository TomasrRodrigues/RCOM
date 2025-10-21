// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

#define MAX_SIZE 256


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;

    // Copy serial port
    snprintf(ll.serialPort, sizeof(ll.serialPort), "%s", serialPort);

    // Map role string to enum
    if (strcmp(role, "tx") == 0)
        ll.role = LlTx;
    else
        ll.role = LlRx;

    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    printf("applicationLayer: calling llopen()...\n");
    if (llopen(ll)<0){
        printf("[APP] llopen() failed\n");
        return;
    }
    printf("[APP] llopen() successful\n");


    if (ll.role == LlTx){
        // Transmitter code
        //printf("[APP] Transmitter: Sending file %s...\n", filename);
        FILE *file = fopen(filename, "rb");
        if (!file) {
            perror("fopen");
            llclose();
            return;
        }
        
        unsigned char startPacket[MAX_SIZE];
        int startLen = 0;
        startPacket[startLen++] = 2; // Control field for start packet

        llwrite(startPacket, startLen);
        printf("[APP] Start packet sent\n");

        unsigned char buffer[MAX_SIZE];
        size_t bytesRead;
        int totalSent = 0;
        while ((bytesRead = fread(buffer, 1, MAX_SIZE, file)) > 0){
            unsigned char dataPacket[MAX_SIZE + 3];
            dataPacket[0] = 1; // control = DATA
            dataPacket[1] = (bytesRead >> 8) & 0xFF; // L2
            dataPacket[2] = bytesRead & 0xFF;        // L1
            memcpy(dataPacket + 3, buffer + 3, bytesRead);
            int sent = llwrite(dataPacket, 3 + bytesRead);
            if (sent > 0){
                totalSent += bytesRead;
                printf("[APP] DATA packet sent (%zu bytes)\n", bytesRead);
            } else {
                printf("[APP] llwrite() failed\n");
                break;
            }
        }
        fclose(file);
        printf("[APP] File transmission finished, total %d bytes sent\n", totalSent);

        unsigned char endPacket[1] = {3};
        llwrite(endPacket, 1);
        printf("[APP] End packet sent\n");
        printf("[APP] File %s sent successfully\n", filename);
        /*
       
         {
            int sent = llwrite(buffer, bytesRead);
            if (sent > 0){
                printf("[APP] llwrite() succeeded (%d bytes sent)\n", sent);
                totalSent += sent;
            } else {
                printf("[APP] llwrite() failed\n");
                break;
            }
        }
        fclose(file);
        printf("[APP] File transmission finished, total %d bytes sent\n", totalSent);

        
        const char *msg = "Hello from transmitter!";
        int sent = llwrite((unsigned char *) msg, strlen(msg));
        if (sent > 0){
            printf("[APP] llwrite() succeeded (%d bytes sent)\n", sent);
        } else
            printf("[APP] llwrite() failed\n");*/
    }


    else if (ll.role == LlRx){
        // Receiver code
        FILE *file = fopen(filename, "wb");
        if (!file){
            perror("[APP] Error creating file");
            llclose();
            return;
        }
        unsigned char buffer[MAX_SIZE];
        int received;
        int totalReceived = 0;

        printf("[APP] Receiver: Receiving file %s...\n", filename);
        while ((received = llread(buffer)) > 0) {

            unsigned char control = buffer[0];
            printf("Control byte: %d\n", control);

            if (control == 3){
                printf("End of transmission control received. Stopping reception.\n");
                break;
            }
            else if (control == 2){
                printf("Start of transmission control received.\n");
                continue;
            } else if (control == 1){
                fwrite(buffer, 1, received, file);
                totalReceived += received;
                printf("[APP] llread() succeeded (%d bytes received)\n", received);
                printf("Received: %d\n", buffer[0] == 3);
            }
            

        }

        printf("Closed llread loop");


        fclose(file);
        printf("[APP] File %s received successfully\n", filename);
    }

    

    if (llclose() == 0){
        printf("[APP] llclose() successful\n");
    } else {
        printf("[APP] llclose() failed\n");
    }
}
