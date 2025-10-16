// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

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

        const char *msg = "Hello from transmitter!";
        int sent = llwrite((unsigned char *) msg, strlen(msg));
        if (sent > 0){
            printf("[APP] llwrite() succeeded (%d bytes sent)\n", sent);
        } else
            printf("[APP] llwrite() failed\n");
    }


    else if (ll.role == LlRx){
        // Receiver code
        unsigned char buffer[2048];

        printf("[APP] Receiver: Receiving file %s...\n", filename);

        int received = llread(buffer);
        if (received > 0){
            buffer[received] = '\0';
            printf("[APP] Received message: \"%s\"\n", buffer);
        }
        else{
            printf("[APP] llread() failed or no data.\n");
        }
    }

    if (llclose() == 0){
        printf("[APP] llclose() successful\n");
    } else {
        printf("[APP] llclose() failed\n");
    }
}
