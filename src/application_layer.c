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
    int res = llopen(ll);
    if (res == 0)
    {
        printf("llopen succeeded\n");
        // For this quick test, immediately close the link
        if (llclose() == 0)
            printf("llclose succeeded\n");
        else
            printf("llclose failed\n");
    }
    else
    {
        printf("llopen failed\n");
    }
}
