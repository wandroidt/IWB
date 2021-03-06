#include <cstdlib>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include "enet/enet.h"

#define PORT  7000

using namespace std;
using namespace std::chrono;

long data[2] = {-1, -1};

int main(int argc, char ** argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    if (enet_initialize() != 0) {
        fprintf(stderr, "An error occurred while initializing ENet.\n");
        return EXIT_FAILURE;
    }

    ENetAddress address;
    ENetHost * server;
    /* Bind the server to the default localhost.     */
    address.host = ENET_HOST_ANY;
    address.port = PORT;
    printf("Creating server on %s:%d\n", address.host, address.port);
    server = enet_host_create(& address /* the address to bind the server host to */,
            32 /* allow up to 32 clients and/or outgoing connections */,
            2 /* allow up to 2 channels to be used, 0 and 1 */,
            0 /* assume any amount of incoming bandwidth */,
            0 /* assume any amount of outgoing bandwidth */);
    if (server == NULL) {
        fprintf(stderr,
                "An error occurred while trying to create an ENet server host.\n");
        exit(EXIT_FAILURE);
    }

    printf("Listening. Press ESC to quit.\n");

    ENetEvent event;
    while (1) {
        // WIP: Clock for buffer.
        auto now = high_resolution_clock::now();
        auto nanos = duration_cast<nanoseconds>(now.time_since_epoch()).count();
        // WIP: Code that utilizes clock.
        if (enet_host_service(server, & event, 100) > 0) {
            switch (event.type) {
                case ENET_EVENT_TYPE_CONNECT:
                    printf("A new client connected from %x:%u.\n",
                            event.peer -> address.host,
                            event.peer -> address.port);
                    /* Store any relevant client information here. */
                    //event.peer -> data = "Client information";
                    break;
                case ENET_EVENT_TYPE_RECEIVE:
                    printf("A packet of length %u containing %s was received from %s on channel %u.\n",
                            event.packet -> dataLength,
                            event.packet -> data,
                            event.peer -> data,
                            event.channelID);
                    /* Clean up the packet now that we're done using it. */
                    enet_packet_destroy(event.packet);
                    break;
                case ENET_EVENT_TYPE_DISCONNECT:
                    printf("%s disconnected.\n", event.peer -> data);
                    /* Reset the peer's client information. */
                    event.peer -> data = NULL;
            }
        }
        if (GetAsyncKeyState(VK_ESCAPE)) break;
        if (GetAsyncKeyState(VK_INSERT)) {
            // WIP: Grab the Mouse (For Testing Only).
            POINT p;
            if (GetCursorPos(&p)) {
                data[0] = p.x;
                data[1] = p.y;
            }
            ENetPacket * packet = enet_packet_create(data,
                    sizeof (data) + 1,
                    0);
            /* Extend the packet so and append the string "foo", so it now */
            /* contains "packetfoo\0"                                      */
            //enet_packet_resize(packet, strlen("packetfoo") + 1);
            //strcpy(& packet -> data [strlen("packet")], "foo");
            /* Send the packet to the peer over channel id 0. */
            /* One could also broadcast the packet by         */
            /* enet_host_broadcast (host, 0, packet);         */
            enet_host_broadcast(server, 0, packet);
        } else {
            data[0] = -1;
            data[1] = -1;
            ENetPacket * packet = enet_packet_create(data,
                    sizeof (data) + 1,
                    0);
            enet_host_broadcast(server, 0, packet);
        }
    }

    enet_host_destroy(server);
    enet_deinitialize();

    return 0;
}
