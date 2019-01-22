#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <unistd.h> // close()

#include <ctime>
#include <sstream>
#include <iostream>

#include "haptic.h"
#include "environment.h"
#include "utils.h"
#include "network.h"

//////////// CONSTANTS ////

#define LOOP_INTERVAL .001  // loop time in secs

//#define SERVER_IP "192.168.1.255"  //localhost
#define SERVER_IP "192.168.20.255"  //localhost rig42 IP

//////////// DECLARED VARIABLES ////

extern ChaiData chai;
extern HapticState haptic;

// UDP ports
const int PORT_IN = 28000;
const int PORT_OUT = 10000;  // rig42, not sure if this matters, emt 4/6/2016

// init UDP buffer packet
static uint8_t rawBinPacketIn[MAX_PACKET_LENGTH];
static uint8_t rawBinPacketOut[MAX_PACKET_LENGTH];

// Setup Socket Variables
struct sockaddr_in si_me, si_other;
int slen_me = sizeof(si_me);
int slen_other = sizeof(si_other);

// declare UDP sockets
int sock_in, sock_out;

bool networkUp = false;

cThread* networkThread;

bool networkIsUp() {
    return networkUp;
}

void networkStart() {
    printf("Network: starting\n");
    // setup incoming socket ***********************************************
    if ((sock_in=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
        diep("socket\n");
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT_IN);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    int bind_sock_in = bind(sock_in,(struct sockaddr*) &si_me, sizeof(si_me));
    if (bind_sock_in ==-1)
        diep("bind sock_in\n");

    // setup outgoing socket ***********************************************
    
    sock_out=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_out==-1) {
        diep("socket out\n");
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_OUT);
    if (inet_aton(SERVER_IP, &si_other.sin_addr)==0){
        fprintf(stderr, "inet_aton() failed \n");
        exit(1);
    }

    int broadcastPermission = 1;
    if (setsockopt(sock_out, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof(broadcastPermission)) < 0){
        fprintf(stderr, "setsockopt error");
        exit(1);
    }

    // start network send thread ********************************************
    networkUp = true;
    networkThread = new cThread();
    networkThread->start(networkUpdate, CTHREAD_PRIORITY_HAPTICS);
}

void networkClose() {
    if(networkUp) {
        printf("Network: stopping\n");
        close(sock_in);
        close(sock_out);
    }
    networkUp = false;
}

// send position update
void networkUpdate(void) {
    double lastTime;
    double_t x = 0, y = 0, z = 0;
    double_t vx = 0, vy = 0, vz = 0;
    double_t fx = 0, fy = 0, fz = 0;
    double_t rfx = 0, rfy = 0, rfz = 0;
    uint8_t isTouching = false;
    uint8_t hitObstacle = 0, hitTarget = 0, atEdge = 0, onPlane = 0, hitConstraint = 0;

    chai.simClockNetwork.reset();
    chai.simClockNetwork.start(true);

    while(chai.simulationRunning)
    {
        // send haptic state every LOOP_INTERVAL via UDP
        if ((chai.simClockNetwork.getCurrentTimeSeconds() - lastTime) > LOOP_INTERVAL) {
            lastTime = chai.simClockNetwork.getCurrentTimeSeconds();

            // read position of device in rig coordinates
            x = (double_t)(haptic.posRig.x());
            y = (double_t)(haptic.posRig.y());
            z = (double_t)(haptic.posRig.z());

            // read velocity of device in rig coordinates
            vx = (double_t)(haptic.velRig.x());
            vy = (double_t)(haptic.velRig.y());
            vz = (double_t)(haptic.velRig.z());

            // read haptic perturbation force
            fx = (double_t)(haptic.totalForce.x());
            fy = (double_t)(haptic.totalForce.y());
            fz = (double_t)(haptic.totalForce.z());

            hitObstacle = (uint8_t)(haptic.hitObstacle);
            hitTarget = (uint8_t)(haptic.hitTarget);
            atEdge = (uint8_t)(haptic.atWorkspaceEdge);
            onPlane = (uint8_t)(haptic.onScreenPlane);
            hitConstraint = (uint8_t)(haptic.hitConstraint);

            //add header info
            uint8_t packet[MAX_PACKET_LENGTH] = HAPTIC_PACKET_PREFIX;
            uint16_t lengthPrefix = strlen(HAPTIC_PACKET_PREFIX);
            uint8_t* pBuf = packet + lengthPrefix;

            memcpy(pBuf, &x, sizeof(x));
            pBuf += sizeof(x);
            memcpy(pBuf, &y, sizeof(y));
            pBuf += sizeof(y);
            memcpy(pBuf, &z, sizeof(z));
            pBuf += sizeof(z);

            memcpy(pBuf, &vx, sizeof(vx));
            pBuf += sizeof(vx);
            memcpy(pBuf, &vy, sizeof(vy));
            pBuf += sizeof(vy);
            memcpy(pBuf, &vz, sizeof(vz));
            pBuf += sizeof(vz);

            memcpy(pBuf, &fx, sizeof(fx));
            pBuf += sizeof(fx);
            memcpy(pBuf, &fy, sizeof(fy));
            pBuf += sizeof(fy);
            memcpy(pBuf, &fz, sizeof(fz));
            pBuf += sizeof(fz);

            memcpy(pBuf, &hitObstacle, sizeof(hitObstacle));
            pBuf += sizeof(hitObstacle);

            memcpy(pBuf, &hitTarget, sizeof(hitTarget));
            pBuf += sizeof(hitTarget);

            memcpy(pBuf, &atEdge, sizeof(atEdge));
            pBuf += sizeof(atEdge);

            memcpy(pBuf, &onPlane, sizeof(onPlane));
            pBuf += sizeof(onPlane);

            memcpy(pBuf, &hitConstraint, sizeof(hitConstraint));
            pBuf += sizeof(hitConstraint);

            uint16_t lengthPacket = pBuf - packet;
            if (sendto(sock_out, packet, lengthPacket, 0, (struct sockaddr*) &si_other, slen_other)==-1){
//                diep("sendto failed\n");
                std::cout << "UDP send error \n ";
            }
        }
    }

    networkClose();
}

int networkReceive(uint8_t * buffer, int bufferLength)
{
    uint8_t rawPacket[MAX_PACKET_LENGTH];
    uint8_t* pBuf = rawPacket;

    // check # bytes in UDP buffer and pull into buffer if available
    int value = 0, bytesRead = 0;
    ioctl(sock_in, FIONREAD, &value);
    if (value > 0) {
        bytesRead = recvfrom(sock_in, rawPacket, MAX_PACKET_LENGTH, 0,
            (struct sockaddr*) &si_me, (socklen_t *)&slen_me);
    }

//    printf("bl %d \n", bytesRead);
    if(bytesRead == 0) {
        return 0;
    }

    int headerLength = 4;
    if(bytesRead < headerLength) {
        fprintf(stderr, "Network: packet too short\n");
        return -1;
    }

    uint16_t packetLength = 0;
    uint16_t checksum = 0;

    // store the length
    STORE_UINT16(pBuf, packetLength);

    if(packetLength > bytesRead - headerLength) {
    	fprintf(stderr, "Network: Invalid packet length!\n");
    	return -1;
    }
    if(packetLength > bufferLength) {
        fprintf(stderr, "Network: Packet length exceeds buffer size\n");
        return -1;
    }

    // store the checksum
    STORE_UINT16(pBuf, checksum);

    // copy the raw data into the data buffer
    STORE_UINT8_ARRAY(pBuf, buffer, packetLength);

    // validate the checksum: sum(bytes as uint8) modulo 2^16
    uint32_t accum = 0;
    for(int i = 0; i < packetLength; i++)
    	accum += buffer[i];
    accum = accum % 65536;

    // return true if checksum valid
    if(accum == checksum)
        return packetLength;
    else {
        fprintf(stderr, "Network: Invalid packet checksum!\n");
        return -1;
    }
}
