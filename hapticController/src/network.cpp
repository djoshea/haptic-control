#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>

#include <ctime>
#include <sstream>
#include <iostream>

#include "haptic.h"
#include "environment.h"
#include "utils.h"
#include "network.h"
#include "hapticController.h"

//////////// CONSTANTS ////

//#define LOOP_INTERVAL .001  // loop time in secs
#define LOOP_INTERVAL .0005  // loop time in secs (modified 2015-07-28 from 0.001)

#define SERVER_IP "192.168.1.255"  //localhost

//////////// DECLARED VARIABLES ////

extern ChaiData chai;
extern HapticState haptic;

// UDP ports
const int PORT_IN = 25000;
const int PORT_OUT = 23000;

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
    if(!networkUp) {
        networkUp = true;
        // start network send thread ********************************************
        networkThread = new cThread();
        networkThread->set(networkUpdate, CHAI_THREAD_PRIORITY_GRAPHICS);
    }
}

int networkInit() {
    // setup incoming socket ***********************************************
    if ((sock_in=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
        printf("Network: socket error\n");
        return -1;
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT_IN);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock_in,(struct sockaddr*) &si_me, sizeof(si_me))==-1) {
        printf("Network: bind socket in error\n");
        return -1;
    }

    // setup outgoing socket ***********************************************
    if ((sock_out=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
        printf("Network: bind socket out error\n");
        return -1;
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_OUT);
    if (inet_aton(SERVER_IP, &si_other.sin_addr)==0){
        printf("inet_aton() failed \n");
        return -1;
    }

    int broadcastPermission = 1;
    if (setsockopt(sock_out, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof(broadcastPermission)) < 0){
        printf("setsockopt error\n");
        return -1;
    }

    return 0;
}

// send position update
void networkUpdate(void) {
    printf("Network: starting\n");
    int ret = networkInit();
    if (ret != 0) {
        printf("Network: error during init\n");
        shutdown();
        networkUp = false;
        return;
    }

    double lastTime; // last time we sent, local time
    double lastHapticTimeUpdateSent = -100000; // haptic time of the last update we sent
    double currentHapticTime;
    bool isNewUpdate = false;
    double_t x = 0, y = 0, z = 0;
    double_t vx = 0, vy = 0, vz = 0;
    double_t fx = 0, fy = 0, fz = 0;
    double_t rfx = 0, rfy = 0, rfz = 0;
    uint8_t isTouching = false;
    uint8_t hitObstacle = 0, hitTarget = 0, atEdge = 0, onPlane = 0, hitConstraint = 0;

    if(!waitForHapticsReady()) {
        printf("Network: aborting\n");
        networkUp = false;
        return;
    }

    chai.simClockNetwork.reset();
    chai.simClockNetwork.start(true);

    LOG("in network loop\n");
    while(!chai.simulationFinished)
    {
        currentHapticTime = haptic.timeLastUpdate;
        isNewUpdate = currentHapticTime > lastHapticTimeUpdateSent;

        // send haptic state every LOOP_INTERVAL via UDP if there is a new update
        //if ((chai.simClockNetwork.getCurrentTimeSeconds() - lastTime) > LOOP_INTERVAL) {
        if(isNewUpdate) {
            lastTime = chai.simClockNetwork.getCurrentTimeSeconds();
            lastHapticTimeUpdateSent = currentHapticTime;

            // read position of device in rig coordinates
            x = (double_t)(haptic.posRig.x);
            y = (double_t)(haptic.posRig.y);
            z = (double_t)(haptic.posRig.z);

            // read velocity of device in rig coordinates
            vx = (double_t)(haptic.velRig.x);
            vy = (double_t)(haptic.velRig.y);
            vz = (double_t)(haptic.velRig.z);

            // read haptic perturbation force
            fx = (double_t)(haptic.totalForce.x);
            fy = (double_t)(haptic.totalForce.y);
            fz = (double_t)(haptic.totalForce.z);

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

            // try sleeping a bit
            sleepMs(LOOP_INTERVAL);

        }
    }

    printf("Network: stopping\n");
    close(sock_in);
    close(sock_out);

    networkUp = false;
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


