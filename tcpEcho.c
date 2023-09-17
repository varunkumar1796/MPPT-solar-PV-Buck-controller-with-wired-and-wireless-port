/*
 * Copyright (c) 2014-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== tcpEcho.c ========
 *    Contains BSD sockets code.
 */

#include <string.h>
#include <stdint.h>

#include <pthread.h>
/* BSD support */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <ti/net/slnetutils.h>

#include <ti/display/Display.h>

#define TCPPACKETSIZE 256
#define NUMTCPWORKERS 3
#define IPADDR             0x0a7b2d01/* 10.123.45.1 */ //server1
char buffer[256] = {0};
double wload_v;
uint32_t load_v_wless;
double wload_v;
int ct10=0;

extern Display_Handle display;

extern void *TaskCreate(void (*pFun)(), char *Name, int Priority,
        uint32_t StackSize, uintptr_t Arg1, uintptr_t Arg2, uintptr_t Arg3);

/*
 *  ======== tcpWorker ========
 *  Task to handle TCP connection. Can be multiple Tasks running
 *  this function.
 */
void tcpWorker(uint32_t arg0, uint32_t arg1)
{

    int  clientfd = (int)arg0;
    int  bytesRcvd;
    int  bytesSent;
    char *hello = "hello from server",*str;


    Display_printf(display, 0, 0, "tcpWorker: start clientfd = 0x%x\n",
            clientfd);

   /* while ((bytesRcvd = recv(clientfd, buffer, TCPPACKETSIZE, 0)) > 0) {
        bytesSent = send(clientfd, buffer, bytesRcvd, 0);
        if (bytesSent < 0 || bytesSent != bytesRcvd) {
            Display_printf(display, 0, 0, "send failed.\n");
            break;
        }*/
    while(1){
                       memset(buffer, '\0', sizeof(buffer));
                       recv( clientfd , buffer, 256, 0);
                     //  Display_printf(display, 0, 0, "%s\n ADC MICROVOLTS \n",buffer );
                       load_v_wless = atol(buffer);
                       wload_v = (double)load_v_wless/1000000 ;
                       send(clientfd , hello , strlen(hello) , 0 );
                       ct10++;
                       //usleep(1);
                    //   Display_printf(display, 0, 0, "Hello message sent\n");
    }
    Display_printf(display, 0, 0, "tcpWorker stop clientfd = 0x%x\n", clientfd);

    //close(clientfd);
}

/*
 *  ======== tcpHandler ========
 *  Creates new Task to handle new TCP connections.
 */
void tcpHandler(uint32_t arg0, uint32_t arg1)
{
    void *thread = NULL;
    int                status;
    int                clientfd;
    int                server,valread;
    struct sockaddr_in localAddr;
    struct sockaddr_in clientAddr;
    int                optval;
    char                 buffer[256]={0};
    int                optlen = sizeof(optval);
    socklen_t          addrlen = sizeof(localAddr);
    char *hello = "Hello from server";

    Display_printf(display, 0, 0, "TCP Echo example started\n");

    server = socket(AF_INET, SOCK_STREAM, 0);
    if (server == -1) {
        Display_printf(display, 0, 0, "tcpHandler: socket failed\n");
        goto shutdown;
    }
    else
        Display_printf(display, 0, 0, "tcpHandler: socket created\n");

    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl((unsigned int)IPADDR);
    localAddr.sin_port = htons(arg0);

    Display_printf(display, 0, 0, "%d\n",arg0 );
    Display_printf(display, 0, 0, "%d\n",localAddr.sin_port );
    Display_printf(display, 0, 0, "%d\n",localAddr.sin_addr.s_addr );
    Display_printf(display, 0, 0, "%d\n",localAddr.sin_family);
    Display_printf(display, 0, 0, "%s\n",&localAddr);

    status = bind(server, (struct sockaddr *)&localAddr, sizeof(localAddr));
    if (status == -1) {
        Display_printf(display, 0, 0, "tcpHandler: bind failed\n");
        goto shutdown;
    }
    else
            Display_printf(display, 0, 0, "tcpHandler: Bind complete\n");

    status = listen(server, NUMTCPWORKERS);
    if (status == -1) {
        Display_printf(display, 0, 0, "tcpHandler: listen failed\n");
        goto shutdown;
    }
    else
            Display_printf(display, 0, 0, "tcpHandler: listen created\n");

    optval = 1;
   status = setsockopt(server, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen);
    if (status == -1) {
        Display_printf(display, 0, 0, "tcpHandler: setsockopt failed\n");
        goto shutdown;
    }
    else
            Display_printf(display, 0, 0, "tcpHandler: socket KEEPALIVe\n");
    /*
          -------------------------------------------------------------------

*/

    while ((clientfd =
            accept(server, (struct sockaddr *)&localAddr,&addrlen)) != -1) {

        Display_printf(display, 0, 0, "ACCEPTED\n");
        Display_printf(display, 0, 0,
                       "tcpHandler: Creating thread clientfd = %x\n", clientfd);

               thread = TaskCreate(tcpWorker, NULL, 3, 2048, (uintptr_t)clientfd,
                   0, 0);

               if (!thread) {
                   Display_printf(display, 0, 0,
                           "tcpHandler: Error - Failed to create new thread.\n");
                   close(clientfd);
               }
               else
                       Display_printf(display, 0, 0, "tcpHandler: Thread created\n");

               /* addrlen is a value-result param, must reset for next accept call*/
               addrlen = sizeof(localAddr);



    }
    Display_printf(display, 0, 0, "tcpHandler: accept failed.\n");

shutdown:
    if (server != -1) {
      //  Display_printf(display, 0, 0, "SOCKET CLOSED\n");
        close(server);
    }
}
