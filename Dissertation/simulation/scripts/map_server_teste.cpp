#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <iostream>
#include <string.h>

#define PORT 3128

using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(){

    int sockfd[2], newsockfd[2], portno;
     socklen_t clilen;
     char buffer[256];
     struct sockaddr_in serv_addr[2], cli_addr;
     int n;
     

     // create a socket
     // socket(int domain, int type, int protocol)
     for (int x = 0; x < 2; x++) {
        sockfd[x] =  socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) 
            error("ERROR opening socket");
     }
     
     // clear address structure
     bzero((char *) &serv_addr, sizeof(serv_addr));

     /* setup the host_addr structure for use in bind call */
     // server byte order
     for (int x = 0; x < 2; x++) {
        serv_addr[x].sin_family = AF_INET;  

        // automatically be filled with current host's IP address
        serv_addr[x].sin_addr.s_addr = INADDR_ANY;  

        // convert short integer value for port must be converted into network byte order
        serv_addr[x].sin_port = htons(PORT);
     }

     // bind(int fd, struct sockaddr *local_addr, socklen_t addr_length)
     // bind() passes file descriptor, the address structure, 
     // and the length of the address structure
     // This bind() call will bind  the socket to the current IP address on port, portno
     for (int x = 0; x < 2; x++) {
        if (bind(sockfd[x], (struct sockaddr *) &serv_addr[x],
                sizeof(serv_addr[x])) < 0) 
                error("ERROR on binding");
     }
     // This listen() call tells the socket to listen to the incoming connections.
     // The listen() function places all incoming connection into a backlog queue
     // until accept() call accepts the connection.
     // Here, we set the maximum size for the backlog queue to 5.
     for (int x = 0; x < 2; x++) {
        listen(sockfd[x],5);
     }
     

     // The accept() call actually accepts an incoming connection
     clilen = sizeof(cli_addr);

     // This accept() function will write the connecting client's address info 
     // into the the address structure and the size of that structure is clilen.
     // The accept() returns a new socket file descriptor for the accepted connection.
     // So, the original socket file descriptor can continue to be used 
     // for accepting new connections while the new socker file descriptor is used for
     // communicating with the connected client.
     for (int x = 0; x < 2; x++) {
        newsockfd[x] = accept(sockfd[x], 
                    (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0) 
            error("ERROR on accept");
     }

     printf("server: got connection from %s port %d\n",
            inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));

     // This send() function sends the 13 bytes of the string to the new socket
     bzero(buffer,256);
     cout << "TESTE DO COMPARE: " << strcmp(buffer,"QUIT\n") << endl; 
     while(strcmp(buffer,"QUIT\n") != 0){
        for (int x = 0; x < 2; x++) {
            send(newsockfd[x], "Hello, world!\n", 13, 0);

            bzero(buffer,256);

            n = recv(newsockfd[x],buffer,255,0);
            if (n < 0) error("ERROR reading from socket");
            printf("Here is the message: %s\n",buffer);
            cout << "TESTE DO COMPARE: " << strcmp(buffer,"QUIT\n") << endl;
            cout << buffer;
        }
     }

    for (int x = 0; x < 2; x++) {
        close(newsockfd[x]);
        close(sockfd[x]);
    }
     return 0; 
}