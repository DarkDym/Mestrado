#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>

#define PORT 3128

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer_in[256];
    char buffer_out[256];
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    serv_addr.sin_addr.s_addr = htons(INADDR_ANY);

    // bcopy((char *)server->h_addr, 
    //      (char *)&serv_addr.sin_addr.s_addr,
    //      server->h_length);
    
    serv_addr.sin_port = htons(strtol(argv[1],NULL,16));
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    do{
        printf("Please enter the message: ");
        bzero(buffer_out,256);
        fgets(buffer_out,255,stdin);
        n = send(sockfd, buffer_out, strlen(buffer_out), 0);
        if (n < 0) 
            error("ERROR writing to socket");
        bzero(buffer_in,256);
        n = recv(sockfd, buffer_in, 255, 0);
        if (n < 0) 
            error("ERROR reading from socket");
        printf("%s\n", buffer_in);
    }while(strcmp(buffer_out,"QUIT\n") != 0);
    
    close(sockfd);
    return 0;
}