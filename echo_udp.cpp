#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <chrono>


//const char* other_ip = "127.0.0.1";
const char* IP_ROBOT = "192.168.1.101";
const char* IP_THIS = "192.168.1.103";

const int PORT = 54003;

const int LEN_READ = 1; //number of floats read from labview
const int LEN_SEND = 1; //number of floats sent to labview

const int buffersize = 1024; //buffer for reading udp


float from_robo[LEN_READ];

float time_check = 0;
struct timeval start;


using namespace std;


int main(){

	unsigned char client_message[buffersize];

	int socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	// int socket_desc = socket(AF_INET, SOCK_STREAM, 0);
	printf("created server socket\n");



	struct sockaddr_in server_addr, client_addr;
	socklen_t client_addr_length = sizeof(client_addr);
	memset(&server_addr, 0, sizeof(server_addr)); 
	memset(&client_addr, 0, sizeof(client_addr)); 
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT);
	server_addr.sin_addr.s_addr = inet_addr(IP_THIS);
	// server_addr.sin_addr.s_addr = INADDR_ANY;
	
	bind(socket_desc, (struct sockaddr*)&server_addr, sizeof(server_addr));
	printf("listener bound to port %i \n", ntohs(server_addr.sin_port));

    int count = 0;

    while(1){

		//fill recieving buffer with NULL 
		memset(client_message, '\0', buffersize);

		//wait for recieve and store c string in client_message. program won't move on until it recieves data.
		recvfrom(socket_desc, client_message, sizeof(client_message), 0, (struct sockaddr*)&client_addr, &client_addr_length);
        memcpy(&from_robo, &client_message, sizeof(from_robo));



        // if(from_robo[23] != time_check){
        //     struct timeval end;
        //     gettimeofday(&end, nullptr);

        //     int diff_us = (end.tv_sec - start.tv_sec)*1e6 + (end.tv_usec - start.tv_usec);
        //     printf("diff_us: %i %f \n", diff_us, time_check);

        //     gettimeofday(&start, nullptr);
        // }
        // time_check = from_robo[23];
    



        float to_robo[LEN_SEND] = {0};

        memcpy(to_robo, from_robo, LEN_SEND*4);


        unsigned char send_char[LEN_SEND*4];
		memcpy(send_char, to_robo, LEN_SEND*4);
		

		//Send char array over socket 
		sendto(socket_desc, send_char, sizeof(send_char), 0, (struct sockaddr*) &client_addr, client_addr_length);
	
    
        // if(count % 100 == 0){
        //     printf("from_robo:");
        //         for(int i=0; i<23; i++){
        //             printf(" %f", from_robo[i]);
        //         } 
        //     printf("\n");

        //     printf("to_robo:");
        //         for(int i=0; i<29; i++){
        //             printf(" %f", to_robo[i]);
        //         } 
        //     printf("\n");
        //     printf("\n");
        // }
        // count++;
    }
}