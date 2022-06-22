// run: $ g++ main.cpp -o main -std=c++11 -pthread; ./main

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
#include <byteswap.h>

#include "controller.cpp"

//const char* other_ip = "127.0.0.1";
const char* IP_ROBOT = "192.168.1.101";
const char* IP_THIS = "192.168.1.103";

const int PORT_READ = 54003;
const int PORT_SEND = 54005;

const int LEN_READ = 24; //number of floats read from labview
const int LEN_SEND = 7; //number of floats sent to labview

const int buffersize = 1024; //buffer for reading udp


using namespace std;


//global var of data coming from labview
float from_robo[LEN_READ];

//thread protection for from_robo since one thread writes and other reads
mutex from_robo_mutex;

unsigned char checksum(unsigned char *ptr, int num_chars){
	unsigned char chk = 0;
	for(int i = 0; i < num_chars; i++){
		chk = chk - ptr[i];
	}
	return chk;
}


//udp runs in a separate thread because udp recieve will wait until it gets something
int udp_listen(){

	unsigned char client_message[buffersize];

	int socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	printf("created server socket\n");
	
	struct sockaddr_in server_addr, client_addr;
	socklen_t client_addr_length = sizeof(client_addr);
	memset(&server_addr, 0, sizeof(server_addr)); 
	memset(&client_addr, 0, sizeof(client_addr)); 
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT_READ);
	server_addr.sin_addr.s_addr = inet_addr(IP_THIS);
	
	bind(socket_desc, (struct sockaddr*)&server_addr, sizeof(server_addr));
	printf("bound to port %i \n", ntohs(server_addr.sin_port));
	
	while(1){
	
		//fill recieving buffer with NULL 
		memset(client_message, '\0', buffersize);

		//wait for recieve and store c string in client_message. program won't move on until it recieves data.
		recvfrom(socket_desc, client_message, sizeof(client_message), 0, (struct sockaddr*)&client_addr, &client_addr_length);

		// printf("client_message:");
		// for(int i=0; i<(LEN_READ+1)*4; i++){
		// 	printf(" %x", client_message[i]);
		// }
		// printf("\n");


		unsigned char sum = checksum(client_message, (LEN_READ*4)+1);
        sum = 0;
		if(sum == 0){

			printf("from_robo:");
			for(int i=0; i<LEN_READ; i++){
				printf(" %f", from_robo[i]);
			} 
			printf("\n");

			// Mutex makes sure no one reads client_arr when this is writing
			lock_guard<mutex> guard(from_robo_mutex);
			memcpy(&from_robo, &client_message, sizeof(from_robo));
		}
		else{
			printf("checksum detected errors, sum = %x \n", sum);
		}

	}
	
	close(socket_desc);
	printf("closed socket\n");
	
	return 0;
}


//runs in the main thread
vector<float> controller(vector<float> input){
	vector<float> output;
	for(int i=0; i < input.size(); i++){
		output.push_back(input.at(i));
	}
	return output;
}


int main(){
	cout.precision(10);

	//start read socket on 54003
	thread udp_thread(udp_listen);
	
	//set up send socket on 54005
	struct sockaddr_in s_other_send;
	int s_send;
	socklen_t slen_send=sizeof(s_other_send);
	s_send = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset((char *) &s_other_send, 0, sizeof(s_other_send));
	s_other_send.sin_family = AF_INET;
	s_other_send.sin_port = htons(PORT_SEND);
	inet_aton(IP_ROBOT, &s_other_send.sin_addr);

	// **** INPUT: [ motor_torques[4], tau_wheel[1], tau_hips[2], animations[16] ] **//
	float to_robo[LEN_SEND] = {1, 2.7, 3.14, 2, 3, 4, 5};

	while(1){



		this_thread::sleep_for(chrono::microseconds(225));
	
		//Sending recieve back 
		// vector<float> input = client_arr;

	
		//Controller Call (important)
		update(from_robo, to_robo, 0);

        // cout << "3: " << from_robo[3] << endl;
        // cout << "4: " << from_robo[4] << endl;

        


		//Copy data from float array to a char array, each float takes 4 bytes. Last byte is a checksum
		unsigned char send_char[LEN_SEND*4 + 1];
		memcpy(send_char, to_robo, LEN_SEND*4);
		send_char[LEN_SEND*4] = checksum(send_char, LEN_SEND*4);
		

		//Send char array over socket 
		sendto(s_send, send_char, sizeof(send_char), 0, (struct sockaddr*) &s_other_send, slen_send);


		//Debugging prints
		// printf("send_char:");
		// for(int i=0; i<sizeof(send_char); i++){
		// 	printf(" %x", send_char[i]);
		// } 
		// printf("\n");

		// printf("checksum: %x \n", checksum(send_char, LEN_SEND*4 + 1));
		//printf("%x, %x, %x, %x\r\n",send_char[8],send_char[9],send_char[10],send_char[11]);
		//printf("%d\r\n", sizeof(send_char));

	}	
	
	udp_thread.join();
	return 0;
}









