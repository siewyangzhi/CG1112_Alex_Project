
// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

//WASD ch input 
//#include <ncurses.h>

// Tells us that the network is running.
static volatile int networkActive=0;

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;
		
		case RESP_COLOUR:
		if(buffer[2] == 0){
			printf("Color Detected is Red");
		}
		else if (buffer[2] == 1){
			printf("Color Detected is Green");
		}
		else {
			printf("Color Detected is Blue");
		}
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, t=stop, c=clear stats, g=get stats q=exit)\n");
		break;
		
		case RESP_ULTRA:
		printf("Front Obstacle Detected. Distance: %d cm \n",buffer[2]);
		break;

		case RESP_ULTRA_BACK:
		printf("Back Obstacle Detected. Distance: %d cm \n",buffer[2]);
		break;
		
		case RESP_SYS_UP:
		printf("Connection between RPi and Arduino successfully established"); 

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Right Forward Ticks:\t\t%d\n", data[1]);
	printf("Left Reverse Ticks:\t\t%d\n", data[2]);
	printf("Right Reverse Ticks:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;

	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		/* TODO: Insert SSL write here to write buffer to network */
	c = sslWrite(conn,buffer,len);

		/* END TODO */	
		networkActive = (c > 0);
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		/* TODO: Insert SSL read here into buffer */
		len=sslRead(conn,buffer,128);

        //	printf("read %d bytes from server.\n", len);
		
		/* END TODO */

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
    
    /* TODO: Stop the client loop and call EXIT_THREAD */

	stopClient();
	EXIT_THREAD(conn);
    /* END TODO */
}


void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &params[0], &params[1]);
	flushInput();
}

void getFixedParam(int32_t *params,char ch)
{
	switch (ch) 
	{

	// w for forward , s for reverse//
		case 'w':
		case 's':
		params[0] = 5;
		params[1] = 100;
		break;
	//a for left , d for right//
		case 'a':
		case 'd':
		params[0] = 30;
		params[1] = 100;
		break;

	//i for forward , k for reverse// Nitro time
		case 'u':
		case 'j':
		params[0] = 20;
		params[1] = 100;
		break;
	//j for left , l for right // Nitro time

		case 'h':
		case 'k':
		params[0] = 30;
		params[1] = 100;
		break;
	}
}

void *writerThread(void *conn)
{
	int quit=0;
	char ch;
	while(!quit)
	{
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, t=stop, c=clear stats, g=get stats q=exit)\n");
		scanf("%c", &ch);
		if (ch != ' '){
	// Purge extraneous characters from input stream
			flushInput();
			char buffer[10];
			int32_t params[2];
			buffer[0] = NET_COMMAND_PACKET;
			switch(ch)
			{
				case 'f':
				case 'F':
				case 'b':
				case 'B':
				case 'l':
				case 'L':
				case 'r':
				case 'R':
		
					getParams(params);
							buffer[1] = ch;
							memcpy(&buffer[2], params, sizeof(params));
							sendData(conn, buffer, sizeof(buffer));
							break;

				case 't':
				case 'T':
				case 'c':
				case 'C':
				case 'g':
				case 'G':
				case 'Z':
				case 'z':
		
						params[0]=0;
						params[1]=0;
						memcpy(&buffer[2], params, sizeof(params));
						buffer[1] = ch;
						sendData(conn, buffer, sizeof(buffer));
						break;
				

				case 'w':
				case 'W':
				case 'a':
				case 'A':
				case 'S':
				case 's':
				case 'd':
				case 'D':
				case 'k':
				case 'K':
				case 'u':
				case 'U':
				case 'j':
				case 'J':
				case 'h':
				case 'H':

						getFixedParam(params, ch);
						if (ch == 'w' || ch == 'u')
							buffer[1] = 'f';
						else if (ch == 'a'|| ch == 'h')
							buffer[1] = 'l';
						else if (ch == 's'|| ch == 'j')
							buffer[1] = 'b';
						else if (ch == 'd' || ch == 'k')
							buffer[1] = 'r';

						memcpy(&buffer[2], params, sizeof(params));
						sendData(conn, buffer, sizeof(buffer));

						break;

				case 'q':
				case 'Q':
					quit=1;
					break;
				default:
					printf("BAD COMMAND\n");
			}
			
		}		
	}
	printf("Exiting keyboard thread\n");

    /* TODO: Stop the client loop and call EXIT_THREAD */
	stopClient();
	EXIT_THREAD(conn);
    /* END TODO */
}

/* TODO: #define filenames for the client private key, certificatea,
   CA filename, etc. that you need to create a client */

#define ClientPVT "laptop.key"
#define ClientCrt "laptop.crt"
#define CA "signing.pem"
#define Server "alex.epp.com"

/* END TODO */
void connectToServer(const char *serverName, int portNum)
{
    /* TODO: Create a new client */
	createClient ("192.168.43.66",5000,1,CA,Server,1,ClientCrt,ClientPVT,readerThread,writerThread);
    /* END TODO */
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    /* TODO: Add in while loop to prevent main from exiting while the
    client loop is running */
	while(client_is_running());


    /* END TODO */
	printf("\nMAIN exiting\n\n");
}
