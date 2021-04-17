/*
  * RFxSerial.c:
 */

#include <unistd.h>  // UNIX standard function definitions
#include <stdlib.h>
#include <stdio.h>   // Standard input/output definitions
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>  // String function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>
#include <wiringSerial.h>
#include <getopt.h>

#define OUR_INPUT_FIFO_NAME "/tmp/RFxSerial.input"

int our_input_fifo_filestream = -1;
char fifo_buffer[2048];
int fifo_length = 0;
char rx_buf[512] = {'.','/','j','e','e','b','a','s','h','.','s','h',' ','"'};
int rx_len = 0;
int rx_other = 0;
int result;
char peekNode1 = '0';
char peekNode2 = '0';

void openPipe() {
// http://www.raspberry-projects.com/pi/programming-in-c/pipes/named-pipes-fifos

	result = mkfifo(OUR_INPUT_FIFO_NAME, 0777);		
  //(This will fail if the fifo already exists in the system from the app 
  // previously running, this is fine)
	if (result == 0) {
		printf("New input pipe created: %s\n", OUR_INPUT_FIFO_NAME);
	}
	our_input_fifo_filestream = open(OUR_INPUT_FIFO_NAME, (O_RDONLY | O_NDELAY));
					//Possible flags:
					//	O_RDONLY - Open for reading only.
					//	O_WRONLY - Open for writing only.
					//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
					//					if there is no input immediately available (instead of blocking). Likewise, write requests can also return
					//					immediately with a failure status if the output can't be written immediately.
	if (our_input_fifo_filestream == -1) {
		printf("Open failed FIFO: %i\n", our_input_fifo_filestream);
		}
}

main (int argc, char **argv) {
  
  
fprintf (stderr, "Start Up\n") ;

  char *cvalue = NULL;
  getopt (argc, argv, "p:");
  cvalue = optarg;

  int fd ;
  struct termios options;  // Terminal options

  time_t rawtime;
  struct tm * timeinfo;
  
// http://wiringpi.com/reference/serial-library/
  if ((fd = serialOpen (cvalue, 115200)/*, O_RDWR | O_NOCTTY*/) < 0) {
    fprintf (stderr, "Unable to open serial device: %s\n", cvalue) ;
	exit(1);
  }  
  printf("Serial device: %s opened\n", cvalue);  

  sleep(2); //required to make flush work, for some reason
  tcflush(fd,TCIOFLUSH);
  
  fprintf (stderr, "Flushed\n") ;
  
  fcntl(fd, F_SETFL);            // Configure port reading
  tcgetattr(fd, &options);       // Get the current options for the port
  cfsetispeed(&options, B115200); // Set the baud rates to 115200
  cfsetospeed(&options, B115200);

	openPipe();

	fprintf (stderr, "Pipe Open\n") ;

    rx_len = 14;

	while ((result = serialGetchar(fd)) == 0 ) ;   
	rx_buf[rx_len++] = result;
	
    for (;;) {
        while ((result = serialGetchar(fd)) != -1 ) {
        	if ((result != 0)) {
        		if (result == 10) break;
            	if (result == 13) {
        			rx_buf[rx_len++] = 32;
            		result = '"';// Convert <cr> to string terminator 
            	}          
            	rx_buf[rx_len++] = result;
//printf("%i ", result);
            	delayMicroseconds(1000);
            }
        }
        
        rx_buf[rx_len++] = 0;	// Null string terminator
        
//printf("%i len=%i\n%s\n", result, rx_len, rx_buf + 14);
        
        if (rx_len > 16){
            
            if (   ((rx_buf[14] == 'O') && (rx_buf[15] == 'K'))	// OK
            	|| ((rx_buf[14] == 'R') && (rx_buf[15] == 'X'))	// RX
//            	|| ((rx_buf[14] == 'R') && (rx_buf[15] == 'F'))	// RFM69
//	      		|| ((rx_buf[14] == 'S') && (rx_buf[15] == 'X'))	// SX1276
//	      		|| ((rx_buf[14] == 'S') && (rx_buf[15] == 'y'))	// Sync
	      		|| ((rx_buf[14] == 'A') && (rx_buf[15] == 'c'))	// Ack
//	      		|| ((rx_buf[14] == 'E') && (rx_buf[15] == 'e'))	// Eeprom
	      		|| ((rx_buf[14] == 'T') && (rx_buf[15] == 'X'))	// TX
	      		|| ((rx_buf[14] == 'R') && (rx_buf[16] == 'I'))	// ReInit
//	      		|| ((rx_buf[14] == 'R') && (rx_buf[15] == 'e'))	// Reset
//	      		|| ((rx_buf[14] == '#') && (rx_buf[15] == ' '))	// # 
	      														)
			{
                int ret = system(rx_buf);
                if (ret) printf("System(1) Returned %i\n", ret);
                
                if (rx_buf[14] == 'O' && rx_buf[15] == 'K' && peekNode1 == rx_buf[17] && peekNode2 == rx_buf[18]) {
//                 	printf("Node detected\n");
               
					rx_buf[rx_len - 2] = '\0';  // Drop trailing double quote
                
                	time ( &rawtime );
                	timeinfo = localtime ( &rawtime );

                	printf("%02d/%02d/%04d %02d:%02d:%02d %s\n", timeinfo->tm_mday,       \
                	  timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,         \
                	  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,  \
                	  rx_buf + 14);
                }
                fflush (stdout);        
            } else {
                /*if ((rx_other++) >= 39){
                    int ret = system("tail /tmp/RFxConsole.txt -n200 > /tmp/RFxTrim");
                    if (ret)
                      printf("System(2)Returned %i\n", ret);
                    fflush (stdout);        
                    ret = system("cp /tmp/RFxTrim /tmp/RFxConsole.txt"); // trim log
                    if (ret)
                      printf("System(3)Returned %i\n", ret);
                    fflush (stdout);
                    rx_other = 0;        
                } */
                
                rx_buf[rx_len - 2] = '\0';  // Drop trailing double quote
                
                time ( &rawtime );
                timeinfo = localtime ( &rawtime );

                printf("%02d/%02d/%04d %02d:%02d:%02d %s\n", timeinfo->tm_mday,       \
                  timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,         \
                  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,  \
                  rx_buf + 14);
                fflush (stdout);        
            }
        } 
        rx_len = 14;
        /*
        
        It is possible to send commands through the serial port to
        the attached JeeLink etc to adjust the configuration or post
        semaphores and the like using:
        echo "o" > /tmp/RFxSerial.input
        to display the current frequency offset or 
        echo "212,17,150p" > /tmp/RFxSerial.input
        to post the value 150 for node 17 in group 212, the value to be
        delivered the next time that node 17 requests a ACK for a packet.
        
        The commands so delivered may be delayed before being issued since
        the fifo queue is only checked when the serial queue times out. This
        timeout is 10 seconds after delivery of the last serial character.
        
        */
        
		    // Read up to 256 characters from the FIFO if they are there
		    if (our_input_fifo_filestream != -1)  {		    
		        fifo_length = read(our_input_fifo_filestream, (void*)fifo_buffer, 256);
		        if (fifo_length > 0) {
		        	if (fifo_buffer[fifo_length - 1] == 10) { 		
						fifo_buffer[fifo_length - 1] = '\0';// Loose <LF>, terminate the char
					} else {
						fifo_buffer[fifo_length] = '\0';  // Terminate the char	string				
					}
                	time ( &rawtime );
                	timeinfo = localtime ( &rawtime );
// deliver the command to the Jeenode                	
                	for (int c = 0; c < fifo_length; c++) {
                		serialPutchar (fd, fifo_buffer[c]);
                		delayMicroseconds(10000);                	
                	}
// Test for a command to this code              
                	if (fifo_buffer[0] == 37) {
                		if (fifo_length > 3) {
//                			printf("Command input detected\n");
                			peekNode1 = fifo_buffer[1];
                			peekNode2 = fifo_buffer[2];
               		 	} else printf("%02d/%02d/%04d %02d:%02d:%02d FIFO bytes=%i buffer=%s\n", \
								timeinfo->tm_mday,timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,\
                  				timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,  \
                  				fifo_length, fifo_buffer);                 		 	
               		 }
               		 
// Display buffer contents

                	printf("%02d/%02d/%04d %02d:%02d:%02d FIFO bytes=%i buffer=%s\n", \
						timeinfo->tm_mday,timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,\
                  		timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,  \
                  		fifo_length, fifo_buffer);   

                		fifo_buffer[0] = '\0';  		// Mark buffer empty
                } else if (fifo_length == 0) delayMicroseconds(10000);
                else printf("FIFO Read error %i\n", errno);
                
                fflush (stdout);
                
        	} else {
        		delayMicroseconds(1000);
			}
		}
}

