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
#include <wiringSerial.h>

#define OUR_INPUT_FIFO_NAME "/tmp/RFxSerial.input"

int our_input_fifo_filestream = -1;
char fifo_buffer[256];
int fifo_length = 0;
char rx_buf[1024] = {'.','/','j','e','e','b','a','s','h','.','s','h',' ','"'};
int rx_len = 0;
int result;

void openPipe() {
// http://www.raspberry-projects.com/pi/programming-in-c/pipes/named-pipes-fifos

	result = mkfifo(OUR_INPUT_FIFO_NAME, 0777);		
  //(This will fail if the fifo already exists in the system from the app 
  // previously running, this is fine)
	if (result == 0)
		printf("New input pipe created: %s\n", OUR_INPUT_FIFO_NAME);

	our_input_fifo_filestream = open(OUR_INPUT_FIFO_NAME, (O_RDONLY | O_NONBLOCK));
					//Possible flags:
					//	O_RDONLY - Open for reading only.
					//	O_WRONLY - Open for writing only.
					//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
					//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
					//											immediately with a failure status if the output can't be written immediately.
	if (our_input_fifo_filestream == -1)
		printf("Open failed FIFO: %i\n", our_input_fifo_filestream);
}

int main () {
  int fd ;
  struct termios options;  // Terminal options
  
// http://wiringpi.com/reference/serial-library/
  if ((fd = serialOpen ("/dev/ttyUSB0", 57600)/*, O_RDWR | O_NOCTTY*/) < 0)
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;


  fcntl(fd, F_SETFL);            // Configure port reading
  tcgetattr(fd, &options);       // Get the current options for the port
  cfsetispeed(&options, B57600); // Set the baud rates to 57600
  cfsetospeed(&options, B57600);

   openPipe();
   
    rx_len = 14;
    for (;;) {
        while ((result = serialGetchar(fd)) != -1) {
            if (result == 13) result = '"';// Convert <cr> to string terminator           
            if (result == 10) break;
            rx_buf[rx_len++] = result;
        }
        if (rx_len > 14){
            rx_buf[rx_len++] = '\0';
            if ((rx_buf[14] == 'O') && (rx_buf[15] == 'K')) {
                int ret = system(rx_buf);
                printf("System Returned %i\n", ret);
            } else {
                rx_buf[rx_len - 2] = '\0';  // Drop trailing double quote
                printf("%s\n", rx_buf + 14);
            }
            rx_len = 14;
        }
        /*
        
        It is possible to send commands through the serial port to
        the attached JeeLink etc to adjust the configuration or post
        semaphores and the like using:
        echo "o" > /tmp/RFxSerial.input
        to display the current frequency offset or 
        echo "212,17,150p" > /tmp/RFxSerial.input
        to post the value 150 for node 17 in group 212 to value to be
        delivered the next time that node 17 requests a ACK with a packet.
        
        The commands so delivered may be delayed before being issued since
        the fifi queue is only checked when the serial queue times out. This
        timeout is 10 seconds after deliver of the last serial character.
        
        */
        
		    // Read up to 255 characters from the FIFO if they are there
		    if (our_input_fifo_filestream != -1)  {
		        fifo_length = read(our_input_fifo_filestream, (void*)fifo_buffer, 255);		
            //Filestream, buffer to store in, number of bytes to read (max)
		        if (fifo_length < 0) printf("FIFO Read error\r");
            else if (fifo_length > 1) {
				        // Bytes received
				        fifo_buffer[fifo_length - 1] = '\0';  // Terminate the char
                serialPrintf (fd, fifo_buffer); // Send string to RFxConsole
            }
			  }
		}
}
