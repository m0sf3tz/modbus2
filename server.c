#include <modbus.h>

#include <string.h> 
#include <stdlib.h> 
#include <errno.h>  
#include <stdio.h>



int main() 
{
		//Prepare a Modbus mapping with 30 holding registers
	//(plus no output coil, one input coil and two input registers)
	//This will also automatically set the value of each register to 0
	modbus_mapping_t *mapping = modbus_mapping_new(0, 1, 30, 2);
	if (!mapping) {
			fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
			exit(1);
	}


	//Example: set register 12 to integer value 623
  mapping->tab_registers[0] = 69;
  mapping->tab_registers[1] = 8008;


	modbus_t *ctx = modbus_new_rtu("/dev/ttyS101", 9600, 'N', 8, 1);
	if (!ctx) {
			fprintf(stderr, "Failed to create the context: %s\n", modbus_strerror(errno));
			exit(1);
	}

	//Set the Modbus address of this slave (to 3)
	modbus_set_slave(ctx, 1);


	if (modbus_connect(ctx) == -1) {
			fprintf(stderr, "Unable to connect: %s\n", modbus_strerror(errno));
			modbus_free(ctx);
			exit(1);
	}


  modbus_set_debug(ctx, 1);

	uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];// request buffer
	int len;// length of the request/response

	while(1) {
			len = modbus_receive(ctx, req);
     	printf("got an indication! \n");
      
      if(len == 0){
        printf("got a message not to me!\n");
        continue;
      }

			len = modbus_reply(ctx, req, len, mapping);
			if (len == -1) break;
	}
	printf("Exit the loop: %s\n", modbus_strerror(errno));
}
