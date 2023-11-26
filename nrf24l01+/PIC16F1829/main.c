#include "main.h"

void main(void) {
    Device_SetUp();			
    Device_Initialize();
    Timer1_Setup();
    spi_init();
    __delay_ms(1000);
    UART_Debug("Device initialized.\r\n");
    StatusLED(LED_ALL);
    __delay_ms(500);
    StatusLED(LED_OFF);
    __delay_ms(500);
    StatusLED(LED_ONLINE);
    __delay_ms(500);
    StatusLED(LED_ALL);
    
	//nRF24 Configuration
    wide_band                   = true;
    p_variant                   = true; //was false
    payload_size                = 32;
    ack_payload_available       = true;
    dynamic_payloads_enabled    = true;
    
    //nRF24 Init and Start
    nrf_begin();
    nrf_enableDynamicPayloads();
    nrf_setAutoAck_all(1);
    nrf_setDataRate(RF24_250KBPS);
    nrf_setPALevel(RF24_PA_MIN); //Does not pin to output
    nrf_setChannel(70); //70
    nrf_setRetries(15,15);
    nrf_setCRCLength(RF24_CRC_8);
    
    //nRF24 Writing address
    uint8_t nRF_pipe_writing[5] = {0xe1, 0xf0, 0xf0, 0xf0, 0xf0}; //random address
    //nRF24 Reading address
    uint8_t nRF_pipe_reading[5] = {0x32, 0x20, 0xaa, 0x3f, 0xab}; //random address
    
	//Open read and write pipes
    nrf_openWritingPipe(nRF_pipe_writing);
    nrf_openReadingPipe(0, nRF_pipe_reading);

    StatusLED(LED_ONLINE);
    
    //i2c_init();
    //ANSELB = 0x50 breaks the code 0x00 (0x50 == RB4 & RB6 as analog inputs [1 = analog, 0 = digital I/O))
    ANSELB = 0x40;
    
    UART_Debug("Printf is working\r\n");

	//Send configuration details over UART
	//Disable when not testing/debugging
    //nrf_powerUp();
    nrf_printDetails();

    __delay_ms(2000);
    
    //printf("THE STATUS IN HEADER FILE: %X", STATUS);
    //uint8_t i;
	//const char text[] = "HELLO WORLD!";
	
    //Very simple example
    while(1) {
        StatusLED(LED_ONLINE);	//Turn on On/Off LED
        nrf_powerUp();			//Power up nRF (wake up)
        
        __delay_ms(50);			//delay so the module has some time to set-up
        sprintf(sendPayload, "HELLO WORLD!");	//Format string
        printf("Sending data: %s\r\n", sendPayload);	//Avoid printf! This is just a simple example
        
        StatusLED(LED_WORKING);	
        
        nrf_stopListening();	// Stop listening and write to radio
        __delay_ms(10);
		
       // UART_Debug("Test3\n\r");
        
        if(nrf_get_status())
            UART_Debug("CONNECTION ESTABLISHED\n\r");
        else if (nrf_get_status() == 0)
            UART_Debug("CONNECTION FAILED\n\r");
        
        printf("%d\n\r", nrf_available(nRF_pipe_writing));
        // Try to send data to hub
        if ( nrf_write(sendPayload, 32) ) { //sendPayload
            UART_Debug("Send successful\n\r");	//Uart debug message
            nrf_write(&sendPayload, sizeof(sendPayload)); 
            StatusLED(LED_ONLINE);				//Set Activity LED to success
        } 
        else {
            UART_Debug("Send failed\n\r");		//Uart debug message
            StatusLED(LED_FAIL);				//Set Activity LED to fail
        }
     
    
       //UART_Debug("Hello2?\n\r");
        
       /* nrf_startListening();					//wait response
        __delay_ms(20);
		
		//Read all available data from reading pipe
        while ( nrf_available(nRF_pipe_reading) ) 		
		{
            uint8_t len = nrf_getDynamicPayloadSize();
            nrf_read(receivePayload, len);
            printf("%c\n\r", receivePayload[0]);

            __delay_ms(1000);
        } // End while */
        
        __delay_ms(5000);						//Delay a bit before going to sleep
        StatusLED(LED_ONLINE);					//Set device On/Off LED
    }
}
