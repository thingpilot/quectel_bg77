/** 
    This is a library for Quectel bg77. Cat M1/Cat NB2
    @file    quectel_bg77.h
    @version 0.1.0
    @author  Rafaella Neofytou
    @brief   Header file of the quectel bg77 driver module
             LTE BG77 Cat M1/NB2 Module
 */

/** Define to prevent recursive inclusion
 */
#pragma once

/** Includes 
 */
#include <mbed.h>
#include "SerialStream.h"

#define MSG_BUFF_SIZE 300
/**
   Communicating with Quectel according to the AT manual
   https://www.quectel.com/UploadImage/Downlad/Quectel_BG95&BG77_AT_Commands_Manual_V1.0.pdf
   https://www.quectel.com/UploadImage/Downlad/Quectel_BG95&BG77&BG600L_Series_QCFG_AT_Commands_Manual_V1.0.pdf
   
   Example code
   #include <quectel_bg77.h>
   QUECTEL_BG77 modem()
   int main(void)
   {
        modem.echo_te_off();    // (optional)

        int status;
        int x = 2;
     // Check module
        modem.at();             
        modem.manufacturer_id();
        int signal_strength;
        int quality;
        //Configure module
        modem.cfun(0);          // basic functionality for configuration
        modem.band_config(QUECTEL_BG77::LTE); // choices between lte, nbiot for gb77
        modem.cfun(1);          // Full functionality, enable radio
        modem.enter_pin(1234);        // Enter SIM pin. 
        modem.creg(x, status);  //get status, we want to see 5

   }
 
 */

 /** TODO List: 1) Error message handle (check AT+CMEE)
                2) Configure the URC (received message) output port (check AT+QURCCFG)
                3) Determine what happens when a msg arrives (RING URC is presented) (check AT+QCFG="urc/ri/ring")
                4) HTTP post or MQTT conf. It's not the same as saran2 check diff
  */


/** Base class for the quectel module
 */ 
class QUECTEL_BG77
{

	public:

        char messageBuff[MSG_BUFF_SIZE];

        /** Band Configurations */
        enum e_band
        {
            GSM     = 0,
            GPRS    = 1,
            LTE     = 2,
            NBIoT   = 3
        };

		/** Constructor. Instantiates an ATCmdParser object
		    on the heap for comms between microcontroller and modem
		   
		   @param txu Pin connected to quectel TXD (This is MCU TXU)
		   @param rxu Pin connected to quectel RXD (This is MCU RXU)
		   @param cts Pin connected to quectel CTS
		   @param rst Pin connected to quectel RST
		   @param vint Pin conencted to quectel VINT
		   @param gpio Pin connected to quectel GPIO1
		   @param baud Baud rate for UART between MCU and quectel
		 */  
		QUECTEL_BG77(PinName txu, PinName rxu, PinName pwkey, PinName ctl1, PinName ctl2, PinName ctl3, int baud = 9600);

		/** Destructor for the Quactel class. Deletes the BufferedSerial (instead of UartDerial) and ATCmdParser
		    objects from the heap to release unused memory
		 */  
		~QUECTEL_BG77();

        /** Lock to enforce a mutual exclusion concurrency control policy
            @return Nothing
         */
        void mutex_lock();

        /** Clears the buffer
        */
        void clearMessageBuff();

        /** Unlock the enforce of mutual exclusion concurrency control policy
            @return Nothing
         */
        void mutex_unlock();

		/** Send "AT" command
            @return Indicates success or failure 
         */
		int at();

        /** Set UE(user equipment) functionality
            @param mode. <fun>  0: Minimum functionality, 
                                1: Full functionality, 
                                4: Disable the UE from both transmitting and receiving RF signals.
         */
        int cfun(int mode);

        /** Query network info and set band configuartion. Default value is LTE
            Configure RAT Searching Sequence, qcfg_configuration
            
            @param  Choose band GSM, LTE, NBIOT..
            @return Indicates success or failure
         */
        int band_config(e_band band = LTE,  uint64_t bands = 80000);

        /** Configure RAT Searching Sequence. (Translate:  Only scan for <scanseq> Networks)
            @param scanseq. <scanseq>       0: Automatic (eMTC -> NB-IoT -> GSM), 
                                            1: GSM (For BG95-M3 only, 
                                            2: eMTC
                                            3: NB-IoT
         */
        int scan_sequence(int scanseq);

        /** Extended Configuration Settings.
            Configure RAT(s) to be Searched.  (Only scan for <scanmode>...)
            @param instruction                 1. nwscanmode,  2. iotopmode,   3. roamservice, 4. servicedomain
            @param scanmode. <scanmode>        0: Automatic,   0: eMTC,        0: Disable
                                               1: GSM,         1: NB-IoT,      1: Enable
                                               3: NB-IoT only  2: eMTC&NBIoT   255: Auto
         */

        int qcfg_configuration(const char *instruction, int scanmode);
    
        /** Reset LTE band configuration. AT+QPRTPARA=3 to default
         *  @return Indicates success or failure
         */
        int reset_band_config();

        /**** Network Service Commands. Check for network registration, operator selection, signal quality ***/

        /** Network Registration Status. (Get connection status)
            @param status       0 Not registered. MT is not currently searching an operator to register to.
                                1 Registered, home network
                                2 Not registered, but MT is currently trying to attach the network or searching an operator to register to.
                                3 Registration denied
                                4 Unknown
                                5 Registered, roaming

            @param act          0 GSM
                                8 eMTC
                                9 NB-IoT

            @return Indicates success or failure 
         */
        int creg(int &status, int &act);


        /** Csq return the 
            @param &signal_strength, rssi. Value = 2-31 for -109 to -51 dBm (linear).  
            @param &quality. Channel bit error rate in percent. (Value = 0 - 7 check RxQUAL BER)
            Value = 99 means Not known or not detectable.
            @return Indicates success or failure 
         */
        int csq(int &signal_strength, int &quality);

        /** Set Command Echo Mode to off, default to 1. Stops the module from echoes 
            chars received from terminal equipment.
         */
        int echo_te_off();

        /** Send "ATE0" command. This should return Quactel BG77XX REVISION XXXX..
            @note   If you see the echo of your AT commands, turn off the echo mode by issuing “ATE0”.
            @return Indicates success or failure 
         */
        int manufacturer_id();

        /** This should return a 15 digit number called, IMEI number. 
            @return Indicates success or failure 
         */
        int imei();

        /** This should return a 15 digit number called, IMSI number. 
           @return Indicates success or failure 
         */
        int imsi();
       
        /** !!IMPORTANT!!
            This command sends to the MT a password which is necessary before it can be operated, or queries
            whether the MT requires a password or not before it can be operated. The password may be (U)SIM PIN,
            (U)SIM PUK, PH-SIM PIN, etc. 
            @return Indicates success or failure 
         */
        int enter_pin(int pin);

        /** Trigger the Module into power saving mode Immediately. TODO: AT+CPSMS Power Saving Mode Setting
            @param mode. <mode>     0: Enter PSM after T3324 expires (active timer), 
                                    1: Enter PSM immediately after RRC connection release is received, 
                                    (An RRC connection release message may be signaled to the UE to put the UE into an RRC idle state)
         */
        int enter_psm(int mode);

        /** Operator Selection. 
           @param mode. <mode>      0: Automatic
                                    1: Manual TODO: do the manual selection
                                    2: 
           @return Indicates success or failure 
         */
        int enable_autoconnect();

        /** Automatic Time Zone Updaten. 
           @return Indicates success or failure 
         */
        int auto_zone_update();

        /** Configure Parameters for HTTP(S) Server. 
           @return Indicates success or failure 
         */
        int configure_http_server();

        /** Configure Parameters for HTTP(S) Server. 
            @return Indicates success or failure 
         */
        int request_http_header();

        /** Configure the PDP context ID and SSL context ID
            @return Indicates success or failure 
         */
        int response_http_header();
        
        /** Set URL of HTTP(S) Server. 
            @return Indicates success or failure 
         */
        int set_http_url();
	
 	/** Power saving mode Settings
         */
        int cpsms();

        /** Sends the post to the server
        */
        int send_http_post();

        int tcpip_startup();
        
        /** Turn of the module.  This procedure is realized by letting the module log off from the network and allowing the software to
            enter a secure and safe data state before disconnecting the power supply
            After this do not send any other AT commands and !!power supply should be disconnected!!
           @return Indicates success or failure 
         */
        int turn_off_module();

        /** Cops
         */ 
        int cops_info();

        /** Access technology 
         */ 
        int acces_tech_info();

        /** Define pdb contex
         */
        int define_pdp();

        /** Define pdb contex
        */
        int cgreg();

        int send_msg();
        
        /** GNSS RELATED AT COMMANDS
        */

        int gnss_configuration();
        int turn_on_gps();
        int get_gps_pos();
        int conf_nmea_output_port();
        int query_satellite_system();
        int nmea_configuration();

        /*Nniot configuration*/
        int disable_psm();
        int nbiot_configuration();
        int define_pdp_nbiot();
        int ping();
        int qnwinfo();
        

    private:
        void _modem_on();


    /**Digital inputs 
     */
    DigitalOut _pwkey; 
    DigitalOut _ctl1;
    DigitalOut _ctl2;
    DigitalOut _ctl3;
   

	private:
    BufferedSerial  *_serial;
    
    /*Parser for at commands*/
    ATCmdParser *_parser;

    BufferedSerial  *_gps_serial;
    
    /*Parser for at commands*/
    ATCmdParser *_gps_parser;
    /*Mutex or lock to enforce mutual exclusion*/
	Mutex _smutex;

};

