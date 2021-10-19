/** 
    This is a library for Quectel bg77. Cat M1/Cat NB2
    @file    quectel_bg77.h
    @version 0.0.3
    @author  Rafaella Neofytou
    @brief   Header file of the quectel bg77 driver module
             LTE BG77 Cat M1/NB2 Module
 */

#ifndef QUECTEL_BG77_H
#define QUECTEL_BG77_H

/** Define to prevent recursive inclusion
 */
#pragma once

/** Includes 
 */
#include <mbed.h>
/**
   Communicating with Quectel according to the AT manual
   https://www.quectel.com/UploadImage/Downlad/Quectel_BG95&BG77_AT_Commands_Manual_V1.0.pdf
   https://www.quectel.com/UploadImage/Downlad/Quectel_BG95&BG77&BG600L_Series_QCFG_AT_Commands_Manual_V1.0.pdf
   
   Example code
   #include <quectel_bg77.h>
   QUECTEL_BG77 modem()
   int main(void)
   {
        modem.at();             
        modem.manufacturer_id();
   }
 
 */

/** TODO List: 1) Better error message handling (divide critical non critical?)
               2) Add new feature by QUECTEL to detect jamming
               3) XTRA enabled? battery?!
 */

/** Base class for the quectel module
 */ 
class QUECTEL_BG77
{
	public:
        enum 
        {
            Q_SUCCESS = 0,
            Q_FAILURE = -1
        };

		/** Constructor. Instantiates an ATCmdParser object
		    on the heap for comms between microcontroller and modem
		   
		   @param txu Pin connected to quectel TXD (This is MCU TXU)
		   @param rxu Pin connected to quectel RXD (This is MCU RXU)
		   @param pwkey Pin connected to quectel powerkey
		   @param baud Baud rate for UART between MCU and quectel
		 */  
		QUECTEL_BG77(PinName txu, PinName rxu, PinName pwkey, int baud = 115200);

		/** Destructor for the Quactel class. Deletes the BufferedSerial (instead of UartDerial) and ATCmdParser
		    objects from the heap to release unused memory
		 */  
		~QUECTEL_BG77();

        /** Lock to enforce a mutual exclusion concurrency control policy
            @return Nothing
         */
        void mutex_lock();

        /** Unlock the enforce of mutual exclusion concurrency control policy
            @return Nothing
         */
        void mutex_unlock();

		/** Send "AT" command
            @return Indicates success or failure 
         */
		int at();

        /** Set Command Echo Mode to off, default to 1. Stops the module from echoes 
            chars received from terminal equipment.
         */
        int echo_te_off();

        /** Ping a url to check if connected
         */
        int ping(const char *url);

        /** Send "ATE0" command. This should return Quactel BG77XX REVISION XXXX..
            @note   If you see the echo of your AT commands, turn off the echo mode by issuing “ATE0”.
            @return Indicates success or failure 
         */
        int manufacturer_id();

        /** Current firmware version
         *  @return firmware version
         */
        int firmware_ver();

        /** Update to the latest firmware
         */
        int update_firmware(const char *url_bin_file);

        /** Set UE(user equipment) functionality
            @param mode. <fun>  0: Minimum functionality, 
                                1: Full functionality, 
                                4: Disable the UE from both transmitting and receiving RF signals.
         */
        int cfun(int mode);

        /** Query network info and set band configuartion. Default value is LTE
            Configure RAT Searching Sequence, qcfg_configuration
        
            @return Indicates success or failure
         */
        int band_config();

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

        /** Network Registration Status. (Get connection status)
            @return Indicates success or failure 
         */
        int creg();

        /** Csq return the 
            @param &signal_strength, rssi. Value = 2-31 for -109 to -51 dBm (linear).  
            @param &quality. Channel bit error rate in percent. (Value = 0 - 7 check RxQUAL BER)
            Value = 99 means Not known or not detectable.
            @return Indicates success or failure 
         */
        int csq(const char *apn);

        /** Query connection 
         *  @return status. Returns true if connected to Nb-iot
         */
        int qnwinfo();

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
        int query_sim();

        /** Trigger the Module into power saving mode Immediately. TODO: AT+CPSMS Power Saving Mode Setting
            @param mode. <mode>     0: Enter PSM after T3324 expires (active timer), 
                                    1: Enter PSM immediately after RRC connection release is received, 
                                    (An RRC connection release message may be signaled to the UE to put the UE into an RRC idle state)
         */
        int enter_psm(int mode);

        /** Exit power saving mode. If enabled it doesn't get a fix for location
         */
        int disable_psm();

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

        /**  Configure TCP
         */
        int tcpip_startup(const char *apn);


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
        int set_http_url(const char *url_m);

        /** Sends the post to the server
            @return true if safe, false if recovery needed
        */
        //bool send_http_post(float lat,  float lon,  const char *stateStr);
        bool send_http_post(const char* http_header, uint8_t *http_body, size_t body_len, const char *stateStr);

 	    /** Power saving mode Settings
         */
        int cpsms();
        
        /** Turn of the module.  This procedure is realized by letting the module log off from the network and allowing the software to
            enter a secure and safe data state before disconnecting the power supply
            After this do not send any other AT commands and !!power supply should be disconnected!!
           @return Indicates success or failure 
         */
        int turn_off_module();

        /** Cops
         */ 
        int cops_info();

        /** Query the pdp connection. If not connected then connect 
         */
        int activate_pdp();
        
        /** TODO: delete?
         */
        int define_pdp_nbiot();

        /** Sync date and time form the ntp server
         */
        char * sync_ntp();

        /** Query location, get a fix
         */
        int parse_latlon(float &lon, float &lat);

        /** Enable the extra mode. Download the xtra file.
         */
        int enable_xtra();

        /** Query
         */
        int query_satellite_system();

        /** 
         */
        int nmea_configuration();
        
        /** Enable the modem with powerkey
         */
        void _modem_on();
    private:
        
        /**Digital inputs*/
        DigitalOut _pwkey; 
        
        /** Uart*/
        BufferedSerial  *_serial;
        BufferedSerial  *_gps_serial;

        /*Parser for at commands*/
        ATCmdParser *_parser;

        /*Parser for at commands*/
        ATCmdParser *_gps_parser;

        /*Mutex or lock to enforce mutual exclusion*/
        Mutex _smutex;

};

#endif