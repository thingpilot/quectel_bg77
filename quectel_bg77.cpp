/**
    @file       quectel_bg77.cpp
    @version    0.1.0
    @author     Rafaella Neofytou
    @brief      Cpp file of the quectel bg77 driver module
    @note       Chooses correct Sim Card for the application. 
                BG77 supports: LTE Cat M1, LTE Cat NB2, integrated GNSS
 */


/** Includes */
#include "quectel_bg77.h"
#include <cstdint>
#include <string>
#include <sstream>  


//TODO: pdp defined by the user
 
QUECTEL_BG77::QUECTEL_BG77(PinName txu, PinName rxu, PinName pwkey, PinName ctl1, PinName ctl2,  
            PinName ctl3, int baud) :_pwkey(pwkey), _ctl1(ctl1, 1), _ctl2(ctl2, 1), _ctl3(ctl3, 0) //001 lte //100 nbiot
{
	_serial = new BufferedSerial(txu, rxu, baud);
	_parser = new ATCmdParser(_serial);
	_parser->set_delimiter("\r\n");
	_parser->set_timeout(600); //TODO: check with ~300
    _modem_on();
}


QUECTEL_BG77::~QUECTEL_BG77()
{
	delete _serial;
	delete _parser;
}

void QUECTEL_BG77::_modem_on()
{
    _pwkey = 0;
    rtos::ThisThread::sleep_for(30ms);
    _pwkey = 1;
    rtos::ThisThread::sleep_for(600ms);
    _pwkey = 0;
    rtos::ThisThread::sleep_for(300ms);
  
}

void QUECTEL_BG77::mutex_lock()
{
    _smutex.lock();
    _parser->flush();
}

void QUECTEL_BG77::mutex_unlock()
{
	_smutex.unlock();
}

int QUECTEL_BG77::tcpip_startup(){

    int status = 0;

    char cpinBuff[16];
    int cereg_n;
    int ceregStatus;
    char qiBuff[64];

    int cregCount = 0;

    mutex_lock();

    // send AT command and wait for response
    _parser->send("AT");
    rtos::ThisThread::sleep_for(300ms);
    // if no response, return fail
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}

    _parser->send("AT");
    rtos::ThisThread::sleep_for(300ms);
    // if no response, return fail
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}

    // make sure we are in full functionality mode
    _parser->send("AT+CFUN=1,0");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}

    _parser->send("AT+QCFG=\"band\",0,0,0x80000"); //band 20
    if(!_parser->recv("OK"))
    {
        status = -1;
    }

    _parser->send("AT+QCFG=\"nb1/bandprior\",14"); //band 20
    if(!_parser->recv("OK"))
    {
        status = -1;
    }

    _parser->send("AT+QCFG=\"iotopmode\",1,1"); //configure network to be searched/ nbiot, take effect immediately
    if(!_parser->recv("OK"))
    {
        status = -1;
    }

    rtos::ThisThread::sleep_for(10s);

    // query the sim card status
    if(status == 0){
        _parser->send("AT+CPIN?");
        rtos::ThisThread::sleep_for(300ms);
        if(!_parser->recv("+CPIN: %s", cpinBuff))
        {
		    status = -1;
	    }
    }

    // if fails to identify, return fail
    if(status == 0){
        if(!std::strcmp(cpinBuff,"READY")){
            status = -1;
        }
    }

    // configure PDP context with QICSGP (vodafone APN)
    if(status == 0){
        _parser->send("AT+QICSGP=1,1,\"lpwa.vodafone.iot\"");
        rtos::ThisThread::sleep_for(300ms);
        if (!_parser->recv("OK"))
        {
            status = -1;	
        }
    }

    //while(status == 0 && (ceregStatus != 1 || ceregStatus != 5)){
    for(int ii = 0; ii < 1; ii++){
        rtos::ThisThread::sleep_for(15s);
        _parser->send("AT+QCSQ");
        rtos::ThisThread::sleep_for(300ms);
        if (!_parser->recv("OK"))
        {
            status = -1;	
        }

        _parser->send("AT+QNWINFO");
        rtos::ThisThread::sleep_for(300ms);
        if (!_parser->recv("OK"))
        {
            status = -1;	
        }

        _parser->send("AT+CEREG?");
        rtos::ThisThread::sleep_for(300ms);
        if(!_parser->recv("+CEREG: %d,%d", &cereg_n, &ceregStatus))
        {
		    status = -1;
	    }

        if(cregCount > 2){
            status = -1;
        }
        cregCount++;
    }

    // activate the PDP context with QIACT=1
    if(status == 0){
        _parser->send("AT+QIACT=1");
        rtos::ThisThread::sleep_for(500ms);
        if (!_parser->recv("OK"))
        {
            status = -1;	
        }
    }

    // query the PDP context with QIACT?
    if(status == 0){
        _parser->send("AT+QIACT?");
        rtos::ThisThread::sleep_for(300ms);
        _parser->recv("%s", qiBuff);
    }

    // if fail, wait 150s and try again
        // if fails to activate a second time, return fail

    // try to open a connection with QIOPEN up to 5 times with 3s delay

    // if fail, deactivate PDP context with QIDEACT
        // return fail

    // try sending some data with QISEND

    // query ACK with QISEND every 5s up to 24 times
        // if no ack, return fail

    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::at()
{
    int status = 0;
    mutex_lock();
	_parser->send("AT");
    rtos::ThisThread::sleep_for(300ms);
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::cfun(int mode)
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+CFUN=%d,0", mode);
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::band_config(QUECTEL_BG77::e_band band, uint64_t bands)
{
    int  status = -1;
    char buffer[100];
    int  size = 100;

    //Query Network Information
    mutex_lock();
    
    // _parser->send("AT+QNWINFO");
    // rtos::ThisThread::sleep_for(10s);
    // _parser->recv(buffer, size);
    // printf("QUERY RESP: %s", buffer);
    // //NBIOT ONLY

    // if (bands == 18)
    // {
    //      _parser->send("AT+QCFG=\"band\",0,0,0x1000000000000000000000"); //band 20 we can have
    // }
    _parser->send("AT+QCFG=\"band\",0,0,0x80000"); //band 20 we can have 
    if(!_parser->recv("OK"))
    {
        status = -1;
    }
    rtos::ThisThread::sleep_for(10s);

    _parser->send("AT+QCFG=\"iotopmode\",1,1"); //configure network to be searched/ nbiot, take effect immediately
    if(!_parser->recv("OK"))
    {
        status = -1;
    }

    _parser->send("AT+QCFG=\"apn/display\",1");//ffect immediately
    if(!_parser->recv("OK"))
    {
        status = -1;
    }
    
    // switch(band)
    // {
    //     case GSM:
    //         printf("Switching to GSM..\r\n");
    //         // Do not change anything  its GSM1800 + GSM1900
    //         _parser->send("(AT+QCFG=\"band\",0,0,0");
    //         if(!_parser->recv("OK"))
    //         {
    //             status = -1;
    //         }
    //     case GPRS:
    //         printf("Switching to GPRS..\r\n");
    //     case LTE:
    //         printf("Switching to LTE..\r\n");
    //         status = scan_sequence(LTE);
    //         if (status != 0)
    //         {
    //             return (status);
    //         }
    //         // LTE only scan
    //         qcfg_configuration("nwscanmode", 0); //for lte is 3

    //         // Search only CAT-M Networks
    //         qcfg_configuration("iotopmode", 2); //for emtc/lte is 0
    //         // Set CATM1 Bands to LTE Band 20
    //         // _parser->send("AT+QCFG=\"band\",0,80000,0");
    //         // if(!_parser->recv("OK"))
    //         // {
    //         //     return (-1);
    //         // }
    //         // qcfg_configuration("roamservice", 255);
    //         // qcfg_configuration("servicedomain", 2);

    //     case NBIoT:
    //         printf("Switching to NBIoT..\r\n");

    //         _parser->send("AT+QCFG=\"iotopmode\",1,1"); //configure network to be searched/ nbiot, take effect immediately
    //         if(!_parser->recv("OK"))
    //         {
    //             status = -1;
    //         }
    //         rtos::ThisThread::sleep_for(5s);
    //         _parser->send("AT+QCFG=\"band\",0,0,0x80000"); //band 20 we can have 
    //         if(!_parser->recv("OK"))
    //         {
    //             status = -1;
    //         }
    //         rtos::ThisThread::sleep_for(5s);
    //     default:
    //         break;
    // }

    mutex_unlock();
    return (status);
}
int QUECTEL_BG77::qnwinfo()
{
    int status = 0;
    mutex_lock();
    rtos::ThisThread::sleep_for(300ms);
    _parser->send("AT+QNWINFO");
    rtos::ThisThread::sleep_for(2s);
    if(!_parser->recv("OK"))
    {
        status = -1;
    }
   
    mutex_unlock();
    return (status);
}
int QUECTEL_BG77::scan_sequence(int scanseq)
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QCFG=\"nwscanseq\",0%d,1",scanseq);//,scanseq);
    rtos::ThisThread::sleep_for(300ms);
    if(!_parser->recv("OK"))
    {
        status = -1;
    }
    
    mutex_unlock();
    return (status);
}

int QUECTEL_BG77::qcfg_configuration(const char *instruction, int scanmode)
{
    int status = 0;
    mutex_lock();
    _parser->send("(AT+QCFG=\"%s\",%d,1)",instruction, scanmode);
    if(!_parser->recv("OK"))
    {
        status = -1;
    }
    mutex_unlock();
    return (status);
}

int QUECTEL_BG77::creg(int &r_status, int &act)
{
    int status = 0;
    char lac[3];
    char ci[5];
    char *buffer;
    mutex_lock();
    // _parser->send("AT+CREG=2");
    // if(!_parser->recv("OK"))
    // {
    //     status = -1;
    // }
    // ThisThread::sleep_for(2s);
    // _parser->send("AT+CREG?");
    // if(!_parser->recv("+CREG: %d,%2[^\n],%4[^\n],%d", &r_status, lac, ci, &act)) //todo: check if \n is needed
    // {
    //     status = -1;
    // }
    _parser->send("AT+CEREG=1");
    _parser->recv("%s", buffer); 
    printf("%s", buffer);

    _parser->send("AT+CEREG?");
    _parser->recv("%s", buffer); 
    printf("%s", buffer);

    mutex_unlock();
    return (status);
}

int QUECTEL_BG77::cgreg()
{
    int status = 0;
    char buffer[250];
   
    _parser->send("AT+CREG?");

    _parser->recv("%s", buffer); 
    printf("%s", buffer);
    mutex_unlock();
    return (status);
}


int QUECTEL_BG77::csq(int &signal_strength, int &quality)
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QCSQ");
    if(!_parser->recv("+CSQ: %d,%d", &signal_strength, &quality))
    {
        status = -1;
    }
    rtos::ThisThread::sleep_for(3s);
    // _parser->send("AT+QCSQ=?");
    // if(!_parser->recv("+CSQ: %d,%d", &signal_strength, &quality))
    // {
    //     status = -1;
    // }
    _parser->send("AT+CSQ");
    if(!_parser->recv("+CSQ: %d,%d", &signal_strength, &quality))
    {
        status = -1;
    }
    // rtos::ThisThread::sleep_for(10s);
    // _parser->send("AT+QICSGP=1");
    // if(!_parser->recv("+CSQ: %d,%d", &signal_strength, &quality))
    // {
    //     status = -1;
    // }
    // rtos::ThisThread::sleep_for(10s);

    // _parser->send("AT+QIACT=?");
    // if(!_parser->recv("+CSQ: %d,%d", &signal_strength, &quality))
    // {
    //     status = -1;
    // }
    // _parser->send("AT+QIACT?");
    // if(!_parser->recv("+CSQ: %d,%d", &signal_strength, &quality))
    // {
    //     status = -1;
    // }
    rtos::ThisThread::sleep_for(3s);
    // _parser->send("AT+QIACT=1");
    // if(!_parser->recv("+CSQ: %d,%d", &signal_strength, &quality))
    // {
    //     status = -1;
    // }
    // rtos::ThisThread::sleep_for(150s);
    mutex_unlock();
    return (status);
}

int QUECTEL_BG77::echo_te_off()
{
    int status = 0;
    mutex_lock();
	_parser->send("ATE0"); 
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::manufacturer_id()
{
    int     status = -1;
    char    *received_data;
    int     str_size; 
    mutex_lock();
	_parser->send("ATI");
    _parser->read(received_data, str_size);
    if (str_size > 0)
    {
        printf("%s", received_data);  
        status = 0;
    }
    mutex_unlock();
	return (status);
}
/* Note: the IMEI can be used to identify ME
*/
int QUECTEL_BG77::imei()
{
    int     status = -1;
    char    imei[15+1];
    int     str_size; 

    printf("Reading IMEI\r\n");
    mutex_lock();
	_parser->send("AT+CGSN");
    if (_parser->recv("%15[^\n]\nOK\n", imei))
    {
        printf("IMEI: %s", imei);  
        status = 0;
    }
    mutex_unlock();
	return (status);
}

/*TODO: it can respond with error, get the error*/
int QUECTEL_BG77::imsi()
{
    int     status = -1;
    char    imsi[16]; //type string without double quotes
    int     str_size; 

    printf("Reading IMSI\r\n");
    mutex_lock();
	_parser->send("AT+CIMI");
    if (_parser->recv("%15[^\n]\nOK\n", imsi))
    {
        printf("IMSI: %s", imsi);  
        status = 0;
    }
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::enter_pin(int pin)
{

    int status = 0;
    mutex_lock();
    _parser->send("AT+CPIN?");
    
    // if (_parser->recv("+CME ERROR: SIM not inserted"))
	// {
    //     printf("SIM NOT INSERTED\n");
	// 	status = -1;	
	// }
    // if (!_parser->recv("OK\n+CPIN: READY"))
    // {
    //     _parser->send("AT+CPIN=%d", pin);
    //     if (!_parser->recv("OK\n+CPIN: READY"))
    //     {
    //         status = -1;	
    //     }
    // }
    mutex_unlock();
	return (status);
}

//TODO check AT+CPSMS=1,,,... or AT+QPSMS
int QUECTEL_BG77::enter_psm(int mode)
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+QCFG=\"psm/enter\",%d", mode);
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::enable_autoconnect()
{
    int     status = -1;
    char buffer[100];
    char buffer2[100];
    char buffer3[500];
    
    mutex_lock();
    // _parser->send("AT+COPS=0");
    // _parser->recv(buffer, sizeof(buffer));
    // printf("%s", buffer);

    uint32_t time_now = time(NULL);
    _parser->send("AT+COPS?");
    _parser->recv(buffer, sizeof(buffer));
    printf("%s", buffer);

    // uint32_t time_after_send = time(NULL);
    // _parser->send("AT%d", time_after_send - time_now);

    // _parser->recv(buffer2, sizeof(buffer2));
    // printf("%s", buffer2);

    // _parser->send("AT+COPS=0"); //TODO: the manual selection etc
	// if (!_parser->recv("OK"))
	// {
	// 	status = -1;	
	// }
    rtos::ThisThread::sleep_for(2s);
    _parser->send("AT+COPS=?");

    rtos::ThisThread::sleep_for(2s);
	_parser->recv(buffer3, sizeof(buffer3));
    printf("%s", buffer3);
    

    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::auto_zone_update()
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+CTZU=1");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::configure_http_server()
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+QHTTPCFG=\"contextid\",1");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::request_http_header()
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+QHTTPCFG=\"requestheader\",1");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::response_http_header()
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+QHTTPCFG=\"responseheader\",1");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::set_http_url()
{
    string url = "https://api.testing.thingpilot.com/v0/metrics";
    string postbody = "{\"uniqueId\":\"0577916f-dfbe-4bb3-948d-f04b7e370953\",\"deviceType_id\":\"6071f29e093b210013071ba8\",\"metrics\":{\"Temp and Humidity\":[{\"startTimestamp\":\"2021-01-01T00:00:00Z\",\"endTimestamp\":\"2021-01-01T00:01:00Z\",\"humidity\":[10,20,30,40],\"temperature\":[100,200,300,400]}]}}";
    string header = "POST /v0/metrics?info=true HTTP/1.1\r\nHost: api.testing.thingpilot.com\r\nAccept: */*\r\nUser-Agent: cav-bike-tracker\r\nConnection: keep-alive\r\nContent-Type: application/json\r\nContent-Length: 265\r\n\r\n";
    int messageSize = postbody.size() + header.size();

    int status = 0;
    mutex_lock();

    _parser->send("AT+QHTTPCFG=\"requestheader\",1"); // set customisation of header
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}

	_parser->send("AT+QHTTPURL=%d,80", url.size()); //TODO: change url length , timeout, url can be inputed aafter this
    rtos::ThisThread::sleep_for(500ms);
	if (!_parser->recv("CONNECT"))
	{
		status = -1;	
	}

    _parser->send("https://api.testing.thingpilot.com/v0/metrics?info=true");
    if (!_parser->recv("OK"))
	{
		status = -1;	
	}

    rtos::ThisThread::sleep_for(2s);

    // POST
    _parser->send("AT+QHTTPPOST=%d,30,30", messageSize);
    rtos::ThisThread::sleep_for(2s);
    if (!_parser->recv("CONNECT:")) 
	{
		status = -1;	
	}
    rtos::ThisThread::sleep_for(500ms);

    _parser->send("POST /v0/metrics?info=true HTTP/1.1\r\nHost: api.testing.thingpilot.com\r\nAccept: */*\r\nUser-Agent: cav-bike-tracker\r\nConnection: keep-alive\r\nContent-Type: application/json\r\nContent-Length: 265\r\n\r\n{\"uniqueId\":\"0577916f-dfbe-4bb3-948d-f04b7e370953\",\"deviceType_id\":\"6071f29e093b210013071ba8\",\"metrics\":{\"Temp and Humidity\":[{\"startTimestamp\":\"2021-01-01T00:00:00Z\",\"endTimestamp\":\"2021-01-01T00:01:00Z\",\"humidity\":[10,20,30,40],\"temperature\":[100,200,300,400]}]}}");
    rtos::ThisThread::sleep_for(2s);
    if (!_parser->recv("OK"))
	{
		status = -1;	
	}

    rtos::ThisThread::sleep_for(5s);

    // get response and wait up to 20s for the HTTP session to close
    _parser->send("AT+QHTTPREAD=30");
    rtos::ThisThread::sleep_for(30s);
    if (!_parser->recv("+QHTTPREAD: 0"))
	{
		status = -1;	
	}

    rtos::ThisThread::sleep_for(5s);



    // // send the GET request and timeout after 20s
    // _parser->send("AT+QHTTPGET=20");
    // rtos::ThisThread::sleep_for(2s);
    // if (!_parser->recv("+QHTTPGET: 0,200")) // 200 means OK response
	// {
	// 	status = -1;	
	// }

    // rtos::ThisThread::sleep_for(2s);

    // // get response and wait up to 20s for the HTTP session to close
    // _parser->send("AT+QHTTPREAD=20");
    // rtos::ThisThread::sleep_for(2s);
    // if (!_parser->recv("+QHTTPREAD: 0"))
	// {
	// 	status = -1;	
	// }

    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::cpsms()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QPSMS=?");
    if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    rtos::ThisThread::sleep_for(300ms);

	_parser->send("AT+QPSMS=1");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    rtos::ThisThread::sleep_for(300ms);

     _parser->send("AT+CPSMS=?");
    if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    rtos::ThisThread::sleep_for(300ms);

	_parser->send("AT+CPSMS=1");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    rtos::ThisThread::sleep_for(300ms);

    // _parser->send("AT+QSCLK=1");
    // if (!_parser->recv("OK"))
	// {
	// 	status = -1;	
	// }
    // rtos::ThisThread::sleep_for(300ms);

    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::turn_off_module()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+CFUN=0");
    if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    rtos::ThisThread::sleep_for(300ms);

    _parser->send("AT+QPOWD");
    if (!_parser->recv("OK"))
    {
	status = -1;	
    }
    rtos::ThisThread::sleep_for(300ms);
    mutex_unlock();
    return (status);
}

int QUECTEL_BG77::cops_info()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+COPS=?");
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
	
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::acces_tech_info()
{
    int status = 0;
    char buffer[1000];
    mutex_lock();
    _parser->send("AT+CEDRXRDP");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);
	
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::define_pdp()
{
    int status = 0;
    char buffer[100];
    mutex_lock();

    rtos::ThisThread::sleep_for(300ms);
    _parser->send("AT+CGDCONT=1,\"IP\",\"data.rewicom.net\"");
    if (!_parser->recv("OK"))
	{
		status = -1;	
	}

    _parser->send("AT+CGDCONT?");
    _parser->recv("%s", buffer);
    printf("%s", buffer);

    _parser->send("AT+CGACT=1,1 ");
    _parser->recv("%s", buffer);
    printf("%s", buffer);
    // rtos::ThisThread::sleep_for(1s);
    // _parser->send("AT+CGACT=1,1");
    // if (!_parser->recv("OK"))
	// {
	// 	status = -1;	
	// }
    // rtos::ThisThread::sleep_for(1s);
    // _parser->send("AT+CGPADDR=1");
    // _parser->recv("%s", buffer);
    // printf("%s", buffer);
    // rtos::ThisThread::sleep_for(1s);

    // _parser->send("AT+CGDCONT?");
    // _parser->recv("%s", buffer);
    // printf("%s", buffer);

    // _parser->send("AT+CGDCONT=?");
    // _parser->recv("%s", buffer);
    // printf("%s", buffer);
	
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::define_pdp_nbiot()
{
    int status = 0;
    char buffer[100];
    mutex_lock();

    _parser->send("AT+CGDCONT=1,\"IP\",\"lpwa.vodafone.iot\"");
    if (!_parser->recv("OK"))
	{
		status = -1;	
	}
    rtos::ThisThread::sleep_for(2s);

    // _parser->send("AT+CGDCONT?");
    // rtos::ThisThread::sleep_for(300ms);
    // if (!_parser->recv("OK"))
	// {
	// 	status = -1;	
	// }
    // _parser->send("AT+CGACT=1,1 ");
    // _parser->recv("%s", buffer);
    // printf("%s", buffer);
   
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::reset_band_config()
{
    int status = 0;

    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT&F0");
    rtos::ThisThread::sleep_for(300ms);
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
	
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::send_msg()
{
    int status = 0;

    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT+CMGS=004407751028943");
    rtos::ThisThread::sleep_for(300ms);
	if (!_parser->recv("OK"))
	{
		status = -1;	
	}
	
    mutex_unlock();
	return (status);
}

/** GNSS
 */
int QUECTEL_BG77::gnss_configuration()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT+QGPSCFG=\"outport\",\"none\"");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    _parser->send("AT+QGPSCFG=?");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::turn_on_gps()
{
    int status = 0;
    char buffer[100];
    mutex_lock();

    _parser->send("AT+QGPSCFG=\"priority\",0");
    rtos::ThisThread::sleep_for(1s);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    _parser->send("AT+QGPS=1");
    rtos::ThisThread::sleep_for(10s);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    _parser->send("AT+QGPS?");
    rtos::ThisThread::sleep_for(1s);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    // _parser->send("AT+QGPSCFG=\"autogps\",1");
    // rtos::ThisThread::sleep_for(300ms);
	// _parser->recv("%s", buffer);
    // printf("%s",buffer);
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::get_gps_pos()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
    
    // _parser->send("AT+QGPSLOC=2");
    // rtos::ThisThread::sleep_for(1s);
	// _parser->recv("%s", buffer);
    // printf("%s",buffer);

    _parser->send("AT+QGPSLOC?");
    rtos::ThisThread::sleep_for(30s);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    // _parser->send("AT+QGPSGNMEA=?");
    // rtos::ThisThread::sleep_for(300ms);
	// _parser->recv("%s", buffer);
    // printf("%s",buffer);

    // _parser->send("AT+QGPSGNMEA=\"RMC\"");
    // rtos::ThisThread::sleep_for(300ms);
	// _parser->recv("%s", buffer);
    // printf("%s",buffer);

    // _parser->send("AT+QGPSEND");
    // rtos::ThisThread::sleep_for(1s);
	// _parser->recv("%s", buffer);
    // printf("%s",buffer);
    mutex_unlock();
	return (status);
}



int QUECTEL_BG77::conf_nmea_output_port()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT+QGPSCFG=\"outport\",\"uartnmea\",115200");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    

    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::query_satellite_system()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT+QGPSCFG=\"gnssconfig\"");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::nmea_configuration()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
  
    _parser->send("AT+QGPSCFG=\"gpsnmeatype\"");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    _parser->send("AT+QGPSGNMEA=\"GSV\"");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    _parser->send("AT+QGPSCFG=\"gnssconfig\"");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);

    mutex_unlock();
	return (status);
}

/*NB-IOT CONFIGURATIONS */
int QUECTEL_BG77::disable_psm()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT+CPSMS=0");
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);
    mutex_unlock();
	return (status);
}
int QUECTEL_BG77::nbiot_configuration()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT+QCFG=\"nb1/bandprior\""); //query the current setting (re)
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);

    _parser->send("AT+QCFG=\"nb1/bandprior\",14"); //query the current setting (re)
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);
    printf("%s",buffer);
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::ping()
{
    int status = 0;
    char buffer[100];
    mutex_lock();
    //_parser->send("AT+QPRTPARA=3");
    _parser->send("AT+QPING=1,\"testing.thingpilot.com\",10,10"); //query the current setting (re)
    rtos::ThisThread::sleep_for(300ms);
	_parser->recv("%s", buffer);

   
    mutex_unlock();
	return (status);
}