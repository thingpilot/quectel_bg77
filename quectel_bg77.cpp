/**
    @file       quectel_bg77.cpp
    @version    0.0.3
    @author     Rafaella Neofytou
    @brief      Cpp file of the quectel bg77 driver module
    @note       Chooses correct Sim Card for the application. 
                BG77 supports: NBIOT && integrated GNSS
 */


/** Includes */
#include "quectel_bg77.h"
#include <cstdint>
#include <string>


QUECTEL_BG77::QUECTEL_BG77(PinName txu, PinName rxu, PinName pwkey, int baud) 
                                :_pwkey(pwkey) 
{
	_serial = new BufferedSerial(txu, rxu, baud);
	_parser = new ATCmdParser(_serial);
	_parser->set_delimiter("\r");
	_parser->set_timeout(12500); 
    _parser->flush();
}

QUECTEL_BG77::~QUECTEL_BG77()
{
	delete _serial;
	delete _parser;
}

void QUECTEL_BG77::_modem_on()
{
    mutex_lock();
    _parser->set_timeout(1000);
    _parser->send("AT");
	if (!_parser->recv("OK"))
	{
        // Modem is not already on, so power key it
        _pwkey = 0;
        ThisThread::sleep_for(30ms);
        _pwkey = 1;
        ThisThread::sleep_for(600ms);
        _pwkey = 0;
        ThisThread::sleep_for(300ms);
    }
    mutex_unlock();
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

int QUECTEL_BG77::at()
{
    int status = 0;
    mutex_lock();
	_parser->send("AT");
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
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
		status = Q_FAILURE;
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::ping(const char *url)
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QPING=1,\"%s\",10,10",url); 
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::manufacturer_id()
{
    int status = -1;
    mutex_lock();
	_parser->send("ATI");
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::firmware_ver()
{   
    int status = 0;
    mutex_lock();
	_parser->send("AT+GMR");
    if (_parser->scanf("BG77LAR02A02") 
        && _parser->recv("OK"))
    {
        status = 2;
    }
    _parser->send("AT+GMR");
    if (_parser->recv("BG77LAR02A04") 
        && _parser->recv("OK"))
    {
        status = 1;
    }
    mutex_unlock();
	return status;
}

int QUECTEL_BG77::update_firmware(const char *url_bin_file)
{
    mutex_lock();
    int error_code = -1;
	_parser->send("AT+QFOTADL=%s", url_bin_file);
    while(!_parser->recv("+QIND: \"FOTA\",\"END\",%d",error_code))
    {
        if(error_code == 0) //todo: works?
        {
            break;
        }
        _parser->flush();
    }
    mutex_unlock();
    return (0);
}


int QUECTEL_BG77::cfun(int mode)
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+CFUN=%d,0", mode);
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
    mutex_unlock();
	return (status);
}


int QUECTEL_BG77::band_config()
{
    int  status = Q_SUCCESS;
    mutex_lock();
    _parser->send("AT+QCFG=\"band\",0,0,0x80000"); //band 20 we can have 
    if(!_parser->recv("OK"))
    {
        mutex_unlock();
        status = Q_FAILURE;
    }

    _parser->send("AT+QCFG=\"nb1/bandprior\",14"); //band 20
    if(!_parser->recv("OK"))
    {
        mutex_unlock();
         status = Q_FAILURE;
    }

    _parser->send("AT+QCFG=\"iotopmode\",1,1"); //configure network to be searched/ nbiot, take effect immediately
    if(!_parser->recv("OK"))
    {
        mutex_unlock();
        status = Q_FAILURE;
    }

    mutex_unlock();
    return Q_SUCCESS;
}

int QUECTEL_BG77::scan_sequence(int scanseq)
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QCFG=\"nwscanseq\",0%d,1",scanseq);
    rtos::ThisThread::sleep_for(300ms);
    if(!_parser->recv("OK"))
    {
        status = Q_FAILURE;
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

int QUECTEL_BG77::reset_band_config()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT&F0");
    rtos::ThisThread::sleep_for(300ms);
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::creg()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+CEREG?");
    if (!_parser->recv("+CEREG: 0,5"))
    {
        ThisThread::sleep_for(1s);
        _parser->send("AT+CEREG=1");
        if (!_parser->recv("OK"))
        {
            return Q_FAILURE;
        }
    }
    mutex_unlock();
    return (status);
}

int QUECTEL_BG77::csq(const char *apn)
{
    int status = 0;
    int cereg_n = -1;
    int ceregStatus = -1;
    mutex_lock();

    //configure PDP context with QICSGP (vodafone APN)
    _parser->send("AT+QICSGP=1,1,\"%s\"",apn);
    rtos::ThisThread::sleep_for(300ms);
    if (!_parser->recv("OK"))
    {
        mutex_unlock();
        status = Q_FAILURE;
    }

    _parser->set_timeout(5000);
    for (int i = 0; i < 10; i++)
    {
        status = 0;
        _parser->send("AT+QCSQ");
        if (!(_parser->recv("+QCSQ: \"NBIoT\"") && _parser->recv("OK")))
        {
            continue;
            status = -1;	
        }
        status = qnwinfo();
        status = creg();
        if (status == 0)
        {
            break;
        }
    }

    mutex_unlock();
    return (status);
}


int QUECTEL_BG77::qnwinfo()
{
    mutex_lock();
    int status = 0;
    _parser->send("AT+QNWINFO");
    if (!(_parser->recv("+QNWINFO: \"NBIoT\"") && _parser->recv("OK")))
    {
        status = Q_FAILURE;	
    }
    mutex_unlock();
    return (status);
}

/* Note: the IMEI can be used to identify ME
*/
int QUECTEL_BG77::imei()
{
    int     status = 0;
    char    imei[15+1];
    int     str_size; 
    mutex_lock();
	_parser->send("AT+CGSN");
    if (_parser->recv("%15[^\n]\nOK\n", imei))
    {
        status = Q_FAILURE;
    }
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::imsi()
{
    int     status = 0;
    char    imsi[16]; //type string without double quotes
    mutex_lock();
	_parser->send("AT+CIMI");
    if (_parser->recv("%15[^\n]\nOK\n", imsi))
    {
        status = Q_FAILURE;
    }
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::query_sim()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+CPIN?");
    if(!(_parser->recv("+CPIN: READY") && _parser->recv("OK")))
    {
        mutex_unlock();
        return Q_FAILURE;
    }
    mutex_unlock();
	return Q_SUCCESS;
}

int QUECTEL_BG77::enter_psm(int mode)
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+QCFGEXT=\"attm2mfeat\"");
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
    _parser->send("AT+CEDRXS=1,5,\"1111\"");
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
	_parser->send("AT+QCFG=\"psm/enter\",%d", mode);
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
     _parser->send("AT+QSCLK = 1");
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::disable_psm()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+CPSMS=0");
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::enable_autoconnect()
{
    int  status = -1;
    mutex_lock();
    _parser->send("AT+COPS=0");
    if(_parser->recv("OK"))
    {
        return Q_FAILURE;
    }
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::auto_zone_update()
{
    int status = 0;
    mutex_lock();
	_parser->send("AT+CTZU=3"); 
	if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::tcpip_startup(const char *apn)
{
    int status = 0;
 
    _parser->send("AT+QGPSCFG=\"priority\",1");
    rtos::ThisThread::sleep_for(100ms);
	if (!_parser->recv("OK"))
    {
        status = -1;	
    }

    if (at() != Q_SUCCESS)
    {
	    status = Q_FAILURE;
    }
    if (cfun(1) != Q_SUCCESS)
    {
        cfun(1);
	    status = Q_FAILURE;
    }
    if (band_config() != QUECTEL_BG77::Q_SUCCESS)
    {
        band_config();
        status = Q_FAILURE;
    }
    if (query_sim() != Q_SUCCESS )
    {
        query_sim();
        status = Q_FAILURE;
    }
    if (csq(apn) != Q_SUCCESS)
    {
        csq(apn);
        status = Q_FAILURE;
    }
    if (activate_pdp() != Q_SUCCESS)
    {
        activate_pdp();
        status = Q_FAILURE;
    }
	return (status);
}

int QUECTEL_BG77::configure_http_server()
{
    int status = 0;
    mutex_lock();
	// _parser->send("AT+QHTTPCFG=\"contextid\",1");
	// if (!_parser->recv("OK"))
	// {
	// 	status = Q_FAILURE;	
	// }
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
		status = Q_FAILURE;
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
		status = Q_FAILURE;	
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::set_http_url(const char *url_m)
{
    int status = 0;
    mutex_lock();
    request_http_header();
    _parser->send("AT+QHTTPURL=%d,80",strlen(url_m));
	if (!_parser->recv("CONNECT"))
	{
		status = Q_FAILURE;	
	}
    _parser->send("%s",url_m);
    if (!_parser->recv("OK"))
    {
        status = Q_FAILURE;	
    }
    mutex_unlock();
	return (status);
}

bool QUECTEL_BG77::send_http_post(const char* http_header, uint8_t *http_body, size_t body_len, const char *stateStr)
{
    mutex_lock();
    _parser->set_timeout(12500);
    int status = 0;
    char isSafeChar[1];
    char contentLength[10];
    sprintf(contentLength, "%d\r\n\r\n", body_len); 

    int totalSize = strlen(http_header) + strlen(contentLength) + body_len; 
    
    _parser->send("AT+QHTTPPOST=%d,20,20", totalSize);
    if (!_parser->recv("CONNECT")) 
	{
		status = Q_FAILURE;
	}
    _parser->write(http_header, strlen(http_header));
    _parser->write(contentLength, strlen(contentLength)); 
       
    char c_buffer[body_len]; 
    memcpy(c_buffer, http_body, body_len);
    _parser->write(c_buffer, body_len);

    if (!_parser->recv("+QHTTPPOST:"))
	{
		status = Q_FAILURE;	
	}
    // get response and wait up to 20s for the HTTP session to close
    _parser->send("AT+QHTTPREAD=5");
    if (!_parser->recv("CONNECT"))
	{
		status = Q_FAILURE;	
	}
    
    char * asset_id;
    asset_id = (char *) malloc(26);
    if (!_parser->scanf("{\"info\":[{\"src\":{\"asset_id\":\"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\"},\"isSafe\":%c", asset_id, 
                                                                asset_id+1, asset_id+2, asset_id+3, asset_id+4, asset_id+5, asset_id+6,
                                                                asset_id+7, asset_id+8, asset_id+9, asset_id+10, asset_id+11,
                                                                asset_id+12, asset_id+13, asset_id+14, asset_id+15, asset_id+16, asset_id+17, asset_id+18,
                                                                asset_id+19, asset_id+20, asset_id+21, asset_id+22, asset_id+23, isSafeChar))
    {
        status = Q_FAILURE; 
    }
    free(asset_id);
    if (!_parser->recv("+QHTTPREAD: 0"))
	{
        status = Q_FAILURE; //if for any reason it fails return that is safe
	}
    
    mutex_unlock();
    if(isSafeChar[0] == 't' || status == -1) //if it failed assume its safe
    {
        return true;
    } 
    else 
    {
        return false;
    }
}



int QUECTEL_BG77::turn_off_module()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QGPSEND");
    if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
    ThisThread::sleep_for(100ms);

    _parser->send("AT+QPOWD");
    if (!_parser->recv("OK"))
    {
	    status = Q_FAILURE;		
    }
    ThisThread::sleep_for(100ms);
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
		status = Q_FAILURE;
	}
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::activate_pdp()
{
    int status = 0;
    char qiBuff[64];
    mutex_lock();
    _parser->set_timeout(1000);
    _parser->send("AT+QIACT?");
    char qibuff[16];
    if(!((_parser->recv("+QIACT: 1,1,1,\"%12s\"", qibuff)) 
        || (_parser->recv("+QIACT: 1,1,1,\"%13s\"", qibuff))
        || (_parser->recv("+QIACT: 1,1,1,\"%14s\"", qibuff))
        || (_parser->recv("OK"))))
    {
        _parser->set_timeout(1000);
        _parser->send("AT+QIACT=1");
        rtos::ThisThread::sleep_for(300ms);
        if (!_parser->recv("OK"))
        {
           status = Q_FAILURE;
        }
    }
    ThisThread::sleep_for(200ms);
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::define_pdp_nbiot()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+CGDCONT=1,\"IP\",\"lpwa.vodafone.iot\"");
    if (!_parser->recv("OK"))
	{
		status = Q_FAILURE;	
	}
    mutex_unlock();
	return (status);
}

char * QUECTEL_BG77::sync_ntp()
{
    mutex_lock();
    int status = 0;
    activate_pdp();
    _parser->flush();
    _parser->set_timeout(60000); //important 
    char * timeBuff;
    timeBuff = (char *) malloc(24); 
    //todo: Check if google ntp is faster? http://time.google.com/ 
    _parser->send("AT+QNTP=1,\"pool.ntp.org\",123,1");
    if (!( _parser->recv("OK") && _parser->scanf("+QNTP: %d,\"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\"", &status, timeBuff, 
        timeBuff+1, timeBuff+2, timeBuff+3, timeBuff+4, timeBuff+5, timeBuff+6, timeBuff+7, timeBuff+8, 
        timeBuff+9, timeBuff+10, timeBuff+11, timeBuff+12, timeBuff+13, timeBuff+14, timeBuff+15, timeBuff+16, 
        timeBuff+17, timeBuff+18, timeBuff+19,timeBuff+20,timeBuff+21)))
    {
        status = Q_FAILURE;
    }
    if (timeBuff[0] != '2' || status == -1)
    {
        _parser->send("AT+QLTS=1");
        if (!(_parser->recv("OK") && _parser->scanf("+QLTS: \"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", timeBuff, timeBuff+1, timeBuff+2, timeBuff+3, timeBuff+4, 
                            timeBuff+5, timeBuff+6, timeBuff+7, timeBuff+8, timeBuff+9, timeBuff+10, timeBuff+11, timeBuff+12, timeBuff+13, timeBuff+14, 
                            timeBuff+15, timeBuff+16, timeBuff+17, timeBuff+18, timeBuff+19)))
        {
            sprintf (timeBuff,"%s", "2021/10/18,15:44:34+08");
        }
    }
    /** Format the time 
     */
    timeBuff[4] = 45;
    timeBuff[7] = 45;
    timeBuff[10] = 84;
    timeBuff[19] = 90; //I need the first 20 bytes

    mutex_unlock();
    return timeBuff;
}

int QUECTEL_BG77::parse_latlon(float &lon, float &lat)
{
    int status = 0;
    mutex_lock();
    _parser->set_timeout(5000);
    _parser->send("AT+QGPSCFG=\"priority\",0");
	if (!_parser->recv("OK"))
    {
       status = Q_FAILURE;
    }
    _parser->send("AT+QGPSCFG=\"gpsnmeatype\",31");
    if (!_parser->recv("OK"))
    {
        status = Q_FAILURE;
    }
    _parser->send("AT+QGPSCFG=\"nmeasrc\",1");
    if (!_parser->recv("OK"))
    {
        status = Q_FAILURE;
    }
    _parser->send("AT+QGPS=1");
    if (!_parser->recv("OK"))
    {
        _parser->send("AT+QGPS=1"); //retry?!
        status = Q_FAILURE;
    }
    char utc[12];
    char cog[7];
    char date[8];
    float hdop, altitude, spkm, spkn;
    int fix, nsat, err;
    float latt,lonn;
   
    for (int i = 0; i < 6; i++)
    {
        _parser->send("AT+QGPSLOC=2");
        if((_parser->scanf("+QGPSLOC: %10s,%f,%f,%f,%f,%d,%4s,%f,%f,%6s,%d", 
                            utc, &latt, &lonn, &hdop, &altitude, &fix,
                            cog,&spkm, &spkn, date, &nsat) && _parser->recv("OK")))
        {
            lat = latt;
            lon = lonn;
            break;
        }
    }
    
    //todo: xtra disabled?
    _parser->send("AT+QGPSXTRA=0");
    if (!_parser->recv("OK"))
    {
        status = Q_FAILURE;
    }
    _parser->send("AT+QGPSEND");
    if (!_parser->recv("OK"))
    {
       status = Q_FAILURE;
    }
    mutex_unlock();
	return status;
}

int QUECTEL_BG77::query_satellite_system()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QGPSCFG=\"gnssconfig\"");
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::nmea_configuration()
{
    int status = 0;
    mutex_lock();
    _parser->send("AT+QGPSCFG=\"gpsnmeatype\"");
    _parser->send("AT+QGPSCFG=\"gnssconfig\"");
    mutex_unlock();
	return (status);
}

int QUECTEL_BG77::enable_xtra()
{
    int status = 0;
    mutex_lock();
    _parser->set_timeout(5000);
    _parser->send("AT+QGPSXTRA?");
	if (!_parser->recv("OK"))
    {
        _parser->send("AT+QGPSXTRA=1");
        if (!_parser->recv("OK"))
        {
            status = Q_FAILURE;	
        }
    }	
    _parser->send("AT+QGPSCFG=\"xtra_info\"");
    if (!_parser->recv("OK"))
    {
        status = Q_FAILURE;	
    }

    _parser->send("AT+QGPSCFG=\"xtra_download\",1");
    rtos::ThisThread::sleep_for(1s);
	if (!_parser->recv("OK"))
    {
        status = Q_FAILURE;
    }
    
    mutex_unlock();
	return (status);
}