/*
 * scs.h
 *
 *  Created on: 21 באפר 2016
 *      Author: USER1
 */

#ifndef SCS_H_
#define SCS_H_

#include <hal/boolean.h>
#include "trxvu.h"
#include "OurTime.h"

#include "Global.h"
#include "main.h"
#define NUMBER_OF_APID 5
#define HIGHEST_APID 16

#define APID_EPS 8
#define APID_COM 9
#define APID_CDMS 10
#define APID_ADCS 11
#define APID_MNLP 16

#define TC_VERIFICATION_SERVICE 1

#define TC_ACCEPT_SERVICE_SUCCESS 1
#define TC_ACCEPT_SERVICE_FAILURE 2
#define TC_EXEC_START_SUCCESS 3
#define TC_EXEC_START_FAILURE 4
#define TC_EXEC_COMPLETE_SUCCESS 7
#define TC_EXEC_COMPLETE_FAILURE 8

#define NO_ERR -1
#define ERR_INCORRECT_CHKSM 2
#define ERR_ILEGAL_DATA 5

extern unsigned int ssc[HIGHEST_APID+1];

typedef struct ccsds_packet
{
	unsigned char* data;
	unsigned int len;
	unsigned int apid;
	unsigned int srvc_type;
	unsigned int srvc_subtype;
	unsigned char c_time[5];
} ccsds_packet;

typedef struct rcvd_packet
{
	unsigned char* data;
	unsigned int len;
	unsigned int apid;
	unsigned int ssc;
	unsigned int srvc_type;
	unsigned int srvc_subtype;
	Boolean isvalidcrc;
} rcvd_packet;

short calc_crc( unsigned char* indata, int len );
unsigned int Crc(unsigned char Data, unsigned short Syndrome);
int send_SCS_pct(ccsds_packet pct_dat);//sends data as per SCS specification
void update_time(unsigned char time[5]);//loads current time into a char array
void parse_comm(rcvd_packet *pct, unsigned char in[]);
void tc_verification_report(rcvd_packet decode,unsigned char type, unsigned int clause,unsigned char in[]);
void ax25send(unsigned char* in,unsigned  int len);



#endif /* SCS_H_ */
