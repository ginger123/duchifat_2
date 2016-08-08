/*
 * scs.c
 *
 *  Created on: 21 באפר 2016
 *      Author: USER1
 */


#include "main.h"

unsigned int ssc[HIGHEST_APID+1];//to keep track of ssc for each apid
unsigned char frame_count;

unsigned char is_sending=0;


unsigned int Crc(unsigned char Data, unsigned short Syndrome)
{
	int i;
	for (i=0; i<8; i++) {
		if ((Data & 0x80) ^ ((Syndrome & 0x8000) >> 8))
		{
			Syndrome = ((Syndrome << 1) ^ 0x1021) & 0xFFFF;
		}
		else
		{
			Syndrome = (Syndrome << 1) & 0xFFFF;
		}
		Data = Data << 1;

	}
	return (Syndrome);
}


short calc_crc( unsigned char* indata, int len )
{
	int j;
	unsigned int Chk; /* CRC syndrome */
	Chk = 0xFFFF; /* Reset syndrome to all ones */
	for (j=0; j<len; j++) {
		Chk = Crc(indata[j], Chk); /* Unoptimized CRC */
	}
	return Chk;
}


int send_SCS_pct(ccsds_packet pct_dat)
{
	while(is_sending==1) vTaskDelay(100);
	is_sending=1;

	// to review format of the below editing reference SCS documentation under ccsds telemetry packet
	int retval=0;
	unsigned char* ret;
	unsigned int i;
	short chksm;
	FRAM_read(ssc,SSC_ADDR,(HIGHEST_APID+1)*4);
	if(pct_dat.apid >= 1<<9)
	{
		printf("invalid apid");
		return -1;
	}
	if(pct_dat.len>(1<<11))
	{
		printf("invalid packet length");
		return -1;
	}
	ret = ( unsigned char*)calloc(pct_dat.len+16,sizeof(char));

	ret[0]= 1<<3; //version number
	ret[0] = ret[0] | pct_dat.apid>>8;//apid
	ret[1]= (char)pct_dat.apid;//apid

	ret[2]= 3<<6;//grouping flags
	ret[2]|= (ssc[pct_dat.apid]>>8) &0xff;//source sequence count
	ret[3]= ssc[pct_dat.apid] &0xff;//source sequence count
	ret[4]=(pct_dat.len + 9)>>8;//length
	ret[5]=(pct_dat.len + 9);//length
	ret[6]= 1<<4;//spare
	ret[7]= pct_dat.srvc_type;//service type
	ret[8]= pct_dat.srvc_subtype;//service type
	ret[9]=  pct_dat.c_time[0];//absolute time
	ret[10]= pct_dat.c_time[1];//absolute time
	ret[11]= pct_dat.c_time[2];//absolute time
	ret[12]= pct_dat.c_time[3];//absolute time
	ret[13]= pct_dat.c_time[4];//absolute time

	//start loading data
	for(i=0;i<pct_dat.len;i++)
	{
		ret[i+14]= pct_dat.data[i];
	}
	//calculate checksum
	chksm= calc_crc(ret,pct_dat.len+14);
	ret[14+i++]=chksm>>8;
	ret[14+i]=chksm;

	ax25send(ret,pct_dat.len+16);

	ssc[pct_dat.apid]++;
	FRAM_write(ssc,SSC_ADDR,(HIGHEST_APID+1)*4);
	free(ret);
	is_sending=0;
	return retval;
}

void ax25send(unsigned char* in, unsigned int len)
{
	int retval;
	unsigned int i=0;
	unsigned char* ret = (unsigned char*)calloc(len+5,sizeof(char));
	FRAM_read(&frame_count,FRAME_COUNT_ADDR,1);
	ret[0]=0x00;// version number+ virtual channel id+ spare
	ret[1]=frame_count;//master count
	ret[2]=frame_count;//virtual channel count(same because we only use 1 channel)
	ret[3]=0x00 ;//first header pointer
	for(i=0;i<len;i++)
	{
		ret[i+4]=in[i];
	}
	ret[len+4]= (0x00 | ( 0x03&tc_count));//frame status (containing time flag spare and tc count




	//sending ret
	retval=TRX_sendFrame(ret, len+5); // first attempt at sending
	while(retval==-1) // try while it didn't succeed
	{
		retval=TRX_sendFrame(ret, len+5);
		vTaskDelay(50 / portTICK_RATE_MS);
	}

	//	printf("PACKET:\r\n");
	//	for(i=0;i<len+5;i++)
	//	{
	//		printf("%02x ",ret[i]);
	//	}
	//	printf("\r\nEND PACKET\r\n");
	frame_count++;
	FRAM_write(&frame_count,FRAME_COUNT_ADDR,1);
	free(ret);
}


void update_time(unsigned char time[5])
{
	unsigned long t;
	Time_getUnixEpoch(&t);
	t=t-30*365*24*3600;
	t=t-24*3600*7;
	time[0]=t>>24;
	time[1]=t>>16;
	time[2]=t>>8;
	time[3]=t;
	time[4]=0;

}


void parse_comm(rcvd_packet *pct, unsigned char in[])
{
	int i=0;
	int len;
	short crc;
	unsigned char* data;
	len=in[4]<<8;
	len+=in[5]+1-5;//length of only the data part
	pct->len=len;
	data=calloc(pct->len,sizeof(unsigned char));
	for(;i<len;i++)
	{
		data[i]=in[i+9];//offset of 9 bytes from beginning
	}
	pct->data=data;
	pct->apid= ((in[0]& 0x07)<<8) +in[1];
	pct->srvc_type=in[7];
	pct->srvc_subtype=in[8];
	pct->ssc=((in[2]& 0x3f)<<8) + in[3];

	pct->isvalidcrc=TRUE;
	crc = calc_crc(in, len+11);
	if(crc!=0)
	{
		printf("checksum of packet failed. aborting command\n");
		pct->isvalidcrc=FALSE;
		tc_verification_report(*pct,TC_ACCEPT_SERVICE_FAILURE,ERR_INCORRECT_CHKSM,in);
		return;

	}
		//by this point the packet has been accepted so sending acceptance report:
	printf("command accepted\n");
	tc_verification_report(*pct,TC_ACCEPT_SERVICE_SUCCESS,NO_ERR,in);
}

void tc_verification_report(rcvd_packet decode,unsigned char type,unsigned int clause, unsigned char in[])
{
	ccsds_packet report;
	unsigned char report_dat[6] = {in[0],in[1],in[2],in[3],0,0};
	update_time(report.c_time);
	report.apid=decode.apid;
	report.data=report_dat;
	report.srvc_type=TC_VERIFICATION_SERVICE;
	report.srvc_subtype=type;
	if(type%2==1)
	{

	report.len=6;
	report_dat[4]=0;
	report_dat[5]=clause;

	}
	else {
		report.len=4;
	}
	send_SCS_pct(report);
}



