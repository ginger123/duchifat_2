#include "main.h"

void InitializeFS()
{
	int ret;
	char HK_packets[] = {"HK_packets"};
	char ADC_comm[] = {"adcs_file"};
	char ADC_tlm[] = {"adcs_tlm_file"};
	char mnlp_file[] ={"mnlp"};
	char wod_file[] = {"wod"};
	F_FILE *file;

	hcc_mem_init(); /* Initialize the memory to be used by filesystem */

	ret = fs_init(); /* Initialize the filesystem */
	ASSERT( (ret == F_NO_ERROR ), "fs_init pb: %d\n\r", ret);

	ret = f_enterFS(); /* Register this task with filesystem */

	ASSERT( (ret == F_NO_ERROR ), "f_enterFS pb: %d\n\r", ret);

	ret = f_initvolume( 0, atmel_mcipdc_initfunc, 0 ); /* Initialize volID as safe */

	//ret = f_format( 0, F_FAT32_MEDIA ); /* Format the filesystem */

	file = f_open( HK_packets, "a" );
	ret = f_close( file ); /* data is also considered safe when file is closed */
	file = f_open( ADC_comm, "a" );
	ret = f_close( file ); /* data is also considered safe when file is closed */
	file = f_open( ADC_tlm, "a" );
	ret = f_close( file ); /* data is also considered safe when file is closed */
	file = f_open( mnlp_file, "a" );
	ret = f_close( file ); /* data is also considered safe when file is closed */
	file = f_open( wod_file, "a" );
	ret = f_close( file ); /* data is also considered safe when file is closed */
	//f_releaseFS(); /* release this task from the filesystem */

}

void DeInitializeFS()
{
	f_delvolume( 0 ); /* delete the volID */

	f_releaseFS(); /* release this task from the filesystem */

	fs_delete(); /* delete the filesystem */

	hcc_mem_delete(); /* free the memory used by the filesystem */

	//DEMO_SD_TRACE_INFO( "SD Card (%d) de-initialization completed!\n\r", volID );
}

void FileWrite(char Filename[], Boolean Ifsafe, char Data[], int _BUFF_SIZE)
{

	int bw;
	F_FILE *file;
	int ret;

	//ret = f_enterFS(); /* Register this task with filesystem */

	//ASSERT( (ret == F_NO_ERROR ), "f_enterFS pb: %d\n\r", ret);


	if(Ifsafe)
	{
		file = f_open( Filename, "a" ); /* open file for writing in safe mode */
	}
	else
	{
		file = f_open_nonsafe( Filename, "a" ); /* open file for writing in nonsafe mode */
	}

	ASSERT( (file), "f_open pb: %d\n\r", f_getlasterror() ); /* if file pointer is NULL, get the error */

	bw = f_write( Data, 1, _BUFF_SIZE, file );
	ASSERT( ( _BUFF_SIZE == bw ),  "f_write pb: %d\n\r", f_getlasterror() ); /* if bytes to write doesn't equal bytes written, get the error */

	f_flush( file ); /* only after flushing can data be considered safe */


	ret = f_close( file ); /* data is also considered safe when file is closed */
	ASSERT( (ret == F_NO_ERROR ), "f_close pb: %d\n\r", ret);

	//f_releaseFS(); /* release this task from the filesystem */

	//DEMO_SD_TRACE_INFO( "SD Card (%d) write operation completed!\n\r", volID );

}

void WritewithEpochtime(char Filename[], Boolean Ifsafe, char Data[], int _BUFF_SIZE)
{
	unsigned char time[5];
	unsigned long t;
	Time_getUnixEpoch(&t);
	convert_time_array(t,time);
	//update_time(time);
	FileWrite( Filename,  Ifsafe, (char*)time,  5);
	FileWrite( Filename,  Ifsafe,  Data,  _BUFF_SIZE);
}


void FileRead(char Filename[],char ToWrite[], int _BUFF_SIZE)
{
	int br;
	F_FILE *file;
	int ret=0;

	//ret = f_enterFS(); /* Register this task with filesystem */

	ASSERT( (ret == F_NO_ERROR ), "f_enterFS pb: %d\n\r", ret);


	file = f_open( Filename, "r" ); /* open file for reading, which is always safe */
	ASSERT( ( file ), "f_open pb: %d\n\r", f_getlasterror() ); /* if file pointer is NULL, get the error */

	br = f_read( ToWrite, 1, _BUFF_SIZE, file );
	ASSERT( ( _BUFF_SIZE == br ),  "f_read pb: %d\n\r", f_getlasterror() ); /* if bytes to read doesn't equal bytes read, get the error */

	ret = f_close( file );
	ASSERT( ( ret == F_NO_ERROR ), "f_close pb: %d\n\r", ret );

	//f_releaseFS(); /* release this task from the filesystem */
	//DEMO_SD_TRACE_INFO( "SD Card (%d) read operation completed!\n\r", volID );
}

void FileReadIndex(char Filename[],char ToWrite[], int _BUFF_SIZE, int index) //file read only in index spot
{
	int br;
	F_FILE *file;
	int ret=0;
	//int i=0;
	file = f_open( Filename, "r" ); /* open file for reading, which is always safe */
	ASSERT( ( file ), "f_open pb: %d\n\r", f_getlasterror() ); /* if file pointer is NULL, get the error */
	//for(; i<index+1;i++)
	//{
	//	br = f_read( ToWrite, 1, _BUFF_SIZE, file );
	//}
	f_seek( file, index*_BUFF_SIZE, SEEK_SET );
	br = f_read( ToWrite, 1, _BUFF_SIZE, file );
	ASSERT( ( _BUFF_SIZE == br ),  "f_read pb: %d\n\r", f_getlasterror() ); /* if bytes to read doesn't equal bytes read, get the error */

	ret = f_close( file );
	ASSERT( ( ret == F_NO_ERROR ), "f_close pb: %d\n\r", ret );

	//DEMO_SD_TRACE_INFO( "SD Card (%d) read operation completed!\n\r", volID );
}

void delete_packets_from_file(char Filename[], int ToDel[],int line_size)
{
	char temp[] = "tmp.txt";
	F_FILE* inFile;

	char line [line_size]; // maybe you have to user better value here
	int lineCount = 0;
	int ret=0;
	int i = 0;

	inFile = f_open(Filename, "r");
	F_FILE* outFile = f_open(temp, "w+");

	if( inFile == NULL )
	{
	    printf("Open Error");
	}


	do
	{
		ret = f_read( line, 1 ,line_size, inFile);
		if(ret == 0)
			continue;

	    if( lineCount != ToDel[i] )
	    {
	    	printf("got here\n");

	    	f_write( line, 1, line_size, outFile );
	    }
	    else
	    {
	    	printf("delete packet\n");
	    	i++;
	    }

	    lineCount++;
	}while( ret != 0 );


	f_close(inFile);
	f_close(outFile);

	// possible you have to remove old file here before
	f_delete(Filename);
	f_rename(temp,Filename);
	//remove(Filename);
	//if( !f_rename(temp,Filename) )
	//{
	   //printf("Rename Error");
	//}
}

int find_number_of_packets(char Filename[],int linesize,unsigned long time_a,unsigned long time_b,int *start_idx)
{
	// loop over packets in file - for each line: a. convert time to long int, b. check if in range

	char temp[linesize];
	F_FILE *file2;
	*start_idx = 0;
	unsigned long curr_time=0;
	unsigned long satt_time;
	int num=0;
	int ret_val;
	file2 = f_open( Filename, "r" ); // open file for reading, which is always safe
	ASSERT( ( file2 ), "f_open pb: %d\n\r", f_getlasterror() ); // if file pointer is NULL, get the error
	Time_getUnixEpoch(&satt_time);
	if(time_b>satt_time) time_b= satt_time-10;
	if(time_a>satt_time-10) return 0;//something went terribly wrong

	while( curr_time < time_b) // do this for every char in the line
	{
		ret_val = f_read( temp, 1, linesize , file2 );
		if(linesize!=ret_val)
		{
			//printf("final time is greater then last packet time!\n");
			break;//edited this so function returns if end of file is reached #BigFuckingIf
		}
		curr_time = convert_epoctime(temp);
		printf("curr time is %lu last time is %lu\n",curr_time,time_b);
		if( (curr_time > time_a) && (curr_time< time_b))
		{
			num++;
		}
		else
		{
			(*start_idx)++;
		}

	}
	f_close(file2);
	return num;
}


void AllinAll()
{

	int i,j;

	printf("starting SD test\n");
	char filename[]={"test_file"};
	int _BUFF_SIZE = 10;
	char ToWrite_a[] = {0x01,0x02,0x03,0x01,0x02,0x03,0x01,0x02,0x03,0x00};
	char ToWrite_b[] = {0x04,0x05,0x06,0x04,0x05,0x06,0x04,0x05,0x06,0x00};
	char ToWrite_c[] = {0x07,0x08,0x09,0x07,0x08,0x09,0x07,0x08,0x09,0x00};
	//char ToWrite_d[] = {0x08,0x09,0x0A,0x08,0x09,0x0A,0x08,0x09,0x0A,0x00};
	//char ToWrite_e[] = {0x0B,0x0C,0x0D,0x0B,0x0C,0x0D,0x0B,0x0C,0x0D,0x00};
	char ToRead[_BUFF_SIZE+5];



	int num_packets;
	int start_idx;
	unsigned long t_start,t_b,t_c,t_finish;

	printf("Initialize File system SD test\n");
	Time_getUnixEpoch(&t_start);
	t_start=t_start-30*365*24*3600;
	t_start=t_start-24*3600*7;

	for (i=0;i<4;i++)
	{
		printf("%d\n",i);
		WritewithEpochtime(filename, 0, ToWrite_a, _BUFF_SIZE);
		vTaskDelay(3000 / portTICK_RATE_MS);
	}
	Time_getUnixEpoch(&t_b);
	t_b=t_b-30*365*24*3600;
	t_b=t_b-24*3600*7;
	for (i=0;i<4;i++)
	{
		printf("%d\n",i);
		WritewithEpochtime(filename, 0, ToWrite_b, _BUFF_SIZE);
		vTaskDelay(3000 / portTICK_RATE_MS);
	}
	Time_getUnixEpoch(&t_c);
	t_c=t_c-30*365*24*3600;
	t_c=t_c-24*3600*7;
	for (i=0;i<4;i++)
	{
		printf("%d\n",i);
		WritewithEpochtime(filename, 0, ToWrite_c, _BUFF_SIZE);
		vTaskDelay(3000 / portTICK_RATE_MS);
	}
	Time_getUnixEpoch(&t_finish);
	t_finish=t_finish-30*365*24*3600;
	t_finish=t_finish-24*3600*7;

	num_packets = find_number_of_packets(filename,_BUFF_SIZE+5,t_b,t_c,&start_idx);
	printf("start time is %lu, finish time is %lu, num packets is %d, start idx is %d\n",t_start,t_finish,num_packets,start_idx);



	// print only relevant packets
	for (i=0;i<num_packets;i++)
	{
		FileReadIndex(filename,ToRead,_BUFF_SIZE+5,start_idx+i);
		for (j=0;j<_BUFF_SIZE;j++)
		{
			printf("%0x ",ToRead[j]);
		}
		printf("l\n");
	}


}


