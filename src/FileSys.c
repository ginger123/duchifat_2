#include "main.h"

#define ENABLE_MAIN_TRACES 1
#if ENABLE_MAIN_TRACES
#define MAIN_TRACE_INFO			TRACE_INFO
#define MAIN_TRACE_DEBUG		TRACE_DEBUG
#define MAIN_TRACE_WARNING		TRACE_WARNING
#define MAIN_TRACE_ERROR		TRACE_ERROR
#define MAIN_TRACE_FATAL		TRACE_FATAL
#else
#define MAIN_TRACE_INFO(...)	{ }
#define MAIN_TRACE_DEBUG(...)	{ }
#define MAIN_TRACE_WARNING(...)	{ }
#define MAIN_TRACE_ERROR		TRACE_ERROR
#define MAIN_TRACE_FATAL		TRACE_FATAL
#endif

void InitializeFS()
{
	int ret;

	hcc_mem_init(); /* Initialize the memory to be used by filesystem */

	ret = fs_init(); /* Initialize the filesystem */
	ASSERT( (ret == F_NO_ERROR ), "fs_init pb: %d\n\r", ret);

	ret = f_enterFS(); /* Register this task with filesystem */

	ASSERT( (ret == F_NO_ERROR ), "f_enterFS pb: %d\n\r", ret);

	ret = f_initvolume( 0, atmel_mcipdc_initfunc, 0 ); /* Initialize volID as safe */

	ret = f_format( 0, F_FAT32_MEDIA ); /* Format the filesystem */

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
	update_time(time);
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
	int i=0;
	file = f_open( Filename, "r" ); /* open file for reading, which is always safe */
	ASSERT( ( file ), "f_open pb: %d\n\r", f_getlasterror() ); /* if file pointer is NULL, get the error */
	for(; i<index+1;i++)
	{
		br = f_read( ToWrite, 1, _BUFF_SIZE, file );
	}
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

int AllinAll()
{

	int i;
	char br[28];
	printf("starting SD test\n");
	char filename[]={"test_file"};

	char ToWrite_a[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
	char ToWrite_b[] = {0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E};
	int _BUFF_SIZE = 14;
	int ToDel[] = {0};
	//InitializeFS();
	printf("Initialize File system SD test\n");

	WritewithEpochtime(filename, 0, ToWrite_a, _BUFF_SIZE);

	vTaskDelay(5000 / portTICK_RATE_MS);

	WritewithEpochtime(filename, 0, ToWrite_b, _BUFF_SIZE);

	//FileWrite(filename, 0, ToWrite_b, _BUFF_SIZE);


	//delete_packets_from_file(filename, ToDel, _BUFF_SIZE);

	FileRead(filename,br, _BUFF_SIZE+5);

	for (i=0;i<_BUFF_SIZE;i++)
	{
		printf("%x ",(int)br[i]);
	}
	printf("\n");
	FileRead(filename,br, _BUFF_SIZE+5);

	for (i=0;i<_BUFF_SIZE;i++)
	{
		printf("%x ",(int)br[i]);
	}
	printf("\n");

	//DeInitializeFS(0);
	//DEMO_SD_TRACE_INFO( "SD Card (%d) de-initialization completed!\n\r", volID );
	return 0;
}

int AllinAll_b()
{

	int i;
	char br[28];
	printf("starting SD test\n");
	char filename[]={"test_file"};

	char ToWrite_a[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
	char ToWrite_b[] = {0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E};
	int _BUFF_SIZE = 14;
	int ToDel[] = {0};
	//InitializeFS();
	printf("Initialize File system SD test\n");

	//FileWrite(filename, 0, ToWrite_a, _BUFF_SIZE);
	//FileWrite(filename, 0, ToWrite_b, _BUFF_SIZE);
	printf("Write to SD test\n");

	//delete_packets_from_file(filename, ToDel, _BUFF_SIZE);

	FileRead(filename,br, _BUFF_SIZE);

	for (i=0;i<_BUFF_SIZE;i++)
	{
		printf("%x ",(int)br[i]);
	}
	printf("\n");
	//DeInitializeFS(0);
	//DEMO_SD_TRACE_INFO( "SD Card (%d) de-initialization completed!\n\r", volID );
	return 0;
}

int AiAaD_Fulltest()
{
	int i;
	char br[20],br_2[20];
	printf("starting SD test\n");
	char filename[]={"test_file"};
	char filename2[]={"test_file2"};
	char ToWrite[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
	int _BUFF_SIZE = 14;
	int ToDel[] = {0};

	InitializeFS();
	printf("Initialize File system SD test\n");

	FileWrite(filename, 0, ToWrite, _BUFF_SIZE);

	FileWrite(filename2, 0, ToWrite, _BUFF_SIZE);
	printf("Write to SD test\n");

	FileRead(filename,br, _BUFF_SIZE);
	FileRead(filename2,br_2, _BUFF_SIZE);

	delete_packets_from_file(filename, ToDel, _BUFF_SIZE);


	//printf("%s\n",br);

	for( i = 0; i < _BUFF_SIZE; i++)
	{
		if(ToWrite[i]!= br[i])
		{
			DeInitializeFS(0);
			//DEMO_SD_TRACE_INFO( "SD Card (%d) de-initialization completed!\n\r", volID );
			return 0;
		}

	}
	DeInitializeFS(0);
	//DEMO_SD_TRACE_INFO( "SD Card (%d) de-initialization completed!\n\r", volID );

	return 0;
}


