#ifndef FILESYS_H_
#define FILESYS_H_

#include <hal/boolean.h>

void InitializeFS(int delete);

void DeInitializeFS();

void FileWrite(char Filename[], Boolean Ifsafe, char Data[], int _BUFF_SIZE);

void WritewithEpochtime(char Filename[], Boolean Ifsafe, char Data[], int _BUFF_SIZE);

void FileRead(char Filename[],char ToWrite[], int _BUFF_SIZE);

void delete_packets_from_file(int file_idx, unsigned long t);

int find_number_of_packets(char Filename[],int linesize,unsigned long time_a,unsigned long time_b,int *start_idx);

void FileReadIndex(char Filename[],char ToWrite[], int _BUFF_SIZE, int index);

void print_file(char filename[],int linesize);


// test functions
void AllinAll();

void check_data_file();
#endif
