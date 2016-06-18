#ifndef FILESYS_H_
#define FILESYS_H_

#include <hal/boolean.h>

void InitializeFS();

void DeInitializeFS();

void FileWrite(char Filename[], Boolean Ifsafe, char Data[], int _BUFF_SIZE);

void WritewithEpochtime(char Filename[], Boolean Ifsafe, char Data[], int _BUFF_SIZE);

void FileRead(char Filename[],char ToWrite[], int _BUFF_SIZE);

void delete_packets_from_file(char Filename[], int ToDel[],int line_size);

int AiAaD_Fulltest();

int AllinAll();

int AllinAll_b();

#endif
