#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

#define DURATION 24000  //100 valori per M1 e A1, 100 per M2 e A2
#define NUMFILES 1000
#define FILEBASE "MEAS"
#define PORTNAME "COM13"

//typedef DWORD *LPDWORD;

HANDLE PortConfig(char *portName); //Il tipo HANDLE è definito in windows.h

int main(int argc, char *argv[])
{
    HANDLE my_Serial;
    char buffer[500], buffer2[200];
    char portName[] = {PORTNAME};  /* Set COM Port Name here*/
    //LPDWORD dwBytesRead; //LPDWORD è un puntatore a DWORD quindi in teoria sarebbero giusti i 2 sotto ma lascio la 3
    //DWORD *dwBytesRead;
    DWORD dwBytesRead; //la seriale è a 32 bit, OK, se è a 16 basta una word...
    int i,j,n, t;
    int cont;
    char filename[30];
    FILE *fout;
    int first;
    int rep = DURATION;

    if (argc == 2)
    {
        sscanf(argv[1], "%d", &rep);
    }

    my_Serial = PortConfig(portName);
    //getchar();

    for (cont = 0; cont<NUMFILES; cont++)
    {
        sprintf(filename,"%s_%03d.txt", FILEBASE, cont);
        fout = fopen(filename, "w");
        if (fout == NULL)
        {
            printf("Error opening %s\n", filename);
            getchar();
        }

       printf("\n\a Ready for acquiring file %d (%s). Press ENTER to continue...", cont, filename);
       getchar();
        first = 1;
        buffer[0]='\0';
        buffer2[0]='\0';
        j = 0;
        t = 0;

        //CLEAR BUFFER!
        PurgeComm(my_Serial, PURGE_RXABORT);
        PurgeComm(my_Serial, PURGE_RXCLEAR);

        for (i=0; i<20; i++)
        {
            if(!ReadFile(my_Serial, &buffer, sizeof(buffer), &dwBytesRead, NULL))
            {
                /* Error occured. Inform user */
                fprintf(stdout,"Error occured during read\n");
            }
        }

       // printf("%d", t);

        while (t<rep)
        {
            if(!ReadFile(my_Serial, &buffer, sizeof(buffer), &dwBytesRead, NULL))
            {
                /* Error occured. Inform user */
                fprintf(stdout, "Error occured during read\n");
            }

            n=dwBytesRead;
            for (i=0; i<n && t<rep; i++)
            {
                buffer2[j++] = buffer[i];
                if (buffer[i]=='\n')
                {
                    if (first)
                        first = 0;
                    else
                    {
                        buffer2[j]='\0';
                        fprintf(fout, "%s", buffer2);
                        fprintf(stdout, "%s", buffer2);

                        //fprintf(stdout, "File %s: %s", filename, buffer2);
                        t++;
                       // printf("\r%d",t);

                    }
                    j = 0;
                }
            }

        } //fine while t<rep

       // printf("\n");
        fclose(fout);
	} //fine for cont = 0

    return 0;
}


HANDLE PortConfig(char *portName)
{
	HANDLE hSerial = NULL;
    //int com = -1;
    COMMTIMEOUTS timeouts={0};
	/* Declare and Init the DCB */
    DCB dcbSerialParams = {0};
 	//LPCWSTR portNamemod[30];
	char portNamemod[30]; //BERT: ho messo char xk sprintf vuole char, const char, ...). LPCWSTR= puntatore a WCHAR


	sprintf(portNamemod, "\\\\.\\%s", portName);
    printf("\nDefault Parameters used:\nPortName=%s\nBaudRate=115200 \nByteSize=8\nStopBits=ONESTOPBIT\nParity=NOPARITY\n\n",portName);
    printf("\nReady to acquire %d samples...\n", NUMFILES);

    /* Open session to COM Port */
    hSerial = CreateFileA(portNamemod, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if(hSerial==INVALID_HANDLE_VALUE)
    {
       if(GetLastError()==ERROR_FILE_NOT_FOUND)
       {
            //serial port does not exist. Inform user.
            printf("Error: Serial port does not exist. Please check the port name and try again\n");
       }
       /* Error. Inform user */
       printf("Error!\n");
    }

    /* Set Timeouts */
    timeouts.ReadIntervalTimeout=1;
    timeouts.ReadTotalTimeoutConstant=1;
    timeouts.ReadTotalTimeoutMultiplier=1;
    timeouts.WriteTotalTimeoutConstant=50;
    timeouts.WriteTotalTimeoutMultiplier=10;

    if(!SetCommTimeouts(hSerial, &timeouts))
    {
        /* Error occured. Inform user */
        printf("Error!\n");
    }

    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        printf("Error Getting State\n");
    }

    //fill up params
    dcbSerialParams.BaudRate=230400;
    dcbSerialParams.ByteSize=8;
    dcbSerialParams.StopBits=ONESTOPBIT;
    dcbSerialParams.Parity=NOPARITY;

     if(!SetCommState(hSerial, &dcbSerialParams))
    {
        /* Error occured. Inform user */
        printf("Error getting state\n");
    }

    return hSerial;
}
