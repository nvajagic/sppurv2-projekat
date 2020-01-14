#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>


#define BUF_LEN 80
#define MAX_MIC 9

char* itoa(int value, char* result, int base);

int main()
{
	FILE* fp;
    int file_desc;
    int ret_val;
    char tmp[BUF_LEN];
	char mic[MAX_MIC];
    int num_of_mic;
	char pom[1];
	char rezim[5];
	int rez;

    /* Open dummy file. */
	fp = fopen("/dev/test", "r+");
	if(fp == NULL) {
		//printf("Error, file not opened\n");
		perror("Error");
		return -1;
	}

	/*file_desc = open("/SPPURV2_projekat/test.txt", O_RDWR);
    if(file_desc < 0)
    {
		perror("Error");
        //printf("Error, 'test' not opened\n");
        return -1;
    }*/

	do {
        printf("Unesite koliko mikrofona ste povezali(MAX = %d):\n", MAX_MIC);
        scanf("%d", &num_of_mic);
	} while(num_of_mic > MAX_MIC);
	

    int mic_array[num_of_mic];
    printf("Unesite na koje pinove ste povezali mikrofone (posle svakog pina enter): ");
    int i;
    for(i = 0; i < num_of_mic; i++) {
        scanf("%d", &mic_array[i]);
        mic[i] = '_' + mic_array[i];
        //printf("%s", mic);
    }
	//mic[i] = '/0';

	itoa(num_of_mic, pom, 10);
	strcpy(tmp, "0,");
	strcat(tmp, pom);
	strcat(tmp, ",");
	strcat(tmp, mic);

	printf("Komanda izgleda: %s\n", tmp);
	
    /* Write to dummy file. */
    //ret_val = write(file_desc, tmp, sizeof(tmp));
	fwrite(tmp, sizeof(char), sizeof(tmp), fp);

	do {

		printf("Unesite rezim rada u kom zelite da radite(0-impuls, 1-konstantan izvor):\n");	
		scanf("%d", &rez);	

	} while(rez > 1 || rez < 0);

	itoa(rez, pom, 10);
	strcpy(rezim, "1,");
	strcat(rezim, pom);

	printf("Rezim: %s\n", rezim);
	frwite(rezim, sizeof(char), sizeof(rezim), fp);

    //printf("%s", tmp);

    /* Close dummy file. */
	fclose(fp);
    //close(file_desc);
    
    return 0;
}

char* itoa(int value, char* result, int base) {  //nemam fje u biblioteci pa sam morao ovako
		// check that the base if valid
		if (base < 2 || base > 36) { *result = '\0'; return result; }

		char* ptr = result, *ptr1 = result, tmp_char;
		int tmp_value;

		do {
			tmp_value = value;
			value /= base;
			*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
		} while ( value );

		// Apply negative sign
		if (tmp_value < 0) *ptr++ = '-';
		*ptr-- = '\0';
		while(ptr1 < ptr) {
			tmp_char = *ptr;
			*ptr--= *ptr1;
			*ptr1++ = tmp_char;
		}
		return result;
	}