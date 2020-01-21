#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h> /* For getch function */
#include <unistd.h>  /* For getch function */
#include <sys/ioctl.h> /* For ioctl */


#define BUF_LEN 80
#define MAX_MIC 9

char* itoa(int value, char* result, int base);
int getch(void);
int get_n_readable_bytes(int fd);

int main(int argc, char *argv[])
{
    int file_desc;
    int ret_val;
	int num_of_mic;
    char tmp[BUF_LEN];
	char mic[MAX_MIC];
	char hlp[1];
	char mode[5];
	int res;
	int ch = 0;


	if(argc != 3) {
		printf("You didn't input enough number of parameters!\n(./program number_of_microphones pins(separated by space or comma)\n");
		return -1;
	}

    /* Open nod file. */
    file_desc = open("/dev/test", O_RDWR);

    if(file_desc < 0)
    {
        printf("Error, nod not opened\n");
        return -1;
    }
	

	int i = 0, j = 0;
	int help;
	char help_str[3];

	while(i < strlen(argv[2])) {
		help_str[0] = argv[2][i];
		if(argv[2][i + 1] != ',') {
			help_str[1] = argv[2][i + 1];
			help_str[2] = '\0';
			i += 3;
		}
		else {
			help_str[1] = '\0';
			i += 2;
		}

		help = atoi(help_str);
		mic[j] = help + '_';
		j++;
	}

	mic[j] = '\0';

	strcpy(tmp, "0,");
    strcat(tmp, argv[1]);
    strcat(tmp, ",");
    strcat(tmp, mic);

    /* Write to nod file. */
    ret_val = write(file_desc, tmp, BUF_LEN);

input_mode:
	do {
		printf("Input mode (0 - impulse, 1 - constant): ");	
		scanf("%d", &res);	
	} while(res > 1 || res < 0);

	itoa(res, hlp, 10);
	strcpy(mode, "1,");
	strcat(mode, hlp);

	ret_val = write(file_desc, mode, BUF_LEN);

	num_of_mic = atoi(argv[1]);
	int mic_array[num_of_mic];
    
	while(1) {
		ret_val = read(file_desc, tmp, BUF_LEN);
		tmp[ret_val] = '\0';	
		if(mode[2] == '0') {
			printf("%s\n", tmp);
			/*If we have something in keyboard buffer, and if its c or C go to mode change*/
			if(get_n_readable_bytes(fileno(stdin))) {
				ch = getch();
				if(ch == 'C' || ch == 'c')
					goto input_mode;
			}
			sleep(2);
		}
		else {
			j = 0;
			for(i = 1; i < num_of_mic + 1; i++) {
				if(tmp[i] == '1') {
					mic_array[j] = i;
					j++;
				}
			}

			switch(j) {
    			case 0:
        			printf("No sound source\n");
        			break;
    			case 1:
        			printf("Sound is coming from ZONE %d\n", mic_array[0]);
        			break;
    			case 2:
        			printf("Sound is coming from ZONE %d, %d\n", mic_array[0], mic_array[1]);
        			break;
    			case 3:
        			printf("Sound is coming from ZONE %d, %d, %d\n", mic_array[0], mic_array[1], mic_array[2]);
        			break;
    			case 4:
        			printf("Sound is coming from ZONE %d, %d, %d, %d\n", mic_array[0], mic_array[1], mic_array[2], mic_array[3]);
        			break;
    			case 5:
        			printf("Sound is coming from ZONE %d, %d, %d, %d, %d\n", mic_array[0], mic_array[1], mic_array[2], mic_array[3], mic_array[4]);
        		break;
    			case 6:
        			printf("Sound is coming from ZONE %d, %d, %d, %d, %d, %d\n", mic_array[0], mic_array[1], mic_array[2], mic_array[3], mic_array[4], mic_array[5]);
        			break;
    			case 7:
        			printf("Sound is coming from ZONE %d, %d, %d, %d, %d, %d, %d\n", mic_array[0], mic_array[1], mic_array[2], mic_array[3], mic_array[4], mic_array[5], mic_array[6]);
        			break;
    			case 8:
        			printf("Sound is coming from ZONE %d, %d, %d, %d, %d, %d, %d, %d\n", mic_array[0], mic_array[1], mic_array[2], mic_array[3], mic_array[4], mic_array[5], mic_array[6], mic_array[7]);
        			break;
    			case 9:
        			printf("Sound is coming from ZONE %d, %d, %d, %d, %d, %d, %d, %d, %d\n", mic_array[0], mic_array[1], mic_array[2], mic_array[3], mic_array[4], mic_array[5], mic_array[6], mic_array[7], mic_array[8]);
        			break;
			}
			/*If we have something in keyboard buffer, and if its c or C go to mode change*/
			if(get_n_readable_bytes(fileno(stdin))) {
				ch = getch();
				if(ch == 'C' || ch == 'c')
					goto input_mode;
			}
			usleep(100000);
		}

	}
    /* Close dummy file. */
    close(file_desc);
    
    return 0;
}

char* itoa(int value, char* result, int base) {  //No itoa function in library
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

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

int get_n_readable_bytes(int fd) {
    int n = 0;
	system ("/bin/stty raw"); /* Putting terminal in raw mode so we don't need to press enter */
    if (ioctl(fd, FIONREAD, &n) < 0) {
        perror("ioctl failed");
		system ("/bin/stty cooked");
        return 2;
    }
	system ("/bin/stty cooked");
    return n;
}