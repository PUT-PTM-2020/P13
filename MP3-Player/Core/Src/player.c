#include "player.h"

extern nr_utworu
void read_song(){

FRESULT res;
    DIR dir;
    UINT i=0;
    UINT z;

    static FILINFO fno;


		res = f_opendir(&dir, "/");
    	if (res == FR_OK) {
    			lcd_clear ();
    		    		lcd_put_cur(0, 0);
    		    		lcd_send_string("FR_OK");
  	  	  	do{
            		res = f_readdir(&dir, &fno);
            		if (res != FR_OK || fno.fname[0] == 0) {i=1; break;}
            		printf("%s\n", fno.fname);
                	z = strlen(fno.fname);
                	i++;
            	}
            	while(i<=nr_utworu ||(fno.fname[z-1]!='V') || (fno.fname[z-2]!='A')|| (fno.fname[z-3]!='W'));
  	  	  	//||
  	  	  	//(fno.fname[z-1]!='3') || (fno.fname[z-2]!='P')|| (fno.fname[z-3]!='M')
  	  	  		sprintf(utwor,"%s",fno.fname);
  	  	  		nr_utworu=i-1;
  	  	  		if(nr_utworu==0)read_song();
            	}

    			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    			sizeutwor = strlen(utwor);
               	return;
}

void next(){
	HAL_TIM_Base_Stop_IT(&htim4);
	f_close(&file);
	nr_utworu++;
	read_song();
	fresult = f_open(&file, &utwor , FA_READ|FA_OPEN_EXISTING);
	f_read(&file, &buf, BUFSIZE, &bytes_read);
	i=0;
	j=0;
	 lcd_clear ();
	lcd_put_cur(0, 0);
	lcd_send_string(&utwor);
	lcd_put_cur(1, 0);
	lcd_send_string("PLAY");
	 HAL_TIM_Base_Start_IT(&htim4);
}

void prev(){
	HAL_TIM_Base_Stop_IT(&htim4);
	f_close(&file);
	nr_utworu--;
	read_song();
	fresult = f_open(&file, &utwor , FA_READ|FA_OPEN_EXISTING);
	f_read(&file, &buf, BUFSIZE, &bytes_read);
	i=0;
	j=0;
	 lcd_clear ();
	lcd_put_cur(0, 0);
	lcd_send_string(&utwor);
	lcd_put_cur(1, 0);
	lcd_send_string("PLAY");
	HAL_TIM_Base_Start_IT(&htim4);
}
