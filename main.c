#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "gd32vf103v_eval.h"
#include "stdio.h"
#include "string.h"
#include "gd32v_tf_card_if.h"
#include "pwm.h"

////////////////////////// Define functions /////////////////////////////////////////
void Initialize_Project();

void SendToSD(int data);

///////////////////////////// Main function /////////////////////////////////////////
int main(void) {   
    Initialize_Project();
	///////////////////////// Acc & Gyro variables ///////////////////////////////////

	/* The related data structure for the IMU, contains a vector of x, y, z floats*/
    mpu_vector_t Acc;
    float y;
    int on = 0;
    int G=0, R=0, S=0;         // States
    uint16_t _color;

    /////////////////////////////////// SD-Card variables ///////////////////////////////////////// 
    
    /* Create an int for transmiting data to SD-Card & Clock to see when we should send it */
    int redReps = 0;
    int period = 7200000;
    int hepticPeriod = 2000;
    int testFreq = 200;
    uint64_t start_mtime, delta_mtime;
    uint64_t startH_mtime, deltaH_mtime;
    uint64_t test_time_start, test_time_delta;
    
    test_time_start = get_timer_value();
	while(1) {
        // Try multiplying y with 10 or 100 to get a better number to compare with, maybe use convert_to_int ?

        ///////////////////////////////////////////////// Accel & Gyro //////////////////////////////////////////////////////
		
        // Read button pressed (GPIO-pin)
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_6) == SET) {          // if Button is pressed        
            if(on == 0) {
                on = 1; 
            } else {
                on = 0;
            }
            while(gpio_input_bit_get(GPIOA, GPIO_PIN_6) == SET);    // while Button is pressed
        }

        if(on == 0) {
            // On a break 
            start_mtime = get_timer_value();
            if(S == 0){
                //LCD_Clear(1);
                SendToSD(redReps);                                  // Send redReps to SD-Card
                S=1;
                G=0;
                R=0;
            }
            /* Blink diod */
            T1setPWMmotorB(1);
            delay_1ms(200);
            T1setPWMmotorB(0);
            delay_1ms(200);
        } else {
            mpu6500_getAccel(&Acc);                     // Get accelleration data (Note: Blocking read) puts a force vector with 1G = 4096 into x, y, z directions respectively
            y = Acc.y / 16384;                          // Scale to G values   

            test_time_delta = get_timer_value() - test_time_start;

            if (test_time_delta >(SystemCoreClock/4000.0 *testFreq)){
                if(y>0.80) {
                    _color = GREEN;
                    if(G==0) {
                        T1setPWMmotorB(0);
                        // LCD_ShowNum1(10,10,y,8,_color);
                        // LCD_ShowNum1(50,50,redReps,8,WHITE);
                    }
                    G=1;
                    R=0;
                } else if(y<0.60) {
                    _color = RED;
                    if(R==0) {
                        redReps++;
                        T1setPWMmotorB(1);
                        // LCD_ShowNum1(10,10,y,8,_color);
                        // if((redReps%10)==0 && (redReps>0)) {
                        //     startH_mtime = get_timer_value();
                        // }
                    }
                    G=0;
                    R=1;
                } else {
                    T1setPWMmotorB(0);
                }
                test_time_start = get_timer_value();
            }
            // deltaH_mtime = get_timer_value() - startH_mtime;
            // if (deltaH_mtime < (SystemCoreClock/4000.0 *hepticPeriod)){
            //     T1setPWMmotorB(1);
            // } else T1setPWMmotorB(0);
            
            S = 0;    
        }
            
        /////////////////////////////////////////////////// SD-Card //////////////////////////////////////////////////////////// 
        // Count up the time passed in ms
        delta_mtime = get_timer_value() - start_mtime;

        // Send to SD-Card after 2 hours of work
        if (delta_mtime >(SystemCoreClock/4000.0 *period)){
            SendToSD(redReps);      // Send redReps to SD-Card
        }
    };
    
}

////////////////////////// Write functions /////////////////////////////////////////

void Initialize_Project(){
    ////////////////////////////////// Initialize  heptic feedback //////////////////////////////////
    T1powerUpInitPWM(0x3);                  //Starts A0 and A1.
    ////////////////////////////////////// Initialize LCD ///////////////////////////////////////////
    // Lcd_SetType(LCD_INVERTED);
    // Lcd_Init();
    // LCD_Clear(1);
    // LCD_DrawPoint(1,1,1);
    // delay_1ms(100);
	//////////////////////////////////// Initialize Acc  ///////////////////////////////////////////
	/* Initialize pins for I2C */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    /* Initialize the IMU (Notice that MPU6500 is referenced, this is due to the fact that ICM-20600
       ICM-20600 is mostly register compatible with MPU6500, if MPU6500 is used only thing that needs
       to change is MPU6500_WHO_AM_I_ID from 0x11 to 0x70. */

    mpu6500_install(I2C0);
    //////////////////////////////////// Initialize Button  ///////////////////////////////////////////
    rcu_periph_clock_enable(RCU_GPIOA);
    /* This configures the A3 and A4 pins as inputs with internal pull ups enabled */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    
    /* Blink diod */
    T1setPWMmotorB(1);
    delay_1ms(1000);
    T1setPWMmotorB(0);
    delay_1ms(1000);
}

void SendToSD(int data){
    FATFS fs;
    volatile FRESULT fr;
    FIL file;

    UINT bw = 0;

    char information[128];
    
    set_fattime(2022,12,6,12,0,0);
    delay_1ms(100);
    
    strcpy(information,"");
    sprintf(&information[strlen(information)], "%d", data);
    strcat(information,"");
    
    f_mount(&fs,"",1);
    f_sync(&file);

    fr = f_open(&file, "DATA.TXT", FA_WRITE | FA_CREATE_ALWAYS);
    fr = f_write(&file, information, strlen(information), &bw);
    delay_1ms(400);

    f_sync(&file);

    f_close(&file);
    delay_1ms(100);

}

