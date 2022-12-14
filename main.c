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

    /* The related data structure for the IMU, contains a vector of x, y, z floats */
    mpu_vector_t Acc;
    
    float y;			// to save the acctual acceleration (G)
    int on = 0;			// to toggle between on / off
    int G=0, R=0, S=0;          // The diffrent States (Green, Red, Stop)

    int redReps = 0;		// to keeping track on the number of bad repetitions 
    
    int period = 3600000;	// the period we update the SD-Card (every hour)
    int hepticPeriod = 2000;	// the period we send heptic-feedback (for 2 seconds)
    int testFreq = 200;		// the sample period (0.2 seconds)

    /* Varibeles to keep track of the periods using the System Core Clock */
    uint64_t start_mtime, delta_mtime;
    uint64_t startH_mtime, deltaH_mtime;
    uint64_t test_time_start, test_time_delta;
    
    test_time_start = get_timer_value();
	while(1) {
        /* Read if button is pressed (connected to GPIO-pin 6) */
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_6) == SET) {          // if Button is pressed        
            if(on == 0) {
                on = 1; 
            } else {
                on = 0;
            }
            while(gpio_input_bit_get(GPIOA, GPIO_PIN_6) == SET);    // while Button is pressed
        }

        if(on == 0) {
            /* System is "Off", Update SD-Card */
            start_mtime = get_timer_value();
            if(S == 0){
                SendToSD(redReps);                                  // Send redReps to SD-Card
                S=1;
                G=0;
                R=0;
            }
            /* Blink LED to let us know it's off */
            T1setPWMmotorB(1);
            delay_1ms(200);
            T1setPWMmotorB(0);
            delay_1ms(200);
        } else {
            mpu6500_getAccel(&Acc);                     // Get accelleration data (Note: Blocking read) puts a force vector with 1G = 16384 into x, y, z directions respectively
            y = Acc.y / 16384;                          // Scale to G values   

            test_time_delta = get_timer_value() - test_time_start;

            if (test_time_delta >(SystemCoreClock/4000.0 *testFreq)){
                if(y>0.80) {
                    if(G==0) {
                        T1setPWMmotorB(0);
                    }
                    G=1;
                    R=0;
                } else if(y<0.60) {
                    if(R==0) {
                        redReps++;
                        T1setPWMmotorB(1);	// Light LED to show we're in RED zone
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
        
        delta_mtime = get_timer_value() - start_mtime;

        // Send to SD-Card after 1 hour of work
        if (delta_mtime >(SystemCoreClock/4000.0 *period)){
            SendToSD(redReps);      // Send redReps to SD-Card
        }
    };
    
}

////////////////////////// Write functions /////////////////////////////////////////

void Initialize_Project(){
    ////////////////////////////////// Initialize  heptic feedback //////////////////////////////////
    T1powerUpInitPWM(0x3);                  //Starts A0 and A1.
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
    
    /* Blink LED to show that the Initialization was successfull */
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

