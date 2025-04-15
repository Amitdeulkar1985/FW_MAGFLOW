//Developed branch testing
#include <Arduino.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <EEPROM.h>

#define ADC_Pin        1                 //ADC input channel     // later it should be 1

#define FLOW_ADC_Pin   4                 //adc input for empty flow 


//******************************* Freq_Geneartion varibales and Pin declarations *****************************************
#define DELAY_MS              80000         // 80 Msec timer 6.25 Hz

#define DEFAULT_FREQENCY      80000         // 6.25Hz
#define FREQENCY_12_5Hz       40000         // 12.5Hz

#define FREQENCY_25Hz         20000         // 25 Hz
#define FREQENCY_37Hz         13514        // 37 Hz   // added on 11_02_2023

#define FREQENCY_2Hz         250000    // 2Hz

#define EMPTY_ADC_VAL               1800
#define RESET_EMPTY_ADC_VAL         1900
#define EMPTY_COUNTER_LIMIT         2
#define EMPTY_RESET_COUNTER_LIMIT   2

//#define DELAY_uS  1000000       // 10 sec timer

#define  INPUT_PIN_2         18        // By default high Default frequency set
#define  INPUT_PIN_1         42 
#define  INPUT_PIN_3         17        // to detect 37hz  // added on 11_02_2023

#define PIN1                  5 //1             // GPIO pin 1         //Excitation PINS 
#define PIN2                  6 //2             // GPIO pin 2

#define MUX_C                42
#define MUX_B                41
#define MUX_A                40

#define PWM_1               7            //46  Frequency Generation with 50% duty

#define EMPTY_PIN       3

//Pins for Calibartion
#define Calibration_Mode  9
#define ADD1              10
#define ADD2              11
#define ADD3              12
#define ADD4              13

#define CAL_TX_PIN        10  // TX on Pin 10
#define CAL_RX_PIN        11  // RX on Pin 11
#define RS_485_RX_TX      8

#define GREEN_LED         38
#define RED_LED           39

HardwareSerial MySerial(1);  // Use UART1

//-----------------------

#define Toggle_For_Coil_2   8              // only for testing
#define Toggle_For_Coil_1   45              // only for testing

#define BAUDRATE            9600          //  baudrate set
#define DEBUG_EN            1             // debug ADC

#define EEPROM_SIZE     150
#define EEPROM_ADDR     10
#define SERIAL_NO_ADD   75
#define PCB_NO_ADD      100
#define FLOW_TUBE_ADD   125

#define BUF_SIZE 100

//operation state
#define Zero_Flow       1
#define Normal_Flow     2
#define Low_Flow        3
#define Pipe_Empty      4
#define Error_State     5
#define Cal_Conf_Mode   6

//led state
#define GREEN_ON            1
#define GREEN_BLINK_TWICE   2
#define GREEN_BLINK_ONCE    3
#define RED_ON              4
#define ORANGE_BLINK        5
#define ORANGE_ON           6

unsigned char Opeation_state;

//#define BUFFER_SIZE    20

volatile char f1=0;       // Flag for toggling 
hw_timer_t *Freq_timer = NULL;  // take hardware timer pointer
hw_timer_t *Empty_Flow_timer = NULL;

volatile unsigned char Check_Empty_Flow;
//*********************************************************************************************************************

 unsigned int ADC_Count_1=0,ADC_Count_2=0;    // was int 15_02_24
 unsigned int X1=0,X2=0;   // added 15_02_24
unsigned int ADC_Count_2_Buff[30];
unsigned int ADC_Count_1_Buff[30];
unsigned int ADC_Avg_1=0;
unsigned int ADC_Avg_2=0;
unsigned int ADC_Avg_Num=1;
unsigned int Var_1=0,Var_2=0;
float Voltage=0;
uint16_t ADC_Average();
unsigned int Temp_Voltage=0;
unsigned int Empty_Flow_Adc_val = 1900;
unsigned int Empty_Adc[5];
 int Delta=0;
 int Delta_ZF=0;
//******************************************************* UART ********************************************************
unsigned char M1[25]="#ADC_Coil_1_is_0000@";             // A1 string
unsigned char M2[25]="#ADC_Coil_2_is_0000@";             // A2 string
unsigned char M3[25]="#Delta_is_0000@";                  // Delta string
unsigned char M4[25]="#A10000A20000D0000@";
//unsigned char Config_Buffer[30]="$S0000E0000G0000&";
unsigned char Config_Buffer[100]="$S062E070EF00.00SD00.00G1F00.123&";
//unsigned char M6[25]="#0000@";          //NA
unsigned char Pulse_Freq[18]="#PWM0000@";
unsigned char Rx_Buffer[250]="#S000@";                   // Receiver buffer for serial

unsigned char Rx_Ch=0;
void IRAM_ATTR Recieve_UART();
volatile bool Rx_Int_Flag=0;
volatile bool Rx_Cal_Int_Flag = 0;
volatile char Rx_String_Complete=0;
void Convert_For_LCD(uint16_t ADC_Value);
uint8_t V1,V2,V3,V4;
uint8_t BCD_4(uint16_t int_16,uint16_t div_16);
void Calculate_Delta();
void Timer2_init(void);
void IRAM_ATTR onTimer();
void Check_Cal_Recived_String(void);
void Led_Operation(void);

//***********************************************************************************************************************
bool Start_ADC_Sampling_Delay=0;
bool ADC_Read_Start=0;
unsigned int Recieve_ADC_Read_Delay=0;

unsigned int ADC_Read_Delay=0;
unsigned int Debug=0;
bool ADC_Read_Stop=0;
unsigned int Recieve_ADC_Read_Stop_Delay=0;
unsigned int ADC_Read_Stop_Delay=0;
unsigned int N_Samples_Set=0,N_Sample_Counter=0;
unsigned int Pulse_Frequency=0,ADC_Calibration_Count=0;

//-------for calibration--------------------
unsigned int Calibration_P1_start;
long int Calibration_P1_Factor;
unsigned int Calibration_P2_start;
long int Calibration_P2_Factor;
unsigned int Calibration_P3_start;
long int Calibration_P3_Factor;
unsigned int Calibration_P4_start;
long int Calibration_P4_Factor;
unsigned int Calibration_P5_start;
long int Calibration_P5_Factor;
unsigned int Calibration_P6_start;
long int Calibration_P6_Factor;
unsigned int Calibration_P7_start;
long int Calibration_P7_Factor;
unsigned int Calibration_P8_start;
long int Calibration_P8_Factor;
unsigned int Calibration_P9_start;
long int Calibration_P9_Factor;
unsigned int Calibration_P10_start;
long int Calibration_P10_Factor;

unsigned int P1_Adc_Count,P2_Adc_Count,P3_Adc_Count,P4_Adc_Count,P5_Adc_Count;
unsigned int P6_Adc_Count,P7_Adc_Count,P8_Adc_Count,P9_Adc_Count,P10_Adc_Count;
float actual_flow = 0;

float Calibrated_Multiplier;
unsigned char Cal_mode;
unsigned char Calibration_counter;
unsigned int READ_FLOW;

unsigned int Calibration_flow[11];
unsigned int Calibration_adc[11];
unsigned char Device_Serial_no[25];
unsigned char Pcb_Serial_no[25];
unsigned char Flowtube_Serial_no[25];

bool Start_calibration;
bool End_calibration;
bool Calibration_on;


//unsigned int FLOW_RATE_RANGE;
//unsigned int PWM_RANGE;
float FLOW_RATE_RANGE;
unsigned int  PWM_RANGE;
unsigned int FLOWRATE_ADC_LOW_VAL =  0;
unsigned int FLOWRATE_ADC_HIGH_VAL = 2860;//1880;//2850;//1500;
//unsigned int  Pwm_Calculator_Val;
float  Pwm_Calculator_Val;

unsigned char Calibration_Points;
bool CALIBRATION_UPDATE;
bool SEND_CALIBRATION_DATA;
bool Update_Frequency;
bool Send_Frequency;
bool Low_Flow_Flag;
void SEND_CAL_DATA(void);
unsigned int Calculate_Flow_Rate(unsigned int,unsigned int,unsigned int,unsigned int);
unsigned int Calculate_Flow_Rate1(unsigned int,unsigned int,unsigned int,unsigned int);
String getValue(char data[], char separator, int index);
void Read_485_Address(void); 
void Store_calb_data(void);
void Send_ok(void);
unsigned int Get_Flow(void);
void Send_flow(void);
void Store_serial_no(void);
void Flush_Rx_Buffer(void);
void Convert_flow_ascii(float flow, char &d1, char &d2, char &d3, char &d4, char &d5);

bool N_Sample_Counter_Flag=0;
void ADC_Sampling_Check();
void Check_Recived_String(void);
void Set_Frequency(void);
void Send_Freq(void);
volatile char receive_byte=0;
volatile char cal_receive_byte = 0;
//********************************************For Timer0 interrupt**********************************************
hw_timer_t *My_timer = NULL;  // take hardware timer pointer
volatile bool Timer0_Int_flag=0;       // Flag for toggling 
void Timer0_init();
void IRAM_ATTR Timer0_Interrupt();
//********************************************For Timer1 interrupt**********************************************************************
void IRAM_ATTR Timer1_Interrupt();
void GPIO_Setup();
void Timer1_init();
//***********************************************For GPIO intterupt**************************************
volatile bool Coil_1_flag=0;
void IRAM_ATTR Coil_1();
volatile bool Coil_2_flag=0;
void IRAM_ATTR Coil_2();
//*****************************PWM Varaibles***************************
//int PWM_Freq = 0;               // generates 1Khz freq  //1000    
float PWM_Freq = 0;

const int ledChannel = 0;
const int resolution = 13;
void PWM_Init();
void PWM_Set();
//*************************************************************************************************************************
unsigned char EEPROM_Save[EEPROM_SIZE]="#0000000000@";  
void printIntArray(unsigned int* arr, int size) ;
void Recall_Memory();
void Pipe_Size_Selection();
//unsigned char Gain_1,Gain_2,Gain_3;

float Form_Multiplier(unsigned char* str,unsigned int Start,unsigned int End);
void Check_Calibration_string(void);
void SEND_CAL_DATA(void);
void Calibrate_PWM(void);
void CONVERT_INTEGER_ASCII(unsigned long int TEMP);
void Calibration_Process(void);

void Update_Config();
bool Update_Config_Flag;
bool Config_Write=0;
void Write_Config_Received(unsigned char* Rx_Buffer);
void IRAM_ATTR caluartISR(void);

unsigned char Pipe_Size=0;
unsigned int Zero_Flow_Offset=80;                //16_03_24
bool Zero_Flow_Update;

//---------------------------
bool ADC_Count_1_Buff_Flag=0;
bool ADC_Count_2_Buff_Flag=0;
//-----------------------------
float Flow_Per_Pulse;
float Flow_Rate;
float Scale_Delta_Factor=1.0 ;
unsigned char Flow_Rate_Buffer[25]="#FR00000.00@";
void Flow_Rate_Calculation();
unsigned char Excitation_Frequency;
float PWM_Multiplier;
unsigned char LOAD1,LOAD2,LOAD3,LOAD4,LOAD5,LOAD6;
unsigned int Zero_Flow_rate_val;
unsigned char RS_485_addr;
unsigned char Empty_Counter;
unsigned char Empty_Reset_Counter;
unsigned char Cal_Rcvd_Data[250];
volatile unsigned char CAL_DATA_COUNTER;
volatile bool Save_data;
volatile bool Compare_data;
bool Empty_Tube_Error;

//**************************************************************************************************************************

void setup()
{  
    #if DEBUG_EN
        Serial.begin(BAUDRATE);     // Serial terminal set baudrate 
    #endif

    MySerial.begin(9600, SERIAL_8N1, CAL_RX_PIN, CAL_TX_PIN);
    pinMode(CAL_RX_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(CAL_RX_PIN), caluartISR, FALLING);
    digitalWrite(RS_485_RX_TX,0);
    
    //--------------------------------

    GPIO_Setup();
    attachInterrupt(44, Recieve_UART, FALLING);        // Rx ISR removed for testing Purpose
    attachInterrupt(PIN1, Coil_1,RISING );        // Rx ISR removed for testing Purpose
    attachInterrupt(PIN2, Coil_2,RISING );        // 
    Start_ADC_Sampling_Delay=0;
    ADC_Count_1=0;
    ADC_Count_2=0;
    X1=X2=0;
    Recall_Memory();

    //initialise delay 30 seconds
    Serial.println("wait for 30 seconds \n");
    delay(10000);
    delay(10000);
    delay(10000);

    Timer0_init();
    //**************************Timer1 functions*******************************************

    Timer1_init();

    Timer2_init();

    digitalWrite(PIN1, 1);                              // set pin 1 to high at first 
    digitalWrite(PIN2, 0);                              // set pin 1 to high at first 
    
  //************************************************************************************
    PWM_Init();
//***************************************************************************************************
  //  Recall_Memory();
    //digitalWrite(MUX_A, 0);
   //digitalWrite(MUX_B, 0);
    //digitalWrite(MUX_C, 0);


}

void loop() 
{
    //unsigned int Var_4=0;
    
  //******************************** Timer 1 checking *****************************************
            if(f1)  // if flag is set then go inside
              {
                f1=0;                                       // clear flag for next check 
                digitalWrite(PIN1, !digitalRead(PIN1));     //   Read pin and toggle 
                digitalWrite(PIN2, !digitalRead(PIN2));     //   Read pin and toggle 
               
                #if DEBUG_EN
                //Serial.println("SET1 \n");
                #endif
              
              } 
  //************************************************************************************************

               if(Rx_Int_Flag==1)
              {
                      Rx_Int_Flag=0;
                      Check_Recived_String();   
              }

              if(Rx_Cal_Int_Flag)
              {
                    Rx_Cal_Int_Flag = 0;
                    Check_Cal_Recived_String();
              }

              //Calibration data 
              if(Compare_data)
              {
                  Compare_data = 0;

                  Calibration_Process();
              }
      
              if(Update_Config_Flag==1)
              {
                Update_Config_Flag=0;
                Pipe_Size_Selection();    //19_02_24
                Update_Config();
              }
              else if( Config_Write==1)
              {
                Write_Config_Received(Rx_Buffer);
                Config_Write=0;                    //23_03_24
              }
              
        if(Zero_Flow_Update==1)
        {
            Zero_Flow_Update=0;   //123456
            Zero_Flow_Offset =Delta_ZF;
             Serial.println("Offset");
             Serial.print(Delta_ZF);
         //************************************ 19_03_24 ***************************************************
            /*Zero_Flow_Offset   =  ((Rx_Buffer[3]&0x0f)*1000); 
            Zero_Flow_Offset   =  Zero_Flow_Offset+((Rx_Buffer[4]&0x0f)*100); 
            Zero_Flow_Offset   =   Zero_Flow_Offset+((Rx_Buffer[5]&0x0f)*10);
            Zero_Flow_Offset   =   Zero_Flow_Offset+((Rx_Buffer[6]&0x0f));*/ 
    
        //**************************************************************************************************
                EEPROM_Save[18] =(Zero_Flow_Offset>>8) & 0xff;   // msb
                EEPROM_Save[17] = Zero_Flow_Offset & 0xff;         //lsb    

                 /*for (int i = 20; i < 22; i++) 
                {
                  EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
                  EEPROM.write(EEPROM_ADDR + i - 20, EEPROM_Save[i]);

                }*/
                EEPROM.write(EEPROM_ADDR + 17, EEPROM_Save[17]);
                EEPROM.write(EEPROM_ADDR + 18, EEPROM_Save[18]);
                EEPROM.commit();

        }
        if(CALIBRATION_UPDATE)
        {
              //#CAL,P10,1)00000,01000,2)01001,02000,3)02001,03000,4)03001,04000,5)04001,05000,6)05001,06000,7)06001,07000,8)07001,08000,9)08001,09000,10)09001,10000@
              CALIBRATION_UPDATE = 0;
              Serial.println("CHECKING STRING  \n");
              Check_Calibration_string();
        }
        if(SEND_CALIBRATION_DATA)
        {
              SEND_CALIBRATION_DATA = 0;
              SEND_CAL_DATA();
        }
        if(Update_Frequency)
        {
            Update_Frequency = 0;
            Set_Frequency();
        }
        if(Send_Frequency)
        {
              Send_Frequency = 0;
              Send_Freq();
        }

        if(Check_Empty_Flow)
        {
              Check_Empty_Flow = 0;
              Serial.println("Checking Empty Pipe\n");
              //timerAlarmDisable(Freq_timer);
              //timerAlarmDisable(My_timer); //Just Enable 

              for (int i = 0; i < 3; i++) 
              {
                  digitalWrite(EMPTY_PIN, HIGH);
                  delay(1);
                  digitalWrite(EMPTY_PIN, LOW);
                  Empty_Adc[i] = analogRead(FLOW_ADC_Pin);
                  delay(1);
              }
              int adcValue = (Empty_Adc[0] + Empty_Adc[1] + Empty_Adc[2]) / 3;
              Serial.print("ADC Value: ");
              Serial.println(adcValue);

              if(adcValue < EMPTY_ADC_VAL)
              {
                    Empty_Counter++;
              }
              else
              {
                  Empty_Counter = 0;
              }
              if(adcValue >= RESET_EMPTY_ADC_VAL && Empty_Tube_Error)
              {
                    Empty_Reset_Counter++;
              }
              else
              {
                  Empty_Reset_Counter = 0;
              }
              //empty pipe 
              if(Empty_Counter > EMPTY_COUNTER_LIMIT && !Empty_Tube_Error)
              {
                    Empty_Tube_Error = 1;
                    Empty_Counter = 0;
                    Serial.print("Pipe is Empty");
              }
              //empty pipe reset
              if(Empty_Reset_Counter  > EMPTY_RESET_COUNTER_LIMIT && Empty_Tube_Error)
              {
                    Empty_Reset_Counter = 0;
                    Empty_Counter = 0;
                    Empty_Tube_Error = 0;
                    Serial.print("Empty Pipe fault reset");
              }

              //timerAlarmEnable(Freq_timer);
              //timerAlarmEnable(My_timer); //Just Enable 
        }
  //**************************************************************************************************        
        if(Timer0_Int_flag==1)
        {
                Timer0_Int_flag=0;        //1 ms flag
                //Debug++;
 
                if((Coil_2_flag == 1) || (Coil_1_flag == 1))
                  {
                      ADC_Sampling_Check();
                  }
        }

      if(Start_ADC_Sampling_Delay==0)
        {
               ADC_Count_2=ADC_Average();

             if(ADC_Count_2>10)             // testing purpose later it should be 0
                {
                  Start_ADC_Sampling_Delay=1;
                 // Serial.print("Read_Start");    // only for testing purpose 
                ADC_Count_2=0;                      //09_05_23
                }

        }
        if((ADC_Read_Start==1 ) && (Start_ADC_Sampling_Delay==1))
        {
          
                if(Coil_2_flag==1)
                {
                
                   if(N_Sample_Counter_Flag==1)
                     {
                       N_Sample_Counter_Flag=0;       //25_03_24
                        digitalWrite(Toggle_For_Coil_2, !digitalRead(Toggle_For_Coil_2));     //   Read pin and toggle 
                        X2=ADC_Average();    // added 15_04_24
                   
                            ADC_Count_2=ADC_Count_2 + X2;   // added 15_04_24
                            //Serial.println("Cnt2Indi");    // only for testing purpose 
                           //  Serial.print(ADC_Count_2);    // only for testing purpose 
                    
                      }
                     
                    
              }
               else if(Coil_1_flag==1)
               {
                     
                   if(N_Sample_Counter_Flag==1)
                     
                      {
                          N_Sample_Counter_Flag=0;       //25_03_24
                        digitalWrite(Toggle_For_Coil_1, !digitalRead(Toggle_For_Coil_1));     //   Read pin and toggle 
                       X1=ADC_Average();// added 15_04_24
                   
                     
                          ADC_Count_1=ADC_Count_1 + X1;// added 15_04_24
                           //Serial.println("Cnt1Indi");    // only for testing purpose
                            //Serial.print(ADC_Count_1);    // only for testing purpose  
                      }
                    
                     
               }
              else
              {
               ADC_Read_Start=0;
               ADC_Read_Stop=0;
              //Start_ADC_Sampling_Delay=0;   

                ADC_Read_Delay=0;                    
                ADC_Read_Stop_Delay=0;

                N_Sample_Counter_Flag=0;   
                N_Sample_Counter=0;
              }
         } 

        if((ADC_Read_Stop==1) && (ADC_Read_Start==1))   //17_02_24
       {
          ADC_Read_Start=0;
          ADC_Read_Stop=0;
          Start_ADC_Sampling_Delay=0;   

          ADC_Read_Delay=0;                    
          ADC_Read_Stop_Delay=0;

          N_Sample_Counter_Flag=0;   
          N_Sample_Counter=0;
          
          Coil_1_flag=0;
          Coil_2_flag=0;
        }

        //calibration mode selection
        Cal_mode = digitalRead(Calibration_Mode);

        if(!Cal_mode)
        {
              Start_calibration = 1;
              Opeation_state = Cal_Conf_Mode;
        }
        else if(Empty_Tube_Error)
        {
              Opeation_state = Pipe_Empty; 
        }
        else if(Low_Flow_Flag)
        {
              Opeation_state = Low_Flow;
        }

        Led_Operation();
//Recieve_UART();
}
//***************************************************** ISR_Routines*********************************************************
void ADC_Sampling_Check(void)
{
      unsigned int Var_4=0;

      if(Start_ADC_Sampling_Delay==1)
      {
            ADC_Read_Delay++;
            ADC_Read_Stop_Delay++;
      }
      //-------------------------------------------------------------------------------------------------------
      if((ADC_Read_Delay>=Recieve_ADC_Read_Delay) && (Recieve_ADC_Read_Delay!=0))  
      {
            ADC_Read_Start=1;
            //   if(N_Sample_Counter<=N_Samples_Set)    //16_02_24   added =  25_03_24
            if(N_Sample_Counter<N_Samples_Set)    //16_02_24   added =  25_03_24
            {
                N_Sample_Counter_Flag=1;
                N_Sample_Counter++;
            }
            else if(Coil_2_flag==1)     //rita
            {
                  if(ADC_Count_2_Buff_Flag==0)
                  {
                        Coil_2_flag=0;
                        N_Sample_Counter_Flag=0;
                        N_Sample_Counter=0;
  
                        ADC_Count_2=ADC_Count_2/N_Samples_Set;

                        //  Serial.print("Cnt2Sample");
                        // Serial.print(ADC_Count_2);

                        if(ADC_Count_2 > 4095)
                        {
                                ADC_Count_2=4095;
                        }
                        ADC_Count_2_Buff[Var_2]=ADC_Count_2;
                        Var_2++;
                        if(Var_2 >= ADC_Avg_Num)     // added 19_02_24
                        {
                              Var_2=0;
                              ADC_Count_2_Buff_Flag=1;
                        }
                        ADC_Count_2=0;       
                        ADC_Count_1=0;   
                  }
              }
              else if(Coil_1_flag==1)
              {
                  if(ADC_Count_1_Buff_Flag==0)
                  {
                      Coil_1_flag=0;
                      N_Sample_Counter_Flag=0;
                      N_Sample_Counter=0; 
                      ADC_Count_1=ADC_Count_1/N_Samples_Set;
                      if(ADC_Count_1>4095)
                      {
                            ADC_Count_1=4095;
                      }
                      ADC_Count_1_Buff[Var_1]=ADC_Count_1;
                      Var_1++;
                     // if(Var_1>=ADC_Avg_Num)     // removed 19_02_24  
                    if(Var_1>=ADC_Avg_Num)        // added 19_02_24
                    {
                          Var_1=0;
                          ADC_Count_1_Buff_Flag=1;
                    }

                  /*  added 16_02_24*/
                  ADC_Count_2=0;       
                  ADC_Count_1=0;

             }

      }

      if(ADC_Count_1_Buff_Flag==1 && ADC_Count_2_Buff_Flag==1 ) 
      {

            Var_4=0;    // added 15_02_24
            for (unsigned int Var_3=0; Var_3 < ADC_Avg_Num; Var_3++)          //was 6
            {
                  Var_4 =ADC_Count_1_Buff[Var_3];
                  ADC_Avg_1=ADC_Avg_1+Var_4;
            }
            ADC_Avg_1=(ADC_Avg_1/ADC_Avg_Num);
            //Serial.println("ADC1/6");
            //Serial.print(ADC_Avg_1);
            //--------------------------------------------------------------------------------------------------------------------
            Var_4=0;    // added 15_02_24
            for (unsigned int  Var_3= 0; Var_3 < ADC_Avg_Num; Var_3++) 
            {
                  Var_4 =ADC_Count_2_Buff[Var_3];    //15_02_24 removed
                  ADC_Avg_2=ADC_Avg_2+Var_4;
                  //Serial.println("ADC2");
                  //Serial.print(ADC_Avg_2);
            }
            ADC_Avg_2=(ADC_Avg_2/ADC_Avg_Num);
            //   Serial.println("ADC2/6");
            //   Serial.print(ADC_Avg_2);
            //--------------------------------------------------------------------------------------------------------------------                                  
            V1 = BCD_4(ADC_Avg_1,1000);
            V2 = BCD_4(ADC_Avg_1,100 );
            V3 = BCD_4(ADC_Avg_1,10  );
            V4 = BCD_4(ADC_Avg_1,1   );
            M4[3]=M1[15]=V1+0x30;
            M4[4]=M1[16]=V2+0x30;
            M4[5]=M1[17]=V3+0x30;
            M4[6]=M1[18]=V4+0x30;
            //****************************************************************************************
            V1 = BCD_4(ADC_Avg_2,1000);
            V2 = BCD_4(ADC_Avg_2,100 );
            V3 = BCD_4(ADC_Avg_2,10  );
            V4 = BCD_4(ADC_Avg_2,1   );
            M4[9]=M2[15]=V1+0x30;
            M4[10]=M2[16]=V2+0x30;
            M4[11]=M2[17]=V3+0x30;
            M4[12]=M2[18]=V4+0x30;
            //M4[18]='@';
            //--------------------------------------------------------------------------------------------------------------------                                     
            Calculate_Delta();
            // PWM_Set();
            Flow_Rate_Calculation();    // 20_03_24

            ADC_Avg_1=0;       
            ADC_Avg_2=0;                         

            if(Config_Write==0)
            {
                  Serial.print((char*)M4);   // only for testing purpose     // send data to LCD  every 1sec
                  Serial.print((char*)Pulse_Freq);   // only for testing purpose     // send data to LCD  every 1sec
                  Serial.print((char*)Flow_Rate_Buffer);   // only for testing purpose     // send data to LCD  every 1sec
            }
            ADC_Count_1_Buff_Flag=0;
            ADC_Count_2_Buff_Flag=0;

      }

      }
      if((ADC_Read_Stop_Delay>=Recieve_ADC_Read_Stop_Delay) && (ADC_Read_Stop_Delay!=0))
      {
            ADC_Read_Stop=1;  
      }  
}
void Calculate_Delta()
{
          //Delta=((ADC_Count_1)-(ADC_Count_2));
           Delta=((ADC_Avg_1)-(ADC_Avg_2));
         // Serial.println("AVG1");
         // Serial.print(ADC_Avg_1);
         // Serial.println("AVG2");
         // Serial.print(ADC_Avg_2);
          if(Delta<0)
          {
            Delta= Delta*(-1);
          }
          Delta_ZF= Delta;

          //Delta=Delta*1;
          V1 = BCD_4(Delta,1000);
          V2 = BCD_4(Delta,100 );
          V3 = BCD_4(Delta,10  );
          V4 = BCD_4(Delta,1   );
          M4[14]=M3[10]=V1+0x30;
          M4[15]=M3[11]=V2+0x30;
          M4[16]=M3[12]=V3+0x30;
          M4[17]=M3[13]=V4+0x30;
          // M4[18]='@';
 
}
void PWM_Init()
{
  ledcSetup(ledChannel, PWM_Freq, resolution);        //resolution is 13 bit 

  ledcAttachPin(PWM_1, ledChannel);                   // attach the channel to the GPIO to be controlled
  ledcWrite(0, 4095);                                 //PWM Duty 50%   2^13−1=8191, for 50% 8191 / 2 = 4095.5
}
void PWM_Set()
{ 
     //Delta= 25000;

     if(Delta <= Zero_Flow_Offset)    //19_02_24 Delta<=50 indicating zero flow //50//250
      {
        PWM_Freq=0; 
        ledcWrite(0 ,0);       // 13_03_24   To make it zero
     //   pinMode(PWM_1, OUTPUT);
      // digitalWrite(PWM_1, 0);
  
         V1 = BCD_4(PWM_Freq,1000);
         V2 = BCD_4(PWM_Freq,100 );
         V3 = BCD_4(PWM_Freq,10  );
         V4 = BCD_4(PWM_Freq,1   );
        Pulse_Freq[0]='#';
        Pulse_Freq[1]='P';
        Pulse_Freq[2]='W';
        Pulse_Freq[3]='M';
        Pulse_Freq[4]=V1+0x30;
        Pulse_Freq[5]=V2+0x30;
        Pulse_Freq[6]=V3+0x30;
        Pulse_Freq[7]=V4+0x30;
        Pulse_Freq[8]='@';

      }
      else
     {
       
       // pinMode(PWM_1, OUTPUT);   //13_03_24

         // Delta    = Delta-Zero_Flow_Offset;
         Delta = Delta;
         /* Serial.println("PWM multiplier");   // only for testing purpose
          Serial.print(PWM_Multiplier,4);
          
          Serial.println("Scale_Delta_Factor");   // only for testing purpose
          Serial.print(Scale_Delta_Factor,4);*/

          PWM_Freq = (int)(Delta* Calibrated_Multiplier)/Scale_Delta_Factor ;

          Calibrate_PWM();

        //PWM_Freq = (int)(Delta* PWM_Multiplier)/Scale_Delta_Factor ;

        //Calibrated_Multiplier = 0.9425;Scale_Delta_Factor = 6.03;
       
        //PWM_Freq = (int)(Delta* Calibrated_Multiplier)/Scale_Delta_Factor ;
        

        if(PWM_Freq <= 16)  //12-06-24 threshold flow 
        {
            Config_Write = 0;
            PWM_Freq=0; 
            ledcWrite(0 ,0);  
             //Serial.println("Frequency less than 16");   // only for testing purpose

            V1 = BCD_4(PWM_Freq,1000);
            V2 = BCD_4(PWM_Freq,100 );
            V3 = BCD_4(PWM_Freq,10  );
            V4 = BCD_4(PWM_Freq,1   );
            Pulse_Freq[0]='#';
            Pulse_Freq[1]='P';
            Pulse_Freq[2]='W';
            Pulse_Freq[3]='M';
            Pulse_Freq[4]=V1+0x30;
            Pulse_Freq[5]=V2+0x30;
            Pulse_Freq[6]=V3+0x30;
            Pulse_Freq[7]=V4+0x30;
            Pulse_Freq[8]='@';
        }
        else
        {
            if (PWM_Freq <= 5)
            {
              PWM_Freq = 6;
            }
            //ledcSetup(ledChannel, PWM_Freq, resolution);
            ledcSetup(ledChannel, PWM_Freq, resolution);
      
            ledcWrite(0, 4095);                                        //PWM Duty 50%   // 13_03_24   To make it zero
            V1 = BCD_4(PWM_Freq,1000);
            V2 = BCD_4(PWM_Freq,100 );
            V3 = BCD_4(PWM_Freq,10  );
            V4 = BCD_4(PWM_Freq,1   );
            Pulse_Freq[0]='#';
            Pulse_Freq[1]='P';
            Pulse_Freq[2]='W';
            Pulse_Freq[3]='M';
            Pulse_Freq[4]=V1+0x30;
            Pulse_Freq[5]=V2+0x30;
            Pulse_Freq[6]=V3+0x30;
            Pulse_Freq[7]=V4+0x30;
            Pulse_Freq[8]='@';
        }

     }
}
//************************************************************************************************
void IRAM_ATTR Timer0_Interrupt()  
{
  Timer0_Int_flag  = 1;      // set flag in interrupt 
   
}
void IRAM_ATTR Recieve_UART()
{
      Rx_Int_Flag=1;
      
}

void IRAM_ATTR Coil_2()  
{
  Coil_2_flag = 1;      // set flag in interrupt 

   
}
void IRAM_ATTR Coil_1()  
{
  Coil_1_flag = 1;      // set flag in interrupt 

}
void Check_Recived_String(void)
{
       while(Serial.available()>0)
      {
              receive_byte=Serial.read();
              //if(receive_byte=='#' && Rx_Ch==0)
              if(receive_byte=='#')
              {
                  Rx_Buffer[0]=receive_byte;
                  Rx_Ch=1;
                 // Serial.println("\nreceived #");

              }
              else if(receive_byte=='@')
              {
                  Rx_Buffer[Rx_Ch]=receive_byte;
                  Rx_Ch=0;
                  Rx_String_Complete=1;
                 //Serial.println("\nreceived @");
              }
              else
              {
                Rx_Buffer[Rx_Ch]=receive_byte;
                Rx_Ch++; 
              }
        }
        if(Rx_String_Complete==1)
        {
                Rx_String_Complete=0;		
                //Serial.print("above flag interrupt \n");
                if((Rx_Buffer[1]=='R') && (Rx_Buffer[2]=='C'))
                {
                      //Serial.println("\nOK");
                      Update_Config_Flag=1;
                }
                else if((Rx_Buffer[1]=='S'))
                {
                      Config_Write=1;                  
                }
                else if ((Rx_Buffer[1]=='F') && (Rx_Buffer[2]=='C'))
                {
                      Update_Frequency = 1;
                }
                else if ((Rx_Buffer[1]=='G') && (Rx_Buffer[2]=='F'))
                {
                      Send_Frequency = 1;
                }
                
                else if ((Rx_Buffer[1]=='Z') && (Rx_Buffer[2]=='F'))
                {
                      Zero_Flow_Update=1;
                }
                else if ((Rx_Buffer[1]=='C') && (Rx_Buffer[2]=='A') && (Rx_Buffer[3]=='L'))
                {
                      CALIBRATION_UPDATE = 1;
                }
                else if ((Rx_Buffer[1]=='G') && (Rx_Buffer[2]=='E') && (Rx_Buffer[3]=='T'))
                {
                      SEND_CALIBRATION_DATA = 1;
                      //Serial.print("below flag interrupt \n");
                }
                  
        //}
   }
}
void Check_Cal_Recived_String(void)
{
       while(Serial1.available()>0)
      {
              cal_receive_byte=Serial1.read();
              //if(receive_byte=='#' && Rx_Ch==0)

                  if(cal_receive_byte == '*')
                  {
                        Save_data = 1;
                  }
                  else if(cal_receive_byte == '#')
                  {
                        Save_data = 0;
                        Compare_data = 1;
                        Cal_Rcvd_Data[CAL_DATA_COUNTER] = cal_receive_byte;
                        //GSM_COUNTER++;
                  }
                  if(Save_data == 1)
                  {
                        Cal_Rcvd_Data[CAL_DATA_COUNTER] = cal_receive_byte;
                        CAL_DATA_COUNTER++;
                        if(CAL_DATA_COUNTER >= 200)
                        {
                            CAL_DATA_COUNTER = 0;
                        }
                        cal_receive_byte = 0;
                  }       
        //}
   }
}
 uint16_t ADC_Average()
{
    uint8_t  avg=0;
    uint16_t vio=0;
    uint16_t tmp=0;
    uint16_t Count=0;
    
 for(avg=1;avg<=10;avg++)				
      {
        
          Count=analogRead(ADC_Pin);
          tmp     = Count;            // load
          vio = (vio + tmp);
      }

      vio =(vio / 10); 
      // Serial.println("Actual ADC is");   // only for testing purpose
      // Serial.print(vio);
    return vio;
}

uint8_t BCD_4(uint16_t int_16,uint16_t div_16)
{
	uint8_t BCD04;
	BCD04 = ((int_16/div_16)%10);
	BCD04 = (BCD04 & 0x0F);
	return BCD04;
}
 void Convert_For_LCD(uint16_t ADC_Value)
 {
      V1 = BCD_4(ADC_Value,1000);
      V2 = BCD_4(ADC_Value,100 );
      V3 = BCD_4(ADC_Value,10  );
      V4 = BCD_4(ADC_Value,1   );

 }
 //****************************Timer Functions**************************
 void Timer0_init()
{
  // Set divider 
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &Timer0_Interrupt, true);     // Interrupt assignement
  timerAlarmWrite(My_timer, 1000, true);        //1000000 //1000
  timerAlarmEnable(My_timer); //Just Enable           // Enable timer 
}
//******************** Functions For Freq Timer ************************************************
 void IRAM_ATTR Timer1_Interrupt()  
{
  f1  = 1;      // set flag in interrupt 
}
void GPIO_Setup()
{
  // SET pin direction as output
    pinMode(PIN1, OUTPUT);              //  Make pin as output
    pinMode(PIN2, OUTPUT);              //  Make pin as output
  
    pinMode(INPUT_PIN_2,INPUT);                         // make pin as input 
    pinMode(INPUT_PIN_1,INPUT_PULLUP);                  // make pin as input pullup
    pinMode(INPUT_PIN_3,INPUT_PULLUP);                  // make pin as input pullup
//*****************************  ADC Channel Pins******************************
    pinMode(ADC_Pin,INPUT);                         // make pin as input 
    pinMode(ADC_Pin,INPUT_PULLDOWN);                // make pin as input pullup


    pinMode(FLOW_ADC_Pin,INPUT);
    pinMode(FLOW_ADC_Pin,INPUT_PULLDOWN);

    pinMode(Toggle_For_Coil_1, OUTPUT);              //  Make pin as output
    pinMode(Toggle_For_Coil_2, OUTPUT);              //  Make pin as output

    pinMode(PWM_1, OUTPUT);              //  Make pin as output
    //***************************** 19_02_24
    pinMode(MUX_C,OUTPUT);                         // make pin as input 
    pinMode(MUX_B,OUTPUT);                  // make pin as input pullup
    pinMode(MUX_A,OUTPUT);                  // make pin as input pullup

    pinMode(EMPTY_PIN,OUTPUT);            //PIN TO GENERATE EMPTY FLOW PULSE

    //PINS FOR CALIBRATION
    pinMode(Calibration_Mode,INPUT);
    pinMode(Calibration_Mode,INPUT_PULLUP);

    pinMode(ADD1,INPUT);
    pinMode(ADD1,INPUT_PULLUP);

    pinMode(ADD2,INPUT);    
    pinMode(ADD2,INPUT_PULLUP);

    pinMode(ADD3,INPUT);
    pinMode(ADD3,INPUT_PULLUP);

    pinMode(ADD4,INPUT);
    pinMode(ADD4,INPUT_PULLUP);

    pinMode(RS_485_RX_TX,OUTPUT);

    pinMode(GREEN_LED,OUTPUT);
    pinMode(RED_LED,OUTPUT);

     digitalWrite(GREEN_LED,LOW);
    digitalWrite(RED_LED,LOW);
}

void Timer1_init()
{
  // Set divider 
  Freq_timer = timerBegin(1, 80, true); 
  timerAttachInterrupt(Freq_timer, &Timer1_Interrupt, true);     // Interrupt assignement
   //Serial.println("Excitation_Frequency is");
   //Serial.print(Excitation_Frequency);
  switch(Excitation_Frequency)
  {
        case 1:
                timerAlarmWrite(Freq_timer,  DEFAULT_FREQENCY, true);         // assign timer value and enable it
                #if DEBUG_EN
                Serial.println("\n6.25hz Set \n");
                #endif
                timerAlarmEnable(Freq_timer); //Just Enable           // Enable timer 
                
            break;
        case 2:
                timerAlarmWrite(Freq_timer,  FREQENCY_12_5Hz, true);         // assign timer value and enable it
                #if DEBUG_EN
                Serial.println("12.5 hz Set \n");
                #endif
                timerAlarmEnable(Freq_timer); //Just Enable           // Enable timer 
                
            break;
        case 3:
                 timerAlarmWrite(Freq_timer, FREQENCY_25Hz , true);         // assign timer value and enable it
                #if DEBUG_EN
                Serial.println("25 hz Set \n");
                #endif
                timerAlarmEnable(Freq_timer); //Just Enable           // Enable timer 
            break;
        case 4:
                 timerAlarmWrite(Freq_timer, FREQENCY_37Hz , true);         // assign timer value and enable it
                #if DEBUG_EN
                Serial.println(" 37 hz Set \n");
                #endif
                timerAlarmEnable(Freq_timer); //Just Enable           // Enable timer 

            break;
        case 5:
                timerAlarmWrite(Freq_timer, FREQENCY_2Hz , true);         // assign timer value and enable it
                #if DEBUG_EN
                Serial.println(" 02hz Set \n");
                #endif
                timerAlarmEnable(Freq_timer); //Just Enable           // Enable timer 
            break;
        default:
               timerAlarmWrite(Freq_timer,  DEFAULT_FREQENCY, true);         // assign timer value and enable it
                #if DEBUG_EN
                Serial.println("6.25hz Set \n");
                #endif
                timerAlarmEnable(Freq_timer); //Just Enable           // Enable timer 

          break;
  }
  /*if(digitalRead(INPUT_PIN_1)==0)
  {
      timerAlarmWrite(Freq_timer,  FREQENCY_25Hz, true);          // assign timer value and enable it
      #if DEBUG_EN
      Serial.println("25 hz Set \n");
      #endif
      ADC_Avg_Num=25;
  }
 else if(digitalRead(INPUT_PIN_3)==0)                                   // added on 11_02_2023
  {
      timerAlarmWrite(Freq_timer,  FREQENCY_37Hz, true);          // assign timer value and enable it
      #if DEBUG_EN
      Serial.println("37 hz Set \n");
      #endif
      ADC_Avg_Num=37;
  }
  else 
  {
    if(digitalRead(INPUT_PIN_2)==1)
    {
          timerAlarmWrite(Freq_timer,  DEFAULT_FREQENCY, true);         // assign timer value and enable it
          #if DEBUG_EN
          Serial.println("6.25hz Set \n");
          #endif
          ADC_Avg_Num=6;
    }else  if(digitalRead(INPUT_PIN_2)==0)
    {
        timerAlarmWrite(Freq_timer,  FREQENCY_12_5Hz, true);         // assign timer value and enable it
        #if DEBUG_EN
        Serial.println("12.5hz Set \n");
        #endif
        ADC_Avg_Num=12;
    }
  }*/
  /* timerAlarmWrite(Freq_timer,  DEFAULT_FREQENCY, true);         // assign timer value and enable it
    #if DEBUG_EN
    Serial.println("6.25hz Set \n");
    #endif*/
    /* timerAlarmWrite(Freq_timer,  FREQENCY_2Hz, true);         // assign timer value and enable it
    #if DEBUG_EN
    Serial.println("2hz Set \n");
    #endif*/
       //  ADC_Avg_Num=10;
//timerAlarmEnable(Freq_timer); //Just Enable           // Enable timer 
}

void printIntArray(unsigned int* arr, int size) 
{
  for (int i = 0; i < size; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println();
}
//*-*-*-*-*-*-*-*-*-*-*-*-
 void Recall_Memory()
 {
        float retrievedFloatValue;
        unsigned char floatBytes3[5];
        EEPROM.begin(150);                                              // Initialize EEPROM
        for (int i = 0; i < EEPROM_SIZE; i++)           //24_02_24
        {
          EEPROM_Save[i] = EEPROM.read(EEPROM_ADDR + i);
        }
          Recieve_ADC_Read_Delay      = (EEPROM_Save[2] << 8) |EEPROM_Save[1];
          Recieve_ADC_Read_Stop_Delay = (EEPROM_Save[4] << 8) |EEPROM_Save[3];
          N_Samples_Set               = (EEPROM_Save[6] << 8) |EEPROM_Save[5];

         
          /*unsigned char floatBytes[]= {EEPROM_Save[10], EEPROM_Save[9], EEPROM_Save[8],EEPROM_Save[7]}; 
          memcpy(&Excitation_Frequency, floatBytes, sizeof(float));*/
          Excitation_Frequency= EEPROM_Save[7];
          //Serial.println("\nRecall excitation frequency");
          //Serial.println(Excitation_Frequency);
           unsigned char floatBytes1[] = {EEPROM_Save[11], EEPROM_Save[10], EEPROM_Save[9],EEPROM_Save[8]}; 
          memcpy(&Scale_Delta_Factor, floatBytes1, sizeof(float));

          Pipe_Size = EEPROM_Save[12];

          if(Pipe_Size == 1)
          {
              Serial.println("Pipe Size: 1  \n");
          }
          else if(Pipe_Size == 2)
          {
              Serial.println("Pipe Size: 2  \n");
          }
          if(Pipe_Size == 3)
          {
              Serial.println("Pipe Size: 3  \n");
          }
          //Pipe_Size = 1;

          //flow rate range selection
          if(Pipe_Size == 1)    //25nb
          {
                FLOW_RATE_RANGE = 10000;
                //PWM_RANGE = 417;
                Zero_Flow_rate_val = 400;
          }
          else if(Pipe_Size == 2) //40nb
          {
                FLOW_RATE_RANGE = 20000;
                //PWM_RANGE = 464;
                Zero_Flow_rate_val = 1000;
          }
          else if(Pipe_Size == 3) //50 nb
          {
                FLOW_RATE_RANGE = 45000;
                //PWM_RANGE = 500;
                Zero_Flow_rate_val = 1600;
          }
         // Pwm_Calculator_Val = float (FLOW_RATE_RANGE / PWM_RANGE);   //factor to calculate pwm frequency
          //Pipe_Size_Selection();

          unsigned char floatBytes2[] = {EEPROM_Save[16], EEPROM_Save[15], EEPROM_Save[14],EEPROM_Save[13]}; 
          memcpy(&PWM_Multiplier, floatBytes2, sizeof(float));

          Zero_Flow_Offset = (EEPROM_Save[18] << 8) |EEPROM_Save[17];
          Serial.println("\nZero_Flow_Offset");
          Serial.println(Zero_Flow_Offset);

          ADC_Avg_Num  =  (EEPROM_Save[20] << 8) |EEPROM_Save[19];       // No of samples for serail out

          PWM_RANGE =  (EEPROM_Save[22] << 8) |EEPROM_Save[21];
          //PWM_RANGE = 464;
          Pwm_Calculator_Val = float (FLOW_RATE_RANGE / PWM_RANGE);     //factor to calculate pwm frequency

          //zero flow rate value 
          Zero_Flow_rate_val = (EEPROM_Save[67] << 8) | EEPROM_Save[66];

          //EEPROM_Save[100] = 11;

          //Serial.println("Zero_Flow_rate_val:");
          //Serial.println(Zero_Flow_rate_val);

          if(EEPROM_Save[140] != 10) //to load default
          {
                Serial.println("setting default");

                PWM_RANGE =  417;
                Pwm_Calculator_Val = float (FLOW_RATE_RANGE / PWM_RANGE);   //factor to calculate pwm frequency

                EEPROM_Save[22] = (PWM_RANGE >>8) & 0xff;   // msb
                EEPROM_Save[21] = PWM_RANGE & 0xff;

                Pipe_Size = 1;
                Calibration_Points = 5;
                Zero_Flow_rate_val =  400;
                if(Pipe_Size == 1)
                {
                      Calibration_P1_start = 800;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P1_start = 1600;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P1_start = 3150;
                }
                P1_Adc_Count = 105;

                if(Pipe_Size == 1)
                {
                      Calibration_P2_start = 1500;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P2_start = 3000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P2_start = 6750;
                }
                P2_Adc_Count = 225;

                if(Pipe_Size == 1)
                {
                      Calibration_P3_start = 3000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P3_start = 6000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P3_start = 13500;
                }
                P3_Adc_Count = 450;

                if(Pipe_Size == 1)
                {
                      Calibration_P4_start = 6000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P4_start = 12000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P4_start = 27000;
                }
                P4_Adc_Count = 900;

                if(Pipe_Size == 1)
                {
                      Calibration_P5_start = 10000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P5_start = 20000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P5_start = 45000;
                } 
                P5_Adc_Count = 1500;

                if(Pipe_Size == 1)
                {
                      Calibration_P6_start = 10000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P6_start = 20000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P6_start = 45000;
                } 
                P6_Adc_Count = 1500;
                
                if(Pipe_Size == 1)
                {
                      Calibration_P7_start = 10000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P7_start = 20000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P7_start = 45000;
                }
                P7_Adc_Count = 1500;

                if(Pipe_Size == 1)
                {
                      Calibration_P8_start = 10000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P8_start = 20000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P8_start = 45000;
                }
                P8_Adc_Count = 1500;

                if(Pipe_Size == 1)
                {
                      Calibration_P9_start = 10000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P9_start = 20000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P9_start = 45000;
                } 
                P9_Adc_Count = 1500;

                 if(Pipe_Size == 1)
                {
                      Calibration_P10_start = 10000;
                }
                else if(Pipe_Size == 2)
                {
                      Calibration_P10_start = 20000;
                }
                else if(Pipe_Size == 3)
                {
                      Calibration_P10_start = 45000;
                }
                P10_Adc_Count = 1500;

                EEPROM_Save[25] = Calibration_Points;;

                EEPROM_Save[27] = (Calibration_P1_start >>8) & 0xff;   // msb
                EEPROM_Save[26] = Calibration_P1_start & 0xff;

                EEPROM_Save[29] = (P1_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[28] = P1_Adc_Count & 0xff;
                //-------------------------------------------------------------------------------------------
                EEPROM_Save[31] = (Calibration_P2_start >>8) & 0xff;   // msb
                EEPROM_Save[30] = Calibration_P2_start & 0xff;

                EEPROM_Save[33] = (P2_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[32] = P2_Adc_Count & 0xff;

                //-----------------------------------------------------------------------------------------------

                EEPROM_Save[35] = (Calibration_P3_start >>8) & 0xff;   // msb
                EEPROM_Save[34] = Calibration_P3_start & 0xff;

                EEPROM_Save[37] = (P3_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[36] = P3_Adc_Count & 0xff;

              //====================================================================================================
                EEPROM_Save[39] = (Calibration_P4_start >>8) & 0xff;   // msb
                EEPROM_Save[38] = Calibration_P4_start & 0xff;

                EEPROM_Save[41] = (P4_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[40] = P4_Adc_Count & 0xff;

              //====================================================================================================

                EEPROM_Save[43] = (Calibration_P5_start >>8) & 0xff;   // msb
                EEPROM_Save[42] = Calibration_P5_start & 0xff;

                EEPROM_Save[45] = (P5_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[44] = P5_Adc_Count & 0xff;

              //====================================================================================================
                EEPROM_Save[47] = (Calibration_P6_start >>8) & 0xff;   // msb
                EEPROM_Save[46] = Calibration_P6_start & 0xff;
                
                EEPROM_Save[49] = (P6_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[48] = P6_Adc_Count & 0xff;
              //====================================================================================================
                EEPROM_Save[51] = (Calibration_P7_start >>8) & 0xff;   // msb
                EEPROM_Save[50] = Calibration_P7_start & 0xff;

                EEPROM_Save[53] = (P7_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[52] = P7_Adc_Count & 0xff;

              //====================================================================================================
                EEPROM_Save[55] = (Calibration_P8_start >>8) & 0xff;   // msb
                EEPROM_Save[54] = Calibration_P8_start & 0xff;

                EEPROM_Save[57] = (P8_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[56] = P8_Adc_Count & 0xff;
              //====================================================================================================

                EEPROM_Save[59] = (Calibration_P9_start >>8) & 0xff;   // msb
                EEPROM_Save[58] = Calibration_P9_start & 0xff;

                EEPROM_Save[61] = (P9_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[60] = P9_Adc_Count & 0xff;

                //====================================================================================================

                EEPROM_Save[63] = (Calibration_P10_start >>8) & 0xff;   // msb
                EEPROM_Save[62] = Calibration_P10_start & 0xff;

                EEPROM_Save[65] = (P10_Adc_Count >>8) & 0xff;   // msb
                EEPROM_Save[64] = P10_Adc_Count & 0xff;

                //zero flow rate value 
                EEPROM_Save[67] = (Zero_Flow_rate_val >>8) & 0xff;   // msb
                EEPROM_Save[66] = Zero_Flow_rate_val & 0xff;

                EEPROM_Save[140] = 10;

              
                  for (int i = 0; i < EEPROM_SIZE; i++) 
                  {
                        EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
                  }
                  EEPROM.commit();
          }  
          else
          {
             // Pipe_Size = 2;
              //Serial.println("Reading from memory");
              Calibration_Points = EEPROM_Save[25];     //no of calibration points

              Calibration_P1_start =  (EEPROM_Save[27] << 8) |EEPROM_Save[26];    //cal1 start point
              P1_Adc_Count =  (EEPROM_Save[29] << 8) |EEPROM_Save[28];     //LOW ADC VALUE FOR ZONE 1 
              //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P2_start =  (EEPROM_Save[31] << 8) |EEPROM_Save[30];    //cal2 start point
              P2_Adc_Count =  (EEPROM_Save[33] << 8) |EEPROM_Save[32];     //LOW ADC VALUE FOR ZONE 1 
              //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P3_start =  (EEPROM_Save[35] << 8) |EEPROM_Save[34];    //cal3 start point
              P3_Adc_Count =  (EEPROM_Save[37] << 8) |EEPROM_Save[36];     //LOW ADC VALUE FOR ZONE 1 
              //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P4_start =  (EEPROM_Save[39] << 8) |EEPROM_Save[38];    //cal4 start point
              P4_Adc_Count =  (EEPROM_Save[41] << 8) |EEPROM_Save[40];     //LOW ADC VALUE FOR ZONE 1 
               //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P5_start =  (EEPROM_Save[43] << 8) |EEPROM_Save[42];    //cal5 start point
              P5_Adc_Count =  (EEPROM_Save[45] << 8) |EEPROM_Save[44];     //LOW ADC VALUE FOR ZONE 1 
               //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P6_start =  (EEPROM_Save[47] << 8) |EEPROM_Save[46];    //cal5 start point
              P6_Adc_Count =  (EEPROM_Save[49] << 8) |EEPROM_Save[48];     //LOW ADC VALUE FOR ZONE 1 
              //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P7_start =  (EEPROM_Save[51] << 8) |EEPROM_Save[50];    //cal5 start poin
              P7_Adc_Count =  (EEPROM_Save[53] << 8) |EEPROM_Save[52];     //LOW ADC VALUE FOR ZONE 1 
              //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P8_start =  (EEPROM_Save[55] << 8) |EEPROM_Save[54];    //cal5 start point
              P8_Adc_Count =  (EEPROM_Save[57] << 8) |EEPROM_Save[56];     //LOW ADC VALUE FOR ZONE 1 
              //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P9_start = (EEPROM_Save[59] << 8) |EEPROM_Save[58];    //cal9 start point
              P9_Adc_Count =  (EEPROM_Save[61] << 8) |EEPROM_Save[60];     //LOW ADC VALUE FOR ZONE 1 
              //----------------------------------------------------------------------------------------------------------------------------------
              Calibration_P10_start = (EEPROM_Save[63] << 8) |EEPROM_Save[62];       //cal10 start point
              //-------------------------------------------------------------------------------------------
              
          }
 }
//*-*-*-*-*-*- 19_02_24
 void Pipe_Size_Selection()
 {
   switch(Pipe_Size)
   {
            case 1:
              digitalWrite(MUX_A, 1);                              // ser pin 1 to high at first 
              digitalWrite(MUX_B, 0);                              // ser pin 1 to high at first 
              digitalWrite(MUX_C, 1);                              // ser pin 1 to high at first 
              Flow_Per_Pulse= 6.66;                               // 1 pulse =6.66ml
              break;
            case 2:
              digitalWrite(MUX_A, 0);                              // ser pin 1 to high at first 
              digitalWrite(MUX_B, 1);                              // ser pin 1 to high at first 
              digitalWrite(MUX_C, 1);                              // ser pin 1 to high at first 
              Flow_Per_Pulse= 12.0;                                // 1 pulse =17.00ml
              break;
            case 3:
              digitalWrite(MUX_A, 1);                              // ser pin 1 to high at first 
              digitalWrite(MUX_B, 1);                              // ser pin 1 to high at first 
              digitalWrite(MUX_C, 1);                              // ser pin 1 to high at first 
              Flow_Per_Pulse= 25.00;                                // 1 pulse =25.00ml
              break;

            default:
            break;
   }

 }
// This function generates a float number from values in buffer 
float  Form_Multiplier(unsigned char* str,unsigned int Start,unsigned int End )
 {
  // Convert unsigned char buffer to a char array
  char charArray[20];
  strncpy((char*)charArray, (char*)str, 20);
  

  //if (strncmp(charArray, "F", 11) == 0 && strstr(charArray, "@") != NULL)
  //if(str[11]=='F')
   //{

    unsigned char* numStart = str + Start;
    unsigned char* numEnd = str + (End+1);

   //unsigned char* numEnd = (unsigned char*)strstr((const char*)str, "@");

    
    // Extract the substring containing the number
    unsigned char numberString[numEnd - numStart + 1];
    strncpy((char*)numberString, (char*)numStart, numEnd - numStart);
    numberString[numEnd - numStart] = '\0'; // Null-terminate the string
    
    // Convert string to float
    float num = atof((char*)numberString);
    
    // Perform calculation
    float result = 1.0000 * num;
    //PWM_Multiplier=result;
   //Serial.println("Number formed");
  // Serial.print(result,4);
    return result;
  //}
  /* else 
   {
    Serial.println("Invalid string format!");
  }*/
}
void Update_Config()
{
  //unsigned char Config_Buffer[100]="$ST062 ET070 EF00.00 SD00.00 G1 F00.123&";
Config_Buffer[0] = '$'; // Start of string
Config_Buffer[1] ='S';
Config_Buffer[2] ='T';
Config_Buffer[3] = ((Recieve_ADC_Read_Delay / 100) % 10) | 0x30;  
Config_Buffer[4] = ((Recieve_ADC_Read_Delay / 10) % 10) | 0x30;    
Config_Buffer[5] = (Recieve_ADC_Read_Delay % 10) | 0x30;  

Config_Buffer[6] ='E';
Config_Buffer[7] ='T';
Config_Buffer[8] = ((Recieve_ADC_Read_Stop_Delay / 100) % 10) | 0x30;  
Config_Buffer[9] = ((Recieve_ADC_Read_Stop_Delay / 10) % 10) | 0x30;    
Config_Buffer[10] = (Recieve_ADC_Read_Stop_Delay % 10) | 0x30;    

//int intPart = (int)Excitation_Frequency;
//int fracPart = (int)((Excitation_Frequency - (int)Excitation_Frequency) * 100); // Assuming 4 decimal places

Config_Buffer[11] ='E'; 
Config_Buffer[12] ='F';
Config_Buffer[13] = Excitation_Frequency+0x30;                  
/*********************************************************************  
Config_Buffer[13] = 1 = 6.25 HZ
Config_Buffer[13] = 2 = 12.5HZ
Config_Buffer[13] = 3 = 25.00 HZ
Config_Buffer[13] = 4 = 37 HZ
Config_Buffer[13] = 5 = 2 HZ


/*********************************************************************  
/*Config_Buffer[13] = ((intPart / 10) % 10) | 0x30; 
Config_Buffer[14] =  (intPart % 10) | 0x30;   
Config_Buffer[15] = '.'; 
Config_Buffer[16] = ((fracPart / 10) % 10) | 0x30; 
Config_Buffer[17] = (fracPart % 10) | 0x30; */



 int intPart = (int)Scale_Delta_Factor;
 int fracPart = (int)((Scale_Delta_Factor - (int)Scale_Delta_Factor) * 100); // Assuming 2 decimal places
Config_Buffer[14] ='S';
Config_Buffer[15] ='D';
Config_Buffer[16] = ((intPart / 10) % 10) | 0x30; 
Config_Buffer[17] =  (intPart % 10) | 0x30;   
Config_Buffer[18] = '.'; 
Config_Buffer[19] = ((fracPart / 10) % 10) | 0x30; 
Config_Buffer[20] = (fracPart % 10) | 0x30;                     


Config_Buffer[21] ='G';
Config_Buffer[22] =Pipe_Size+0x30;   // 20_03_24   //23_03_24

float X = PWM_Multiplier*1.0000; // Example float value
intPart = (int)X;
fracPart = (int)((X - (int)X) * 10000); // Assuming 4 decimal places

Config_Buffer[23] ='M';
Config_Buffer[24] ='F';
Config_Buffer[25] = ((intPart / 10) % 10) | 0x30; 
Config_Buffer[26] =  (intPart % 10) | 0x30;   
Config_Buffer[27] = '.'; 
Config_Buffer[28] = ((fracPart / 1000) % 10) | 0x30; 
Config_Buffer[29] = ((fracPart / 100) % 10) | 0x30; 
Config_Buffer[30] = ((fracPart / 10) % 10) | 0x30; 
Config_Buffer[31] = (fracPart % 10) | 0x30; 
//************  19_03_24 *******************
Config_Buffer[32] = 'Z';
Config_Buffer[33] = 'F';
Config_Buffer[34] = ((Zero_Flow_Offset / 1000) % 10) | 0x30;  
Config_Buffer[35] = ((Zero_Flow_Offset / 100) % 10) | 0x30;  
Config_Buffer[36] =((Zero_Flow_Offset / 10) % 10) | 0x30;  
Config_Buffer[37] = (Zero_Flow_Offset % 10) | 0x30; 

Config_Buffer[38] = 'S';
Config_Buffer[39] = 'O';
Config_Buffer[40] = ((ADC_Avg_Num / 100) % 10) | 0x30;  
Config_Buffer[41] = ((ADC_Avg_Num / 10) % 10) | 0x30;    
Config_Buffer[42] = (ADC_Avg_Num % 10) | 0x30; 



//***************************************


Config_Buffer[43] = '&';

Serial.print((char*)Config_Buffer);   // only for testing purpose     // send data to LCD  every 1sec
Serial.println("\nOK");

//delay(5000);         // removed 23_02_24

if(Pipe_Size == 1)
{
    PWM_RANGE = 417;
}
else if(Pipe_Size == 2)
{
    PWM_RANGE = 464;
}
else if(Pipe_Size == 3)
{
    PWM_RANGE = 500;
}

}


void Write_Config_Received(unsigned char* Rx_Buffer)
{
                Recieve_ADC_Read_Delay   =  ((Rx_Buffer[3]&0x0f)*100); 
                Recieve_ADC_Read_Delay   =   Recieve_ADC_Read_Delay+((Rx_Buffer[4]&0x0f)*10);
                Recieve_ADC_Read_Delay   =   Recieve_ADC_Read_Delay+((Rx_Buffer[5]&0x0f)); 

                ADC_Read_Delay=0;

                EEPROM_Save[2] = (Recieve_ADC_Read_Delay>>8) & 0xff;   // msb
                EEPROM_Save[1]= Recieve_ADC_Read_Delay & 0xff;         //lsb
              //-------------------------------
                Recieve_ADC_Read_Stop_Delay  = ((Rx_Buffer[8]&0x0f)*100);  
                Recieve_ADC_Read_Stop_Delay  = Recieve_ADC_Read_Stop_Delay+((Rx_Buffer[9]&0x0f)*10);
                Recieve_ADC_Read_Stop_Delay  = Recieve_ADC_Read_Stop_Delay+((Rx_Buffer[10]&0x0f)); 
              
                ADC_Read_Stop_Delay=0;
                EEPROM_Save[4] = (Recieve_ADC_Read_Stop_Delay>>8) & 0xff;   // msb
                EEPROM_Save[3]= Recieve_ADC_Read_Stop_Delay & 0xff;         //lsb
              //-------------------------------------------------------------------------------------------------      
                 N_Samples_Set      = (Recieve_ADC_Read_Stop_Delay-Recieve_ADC_Read_Delay)-1;
                 N_Sample_Counter   = 0;
                EEPROM_Save[6] = (N_Samples_Set>>8) & 0xff;   // msb
                EEPROM_Save[5]= N_Samples_Set & 0xff;         //lsb
                //------------------- 25_03_24-----------------------------------------------------------------
                N_Sample_Counter_Flag=0;
                N_Sample_Counter=0;
                ADC_Avg_1=0;
                ADC_Avg_2=0;
                ADC_Count_1_Buff_Flag=0;
                ADC_Count_2_Buff_Flag=0;
                Var_2=0;
                Var_1=0;
                ADC_Count_2=0;       
                ADC_Count_1=0;
             
                //---------------------------------------------------------------------------------------------
               /* Serial.println("Start Time");
                Serial.print(Recieve_ADC_Read_Delay);
                Serial.println("Stop Time");
                Serial.print(Recieve_ADC_Read_Stop_Delay);
                 Serial.println("Number of samples");
                Serial.print(N_Samples_Set);*/

              //-----------------------------------22_03_24----------------------------------
               if((Rx_Buffer[11]=='E') && (Rx_Buffer[12]=='F'))
                Excitation_Frequency= Rx_Buffer[13]-0x30;
                   EEPROM_Save[7] = Excitation_Frequency;

            /*   Excitation_Frequency =  Form_Multiplier(Rx_Buffer,13,17);
              unsigned char *floatBytes=(unsigned char *)&Excitation_Frequency;
                EEPROM_Save[10] = floatBytes[0]; // Least significant byte
                EEPROM_Save[9] = floatBytes[1];
                EEPROM_Save[8] = floatBytes[2];
                EEPROM_Save[7] = floatBytes[3]; // Most significant byte*/


                Scale_Delta_Factor =  Form_Multiplier(Rx_Buffer,16,20);
                unsigned char *floatBytes = (unsigned char *)&Scale_Delta_Factor;
                EEPROM_Save[11] = floatBytes[0]; // Least significant byte
                EEPROM_Save[10] = floatBytes[1];
                EEPROM_Save[9] = floatBytes[2];
                EEPROM_Save[8] = floatBytes[3]; // Most significant byte

              //-------------------------------------------------
                      if((Rx_Buffer[21]=='G') && (Rx_Buffer[22]=='1'))
                      {
                       // Gain_1=1;
                       // Gain_2=0;
                       // Gain_3=0;
                       // EEPROM_Save[7]= Gain_1;  
                       // EEPROM_Save[8]= Gain_2;
                       // EEPROM_Save[9]= Gain_3;
                        Pipe_Size=1;
                      }
                     else if((Rx_Buffer[21]=='G') && (Rx_Buffer[22]=='2'))
                     {
                     // Gain_1=0;
                     // Gain_2=1;
                    //  Gain_3=0;
                     // EEPROM_Save[7]= Gain_1;  
                      //EEPROM_Save[8]= Gain_2;
                    //  EEPROM_Save[9]= Gain_3;
                       Pipe_Size=2;
                    }
                    else if((Rx_Buffer[21]=='G') && (Rx_Buffer[22]=='3'))
                   {
                  //Gain_1=0;
                 // Gain_2=0;
                 // Gain_3=1;
                  //EEPROM_Save[7]= Gain_1;  
                 // EEPROM_Save[8]= Gain_2;
                 // EEPROM_Save[9]= Gain_3;
                   Pipe_Size=3;
                  }
                EEPROM_Save[12]= Pipe_Size;


               PWM_Multiplier= Form_Multiplier(Rx_Buffer,25,31);
                floatBytes = (unsigned char *)&PWM_Multiplier;
                //Serial.println("Multiplier_PWM");
                //Serial.print(PWM_Multiplier);
               
                EEPROM_Save[16] = floatBytes[0]; // Least significant byte
                EEPROM_Save[15] = floatBytes[1];
                EEPROM_Save[14] = floatBytes[2];
                EEPROM_Save[13] = floatBytes[3]; // Most significant byte
                // EEPROM.put(10, PWM_Multiplier);
                 //EEPROM.commit();
               // ADC_Avg_Num=1; 

                ADC_Avg_Num   =  ((Rx_Buffer[34]&0x0f)*100); 
                ADC_Avg_Num   =   ADC_Avg_Num+((Rx_Buffer[35]&0x0f)*10);
                ADC_Avg_Num   =   ADC_Avg_Num+((Rx_Buffer[36]&0x0f));  
                
                EEPROM_Save[20] = (ADC_Avg_Num>>8) & 0xff;   // msb
                EEPROM_Save[19]= ADC_Avg_Num & 0xff;         //lsb

                EEPROM_Save[0]= 0x23;
                EEPROM_Save[21]= 0x40;            // was 14 22_03_24

                for (int i = 0; i < EEPROM_SIZE; i++) 
                {
                    EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
                }
               // EEPROM.put(10, PWM_Multiplier);
                EEPROM.commit();
               // Serial.println("\nOK");
            //   Update_Config_Flag=1;                     //   25_03_24
                Config_Write=0;
              

}
void Flow_Rate_Calculation()
{
      long int temp = 0;
      
      if(Delta <= Zero_Flow_Offset)    //19_02_24 Delta<=50 indicating zero flow //50//250
      {
            Flow_Rate =0;
            PWM_Freq=0; 
            ledcWrite(0 ,0);       // 13_03_24   To make it zero
      }
      else
      {
            FLOWRATE_ADC_LOW_VAL = Zero_Flow_Offset;

           // Delta    = Delta - Zero_Flow_Offset;
           Delta    = Delta;

            if(Delta <= FLOWRATE_ADC_LOW_VAL)
            {
                  Flow_Rate = 0;
            }
            else
            {
                  Calibrate_PWM();

                  //if(Flow_Rate > 0)
                  if(Flow_Rate > Zero_Flow_rate_val)
                  {
                        //PWM_Freq = (unsigned int)Flow_Rate / Pwm_Calculator_Val;

                        PWM_Freq = (float)Flow_Rate / (float)Pwm_Calculator_Val;

                        //PWM_Freq = (float)Flow_Rate / 23.98;

                        Serial.print(Pwm_Calculator_Val); //testing only

                       // PWM_Freq = PWM_Freq - 14;

                        ledcSetup(ledChannel, PWM_Freq, resolution);
            
                        ledcWrite(0, 4095); 

                        Low_Flow_Flag = 0;
                  }
                  else
                  {
                        Low_Flow_Flag = 1;
                        PWM_Freq=0; 
                        ledcWrite(0 ,0);       // 13_03_24   To make it zero
                  }
            }    
      }

      //Serial.println("\n calculated flow");
      int intPart = (int)Flow_Rate;
      int fracPart = (int)((Flow_Rate - (int)Flow_Rate) * 10000); // Assuming 3 decimal places
      Flow_Rate_Buffer[0] ='#';
      Flow_Rate_Buffer[1] ='F';
      Flow_Rate_Buffer[2]= 'R';
      Flow_Rate_Buffer[3] = ((intPart / 10000) % 10) | 0x30;
      Flow_Rate_Buffer[4] = ((intPart / 1000) % 10) | 0x30;  
      Flow_Rate_Buffer[5] = ((intPart / 100) % 10) | 0x30; 
      Flow_Rate_Buffer[6] = ((intPart / 10) % 10) | 0x30;  
      Flow_Rate_Buffer[7] =  (intPart % 10) | 0x30;
      Flow_Rate_Buffer[8] = '.';
      Flow_Rate_Buffer[9] = ((fracPart / 10) % 10) | 0x30; 
      Flow_Rate_Buffer[10] = (fracPart % 10) | 0x30;
      Flow_Rate_Buffer[11] ='@';

          V1 = BCD_4(PWM_Freq,1000);
          V2 = BCD_4(PWM_Freq,100 );
          V3 = BCD_4(PWM_Freq,10  );
          V4 = BCD_4(PWM_Freq,1   );
          Pulse_Freq[0]='#';
          Pulse_Freq[1]='P';
          Pulse_Freq[2]='W';
          Pulse_Freq[3]='M';
          Pulse_Freq[4]=V1+0x30;
          Pulse_Freq[5]=V2+0x30;
          Pulse_Freq[6]=V3+0x30;
          Pulse_Freq[7]=V4+0x30;
          Pulse_Freq[8]='@';
}

void Check_Calibration_string(void)
{

            //Serial.print((char*)Rx_Buffer);
             unsigned int temp,temp1,final;
             unsigned char val1,val2,val3,val4,val5,val6;
             float floatval;
             long int final1;
             char buffer[50];
             unsigned char counter = 0;

              //calibration points
              //#CAL,P10,1)00000,01000,2)01001,02000,3)02001,03000,4)03001,04000,5)04001,05000,6)05001,06000,7)06001,07000,8)07001,08000,9)08001,09000,10)09001,10000@
              //Serial.println((char *)Rx_Buffer);

             // while(1);
              temp = Rx_Buffer[6] - 0x30;
              temp1 = Rx_Buffer[7] - 0x30;
              final = (temp * 10) + temp1;
              if(final >= 1 && final <= 10)
              {
                  EEPROM_Save[25] = Calibration_Points = (unsigned char)final;
              }
              
              //Serial.println("Calibration points:");
              //Serial.println(Calibration_Points);
              //Serial.println(" \n");

              //cal point-1 start
              val1 = Rx_Buffer[11] - 0x30;
              val2 = Rx_Buffer[12] - 0x30;
              val3 = Rx_Buffer[13] - 0x30;
              val4 = Rx_Buffer[14] - 0x30;
              val5 = Rx_Buffer[15] - 0x30;

              final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

              //if(final >= 0 && final <= 45000)
              {
                    Calibration_P1_start = final;

                    EEPROM_Save[27] = (Calibration_P1_start >>8) & 0xff;   // msb
                    EEPROM_Save[26] = Calibration_P1_start & 0xff;
              }

              if(Calibration_Points >= 1)
              {
                  val1 = Rx_Buffer[17] - 0x30;
                  val2 = Rx_Buffer[18] - 0x30;
                  val3 = Rx_Buffer[19] - 0x30;
                  val4 = Rx_Buffer[20] - 0x30;
                  val5 = Rx_Buffer[21] - 0x30;

                  P1_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                  EEPROM_Save[29] = (P1_Adc_Count >>8) & 0xff;   // msb
                  EEPROM_Save[28] = P1_Adc_Count & 0xff;
              }
              //----------------------------------------------------------------------
              if(Calibration_Points >= 2)
              {
                    //cal point-2 start
                    val1 = Rx_Buffer[25] - 0x30;
                    val2 = Rx_Buffer[26] - 0x30;
                    val3 = Rx_Buffer[27] - 0x30;
                    val4 = Rx_Buffer[28] - 0x30;
                    val5 = Rx_Buffer[29] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= 45000)
                    {
                          Calibration_P2_start = final;

                          EEPROM_Save[31] = (Calibration_P2_start >>8) & 0xff;   // msb
                          EEPROM_Save[30] = Calibration_P2_start & 0xff;
                    }
              
                    val1 = Rx_Buffer[31] - 0x30;
                    val2 = Rx_Buffer[32] - 0x30;
                    val3 = Rx_Buffer[33] - 0x30;
                    val4 = Rx_Buffer[34] - 0x30;
                    val5 = Rx_Buffer[35] - 0x30;

                  P2_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                  EEPROM_Save[33] = (P2_Adc_Count >>8) & 0xff;   // msb
                  EEPROM_Save[32] = P2_Adc_Count & 0xff;

              }

              //----------------------------------------------------------------------
              if(Calibration_Points >= 3)
              {
                    //cal point-3 start
                    val1 = Rx_Buffer[39] - 0x30;
                    val2 = Rx_Buffer[40] - 0x30;
                    val3 = Rx_Buffer[41] - 0x30;
                    val4 = Rx_Buffer[42] - 0x30;
                    val5 = Rx_Buffer[43] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= 45000)
                    {
                          Calibration_P3_start = final;

                          EEPROM_Save[35] = (Calibration_P3_start >>8) & 0xff;   // msb
                          EEPROM_Save[34] = Calibration_P3_start & 0xff;
                    }
              
                    val1 = Rx_Buffer[45] - 0x30;
                    val2 = Rx_Buffer[46] - 0x30;
                    val3 = Rx_Buffer[47] - 0x30;
                    val4 = Rx_Buffer[48] - 0x30;
                    val5 = Rx_Buffer[49] - 0x30;

                    P3_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5; 

                    EEPROM_Save[37] = (P3_Adc_Count >>8) & 0xff;   // msb
                    EEPROM_Save[36] = P3_Adc_Count & 0xff;
                  
              }

              //----------------------------------------------------------------------
              if(Calibration_Points >= 4)
              {
                    //cal point-4 start
                    val1 = Rx_Buffer[53] - 0x30;
                    val2 = Rx_Buffer[54] - 0x30;
                    val3 = Rx_Buffer[55] - 0x30;
                    val4 = Rx_Buffer[56] - 0x30;
                    val5 = Rx_Buffer[57] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= 45000)
                    {
                          Calibration_P4_start = final;

                          EEPROM_Save[39] = (Calibration_P4_start >>8) & 0xff;   // msb
                          EEPROM_Save[38] = Calibration_P4_start & 0xff;
                    }

                    val1 = Rx_Buffer[59] - 0x30;
                    val2 = Rx_Buffer[60] - 0x30;
                    val3 = Rx_Buffer[61] - 0x30;
                    val4 = Rx_Buffer[62] - 0x30;
                    val5 = Rx_Buffer[63] - 0x30;

                    P4_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5; 

                    EEPROM_Save[41] = (P4_Adc_Count >>8) & 0xff;   // msb
                    EEPROM_Save[40] = P4_Adc_Count & 0xff;
                 
              }

              //----------------------------------------------------------------------
              if(Calibration_Points >= 5)
              {

                    //Serial.println("Inside cal point 5");

                    //cal point-5 start
                    val1 = Rx_Buffer[67] - 0x30;
                    val2 = Rx_Buffer[68] - 0x30;
                    val3 = Rx_Buffer[69] - 0x30;
                    val4 = Rx_Buffer[70] - 0x30;
                    val5 = Rx_Buffer[71] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= 50000)
                    {
                          Calibration_P5_start = final;

                          EEPROM_Save[43] = (Calibration_P5_start >>8) & 0xff;   // msb
                          EEPROM_Save[42] = Calibration_P5_start & 0xff;
                    }
              
                    val1 = Rx_Buffer[73] - 0x30;
                    val2 = Rx_Buffer[74] - 0x30;
                    val3 = Rx_Buffer[75] - 0x30;
                    val4 = Rx_Buffer[76] - 0x30;
                    val5 = Rx_Buffer[77] - 0x30;

                    P5_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    EEPROM_Save[45] = (P5_Adc_Count >>8) & 0xff;   // msb
                    EEPROM_Save[44] = P5_Adc_Count & 0xff;
                    
              }

              //----------------------------------------------------------------------
              if(Calibration_Points >= 6)
              {
                    //cal point-6 start
                    val1 = Rx_Buffer[81] - 0x30;
                    val2 = Rx_Buffer[82] - 0x30;
                    val3 = Rx_Buffer[83] - 0x30;
                    val4 = Rx_Buffer[84] - 0x30;
                    val5 = Rx_Buffer[85] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= )
                    {
                          Calibration_P6_start = final;

                          EEPROM_Save[47] = (Calibration_P6_start >>8) & 0xff;   // msb
                          EEPROM_Save[46] = Calibration_P6_start & 0xff;
                    }
              
                    val1 = Rx_Buffer[87] - 0x30;
                    val2 = Rx_Buffer[88] - 0x30;
                    val3 = Rx_Buffer[89] - 0x30;
                    val4 = Rx_Buffer[90] - 0x30;
                    val5 = Rx_Buffer[91] - 0x30;

                    P6_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5; 

                    EEPROM_Save[49] = (P6_Adc_Count >>8) & 0xff;   // msb
                    EEPROM_Save[48] = P6_Adc_Count & 0xff;
              }

              //----------------------------------------------------------------------
              if(Calibration_Points >= 7)
              {
                    //cal point-7 start
                    val1 = Rx_Buffer[95] - 0x30;
                    val2 = Rx_Buffer[96] - 0x30;
                    val3 = Rx_Buffer[97] - 0x30;
                    val4 = Rx_Buffer[98] - 0x30;
                    val5 = Rx_Buffer[99] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= 45000)
                    {
                          Calibration_P7_start = final;

                          EEPROM_Save[51] = (Calibration_P7_start >>8) & 0xff;   // msb
                          EEPROM_Save[50] = Calibration_P7_start & 0xff;
                    }
              
                    val1 = Rx_Buffer[101] - 0x30;
                    val2 = Rx_Buffer[102] - 0x30;
                    val3 = Rx_Buffer[103] - 0x30;
                    val4 = Rx_Buffer[104] - 0x30;
                    val5 = Rx_Buffer[105] - 0x30;

                    P7_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    EEPROM_Save[53] = (P7_Adc_Count >>8) & 0xff;   // msb
                    EEPROM_Save[52] = P7_Adc_Count & 0xff;
              }

              //----------------------------------------------------------------------
              if(Calibration_Points >= 8)
              {
                    //cal point-8 start
                    val1 = Rx_Buffer[109] - 0x30;
                    val2 = Rx_Buffer[110] - 0x30;
                    val3 = Rx_Buffer[111] - 0x30;
                    val4 = Rx_Buffer[112] - 0x30;
                    val5 = Rx_Buffer[113] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= 45000)
                    {
                          Calibration_P8_start = final;

                          EEPROM_Save[55] = (Calibration_P8_start >>8) & 0xff;   // msb
                          EEPROM_Save[54] = Calibration_P8_start & 0xff;
                    }
              
                    val1 = Rx_Buffer[115] - 0x30;
                    val2 = Rx_Buffer[116] - 0x30;
                    val3 = Rx_Buffer[117] - 0x30;
                    val4 = Rx_Buffer[118] - 0x30;
                    val5 = Rx_Buffer[119] - 0x30;

                    P8_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    EEPROM_Save[57] = (P8_Adc_Count >>8) & 0xff;   // msb
                    EEPROM_Save[56] = P8_Adc_Count & 0xff;
                    
              }

              //----------------------------------------------------------------------
              if(Calibration_Points >= 9)
              {
                    //cal point-9 start
                    val1 = Rx_Buffer[123] - 0x30;
                    val2 = Rx_Buffer[124] - 0x30;
                    val3 = Rx_Buffer[125] - 0x30;
                    val4 = Rx_Buffer[126] - 0x30;
                    val5 = Rx_Buffer[127] - 0x30;

                    final = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    //if(final >= 0 && final <= 45000)
                    {
                          Calibration_P9_start = final;

                          EEPROM_Save[59] = (uint8_t)(Calibration_P9_start >>8) & 0xff;   // msb
                          EEPROM_Save[58] = (uint8_t)Calibration_P9_start & 0xff;

                    }
              
                    val1 = Rx_Buffer[129] - 0x30;
                    val2 = Rx_Buffer[130] - 0x30;
                    val3 = Rx_Buffer[131] - 0x30;
                    val4 = Rx_Buffer[132] - 0x30;
                    val5 = Rx_Buffer[133] - 0x30;

                    P9_Adc_Count = final1 = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    EEPROM_Save[61] = (P9_Adc_Count >>8) & 0xff;   // msb
                    EEPROM_Save[60] = P9_Adc_Count & 0xff;

              }

              for (int i = 0; i < EEPROM_SIZE; i++) 
              {
                    EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
              }
              // EEPROM.put(10, PWM_Multiplier);
              EEPROM.commit();

              //Serial.println(Rx_Buffer);

              //while(1);
}
void SEND_CAL_DATA(void)
{
      char buffer[250];
      unsigned char count;
      unsigned int i;
      unsigned char buffer_counter = 0;
      //String convVal;


      //Serial.println("FILLING STRING  \n");
      buffer[buffer_counter] = '*';

      CONVERT_INTEGER_ASCII(Calibration_Points);
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '1';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P1_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P1_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;


      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '2';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P2_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P2_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '3';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P3_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P3_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '4';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P4_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P4_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '5';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P5_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P5_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '6';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P6_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P6_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '7';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P7_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P7_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '8';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P8_start);
     buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P8_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '9';
       buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P9_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(P9_Adc_Count);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = ',';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = '1';
      buffer[++buffer_counter] = '0';
      buffer[++buffer_counter] = ':';

      CONVERT_INTEGER_ASCII(Calibration_P10_start);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      CONVERT_INTEGER_ASCII(FLOWRATE_ADC_HIGH_VAL);
      buffer[++buffer_counter] = LOAD2;
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;

      buffer[++buffer_counter] = '@';

       //Serial.println("SENDING STRING  \n");

       Serial.println(buffer);

      //while(1);
      
      
}
void CONVERT_INTEGER_ASCII(unsigned long int TEMP1)
{
      unsigned long int TEMP = 0;
      unsigned char POS = 0;

      TEMP = TEMP1;

      TEMP = TEMP % 1000000;
      POS = TEMP / 100000;
      LOAD1 = 0x30 + POS;

      TEMP = TEMP % 100000;
      POS = TEMP / 10000;
      LOAD2 = 0x30 + POS;

      TEMP = TEMP % 10000;
      POS = TEMP / 1000;
      LOAD3 = 0x30 + POS;

      TEMP = TEMP % 1000;
      POS = TEMP / 100;
      LOAD4 = 0x30 + POS;

      TEMP = TEMP % 100;
      POS = TEMP / 10;
      LOAD5 = 0x30 + POS;

      POS = TEMP % 10;
      LOAD6 = 0x30 + POS;
}
void Calibrate_PWM(void)
{
      float NA_Flow=800.0;
      unsigned int temp,temp1;
      unsigned int Start_flow,End_flow,Start_Adc,End_Adc;

        /*switch(Pipe_Size)
        {
                case 1:
                NA_Flow=883 ;
                break;
                case 2:
                NA_Flow=2171;
                break;
                case 3:
                NA_Flow=3256;
                break;
                default:

                break;
        }*/

        /*if(Flow_Rate<= NA_Flow)
        {
          Calibrated_Multiplier=0;
        }
        
        else */

        if(Calibration_Points == 1)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P1_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P1_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P1_start = 45000;
              }*/
              FLOWRATE_ADC_HIGH_VAL = P1_Adc_Count;
        }
        else if(Calibration_Points == 2)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P2_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P2_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P2_start = 45000;
              }*/
              FLOWRATE_ADC_HIGH_VAL = P2_Adc_Count;
        }
        else if(Calibration_Points == 3)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P3_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P3_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P3_start = 45000;
              }*/
             FLOWRATE_ADC_HIGH_VAL = P3_Adc_Count;
        }
        else if(Calibration_Points == 4)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P4_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P4_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P4_start = 45000;
              }*/
              FLOWRATE_ADC_HIGH_VAL = P4_Adc_Count;
        }
        else if(Calibration_Points == 5)
        {
             /* if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P5_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P5_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P5_start = 45000;
              }*/
             FLOWRATE_ADC_HIGH_VAL =  P5_Adc_Count;
        }
        else if(Calibration_Points == 6)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P6_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P6_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P6_start = 45000;
              }*/
             FLOWRATE_ADC_HIGH_VAL = P6_Adc_Count;
        }
        else if(Calibration_Points == 7)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P7_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P7_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P7_start = 45000;
              }*/
              FLOWRATE_ADC_HIGH_VAL = P7_Adc_Count;
        }
        else if(Calibration_Points == 8)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P8_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P8_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P8_start = 45000;
              }*/
              FLOWRATE_ADC_HIGH_VAL = P8_Adc_Count;
        }
        else if(Calibration_Points == 9)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P9_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P9_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P9_start = 45000;
              }*/
              FLOWRATE_ADC_HIGH_VAL = P9_Adc_Count;
        }
        else if(Calibration_Points == 10)
        {
              /*if(Pipe_Size == 1)    //25nb
              {
                  Calibration_P10_start = 10000;
              }
              else if(Pipe_Size == 2) //40nb
              {
                  Calibration_P10_start = 20000;
              }
              else if(Pipe_Size == 3) //50 nb
              {
                  Calibration_P10_start = 45000;
              }*/
             FLOWRATE_ADC_HIGH_VAL = P10_Adc_Count;
        }
        FLOWRATE_ADC_LOW_VAL = Zero_Flow_Offset;

        if(Delta > FLOWRATE_ADC_HIGH_VAL)
        {
                 if(Calibration_Points == 1)
                 {
                        Start_flow = Calibration_P1_start;
                        End_flow = 0;
                        Start_Adc = P1_Adc_Count;
                        End_Adc = 0;
                 }
                 else if(Calibration_Points == 2)
                 {
                        Start_flow = Calibration_P2_start;
                        End_flow = Calibration_P1_start;
                        Start_Adc = P2_Adc_Count;
                        End_Adc = P1_Adc_Count;
                 }
                else if(Calibration_Points == 3)
                 {
                        Start_flow = Calibration_P3_start;
                        End_flow = Calibration_P2_start;
                        Start_Adc = P3_Adc_Count;
                        End_Adc = P2_Adc_Count;
                 }
                 else if(Calibration_Points == 4)
                 {
                        Start_flow = Calibration_P4_start;
                        End_flow = Calibration_P3_start;
                        Start_Adc = P4_Adc_Count;
                        End_Adc = P3_Adc_Count;
                 }
                 else if(Calibration_Points == 5)
                 {
                        Start_flow = Calibration_P5_start;
                        End_flow = Calibration_P4_start;
                        Start_Adc = P5_Adc_Count;
                        End_Adc = P4_Adc_Count;
                 }
                 else if(Calibration_Points == 6)
                 {
                        Start_flow = Calibration_P6_start;
                        End_flow = Calibration_P5_start;
                        Start_Adc = P6_Adc_Count;
                        End_Adc = P5_Adc_Count;
                 }
                 else if(Calibration_Points == 7)
                 {
                        Start_flow = Calibration_P7_start;
                        End_flow = Calibration_P6_start;
                        Start_Adc = P7_Adc_Count;
                        End_Adc = P6_Adc_Count;
                 }
                 else if(Calibration_Points == 8)
                 {
                        Start_flow = Calibration_P8_start;
                        End_flow = Calibration_P7_start;
                        Start_Adc = P8_Adc_Count;
                        End_Adc = P7_Adc_Count;
                 }
                 else if(Calibration_Points == 9)
                 {
                        Start_flow = Calibration_P9_start;
                        End_flow = Calibration_P8_start;
                        Start_Adc = P9_Adc_Count;
                        End_Adc = P8_Adc_Count;
                 }
                 else if(Calibration_Points == 10)
                 {
                        Start_flow = Calibration_P10_start;
                        End_flow = Calibration_P9_start;
                        Start_Adc = P10_Adc_Count;
                        End_Adc = P9_Adc_Count;
                 }
                
                temp = (Start_flow - End_flow);
                temp = temp + Start_flow;

                temp1 = Start_Adc - End_Adc;
                temp1 = temp1 + Start_Adc;
                Flow_Rate = Calculate_Flow_Rate(Start_Adc,temp1,Start_flow,temp);


                 //temp = (Calibration_P5_start - Calibration_P4_start);
                 //temp = temp + Calibration_P5_start;

                 //temp1 = P5_Adc_Count - P4_Adc_Count;
                 //temp1 = temp1 + P5_Adc_Count;
                 //Flow_Rate = Calculate_Flow_Rate(P5_Adc_Count,temp1,Calibration_P5_start,temp);
        }
        else if (Delta >= FLOWRATE_ADC_LOW_VAL && Delta < P1_Adc_Count && Calibration_Points >= 1)
        {
                Flow_Rate = Calculate_Flow_Rate(FLOWRATE_ADC_LOW_VAL,P1_Adc_Count,0,Calibration_P1_start);
                Serial.print("In zone-1"); 
                
        }
        else if (Delta > P1_Adc_Count && Delta <= P2_Adc_Count && Calibration_Points >= 2)
        {
               Flow_Rate = Calculate_Flow_Rate(P1_Adc_Count,P2_Adc_Count,Calibration_P1_start,Calibration_P2_start);
               Serial.print("In zone-2");
        }
        else if (Delta > P2_Adc_Count && Delta <= P3_Adc_Count && Calibration_Points >= 3)
        {
               Flow_Rate = Calculate_Flow_Rate(P2_Adc_Count,P3_Adc_Count,Calibration_P2_start,Calibration_P3_start);
               Serial.print("In zone-3");
        }
        else if (Delta > P3_Adc_Count && Delta <= P4_Adc_Count && Calibration_Points >= 4)
        {
               Flow_Rate = Calculate_Flow_Rate(P3_Adc_Count,P4_Adc_Count,Calibration_P3_start,Calibration_P4_start);
               Serial.print("In zone-4");
        }
        else if (Delta > P4_Adc_Count && Delta <= P5_Adc_Count && Calibration_Points >= 5)
        {
               Flow_Rate = Calculate_Flow_Rate(P4_Adc_Count,P5_Adc_Count,Calibration_P4_start,Calibration_P5_start);
               Serial.print("In zone-5");
        }
        else if (Delta > P5_Adc_Count && Delta <= P6_Adc_Count && Calibration_Points >= 6)
        {
               Flow_Rate = Calculate_Flow_Rate(P5_Adc_Count,P6_Adc_Count,Calibration_P5_start,Calibration_P6_start);
               Serial.print("In zone-6");
        }
        else if (Delta > P6_Adc_Count && Delta <= P7_Adc_Count && Calibration_Points >= 7)
        {
               Flow_Rate = Calculate_Flow_Rate(P6_Adc_Count,P7_Adc_Count,Calibration_P6_start,Calibration_P7_start);
               Serial.print("In zone-7");
        }
        else if (Delta > P7_Adc_Count && Delta <= P8_Adc_Count && Calibration_Points >= 8)
        {
               Flow_Rate = Calculate_Flow_Rate(P7_Adc_Count,P8_Adc_Count,Calibration_P7_start,Calibration_P8_start);
        }
        else if (Delta > P8_Adc_Count && Delta <= P9_Adc_Count && Calibration_Points >= 9)
        {
               Flow_Rate = Calculate_Flow_Rate(P8_Adc_Count,P9_Adc_Count,Calibration_P8_start,Calibration_P9_start);
               Serial.print("In zone-8");
        }
        else if (Delta > P9_Adc_Count && Delta <= P10_Adc_Count && Calibration_Points >= 10)
        {
               Flow_Rate = Calculate_Flow_Rate(P9_Adc_Count,P10_Adc_Count,Calibration_P9_start,Calibration_P10_start);
               Serial.print("In zone-9");
        }
        /*else if (Delta > P10_Adc_Count && )
        {
              Flow_Rate = Calculate_Flow_Rate(P9_Adc_Count,P10_Adc_Count,Calibration_P9_start,Calibration_P10_start);
              Serial.print("In zone-10");
        }*/
        
}

unsigned int Calculate_Flow_Rate(unsigned int adc_low,unsigned int adc_high,unsigned int flow_low ,unsigned int flow_high)
{

    unsigned int span = adc_high - adc_low;
    unsigned int offset = flow_low;
    unsigned int flow;
    long int temp;
    unsigned int flow_size = flow_high - flow_low;


    if(Delta <= adc_low)
    {
        Delta = adc_low + 1;
    }

    temp = (Delta - adc_low) * flow_size;
    flow = temp / span;
    flow = flow + offset;

    return flow;
}

unsigned int Calculate_Flow_Rate1(unsigned int adc_low,unsigned int adc_high,unsigned int flow_low ,unsigned int flow_high)
{

    unsigned int span = adc_high - adc_low;
    unsigned int offset; //= //flow_size;
    unsigned int flow;
    long int temp;
    unsigned int flow_size = FLOW_RATE_RANGE;


    if(Delta <= adc_low)
    {
        Delta = adc_low + 1;
    }

    temp = (Delta - adc_low) * flow_size;
    flow = temp / span;
    flow = flow + offset;

    return flow;
}

 void Set_Frequency(void)
 {
      //#FC1234@
      unsigned char val1,val2,val3,val4;
      unsigned int final;

      Serial.println("SETTING FREQUENCY  \n");

      val1 = Rx_Buffer[3] - 0x30;
      val2 = Rx_Buffer[4] - 0x30;
      val3 = Rx_Buffer[5] - 0x30;
      val4 = Rx_Buffer[6] - 0x30;

      final = (val1 * 1000) + (val2 * 100) + (val3 * 10) + val4;

      PWM_RANGE = final;

      Pwm_Calculator_Val = float (FLOW_RATE_RANGE / PWM_RANGE);   //factor to calculate pwm frequency

      EEPROM_Save[22] = (PWM_RANGE >>8) & 0xff;   // msb
      EEPROM_Save[21] = PWM_RANGE & 0xff;

      for (int i = 0; i < EEPROM_SIZE; i++) 
      {
            EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
      }
      // EEPROM.put(10, PWM_Multiplier);
      EEPROM.commit();
      
 }
void Send_Freq(void)
{
      unsigned char buffer_counter = 0;
      char buffer[10];

      Serial.println("SENDING FREQUENCY  \n");

      buffer[buffer_counter] = '*';
      CONVERT_INTEGER_ASCII(PWM_RANGE);
  
      buffer[++buffer_counter] = LOAD3;
      buffer[++buffer_counter] = LOAD4;
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = '@';

      Serial.println(buffer);
}

void IRAM_ATTR onTimer() 
{
    Check_Empty_Flow = 1;  // Set flag when timer fires
}

void Timer2_init(void)
{
      // Create a timer (Timer 0, Divider = 80, Count Up)
    Empty_Flow_timer = timerBegin(2, 80, true);  // 80MHz / 80 = 1MHz (1µs per tick)

    // Attach the ISR function
    timerAttachInterrupt(Empty_Flow_timer, &onTimer, true);

    // Set timer alarm every 1 min (1000000 µs)
    timerAlarmWrite(Empty_Flow_timer, 30000000, true);
    //timerAlarmWrite(Empty_Flow_timer, 10000000, true);

    // Enable the timer
    timerAlarmEnable(Empty_Flow_timer);
}

void IRAM_ATTR caluartISR(void)
{
    Rx_Cal_Int_Flag=1;
}

void Read_485_Address(void)
{
     
     unsigned char add1,add2,add3,add4;

     add1 = digitalRead(ADD1);
     add2 = digitalRead(ADD2);
     add4 = digitalRead(ADD3);
     add3 = digitalRead(ADD4);

     
     if(!add4 && !add3 && !add2 && add1)
     {
            RS_485_addr = 1;
     }
     else if(!add4 && !add3 && add2 && !add1)
     {
            RS_485_addr = 2;
     }
     else if(!add4 && !add3 && add2 && add1)
     {
            RS_485_addr = 3;
     }
     else if(!add4 && add3 && !add2 && !add1)
     {
            RS_485_addr = 4;
     }
     else if(!add4 && add3 && !add2 && add1)
     {
            RS_485_addr = 5;
     }
     else if(!add4 && add3 && add2 && !add1)
     {
            RS_485_addr = 6;
     }
     else if(!add4 && add3 && add2 && add1)
     {
            RS_485_addr = 7;
     }
     else if(add4 && !add3 && !add2 && !add1)
     {
            RS_485_addr = 8;
     }
     else if(add4 && !add3 && !add2 && add1)
     {
            RS_485_addr = 9;
     }
      else if(add4 && !add3 && add2 && !add1)
     {
            RS_485_addr = 10;
     }
    // RS_485_addr = 1;
}

void Calibration_Process(void)
{
      unsigned char val1,val2,val3,val4,val5;
      unsigned int final_val;
      unsigned char count,count1;

          if(Cal_Rcvd_Data[1] == 'C' && Cal_Rcvd_Data[2] == 'F')
          {
                  Read_Configuration();
          }
          if(Cal_Rcvd_Data[1] == 'C' && Cal_Rcvd_Data[2] == 'S')
          {
                Calibration_on = 1;
                Calibration_counter = 0;
          }
          else if(Cal_Rcvd_Data[1] == 'C' && Cal_Rcvd_Data[2] == 'E')
          {
                  Calibration_on = 0;

                  Store_calb_data();
          }
          else if(Cal_Rcvd_Data[1] == 'F')
          {
                
                val1 = Cal_Rcvd_Data[2] - 0x30; 
                val2 = Cal_Rcvd_Data[3] - 0x30;
                Calibration_counter = (val1 * 10) + val2;

                val1 = Cal_Rcvd_Data[12] - 0x30; 
                val2 = Cal_Rcvd_Data[13] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                    digitalWrite(RS_485_RX_TX,1);
                    Send_ok();
                    digitalWrite(RS_485_RX_TX,0);

                    val1 = Cal_Rcvd_Data[5] - 0x30; 
                    val2 = Cal_Rcvd_Data[6] - 0x30;
                    val3 = Cal_Rcvd_Data[8] - 0x30;
                    val4 = Cal_Rcvd_Data[9] - 0x30;
                    val5 = Cal_Rcvd_Data[10] - 0x30;

                    final_val = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    Calibration_flow[Calibration_counter] = final_val;
                    READ_FLOW = Get_Flow();
                    Calibration_adc[Calibration_counter] = Delta;
                }
          }
          else if(Cal_Rcvd_Data[1] == 'R' && Cal_Rcvd_Data[2] == 'F')
          {
                val1 = Cal_Rcvd_Data[3] - 0x30; 
                val2 = Cal_Rcvd_Data[4] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                      digitalWrite(RS_485_RX_TX,1);
                      Send_flow();
                      digitalWrite(RS_485_RX_TX,0);
                }
          }
          else if(Cal_Rcvd_Data[1] == 'R' && Cal_Rcvd_Data[2] == 'C' && Cal_Rcvd_Data[3] == 'F')
          {
                val1 = Cal_Rcvd_Data[3] - 0x30; 
                val2 = Cal_Rcvd_Data[4] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                      digitalWrite(RS_485_RX_TX,1);
                      Send_Configuration();
                      digitalWrite(RS_485_RX_TX,0);
                }
          }
          else if(Cal_Rcvd_Data[1] == 'G' && Cal_Rcvd_Data[2] == 'E' && Cal_Rcvd_Data[3] == 'T' && Cal_Rcvd_Data[4] == 'C')
          {
                val1 = Cal_Rcvd_Data[3] - 0x30; 
                val2 = Cal_Rcvd_Data[4] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                      digitalWrite(RS_485_RX_TX,1);
                      Send_Calibration();
                      digitalWrite(RS_485_RX_TX,0);
                }
          }
          else if(Cal_Rcvd_Data[1] == 'S' && Cal_Rcvd_Data[2] == 'R') //DEVICE SERIAL NO
          {
                val1 = Cal_Rcvd_Data[3] - 0x30; 
                val2 = Cal_Rcvd_Data[4] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                    count = 6;
                    count1 = 0;
                    while(Cal_Rcvd_Data[count] != '#')
                    {
                          Device_Serial_no[count1] = Cal_Rcvd_Data[count];
                          count++;count1++;
                    }
                    count1++;
                    Device_Serial_no[count1] = '#';
                    
                      digitalWrite(RS_485_RX_TX,1);
                      Send_ok();
                      digitalWrite(RS_485_RX_TX,0);

                      Store_serial_no();
                }

          }
          else if(Cal_Rcvd_Data[1] == 'P' && Cal_Rcvd_Data[2] == 'S' && Cal_Rcvd_Data[3] == 'R')  //PCB SERIAL NO
          {
                val1 = Cal_Rcvd_Data[3] - 0x30; 
                val2 = Cal_Rcvd_Data[4] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                    count = 7;
                    count1 = 0;
                    while(Cal_Rcvd_Data[count] != '#')
                    {
                          Pcb_Serial_no[count1] = Cal_Rcvd_Data[count];
                          count++;count1++;
                    }
                    count1++;
                    Device_Serial_no[count1] = '#';
                    
                      digitalWrite(RS_485_RX_TX,1);
                      Send_ok();
                      digitalWrite(RS_485_RX_TX,0);

                      Store_pcb_serial_no();
                }

          }
          else if(Cal_Rcvd_Data[1] == 'T' && Cal_Rcvd_Data[2] == 'S' && Cal_Rcvd_Data[3] == 'R')  //FLOW TUBE SERIAL NO
          {
                val1 = Cal_Rcvd_Data[3] - 0x30; 
                val2 = Cal_Rcvd_Data[4] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                    count = 7;
                    count1 = 0;
                    while(Cal_Rcvd_Data[count] != '#')
                    {
                          Flowtube_Serial_no[count1] = Cal_Rcvd_Data[count];
                          count++;count1++;
                    }
                    count1++;
                    Device_Serial_no[count1] = '#';
                    
                      digitalWrite(RS_485_RX_TX,1);
                      Send_ok();
                      digitalWrite(RS_485_RX_TX,0);

                      Store_flow_tube_serial_no();
                }

          }
          else if(Cal_Rcvd_Data[1] == 'T')
          {
                val1 = Cal_Rcvd_Data[2] - 0x30; 
                val2 = Cal_Rcvd_Data[3] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                      digitalWrite(RS_485_RX_TX,1);
                      Send_ok();
                      digitalWrite(RS_485_RX_TX,0);
                }

          }
          Flush_Rx_Buffer();   
} 

void Store_calb_data(void)
{
     Calibration_P2_start = Calibration_flow[1];
        P2_Adc_Count = Calibration_adc[1];

        Calibration_P3_start = Calibration_flow[2];
        P3_Adc_Count = Calibration_adc[2];

        Calibration_P4_start = Calibration_flow[3];
        P4_Adc_Count = Calibration_adc[3];

        Calibration_P5_start = Calibration_flow[4];
        P5_Adc_Count = Calibration_adc[4];

        Calibration_P6_start = Calibration_flow[5];
        P6_Adc_Count = Calibration_adc[5];

        Calibration_P7_start = Calibration_flow[6];
        P7_Adc_Count = Calibration_adc[6];

        Calibration_P8_start = Calibration_flow[7];
        P8_Adc_Count = Calibration_adc[7];

        Calibration_P9_start = Calibration_flow[8];
        P9_Adc_Count = Calibration_adc[8];

        Calibration_P10_start = Calibration_flow[9];
        P10_Adc_Count = Calibration_adc[9];

        EEPROM_Save[27] = (Calibration_P1_start >>8) & 0xff;   // msb
        EEPROM_Save[26] = Calibration_P1_start & 0xff;
        
        EEPROM_Save[29] = (P1_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[28] = P1_Adc_Count & 0xff;
        
        EEPROM_Save[31] = (Calibration_P2_start >>8) & 0xff;   // msb
        EEPROM_Save[30] = Calibration_P2_start & 0xff;
                    
        EEPROM_Save[33] = (P2_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[32] = P2_Adc_Count & 0xff;
        
        EEPROM_Save[35] = (Calibration_P3_start >>8) & 0xff;   // msb
        EEPROM_Save[34] = Calibration_P3_start & 0xff;
        
        EEPROM_Save[37] = (P3_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[36] = P3_Adc_Count & 0xff;
                  
        EEPROM_Save[39] = (Calibration_P4_start >>8) & 0xff;   // msb
        EEPROM_Save[38] = Calibration_P4_start & 0xff;
        
        EEPROM_Save[41] = (P4_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[40] = P4_Adc_Count & 0xff;
                 
        EEPROM_Save[43] = (Calibration_P5_start >>8) & 0xff;   // msb
        EEPROM_Save[42] = Calibration_P5_start & 0xff;
        
        EEPROM_Save[45] = (P5_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[44] = P5_Adc_Count & 0xff;
                    
        EEPROM_Save[47] = (Calibration_P6_start >>8) & 0xff;   // msb
        EEPROM_Save[46] = Calibration_P6_start & 0xff;
        
        EEPROM_Save[49] = (P6_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[48] = P6_Adc_Count & 0xff;
        
        EEPROM_Save[51] = (Calibration_P7_start >>8) & 0xff;   // msb
        EEPROM_Save[50] = Calibration_P7_start & 0xff;
        
        EEPROM_Save[53] = (P7_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[52] = P7_Adc_Count & 0xff;
        
        EEPROM_Save[55] = (Calibration_P8_start >>8) & 0xff;   // msb
        EEPROM_Save[54] = Calibration_P8_start & 0xff;
        
        EEPROM_Save[57] = (P8_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[56] = P8_Adc_Count & 0xff;
                    
        EEPROM_Save[59] = (uint8_t)(Calibration_P9_start >>8) & 0xff;   // msb
        EEPROM_Save[58] = (uint8_t)Calibration_P9_start & 0xff;

        EEPROM_Save[61] = (P9_Adc_Count >>8) & 0xff;   // msb
        EEPROM_Save[60] = P9_Adc_Count & 0xff;

        for (int i = 0; i < 60; i++) 
        {
              EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
        }
        // EEPROM.put(10, PWM_Multiplier);
        EEPROM.commit();
}
 void Send_ok(void)
 {
    unsigned char STRING[10];
    unsigned char cnt;
    STRING[0] = ' ';
    STRING[1] = 'O';
    STRING[2] = 'K';

    STRING[3] = (RS_485_addr / 10) + '0';  
    STRING[4] = (RS_485_addr % 10) + '0';  
    STRING[5] = '\0'; // Null terminator if needed for string functions

    // Send each character with 1ms delay
    for (cnt = 0; cnt <= 4; cnt++) 
    {
      MySerial.write(STRING[cnt]);
      delay(1);
    }
}
 unsigned int Get_Flow(void)
 {

 }
 void Send_flow(void)
 {
        unsigned char STRING[15];

        Calibrate_PWM();

        STRING[0] = ' ';
        STRING[1] = 'F';

        char LOAD5, LOAD6;
    
        STRING[2] = (RS_485_addr / 10) + '0'; 
        STRING[3] = (RS_485_addr % 10) + '0'; 
        STRING[4] = ',';

        char LOAD2, LOAD3, LOAD4;
        Convert_flow_ascii(Flow_Rate, LOAD2, LOAD3, LOAD4, LOAD5, LOAD6);
        STRING[5] = LOAD2;
        STRING[6] = LOAD3;
        STRING[8] = LOAD4;
        STRING[9] = LOAD5;
        STRING[10] = LOAD6; 

        for (int cnt = 0; cnt <= 10; cnt++)
         {
          MySerial.write(STRING[cnt]);
          delay(1); 
        }

        delay(100); 
 }

void Store_serial_no(void)
{
    for (int i = 0; i < 25; i++) 
    {
        EEPROM.write(SERIAL_NO_ADD + i, EEPROM_Save[i]); 
    }
    EEPROM.commit(); // Save changes
}

void Store_pcb_serial_no(void)
{
    for (int i = 0; i < 25; i++) 
    {
        EEPROM.write(PCB_NO_ADD + i, EEPROM_Save[i]); 
    }
    EEPROM.commit(); // Save changes
}

void Store_flow_tube_serial_no(void)
{
    for (int i = 0; i < 25; i++) 
    {
        EEPROM.write(FLOW_TUBE_ADD + i, EEPROM_Save[i]); 
    }
    EEPROM.commit(); // Save changes
}

void Flush_Rx_Buffer(void)
{
      unsigned char count;
      for(count = 0;count<250;count++)
      {
            Cal_Rcvd_Data[count] = 0;
      }
} 

void Convert_flow_ascii(float flow, char &d1, char &d2, char &d3, char &d4, char &d5) 
{
  int pres = flow * 100; // Shift decimal: 123.45 → 12345

  d1 = (pres / 10000) % 10 + '0'; // hundreds
  d2 = (pres / 1000) % 10 + '0';  // tens
  d3 = (pres / 100) % 10 + '0';   // units
  d4 = (pres / 10) % 10 + '0';    // tenths
  d5 = (pres % 10) + '0';         // hundredths
}

void Read_Configuration(void)
{
       //*CF01,PS01,OF1234,ZF1234,ST55,EN70,ZFR1234,EF01#
       //*CF<device_no>, PS<Pipe_Size>, OF<Output Frequency>, ZF<Zero_Flow_Delta>, ST<Measurment_Start>, EN<Measurment_End>, ZFR<Zero_Flow_rate>, EF<Exitation_Freq#

       unsigned char val1,val2,val3,val4,val5,val6;
       unsigned int final_val;
       val1 = Cal_Rcvd_Data[3] - 0x30; 
       val2 = Cal_Rcvd_Data[4] - 0x30;

      final_val = (val1 * 10) + val2;

      if(final_val == RS_485_addr)
      {
          if(Cal_Rcvd_Data[6] == 'P' &&  Cal_Rcvd_Data[7] == 'S')   //PIPE SIZE
          {
                  val1 = Cal_Rcvd_Data[8] - 0x30; 
                  val2 = Cal_Rcvd_Data[9] - 0x30;

                   final_val = (val1 * 10) + val2;

                   if(final_val > 0 && final_val <= 2)
                   {
                        Pipe_Size = final_val;
                   }
                   if(Pipe_Size == 1)    //25nb
                    {
                          FLOW_RATE_RANGE = 10000;
                          //PWM_RANGE = 417;
                          Zero_Flow_rate_val = 400;
                    }
                    else if(Pipe_Size == 2) //40nb
                    {
                          FLOW_RATE_RANGE = 20000;
                          //PWM_RANGE = 464;
                          Zero_Flow_rate_val = 1000;
                    }
                    else if(Pipe_Size == 3) //50 nb
                    {
                          FLOW_RATE_RANGE = 45000;
                          //PWM_RANGE = 500;
                          Zero_Flow_rate_val = 1600;
                    }
                   
          }
          if(Cal_Rcvd_Data[11] == 'O' &&  Cal_Rcvd_Data[12] == 'F')   //output frequency
          {
                  val1 = Cal_Rcvd_Data[13] - 0x30; 
                  val2 = Cal_Rcvd_Data[14] - 0x30;
                  val3 = Cal_Rcvd_Data[15] - 0x30;
                  val4 = Cal_Rcvd_Data[16] - 0x30;

                   final_val = (val1 * 1000) + (val2 * 100) + (val3 * 10) + val4;

                  PWM_RANGE = final_val;
                  Pwm_Calculator_Val = float (FLOW_RATE_RANGE / PWM_RANGE);   //factor to calculate pwm frequency
                   
          }
          if(Cal_Rcvd_Data[18] == 'Z' &&  Cal_Rcvd_Data[19] == 'F')   //zero flow 
          {
                  val1 = Cal_Rcvd_Data[20] - 0x30; 
                  val2 = Cal_Rcvd_Data[21] - 0x30;
                  val3 = Cal_Rcvd_Data[22] - 0x30;
                  val4 = Cal_Rcvd_Data[23] - 0x30;
                   
                  final_val = (val1 * 1000) + (val2 * 100) + (val3 * 10) + val4;

                  Zero_Flow_Offset = final_val;   
          }
          if(Cal_Rcvd_Data[25] == 'S' &&  Cal_Rcvd_Data[26] == 'T')   //START TIME
          {
                  val1 = Cal_Rcvd_Data[27] - 0x30; 
                  val2 = Cal_Rcvd_Data[28] - 0x30;

                  final_val = (val1 * 10) + val2;
                  
                  Recieve_ADC_Read_Delay = final_val;       
          }

          if(Cal_Rcvd_Data[30] == 'E' &&  Cal_Rcvd_Data[31] == 'N')   //END TIME
          {
                  val1 = Cal_Rcvd_Data[32] - 0x30; 
                  val2 = Cal_Rcvd_Data[33] - 0x30;

                  final_val = (val1 * 10) + val2;
                  
                  Recieve_ADC_Read_Delay = final_val;       
          }

          if(Cal_Rcvd_Data[35] == 'Z' &&  Cal_Rcvd_Data[36] == 'F' &&  Cal_Rcvd_Data[37] == 'R')   //ZERO FLOW VALUE
          {
                  val1 = Cal_Rcvd_Data[38] - 0x30; 
                  val2 = Cal_Rcvd_Data[39] - 0x30;
                  val3 = Cal_Rcvd_Data[40] - 0x30;
                  val4 = Cal_Rcvd_Data[41] - 0x30;

                   final_val = (val1 * 1000) + (val2 * 100) + (val3 * 10) + val4;

                  Zero_Flow_rate_val = final_val;  
          }

          if(Cal_Rcvd_Data[43] == 'E' &&  Cal_Rcvd_Data[44] == 'F')   //EXITATATION FREQUENCY
          {
                  val1 = Cal_Rcvd_Data[45] - 0x30; 
                  val2 = Cal_Rcvd_Data[46] - 0x30;

                  final_val = (val1 * 10) + val2;
                  
                  Excitation_Frequency  = final_val;       

                  Timer1_init();
          }

          digitalWrite(RS_485_RX_TX,1);
          Send_ok();
          digitalWrite(RS_485_RX_TX,0);
      }

}

void Send_Configuration(void)
{
        unsigned char STRING[50];
        char LOAD1, LOAD2, LOAD3, LOAD4,LOAD5, LOAD6;
        unsigned char count,count1;
        unsigned char string_counter = 0;
        unsigned int string_size;

        //*CF01,PS01,OF1234,ZF1234,ST55,EN70,ZFR1234,EF01#

        STRING[string_counter] = ' ';
        STRING[++string_counter] = 'C';
        STRING[++string_counter] = 'F';
      
        //DEVICE NO
        STRING[++string_counter] = (RS_485_addr / 10) + '0'; 
        STRING[++string_counter] = (RS_485_addr % 10) + '0'; 
        STRING[++string_counter] = ',';

        //PIPE SIZE
        STRING[++string_counter] = 'P';
        STRING[++string_counter] = 'S';
        STRING[++string_counter] = (Pipe_Size / 10) + '0'; 
        STRING[++string_counter] = (Pipe_Size % 10) + '0'; 
        STRING[++string_counter] = ',';

        //output frequency
        STRING[++string_counter] = 'O';
        STRING[++string_counter] = 'F';

        Convert_flow_ascii(PWM_RANGE, LOAD2, LOAD3, LOAD4, LOAD5, LOAD6);
        STRING[++string_counter] = LOAD3;
        STRING[++string_counter] = LOAD4;
        STRING[++string_counter] = LOAD5;
        STRING[++string_counter] = LOAD6;
        STRING[++string_counter] = ',';

        //ZERO FLOW OFFSET
        STRING[++string_counter] = 'O';
        STRING[++string_counter] = 'F';

        Convert_flow_ascii(Zero_Flow_Offset, LOAD2, LOAD3, LOAD4, LOAD5, LOAD6);
        STRING[++string_counter] = LOAD3;
        STRING[++string_counter] = LOAD4;
        STRING[++string_counter] = LOAD5;
        STRING[++string_counter] = LOAD6;
        STRING[++string_counter] = ',';

        //START TIME
        STRING[++string_counter] = 'S';
        STRING[++string_counter] = 'T';

        STRING[++string_counter] = (Recieve_ADC_Read_Delay / 10) + '0'; 
        STRING[++string_counter] = (Recieve_ADC_Read_Delay % 10) + '0'; 
        STRING[++string_counter] = ',';

        //END TIME
        STRING[++string_counter] = 'E';
        STRING[++string_counter] = 'N';

        STRING[++string_counter] = (Recieve_ADC_Read_Stop_Delay / 10) + '0'; 
        STRING[++string_counter] = (Recieve_ADC_Read_Stop_Delay % 10) + '0'; 
        STRING[++string_counter] = ',';

        //ZERO FLOW VALUE
        STRING[++string_counter] = 'Z';
        STRING[++string_counter] = 'F';
        STRING[++string_counter] = 'R';

        Convert_flow_ascii(Zero_Flow_rate_val, LOAD2, LOAD3, LOAD4, LOAD5, LOAD6);
        STRING[++string_counter] = LOAD3;
        STRING[++string_counter] = LOAD4;
        STRING[++string_counter] = LOAD5;
        STRING[++string_counter] = LOAD6;
        STRING[++string_counter] = ',';

        //EXITATION FREQUENCY
        STRING[++string_counter] = 'E';
        STRING[++string_counter] = 'F';
        STRING[++string_counter] = (Excitation_Frequency / 10) + '0'; 
        STRING[++string_counter] = (Excitation_Frequency % 10) + '0'; 
        STRING[++string_counter] = ',';

        //DEVICE SERIAL NO
        STRING[++string_counter] = 'D';
        STRING[++string_counter] = 'S';
        STRING[++string_counter] = 'R';

        string_counter++;
        for(count = 0,count1=string_counter;count<25;count++,count1++)
        {
            STRING[count1] = Device_Serial_no[count];
        }
        STRING[++string_counter] = ',';

        //PCB SERIAL NO
        STRING[++string_counter] = 'P';
        STRING[++string_counter] = 'S';
        STRING[++string_counter] = 'R';

        string_counter++;
        for(count = 0,count1=string_counter;count<25;count++,count1++)
        {
            STRING[count1] = Pcb_Serial_no[count];
        }
        STRING[++string_counter] = ',';

        //FLOW TUBE SERIAL NO
        STRING[++string_counter] = 'F';
        STRING[++string_counter] = 'S';
        STRING[++string_counter] = 'R';

        string_counter++;
        for(count = 0,count1=string_counter;count<25;count++,count1++)
        {
            STRING[count1] = Flowtube_Serial_no[count];
        }
        STRING[++string_counter] = '#';

        string_size = string_counter+1;

        for(count = 0;count < string_size; count++)
        {
            MySerial.write(STRING[count]);
            delay(1); 
        }
        
}
void Send_Calibration(void)
{
      char buffer[250];
      unsigned char count;
      unsigned int i;
      unsigned char buffer_counter = 0;
      unsigned int string_size;
      //String convVal;


      //Serial.println("FILLING STRING  \n");
      buffer[buffer_counter] = ' ';
      buffer[++buffer_counter] = 'C';
      buffer[++buffer_counter] = 'A';
      buffer[++buffer_counter] = 'L';
      buffer[++buffer_counter] = ' ';

      buffer[++buffer_counter] = 'P';
      buffer[++buffer_counter] = 'T';
      buffer[++buffer_counter] = 'S';

      CONVERT_INTEGER_ASCII(Calibration_Points);
      buffer[++buffer_counter] = LOAD5;
      buffer[++buffer_counter] = LOAD6;
      buffer[++buffer_counter] = ',';

      for (int i = 1; i <= Calibration_Points; i++)
    {
        buffer[buffer_counter++] = 'P';
        buffer[buffer_counter++] = i + 0x30;  // Assumes P1 to P9 (you may need to adjust for P10)
        buffer[buffer_counter++] = ':';

        // Convert and append Start Value
        if(i == 1)
        {
              CONVERT_INTEGER_ASCII(Calibration_P1_start);
        }
        else if(i == 2)
        {
              CONVERT_INTEGER_ASCII(Calibration_P2_start);
        }
        else if(i == 3)
        {
              CONVERT_INTEGER_ASCII(Calibration_P3_start);
        }
        else if(i == 4)
        {
              CONVERT_INTEGER_ASCII(Calibration_P4_start);
        }
        else if(i == 5)
        {
              CONVERT_INTEGER_ASCII(Calibration_P5_start);
        }
        else if(i == 6)
        {
              CONVERT_INTEGER_ASCII(Calibration_P6_start);
        }
        else if(i == 7)
        {
              CONVERT_INTEGER_ASCII(Calibration_P7_start);
        }
        else if(i == 8)
        {
              CONVERT_INTEGER_ASCII(Calibration_P8_start);
        }
        else if(i == 9)
        {
              CONVERT_INTEGER_ASCII(Calibration_P9_start);
        }
        else if(i == 10)
        {
              CONVERT_INTEGER_ASCII(Calibration_P10_start);
        }
        buffer[buffer_counter++] = LOAD2;
        buffer[buffer_counter++] = LOAD3;
        buffer[buffer_counter++] = LOAD4;
        buffer[buffer_counter++] = LOAD5;
        buffer[buffer_counter++] = LOAD6;
        buffer[buffer_counter++] = ',';

        // Convert and append ADC Count
        if(i == 1)
        {
              CONVERT_INTEGER_ASCII(P1_Adc_Count);
        }
        else if(i == 2)
        {
              CONVERT_INTEGER_ASCII(P2_Adc_Count);
        }
        else if(i == 3)
        {
              CONVERT_INTEGER_ASCII(P3_Adc_Count);
        }
        else if(i == 4)
        {
              CONVERT_INTEGER_ASCII(P4_Adc_Count);
        }
        else if(i == 5)
        {
              CONVERT_INTEGER_ASCII(P5_Adc_Count);
        }
        else if(i == 6)
        {
              CONVERT_INTEGER_ASCII(P6_Adc_Count);
        }
        else if(i == 7)
        {
              CONVERT_INTEGER_ASCII(P7_Adc_Count);
        }
        else if(i == 8)
        {
              CONVERT_INTEGER_ASCII(P8_Adc_Count);
        }
        else if(i == 9)
        {
              CONVERT_INTEGER_ASCII(P9_Adc_Count);
        }
        else if(i == 10)
        {
              CONVERT_INTEGER_ASCII(P10_Adc_Count);
        }
        buffer[buffer_counter++] = LOAD2;
        buffer[buffer_counter++] = LOAD3;
        buffer[buffer_counter++] = LOAD4;
        buffer[buffer_counter++] = LOAD5;
        buffer[buffer_counter++] = LOAD6;

        buffer[buffer_counter++] = ',';
    }

    buffer[--buffer_counter] = '#';  // Replace the last ',' with '#'

    buffer[buffer_counter + 1] = '#'; // Null-terminate the string

    string_size = buffer_counter+1;

    for(count = 0;count < string_size; count++)
    {
        MySerial.write(buffer[count]);
        delay(1); 
    }

    //Serial.println(buffer);  // Or send via communication

      //while(1);
}

void Led_Operation(void)
{
        unsigned char Led_State;

        if(Opeation_state == Zero_Flow)
        {
              Led_State = GREEN_ON;
        }
        else if(Opeation_state == Normal_Flow)
        {
              Led_State = GREEN_BLINK_TWICE;
        }
        else if(Opeation_state == Low_Flow)
        {
              Led_State = GREEN_BLINK_ONCE;
        }
        else if(Opeation_state == Pipe_Empty)
        {
              Led_State = RED_ON;
        }
        else if(Opeation_state == Error_State)
        {
              Led_State = ORANGE_BLINK;
        }
        else if(Opeation_state == Cal_Conf_Mode)
        {
              Led_State = ORANGE_ON;
        }

        if(Led_State == GREEN_ON)
        {
                digitalWrite(GREEN_LED, HIGH);
                digitalWrite(RED_LED,LOW);
        }
        else if(Led_State == RED_ON)
        {
                digitalWrite(GREEN_LED,LOW);
                digitalWrite(RED_LED,HIGH);
        }
        else if(Led_State == RED_ON)
        {
                digitalWrite(GREEN_LED,LOW);
                digitalWrite(RED_LED,HIGH);
        }
        else if(Led_State == GREEN_BLINK_ONCE)
        {
                digitalWrite(GREEN_LED, HIGH);
                digitalWrite(RED_LED,HIGH);
        }
        else if(Led_State == GREEN_BLINK_TWICE)
        {
                digitalWrite(GREEN_LED, HIGH);
                digitalWrite(RED_LED,HIGH);
        }
        else if(Led_State == ORANGE_BLINK)
        {
                digitalWrite(GREEN_LED, HIGH);
                digitalWrite(RED_LED,HIGH);
                delay(100);
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(RED_LED,LOW);
                delay(100);
        }
        else
        {
              digitalWrite(GREEN_LED,LOW);
              digitalWrite(RED_LED,LOW);
        }
}