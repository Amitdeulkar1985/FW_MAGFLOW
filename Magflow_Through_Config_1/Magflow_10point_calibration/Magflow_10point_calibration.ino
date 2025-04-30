//Developed branch testing
#include <Arduino.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WebServer.h> 
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Update.h>

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

//#define  INPUT_PIN_2         18        // By default high Default frequency set
#define  INPUT_PIN_1         42 
//#define  INPUT_PIN_3         17        // to detect 37hz  // added on 11_02_2023

#define PIN1                  5 //1             // GPIO pin 1         //Excitation PINS 
#define PIN2                  6 //2             // GPIO pin 2

#define MUX_C                42
#define MUX_B                41
#define MUX_A                40

#define PWM_1               7            //46  Frequency Generation with 50% duty

#define EMPTY_PIN           3

//Pins for Calibartion
#define Calibration_Mode  10    //SJ2
#define ADD1              9     //SJ6
#define ADD2              11    //SJ5
#define ADD3              12    //SJ4
#define ADD4              13    //SJ3

#define CAL_TX_PIN        17  // TX on Pin 10
#define CAL_RX_PIN        18  // RX on Pin 11
#define RS_485_RX_TX      21

#define GREEN_LED         38
#define RED_LED           39

#define REED_SWITCH        37

HardwareSerial MySerial(1);  // Use UART1

//------------------BLUETOOTH CONFIGURATION-------------------------
BLECharacteristic *pCharacteristic;
BLEServer* pServer;
bool deviceConnected = false;
unsigned long connectionStartTime = 0; 
static bool disconnectedManually = false;

#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

//-----------------------WIFI DETAILS----------------
const char* ssid = "";          // Wi-Fi SSID
const char* password = "";     // Wi-Fi password

const char* ssid1 = "MAGFLOW_Config_AP";
const char* password1 = "12345678";

String apiKey = "k5GS7O2nA7Hop0x";
String apiSecret = "3bVNEbHmtRnHuOa";
String auth = apiKey + ":" + apiSecret;

int authLen = auth.length();
//char encodedAuth[base64_enc_len(authLen)];
//base64_encode(encodedAuth, (char*)auth.c_str(), authLen);
//web server
// Create a web server on port 80
WebServer server(80);

//-----------------------

#define Toggle_For_Coil_2   8              // only for testing
#define Toggle_For_Coil_1   45             // only for testing

#define BAUDRATE            9600          //  baudrate set
#define DEBUG_EN            1             // debug ADC

#define EEPROM_SIZE     512
#define EEPROM_ADDR     10
#define SERIAL_NO_ADD   75
#define PCB_NO_ADD      100
#define FLOW_TUBE_ADD   125

#define SSID_ADDR       200             // starting byte for SSID
#define PASS_ADDR       300

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
unsigned char Pin_Counter;
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
bool Server_Connected;
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
unsigned char Device_Serial_no[25] = "111111";
unsigned char Pcb_Serial_no[25] = "111111";;
unsigned char Flowtube_Serial_no[25] = "111111";;

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
String chipId;
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
//------------------------------WIFI FUNCTION--------------------------
// HTML content from files
const char* wifiPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Wi-Fi Configuration</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 0;
      background-color: #006622;
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
      color: #fff;
    }
    h2 { text-align: center; color: #fff; }
    form {
      width: 100%;
      max-width: 400px;
      background: #fff;
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
    }
    label {
      font-weight: bold;
      display: inline-block;
      color: #333;
    }
    input {
      width: 100%;
      padding: 10px;
      margin-bottom: 15px;
      border: 1px solid #ccc;
      border-radius: 4px;
    }
    input[type="submit"] {
      background-color: #006622;
      color: white;
      cursor: pointer;
    }
  </style>
</head>
<body>
  <div>
    <h2>Wi-Fi Credentials</h2>
    <form action="/submit" method="POST">
      <label for="wifi_ssid">Wi-Fi SSID:</label>
      <input type="text" id="wifi_ssid" name="wifi_ssid" required>
      <label for="wifi_password">Wi-Fi Password:</label>
      <input type="password" id="wifi_password" name="wifi_password" required>
      <input type="submit" value="Submit">
    </form>
  </div>
</body>
</html>
)rawliteral";

const char* successPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Wi-Fi Configuration</title>
  <style>
    /* General reset and styling */
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 0;
      background-color: #006622; /* Set to green */
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
      color: #fff; /* Set text color to white for contrast */
    }

    h2 {
      text-align: center;
      color: #fff; /* Keep header text white */
    }

    form {
      width: 100%;
      max-width: 400px;
      background: #fff; /* White form background for contrast */
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
      text-align: left;
    }

    label {
      font-weight: bold;
      margin-bottom: 5px;
      display: inline-block;
      color: #333; /* Dark label text for readability */
    }

    input[type="text"],
    input[type="password"] {
      width: 100%;
      padding: 10px;
      margin-bottom: 15px;
      border: 1px solid #ccc;
      border-radius: 4px;
      box-sizing: border-box;
    }

    input[type="submit"] {
      width: 100%;
      padding: 10px;
      background-color: #006622; /* Match background color */
      color: white;
      border: none;
      border-radius: 4px;
      font-size: 16px;
      cursor: pointer;
      transition: background-color 0.3s ease, transform 0.2s ease;
    }

    input[type="submit"]:hover {
      background-color: #004d1a; /* Darker green on hover */
      transform: scale(1.05); /* Slight zoom effect */
    }

    @media (max-width: 480px) {
      form {
        padding: 15px;
      }

      input[type="text"],
      input[type="password"],
      input[type="submit"] {
        font-size: 14px;
        padding: 8px;
      }
    }
  </style>
</head>
<body>
  <div>
    <form action="/submit" method="POST">
      <label for="wifi_ssid">
        <h2></h2>
        <p>Wi-Fi Configuration Received...! 😊</p>
      </label>
    </form>
  </div>
</body>
</html>
)rawliteral";

const char* failedPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Wi-Fi Configuration</title>
  <style>
    /* General reset and styling */
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 0;
      background-color: #006622; /* Set to green */
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
      color: #fff; /* Set text color to white for contrast */
    }

    h2 {
      text-align: center;
      color: #fff; /* Keep header text white */
    }

    .error-message {
      color: red;
      font-weight: bold;
      text-align: center;
      margin-bottom: 20px;
    }

    form {
      width: 100%;
      max-width: 400px;
      background: #fff; /* White form background for contrast */
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
      text-align: left;
    }

    label {
      font-weight: bold;
      margin-bottom: 5px;
      display: inline-block;
      color: #333; /* Dark label text for readability */
    }

    input[type="text"],
    input[type="password"] {
      width: 100%;
      padding: 10px;
      margin-bottom: 15px;
      border: 1px solid #ccc;
      border-radius: 4px;
      box-sizing: border-box;
    }

    input[type="submit"] {
      width: 100%;
      padding: 10px;
      background-color: #006622; /* Match background color */
      color: white;
      border: none;
      border-radius: 4px;
      font-size: 16px;
      cursor: pointer;
      transition: background-color 0.3s ease, transform 0.2s ease;
    }

    input[type="submit"]:hover {
      background-color: #004d1a; /* Darker green on hover */
      transform: scale(1.05); /* Slight zoom effect */
    }

    @media (max-width: 480px) {
      form {
        padding: 15px;
      }

      input[type="text"],
      input[type="password"],
      input[type="submit"] {
        font-size: 14px;
        padding: 8px;
      }
    }
  </style>
</head>


<body>
  <div>
    <form action="/submit" method="POST">
      <label for="wifi_ssid">
        <h2></h2>
        <p> Wi-Fi Configuration Failed...! 😢</p>
      </label>
    </form>
  </div>
</body>
</html>
)rawliteral";

// Function to handle the root URL
void handleRoot() {
  server.send(200, "text/html", wifiPage);
}

// Function to handle form submission
void handleSubmit() {
  if (server.method() == HTTP_POST) {
    String wifi_ssid = server.arg("wifi_ssid");
    String wifi_password = server.arg("wifi_password");

    // Validate inputs
    if (wifi_ssid.isEmpty() || wifi_password.isEmpty()) {
      server.send(400, "text/html", failedPage);
      return;
    }

    // Print received parameters to Serial Monitor
    Serial.println("Configuration Received:");
    Serial.println("Wi-Fi SSID: " + wifi_ssid);
    Serial.println("Wi-Fi Password: " + wifi_password);

    // Send confirmation response to the client
    server.send(200, "text/html", successPage);

    EEPROM.begin(EEPROM_SIZE);
    writeStringToEEPROM(SSID_ADDR, wifi_ssid);
    writeStringToEEPROM(PASS_ADDR, wifi_password);
    EEPROM.commit(); // IMPORTANT!


  } else 
  {
    server.send(405, "text/html", failedPage);
  }


  delay(5000);
  WiFi.softAPdisconnect(true);  // Stop AP
  WiFi.mode(WIFI_STA);          // Set mode to Station (normal WiFi client)
  //WiFi.begin(ssid, password); 

  String s = readStringFromEEPROM(SSID_ADDR);
  ssid = s.c_str();
  
  String p = readStringFromEEPROM(PASS_ADDR);
  password = p.c_str();

   connectWiFi();
   int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) 
    {
      delay(500);
      Serial.print(".");
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED) 
    {
      Serial.println("\nConnected!");
      Serial.println(WiFi.localIP());

      // Step 1: Retrieve API credentials (api_key and api_secret) from the device details API.
        String api_key, api_secret;
        if (!getDeviceDetails(chipId, api_key, api_secret)) 
        {
          Serial.println("[ERROR] Failed to obtain device details. Aborting.");
          return;
        }

        // Step 2: Retrieve the firmware download URL using the API credentials.
        String firmware_url;
        if (!getFirmwareUrl(chipId, api_key, api_secret, firmware_url)) 
        {
          Serial.println("[ERROR] Failed to obtain firmware URL. Aborting.");
          return;
        }

        Serial.println("Firmware URL: " + firmware_url);
        delay(10000); // Optional delay for debugging or observation.

        // Step 3: Download and update firmware via OTA using the obtained firmware URL.
        performOTAUpdate(firmware_url, chipId, api_key, api_secret);
    } 
    else 
    {
      Serial.println("\nFailed to connect.");
    }
  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.println("[ERROR] Wi-Fi connection failed. Aborting.");
    return;
  }

  Pin_Counter = 0;
  Server_Connected = 0;

}

//***************************************************************************
// Function: connectWiFi()
// Purpose:  Establish a Wi-Fi connection using the specified SSID and password.
//***************************************************************************
void connectWiFi() {
  Serial.print("[BOOT] Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  int retries = 0;
  
  // Wait for Wi-Fi connection with a maximum of 20 retries.
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  
  // Check if Wi-Fi is connected, else log an error.
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[BOOT] Wi-Fi Connected!");
  } else {
    Serial.println("\n[ERROR] Failed to connect to Wi-Fi.");
  }
}

//-----------------BLE FUNCTIONS
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    disconnectedManually = false;
    connectionStartTime = millis();
    Serial.println("Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected");
  }
};;

void setup()
{  
    #if DEBUG_EN
        Serial.begin(BAUDRATE);     // Serial terminal set baudrate 
    #endif

    MySerial.begin(9600, SERIAL_8N1, CAL_RX_PIN, CAL_TX_PIN);
    //pinMode(CAL_RX_PIN, INPUT);
    //attachInterrupt(digitalPinToInterrupt(CAL_RX_PIN), caluartISR, FALLING);
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

    //----------BLE INIT--------------

    String bleName = "VMAG-" + String((char*)Device_Serial_no);

    //BLEDevice::init("VATS MAGFLOW");
     BLEDevice::init(bleName.c_str());

    pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->setValue("Flow Rate: 0 lph");
  pService->start();

  // Start Advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Waiting for a client connection to notify...");

   /*uint64_t chipIdVal = ESP.getEfuseMac();
  uint32_t highPart = (uint32_t)(chipIdVal >> 32);
  uint32_t lowPart = (uint32_t)chipIdVal;
  char chipIdBuffer[24];
  // Format the chip ID as "AVY_BC_<highPart><lowPart>".
  snprintf(chipIdBuffer, sizeof(chipIdBuffer), "AVY_BC_%08X%08X", highPart, lowPart);
  chipId = String(chipIdBuffer);

  Serial.print("Custom Chip ID: ");
  Serial.println(chipId);*/
  chipId = "1224-045-00001";
 


}

void loop() 
{
    //unsigned int Var_4=0;
    
               //digitalWrite(RS_485_RX_TX,1);
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

              /*if(Rx_Cal_Int_Flag)
              {
                    Rx_Cal_Int_Flag = 0;
                    
                    
                    Check_Cal_Recived_String();
              }*/

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
             // digitalWrite(PIN2, 0); 
              //digitalWrite(PIN1, 0); 
              //f1 = 0;
              Check_Empty_Flow = 0;
              Serial.println("Checking Empty Pipe\n");
              //timerAlarmDisable(Freq_timer);
              //timerAlarmDisable(My_timer); //Just Enable 
              //timerAlarmDisable(timer1);

              delay(10);
              for (int i = 0; i < 3; i++) 
              {
                  digitalWrite(EMPTY_PIN, HIGH);
                  Empty_Adc[i] = analogRead(FLOW_ADC_Pin);
                  delay(1);
                  digitalWrite(EMPTY_PIN, LOW);
                  //Empty_Adc[i] = analogRead(FLOW_ADC_Pin);
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
                //timerAlarmEnable(timer1);
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
              //Serial.println("Calibration on \n");
               Read_485_Address();
        }
        else if(Empty_Tube_Error)
        {
              Opeation_state = Pipe_Empty; 
        }
        else if(Low_Flow_Flag)
        {
              Opeation_state = Low_Flow;
        }
        else
        {
              Opeation_state = Normal_Flow;
        }

        Led_Operation();

        //digitalWrite(RS_485_RX_TX,1);
        //Send_Calibration();
        //digitalWrite(RS_485_RX_TX,0);

        if (MySerial.available())   //RS485 SERIAL PORT
        {
              Check_Cal_Recived_String();
        }

        //IF CONNECTED SEND DATA
        if (deviceConnected && !disconnectedManually) 
        {
            // Send flow rate
            String data = "Flow Rate: " + String(Flow_Rate) + " lph";
            pCharacteristic->setValue(data.c_str());
            pCharacteristic->notify();
            Serial.println("Sent: " + data);

            // Check if 1 minute has passed
            if (millis() - connectionStartTime > 10000) 
            {
              Serial.println("Auto disconnecting client after 10 seconds...");
              if (pServer != nullptr) 
              {
                pServer->disconnect(0);  // Use pServer instead of BLEDevice::getServer()
                disconnectedManually = true;  // prevent repeat

                // Restart advertising after disconnection
                BLEDevice::getAdvertising()->start();
                Serial.println("Advertising restarted...");
                connectionStartTime = 0;
              }
            }
  }

    int pinState = digitalRead(REED_SWITCH);

     if(pinState == LOW)
     {
          delay(500);
          Pin_Counter++;
     }
     else
     {
          Pin_Counter = 0;
     }

     if(Pin_Counter >= 4 && !Server_Connected)
     {
          Serial.println("Starting Webserver");
          Pin_Counter = 0;
          Server_Connected = 1;
          Web_server();
     }
     if(Server_Connected)
     {
        server.handleClient();
     }
             
}
//***************************************************** ISR_Routines*********************************************************

/**************************************************************************************
**  Function Prototype: void ADC_Sampling_Check(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to make flags for adc calulation
***************************************************************************************/
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

                        //if(ADC_Count_2 > 4095)
                       // {
                               // ADC_Count_2=4095;
                        //}
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
                      //if(ADC_Count_1>4095)
                      //{
                            //ADC_Count_1=4095;
                      //}
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

/**************************************************************************************
**  Function Prototype: void Calculate_Delta(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to calculate delata from two coil adc readings
***************************************************************************************/
void Calculate_Delta(void)
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

/**************************************************************
**  Function Prototype: void PWM_Init(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to initialise pwm 
****************************************************************/
void PWM_Init(void)
{
  ledcSetup(ledChannel, PWM_Freq, resolution);        //resolution is 13 bit 

  ledcAttachPin(PWM_1, ledChannel);                   // attach the channel to the GPIO to be controlled
  ledcWrite(0, 4095);                                 //PWM Duty 50%   2^13−1=8191, for 50% 8191 / 2 = 4095.5
}

/**************************************************************
**  Function Prototype: void PWM_Set(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to set pwm frequency
****************************************************************/
void PWM_Set(void)
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

/**************************************************************
**  Function Prototype: void IRAM_ATTR Coil_2(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Isr routine for coil -2 input
****************************************************************/
void IRAM_ATTR Timer0_Interrupt(void)  
{
  Timer0_Int_flag  = 1;      // set flag in interrupt 
   
}

/**************************************************************
**  Function Prototype: void IRAM_ATTR Coil_2(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Isr routine for coil -2 input
****************************************************************/
void IRAM_ATTR Recieve_UART(void)
{
      Rx_Int_Flag=1;
      
}

/****************************************************************************************************
**  Function Prototype: void IRAM_ATTR Coil_2(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Isr routine for coil -2 input
*****************************************************************************************************/
void IRAM_ATTR Coil_2(void)  
{
  Coil_2_flag = 1;      // set flag in interrupt 

   
}

/****************************************************************************************************
**  Function Prototype: void IRAM_ATTR Coil_1(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Isr routine for coil -1 input
*****************************************************************************************************/
void IRAM_ATTR Coil_1(void)  
{
  Coil_1_flag = 1;      // set flag in interrupt 

}

/****************************************************************************************************
**  Function Prototype: void Check_Cal_Recived_String(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to read decode commands received on serial port 0
*****************************************************************************************************/
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

/****************************************************************************************************
**  Function Prototype: void Check_Cal_Recived_String(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to read decode commands received on serial port 0
*****************************************************************************************************/
void Check_Cal_Recived_String(void)
{
      // Serial.print("above while\n");
       while(MySerial.available()>0)
      {
              cal_receive_byte=MySerial.read();
              //if(receive_byte=='#' && Rx_Ch==0)

                 //Serial.print("in side interrupt loop\n");

                  if(cal_receive_byte == '*')
                  {
                        Save_data = 1;
                        CAL_DATA_COUNTER = 0;
                  }
                  else if(cal_receive_byte == '#')
                  {
                        Save_data = 0;
                        Compare_data = 1;
                        Cal_Rcvd_Data[CAL_DATA_COUNTER] = cal_receive_byte;
                        CAL_DATA_COUNTER++;
                        Cal_Rcvd_Data[CAL_DATA_COUNTER] = '#';
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
 /*uint16_t ADC_Average()
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
}*/

 /****************************************************************************************************
**  Function Prototype: uint16_t ADC_Average(void) 
**  Passed Parameter  : None
**  Returned Parameter: adc avarge value
**  Date of Creation  : 30/04/2025
**  Discription       : Function to read adc readings and return average value
*****************************************************************************************************/
uint16_t ADC_Average(void) 
{
    const uint8_t samples = 20;
    const uint8_t avg_no = samples / 2;
    uint32_t total = 0;

    for (uint8_t i = 0; i < samples; i++) 
    {
        total += analogRead(ADC_Pin);
    }

    return total / avg_no;
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
 
 /****************************************************************************************************
**  Function Prototype: vvoid Timer0_init(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function for timer0 initilisation
*****************************************************************************************************/
 void Timer0_init(void)
{
  // Set divider 
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &Timer0_Interrupt, true);     // Interrupt assignement
  timerAlarmWrite(My_timer, 1000, true);        //1000000 //1000
  timerAlarmEnable(My_timer); //Just Enable           // Enable timer 
}
//******************** Functions For Freq Timer ************************************************
/****************************************************************************************************
**  Function Prototype: void IRAM_ATTR Timer1_Interrupt(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : ISR routine for timer-1
*****************************************************************************************************/
 void IRAM_ATTR Timer1_Interrupt(void)  
{
  f1  = 1;      // set flag in interrupt 
}

/****************************************************************************************************
**  Function Prototype: void GPIO_Setup(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to initialize all gpio pins
*****************************************************************************************************/
void GPIO_Setup(void)
{
  // SET pin direction as output
    pinMode(PIN1, OUTPUT);              //  Make pin as output
    pinMode(PIN2, OUTPUT);              //  Make pin as output
  
    //pinMode(INPUT_PIN_2,INPUT);                         // make pin as input 
    pinMode(INPUT_PIN_1,INPUT_PULLUP);                  // make pin as input pullup
    //pinMode(INPUT_PIN_3,INPUT_PULLUP);                  // make pin as input pullup
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

    pinMode(REED_SWITCH,INPUT);
    pinMode(REED_SWITCH,INPUT_PULLUP);

    pinMode(ADD4,INPUT);
    pinMode(ADD4,INPUT_PULLUP);

    pinMode(RS_485_RX_TX,OUTPUT);

    pinMode(GREEN_LED,OUTPUT);
    pinMode(RED_LED,OUTPUT);

     digitalWrite(GREEN_LED,LOW);
    digitalWrite(RED_LED,LOW);
}


/****************************************************************************************************
**  Function Prototype: void Timer1_init(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to initialize timer 1 for excitation frequency
*****************************************************************************************************/
void Timer1_init(void)
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

/****************************************************************************************************
**  Function Prototype: void printIntArray(unsigned int* arr, int size) 
**  Passed Parameter  : array adress, size of array
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to print integer array
*****************************************************************************************************/
void printIntArray(unsigned int* arr, int size) 
{
  for (int i = 0; i < size; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println();
}
//*-*-*-*-*-*-*-*-*-*-*-*-

/****************************************************************************************************
**  Function Prototype: void Recall_Memory(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to read eeprom memory for configuration and calibration parameters
*****************************************************************************************************/
 void Recall_Memory(void)
 {
        float retrievedFloatValue;
        unsigned char floatBytes3[5];
        EEPROM.begin(250);                                              // Initialize EEPROM
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

                EEPROM_Save[25] = Calibration_Points;

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
              int i;
             // Pipe_Size = 2;
              Serial.println("Reading from memory");
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
             Serial.println("\nDevice Serial no:");
             for (i = 0; i < 25; i++) 
             {
                   Device_Serial_no[i] = EEPROM_Save[i + SERIAL_NO_ADD - EEPROM_ADDR];
                   Serial.write(Device_Serial_no[i]);
                   if (Device_Serial_no[i] == '#') break;
              }
              Device_Serial_no[i] = '\0';
              Serial.println();

            Serial.println("\nPCB Serial no:");
             for (i = 0; i < 25; i++) 
             {
                   Pcb_Serial_no[i] = EEPROM_Save[i + PCB_NO_ADD - EEPROM_ADDR];
                   Serial.write(Pcb_Serial_no[i]);
                  if (Pcb_Serial_no[i] == '#') break;
            }
            Serial.println();

            Serial.println("\nFlow Tube Serial no:");
             for (i = 0; i < 25; i++) 
             {
                   Flowtube_Serial_no[i] = EEPROM_Save[i + FLOW_TUBE_ADD - EEPROM_ADDR];
                   Serial.write(Flowtube_Serial_no[i]);
                   if (Flowtube_Serial_no[i] == '#') break;
            }
            Serial.println();

          }
 }
//*-*-*-*-*-*- 19_02_24
 /************************************************************************************************
**  Function Prototype: void Pipe_Size_Selection(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to select pipe size gain using multiplexer
**************************************************************************************************/
 void Pipe_Size_Selection(void)
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

 /************************************************************************************************
**  Function Prototype: float  Form_Multiplier(unsigned char* str,unsigned int Start,unsigned int End )
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to send configuration parameters serially
**************************************************************************************************/
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

/************************************************************************************************
**  Function Prototype: void Update_Config(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to send configuration parameters serially
**************************************************************************************************/
void Update_Config(void)
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

/************************************************************************************************
**  Function Prototype: void Write_Config_Received(unsigned char* Rx_Buffer)
**  Passed Parameter  : Receive buffer
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to read configuration serially and store to eeprom
**************************************************************************************************/
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

/************************************************************************************************
**  Function Prototype: void Flow_Rate_Calculation(void )
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to calculate flow rate
**************************************************************************************************/
void Flow_Rate_Calculation(void )
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


/************************************************************************************************
**  Function Prototype: void Check_Calibration_string(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to read calibration parameters serially and store in the eeprom
**************************************************************************************************/
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

/*********************************************************************
**  Function Prototype: void SEND_CAL_DATA(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to send calibration data serially
**********************************************************************/
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

/*****************************************************************************************************
**  Function Prototype: void CONVERT_INTEGER_ASCII(unsigned long int TEMP1)
**  Passed Parameter  : long int value
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to convert integer to ascii
******************************************************************************************************/
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

/*****************************************************************************************************
**  Function Prototype: void Calibrate_PWM(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to decide calibration zone based on delta and calibration points
******************************************************************************************************/
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

/**************************************************************************************************************************************************
**  Function Prototype: unsigned int Calculate_Flow_Rate(unsigned int adc_low,unsigned int adc_high,unsigned int flow_low ,unsigned int flow_high)
**  Passed Parameter  : adc low: 
                        adc high:
                        flow low:
                        flow high:
**  Returned Parameter: Calculated Flow rate
**  Date of Creation  : 30/04/2025
**  Discription       : Function to calculate flow rate
*****************************************************************************************************************************************************/
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

/****************************************************************************************************************************
**  Function Prototype: unsigned int Calculate_Flow_Rate1(unsigned int adc_low,unsigned int adc_high,unsigned int flow_low ,unsigned int flow_high)
**  Passed Parameter  : adc low: 
                        adc high:
                        flow low:
                        flow high:
**  Returned Parameter: Calculated Flow rate
**  Date of Creation  : 30/04/2025
**  Discription       : Function to calculate flow rate for readings above calibration values
*****************************************************************************************************************************/
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

 /***************************************************************************************************
**  Function Prototype: void Set_Frequency(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to read output frequency command over serially and store to eeprom
****************************************************************************************************/
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

 /***************************************************************************************************
**  Function Prototype: void Send_Freq(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to send output frequency setting serially
****************************************************************************************************/
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


/***************************************************************************************************
**  Function Prototype: void IRAM_ATTR onTimer(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : ISR routine for timer 2
****************************************************************************************************/
void IRAM_ATTR onTimer(void) 
{
    Check_Empty_Flow = 1;  // Set flag when timer fires
}


/***************************************************************************************************
**  Function Prototype: void Timer2_init(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function to initialise timer 2 for 30 seconds to detect empty pipe 
****************************************************************************************************/
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

/***************************************************************************************************
**  Function Prototype: void IRAM_ATTR caluartISR(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : ISR routine for serial 0 port
****************************************************************************************************/
void IRAM_ATTR caluartISR(void)
{
    Rx_Cal_Int_Flag=1;
}


/***************************************************************************************************
**  Function Prototype: void Read_485_Address(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function To create device RS485 address for RS485 communication
****************************************************************************************************/
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

   // Serial.print("RS_485_addr = ");
    //Serial.println(RS_485_addr);
}

/***************************************************************************************************
**  Function Prototype: void Calibration_Process(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function To decode received command over RS485 and take appropriate action
****************************************************************************************************/
void Calibration_Process(void)
{
      unsigned char val1,val2,val3,val4,val5;
      unsigned int final_val;
      unsigned char count,count1;

          if(Cal_Rcvd_Data[1] == 'C' && Cal_Rcvd_Data[2] == 'F')
          {
                  Serial.println("\nreading configuration \n");
                  val1 = Cal_Rcvd_Data[3] - 0x30; 
                  val2 = Cal_Rcvd_Data[4] - 0x30;

                  final_val = (val1 * 10) + val2;

                  if(final_val == RS_485_addr)
                  {
                      Read_Configuration();
                  }
                  
          }
          else if(Cal_Rcvd_Data[1] == 'C' && Cal_Rcvd_Data[2] == 'S')
          {
                Calibration_on = 1;
                Calibration_counter = 0;

                Serial.println("\nCalibration start \n");
          }
          else if(Cal_Rcvd_Data[1] == 'C' && Cal_Rcvd_Data[2] == 'E')
          {
                  Serial.println("\nCalibration stop \n");

                  Calibration_on = 0;

                  Store_calb_data();
          }
          else if(Cal_Rcvd_Data[1] == 'C' && Cal_Rcvd_Data[2] == 'P')
          {
                  Serial.print("\nCalibration points:\n");

                  val1 = Cal_Rcvd_Data[3] - 0x30; 
                  val2 = Cal_Rcvd_Data[4] - 0x30;

                  final_val = (val1 * 10) + val2;

                  if(final_val == RS_485_addr)
                  {
                        //Serial.println("\naddress matched \n");
                        val1 = Cal_Rcvd_Data[6] - 0x30; 
                        val2 = Cal_Rcvd_Data[7] - 0x30;

                        final_val = (val1 * 10) + val2;

                        if(final_val >= 1 && final_val <= 10)
                        {
                            Calibration_Points = final_val;

                            EEPROM_Save[25] = Calibration_Points;

                              for (int i = 0; i < EEPROM_SIZE; i++) 
                              {
                                      EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
                              }
                              // EEPROM.put(10, PWM_Multiplier);
                              EEPROM.commit();

                            Serial.println(Calibration_Points);

                            digitalWrite(RS_485_RX_TX,1);
                            Send_ok();
                            digitalWrite(RS_485_RX_TX,0);


                        }
                  }   
          }
          else if(Cal_Rcvd_Data[1] == 'F')
          {
                
                Serial.println("\n received calibration value \n");

                val1 = Cal_Rcvd_Data[2] - 0x30; 
                val2 = Cal_Rcvd_Data[3] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)    //*F01,01,01234#
                {

                      val1 = Cal_Rcvd_Data[5] - 0x30; 
                      val2 = Cal_Rcvd_Data[6] - 0x30; 

                     Calibration_counter = (val1 * 10) + val2;

                    digitalWrite(RS_485_RX_TX,1);
                    Send_ok();
                    digitalWrite(RS_485_RX_TX,0);

                    val1 = Cal_Rcvd_Data[8] - 0x30; 
                    val2 = Cal_Rcvd_Data[9] - 0x30;
                    val3 = Cal_Rcvd_Data[10] - 0x30;
                    val4 = Cal_Rcvd_Data[11] - 0x30;
                    val5 = Cal_Rcvd_Data[12] - 0x30;

                    final_val = (val1 * 10000) + (val2 * 1000) + (val3 * 100) + (val4 * 10) + val5;

                    Serial.println("\n received flow: \n");

                    Serial.println(final_val);

                    Calibration_flow[Calibration_counter] = final_val;
                    Calibration_adc[Calibration_counter] = Delta;

                    Serial.println("\n calibration Counter: \n");

                    Serial.println(Calibration_counter);
                }
          }
          else if(Cal_Rcvd_Data[1] == 'R' && Cal_Rcvd_Data[2] == 'C'  && Cal_Rcvd_Data[3] == 'F')
          {
                Serial.println("\nsendinG configuration \n");

                val1 = Cal_Rcvd_Data[4] - 0x30; 
                val2 = Cal_Rcvd_Data[5] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                      //Serial.println("\naddress matched \n");
                      digitalWrite(RS_485_RX_TX,1);
                      Send_Configuration();
                      digitalWrite(RS_485_RX_TX,0);
                }
                
          }
          else if(Cal_Rcvd_Data[1] == 'R' && Cal_Rcvd_Data[2] == 'F')
          {
                Serial.println("\nsending flow \n");

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

          else if(Cal_Rcvd_Data[1] == 'G' && Cal_Rcvd_Data[2] == 'E' && Cal_Rcvd_Data[3] == 'T' && Cal_Rcvd_Data[4] == 'C')
          {
                Serial.println("\n sending configuration \n");
                val1 = Cal_Rcvd_Data[5] - 0x30; 
                val2 = Cal_Rcvd_Data[6] - 0x30;

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
                Serial.println("\nsetting device serial no \n");
                val1 = Cal_Rcvd_Data[3] - 0x30; 
                val2 = Cal_Rcvd_Data[4] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                    count = 6;
                    count1 = 0;
                    while(Cal_Rcvd_Data[count] != '#' && count1 < 25)
                    {
                          Device_Serial_no[count1] = Cal_Rcvd_Data[count];
                          //Serial.write(Device_Serial_no[count1]);
                          count++;count1++;
                    }
                    count1++;
                    Device_Serial_no[count1] = '#';
                    
                      digitalWrite(RS_485_RX_TX,1);
                      Send_ok();
                      digitalWrite(RS_485_RX_TX,0);
                     
                      
                      for (int i = 0; i < 25; i++) 
                      {
                            EEPROM_Save[SERIAL_NO_ADD+i] = Device_Serial_no[i];
                      }

                      Store_serial_no();

                        // Print for confirmation
                      Serial.print("\nStored Serial No: ");
                      for(count1 = 0;count1<25;count1++)
                          Serial.write(Device_Serial_no[count1]);
                }

          }
          else if(Cal_Rcvd_Data[1] == 'P' && Cal_Rcvd_Data[2] == 'S' && Cal_Rcvd_Data[3] == 'R')  //PCB SERIAL NO
          {
                Serial.println("\nsetting pcb serial no \n");

                val1 = Cal_Rcvd_Data[4] - 0x30; 
                val2 = Cal_Rcvd_Data[5] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                    count = 7;
                    count1 = 0;
                    while(Cal_Rcvd_Data[count] != '#' && count1 < 25)
                    {
                          Pcb_Serial_no[count1] = Cal_Rcvd_Data[count];
                          count++;count1++;
                    }
                    //count1++;
                    Pcb_Serial_no[count1] = '#';
                    
                      digitalWrite(RS_485_RX_TX,1);
                      Send_ok();
                      digitalWrite(RS_485_RX_TX,0);

                      for (int i = 0; i < 25; i++) 
                      {
                            EEPROM_Save[PCB_NO_ADD+i] = Pcb_Serial_no[i];
                      }

                      Store_pcb_serial_no();

                       // Print for confirmation
                      Serial.print("\nStored pcb Serial No: ");
                      for(count1 = 0;count1<25;count1++)
                          Serial.write(Pcb_Serial_no[count1]);
                }

          }
          else if(Cal_Rcvd_Data[1] == 'T' && Cal_Rcvd_Data[2] == 'S' && Cal_Rcvd_Data[3] == 'R')  //FLOW TUBE SERIAL NO
          {
                Serial.println("\nsetting flow tube serial no \n"); 

                val1 = Cal_Rcvd_Data[4] - 0x30; 
                val2 = Cal_Rcvd_Data[5] - 0x30;

                final_val = (val1 * 10) + val2;

                if(final_val == RS_485_addr)
                {
                    count = 7;
                    count1 = 0;
                    while(Cal_Rcvd_Data[count] != '#' && count1 < 25)
                    {
                          Flowtube_Serial_no[count1] = Cal_Rcvd_Data[count];
                          count++;count1++;
                    }
                    //count1++;
                    Flowtube_Serial_no[count1] = '#';
                    
                      digitalWrite(RS_485_RX_TX,1);
                      Send_ok();
                      digitalWrite(RS_485_RX_TX,0);

                      for (int i = 0; i < 25; i++) 
                      {
                            EEPROM_Save[FLOW_TUBE_ADD+i] = Flowtube_Serial_no[i];
                      }

                      Store_flow_tube_serial_no();

                       // Print for confirmation
                      Serial.print("\nStored pcb Serial No: ");
                      for(count1 = 0;count1<25;count1++)
                          Serial.write(Flowtube_Serial_no[count1]);
                }

          }
          else if(Cal_Rcvd_Data[1] == 'T')
          {
                Serial.println("\ntesting communication \n");
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

/************************************************************************************
**  Function Prototype: void Store_calb_data(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function To store received calibration data in to eeprom
*************************************************************************************/
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

/************************************************************************************
**  Function Prototype: void Send_ok(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function To send ok acknowledment over RS485
*************************************************************************************/
 void Send_ok(void)
 {
    unsigned char STRING[10];
    unsigned char cnt;
    STRING[0] = '*';
    STRING[1] = 'O';
    STRING[2] = 'K';

    STRING[3] = (RS_485_addr / 10) + '0';  
    STRING[4] = (RS_485_addr % 10) + '0';  
    STRING[5] = '/n'; // Null terminator if needed for string functions

    // Send each character with 1ms delay
    for (cnt = 0; cnt <= 4; cnt++) 
    {
      MySerial.write(STRING[cnt]);
      //Serial.write(STRING[cnt]);
      delay(5);
    }
}

 /************************************************************************************
**  Function Prototype: void Send_flow(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function To send flow rate over RS485
*************************************************************************************/
 void Send_flow(void)
{
    unsigned char STRING[20]; // Make slightly larger for safety
    char flow_buffer[10];
    unsigned char string_counter = 0;

    // Prepare flow data
    dtostrf(Flow_Rate, 5, 2, flow_buffer);  // e.g., "12.34"

    // Header
    STRING[string_counter++] = ' ';
    STRING[string_counter++] = 'F';
    STRING[string_counter++] = (RS_485_addr / 10) + '0'; 
    STRING[string_counter++] = (RS_485_addr % 10) + '0'; 
    STRING[string_counter++] = ',';

    // Copy float string
    for (int i = 0; flow_buffer[i] != '\0'; i++) 
    {
        STRING[string_counter++] = flow_buffer[i];
    }

    // Optional: add end character
    //STRING[string_counter++] = '#'; // or \n or \r, etc.

    // Send out the data
    for (int cnt = 0; cnt < string_counter; cnt++) 
    {
        MySerial.write(STRING[cnt]);
        Serial.write(STRING[cnt]);
        delay(5); 
    }

    Serial.println(); // for clarity
}

/************************************************************************************
**  Function Prototype: void Store_serial_no(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : Function To store device serial no in EEPROM
*************************************************************************************/
void Store_serial_no(void)
{
    for (int i = 0; i < 25; i++) 
    {
        EEPROM.write(SERIAL_NO_ADD + i, EEPROM_Save[SERIAL_NO_ADD + i]); 
    }
    EEPROM.commit(); 
}

/************************************************************************************
**  Function Prototype: void Store_pcb_serial_no(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To store PCB serial no in EEPROM
*************************************************************************************/
void Store_pcb_serial_no(void)
{
    for (int i = 0; i < 25; i++) 
    {
         EEPROM.write(PCB_NO_ADD + i, EEPROM_Save[PCB_NO_ADD + i]); 
    }
    EEPROM.commit(); // Save changes
}

/************************************************************************************
**  Function Prototype: void Store_flow_tube_serial_no(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To store flow tube serial no in EEPROM
*************************************************************************************/
void Store_flow_tube_serial_no(void)
{
    for (int i = 0; i < 25; i++) 
    {
         EEPROM.write(FLOW_TUBE_ADD + i, EEPROM_Save[FLOW_TUBE_ADD + i]);   
    }
    EEPROM.commit(); // Save changes
}

/************************************************************************************
**  Function Prototype: void Flush_Rx_Buffer(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To clear Cal_Rcvd_Data array
*************************************************************************************/
void Flush_Rx_Buffer(void)
{
      unsigned char count;
      for(count = 0;count<250;count++)
      {
            Cal_Rcvd_Data[count] = 0;
      }
} 

/***************************************************************************************************************
**  Function Prototype: void Convert_flow_ascii(float flow, char &d1, char &d2, char &d3, char &d4, char &d5) 
**  Passed Parameter  : Flow: Current flow rate
                        d1 - d5: Ascii charecter address
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To connver flow rate to ascii
***************************************************************************************************************/

void Convert_flow_ascii(float flow, char &d1, char &d2, char &d3, char &d4, char &d5) 
{
  int pres = flow * 100; // Shift decimal: 123.45 → 12345

  d1 = (pres / 10000) % 10 + '0'; // hundreds
  d2 = (pres / 1000) % 10 + '0';  // tens
  d3 = (pres / 100) % 10 + '0';   // units
  d4 = (pres / 10) % 10 + '0';    // tenths
  d5 = (pres % 10) + '0';         // hundredths
}


/************************************************************************************
**  Function Prototype: void Read_Configuration(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To read configuration data over RS485 and strore it in eeprom
*************************************************************************************/
void Read_Configuration(void)
{
       //*CF01,PS01,OF1234,ZF1234,ST55,EN70,ZFR1234,EF01#
       //*CF<device_no>, PS<Pipe_Size>, OF<Output Frequency>, ZF<Zero_Flow_Delta>, ST<Measurment_Start>, EN<Measurment_End>, ZFR<Zero_Flow_rate>, EF<Exitation_Freq#

       unsigned char val1,val2,val3,val4,val5,val6;
       unsigned int final_val;
       val1 = Cal_Rcvd_Data[3] - 0x30; 
       val2 = Cal_Rcvd_Data[4] - 0x30;

      final_val = (val1 * 10) + val2;

      //if(final_val == RS_485_addr)
      {
          if(Cal_Rcvd_Data[6] == 'P' &&  Cal_Rcvd_Data[7] == 'S')   //PIPE SIZE
          {
                  val1 = Cal_Rcvd_Data[8] - 0x30; 
                  val2 = Cal_Rcvd_Data[9] - 0x30;

                   final_val = (val1 * 10) + val2;

                   if(final_val > 0 && final_val <= 2)
                   {
                        Pipe_Size = final_val;

                        EEPROM_Save[12]= Pipe_Size;
                   }
                   Serial.print("Pipe Size: ");
                  Serial.println(Pipe_Size);

                   //while(1);
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

                   Serial.print("Output frequency: ");
                   Serial.println(PWM_RANGE);

                    EEPROM_Save[22] = (PWM_RANGE >>8) & 0xff;   // msb
                    EEPROM_Save[21] = PWM_RANGE & 0xff;

                   //while(1);
                   
          }
          if(Cal_Rcvd_Data[18] == 'Z' &&  Cal_Rcvd_Data[19] == 'F')   //zero flow 
          {
                  val1 = Cal_Rcvd_Data[20] - 0x30; 
                  val2 = Cal_Rcvd_Data[21] - 0x30;
                  val3 = Cal_Rcvd_Data[22] - 0x30;
                  val4 = Cal_Rcvd_Data[23] - 0x30;
                   
                  final_val = (val1 * 1000) + (val2 * 100) + (val3 * 10) + val4;

                  Zero_Flow_Offset = final_val;   

                   Serial.print("Zero Flow Offset: ");
                   Serial.println(Zero_Flow_Offset);

                  EEPROM_Save[18] =(Zero_Flow_Offset>>8) & 0xff;   // msb
                  EEPROM_Save[17] = Zero_Flow_Offset & 0xff;         //lsb   

                  // while(1);
          }
          if(Cal_Rcvd_Data[25] == 'S' &&  Cal_Rcvd_Data[26] == 'T')   //START TIME
          {
                  val1 = Cal_Rcvd_Data[27] - 0x30; 
                  val2 = Cal_Rcvd_Data[28] - 0x30;

                  final_val = (val1 * 10) + val2;
                  
                  Recieve_ADC_Read_Delay = final_val;    

                  Serial.print("Start time: ");
                  Serial.println(Recieve_ADC_Read_Delay);

                  EEPROM_Save[2] = (Recieve_ADC_Read_Delay>>8) & 0xff;   // msb
                  EEPROM_Save[1]= Recieve_ADC_Read_Delay & 0xff;            //lsb
  
          }

          if(Cal_Rcvd_Data[30] == 'E' &&  Cal_Rcvd_Data[31] == 'N')   //END TIME
          {
                  val1 = Cal_Rcvd_Data[32] - 0x30; 
                  val2 = Cal_Rcvd_Data[33] - 0x30;

                  final_val = (val1 * 10) + val2;
                  
                  Recieve_ADC_Read_Stop_Delay = final_val;    

                  Serial.print("end time: ");
                  Serial.println(Recieve_ADC_Read_Stop_Delay);

                  EEPROM_Save[4] = (Recieve_ADC_Read_Stop_Delay>>8) & 0xff;   // msb
                  EEPROM_Save[3]= Recieve_ADC_Read_Stop_Delay & 0xff;         //lsb
          }

          if(Cal_Rcvd_Data[35] == 'Z' &&  Cal_Rcvd_Data[36] == 'F' &&  Cal_Rcvd_Data[37] == 'R')   //ZERO FLOW VALUE
          {
                  val1 = Cal_Rcvd_Data[38] - 0x30; 
                  val2 = Cal_Rcvd_Data[39] - 0x30;
                  val3 = Cal_Rcvd_Data[40] - 0x30;
                  val4 = Cal_Rcvd_Data[41] - 0x30;

                   final_val = (val1 * 1000) + (val2 * 100) + (val3 * 10) + val4;

                  Zero_Flow_rate_val = final_val;  

                  Serial.print("Zero flow value: ");
                  Serial.println(Zero_Flow_rate_val);

                  //zero flow rate value 
                  EEPROM_Save[67] = (Zero_Flow_rate_val >>8) & 0xff;   // msb
                  EEPROM_Save[66] = Zero_Flow_rate_val & 0xff;

          }

          if(Cal_Rcvd_Data[43] == 'E' &&  Cal_Rcvd_Data[44] == 'F')   //EXITATATION FREQUENCY
          {
                  val1 = Cal_Rcvd_Data[45] - 0x30; 
                  val2 = Cal_Rcvd_Data[46] - 0x30;

                  final_val = (val1 * 10) + val2;
                  
                  Excitation_Frequency  = final_val;    
  

                  Serial.print("Excitation Frequency: ");
                  Serial.println(Excitation_Frequency);

                  EEPROM_Save[7] = Excitation_Frequency;
                   //while(1);

                  //Timer1_init();
          }

          for (int i = 0; i < EEPROM_SIZE; i++) 
          {
                  EEPROM.write(EEPROM_ADDR + i, EEPROM_Save[i]);
          }
          // EEPROM.put(10, PWM_Multiplier);
          EEPROM.commit();

          digitalWrite(RS_485_RX_TX,1);
          Send_ok();
          digitalWrite(RS_485_RX_TX,0);
      }

}
/************************************************************************************
**  Function Prototype: void Send_Configuration(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To send configuration data over RS485 in response to command
*************************************************************************************/
void Send_Configuration(void)
{
        unsigned char STRING[200];
        //char LOAD1, LOAD2, LOAD3, LOAD4,LOAD5, LOAD6;
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

        CONVERT_INTEGER_ASCII(PWM_RANGE);
  
        STRING[++string_counter] = LOAD3;
        STRING[++string_counter] = LOAD4;
        STRING[++string_counter] = LOAD5;
        STRING[++string_counter] = LOAD6;
        STRING[++string_counter] = ',';

        //ZERO FLOW OFFSET
        STRING[++string_counter] = 'Z';
        STRING[++string_counter] = 'F';

        CONVERT_INTEGER_ASCII(Zero_Flow_Offset);
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

        CONVERT_INTEGER_ASCII(Zero_Flow_rate_val);
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

        //string_counter++;
       for (int i = 0; i < 25; i++) 
       {
            if (Device_Serial_no[i] == '#') break;
            STRING[++string_counter] = Device_Serial_no[i];  // increment + copy
       }
        string_counter--;
        STRING[++string_counter] = ',';

        //PCB SERIAL NO
        STRING[++string_counter] = 'P';
        STRING[++string_counter] = 'S';
        STRING[++string_counter] = 'R';

        //string_counter++;
        for (int i = 0; i < 25; i++) 
        {
            if (Pcb_Serial_no[i] == '#') break;
            STRING[++string_counter] = Pcb_Serial_no[i];
        }
        STRING[++string_counter] = ',';

        //FLOW TUBE SERIAL NO
        STRING[++string_counter] = 'F';
        STRING[++string_counter] = 'S';
        STRING[++string_counter] = 'R';

        //string_counter++;
        for (int i = 0; i < 25; i++)
        {
            if (Flowtube_Serial_no[i] == '#') break;
            STRING[++string_counter] = Flowtube_Serial_no[i];
        }
        STRING[++string_counter] = '#';

        string_size = string_counter+1;

        for(count = 0;count < string_size; count++)
        {
            MySerial.write(STRING[count]);
            Serial.write(STRING[count]);
            delay(5); 
        }
        
}

/************************************************************************************
**  Function Prototype: void Send_Calibration(void)
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To send calibration data over RS485 in response to command
*************************************************************************************/
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

/************************************************************************************
**  Function Prototype: void Led_Operation(void) 
**  Passed Parameter  : None
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To control led indication based on device operation status
*************************************************************************************/
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

/**************************************************************************
**  Function Prototype: void writeStringToEEPROM(int startAddr, const String& data)  
**  Passed Parameter  : Starting adrees of memory
                        data array
**  Returned Parameter: None
**  Date of Creation  : 30/04/2025
**  Discription       : To write characters to memory from string
***************************************************************************/
void writeStringToEEPROM(int startAddr, const String& data) 
{
  int len = data.length();
  for (int i = 0; i < len; i++) 
  {
    EEPROM.write(startAddr + i, data[i]);
  }
  EEPROM.write(startAddr + len, '\0'); // null-terminate
}

/**************************************************************************
**  Function Prototype: String readStringFromEEPROM(int startAddr) 
**  Passed Parameter  : Starting adrees of memory
**  Returned Parameter: read sring from memory
**  Date of Creation  : 30/04/2025
**  Discription       : To read characters from memory and form string
***************************************************************************/
String readStringFromEEPROM(int startAddr) 
{
  String value;
  char ch;
  while ((ch = EEPROM.read(startAddr++)) != '\0') 
  {
    value += ch;
  }
  return value;
}

/******************************************************************************************************************************
**  Function Prototype: void Web_server(void) 
**  Passed Parameter  : None
**  Returned Parameter: none
**  Date of Creation  : 30/04/2025
**  Discription       : To put device in Access point mode to receive wifi ssid and password
*********************************************************************************************************************************/
void Web_server(void)
{

   WiFi.disconnect(true);
  
    // Start Wi-Fi in AP mode
  WiFi.softAP(ssid1, password1);

  // Print the AP IP address
  Serial.print("Access Point Started. Connect to Wi-Fi with SSID: ");
  Serial.println(ssid1);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Configure web server routes
  server.on("/", handleRoot);        // Serve the configuration form
  server.on("/submit", handleSubmit); // Handle form submissions

  // Start the web server
  server.begin();
  Serial.println("HTTP server started");
}

//-------------------------bootloader functions-----------------------------------

/******************************************************************************************************************************
**  Function Prototype: bool getDeviceDetails(const String &chipId, String &api_key, String &api_secret) 
**  Passed Parameter  : chipId     - The unique chip ID of the device.
//                      api_key    - address of API key.
//                      api_secret - address of API secret.
**  Returned Parameter: true if notification is successfully sent, false otherwise
**  Date of Creation  : 30/04/2025
**  Discription       : Retrieve the api key and secret key from sever
*********************************************************************************************************************************/
bool getDeviceDetails(const String &chipId, String &api_key, String &api_secret) 
{
  HTTPClient http;
  const char* url = "https://iotweet.io/api/method/beetwin_iot.beetwin_iot.api.device_details.get_device_details";
  
  // Initialize HTTP connection
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Build JSON body for the POST request, including chip ID and DACK flag.
  String requestBody = "{\"ts\":1734348733000,\"values\":{\"IM\":\"" + chipId + "\",\"DACK\":0}}";
  //String requestBody = "{\"ts\":1734348733000,\"values\":{\"IM\":\"1224-045-00001\", \"DACK\":0}}";


  Serial.println("[BOOT] Requesting device details...");
  Serial.println("Request Body: " + requestBody);

  // Send POST request to the API.
  int httpCode = http.POST(requestBody);
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[ERROR] Device details request failed, HTTP code: %d\n", httpCode);
    http.end();
    return false;
  }

  // Retrieve and log the response payload.
  String payload = http.getString();
  Serial.println("Device Details Response: " + payload);
  http.end();

  // Parse the JSON response.
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("[ERROR] JSON parse failed: ");
    Serial.println(error.c_str());
    return false;
  }

  // Extract the 'message' object from the JSON.
  JsonObject message = doc["message"];
  if (message.isNull()) {
    Serial.println("[ERROR] 'message' field not found in device details response.");
    return false;
  }

  // Retrieve API credentials from the JSON response.
  api_key   = message["api_key"].as<String>();
  api_secret = message["api_secret"].as<String>();

  // Validate that both API credentials are not empty.
  if (api_key == "" || api_secret == "") {
    Serial.println("[ERROR] API credentials missing in device details response.");
    return false;
  }

  Serial.println("[BOOT] Received API credentials.");
  return true;
}


/******************************************************************************************************************************
**  Function Prototype: bool getFirmwareUrl(const String &chipId, const String &api_key, const String &api_secret, String &firmware_url) 
**  Passed Parameter  : chipId     - The unique chip ID of the device.
//                      api_key    - API key for HTTP Basic Authentication.
//                      api_secret - API secret for HTTP Basic Authentication.
                        firmware_url - (Output) Firmware URL retrieved from the API response
**  Returned Parameter: true if notification is successfully sent, false otherwise
**  Date of Creation  : 30/04/2025
**  Discription       : Retrieve the OTA firmware URL from the configuration API using HTTP Basic Authentication.
*********************************************************************************************************************************/

bool getFirmwareUrl(const String &chipId, const String &api_key, const String &api_secret, String &firmware_url) 
{
  HTTPClient http;
  const char* url = "https://iotweet.io/api/method/beetwin_iot.beetwin_iot.api.device_config.process_new_config_handle_request";
  
  // Begin HTTP connection and set necessary headers.
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  // Set HTTP Basic Authentication credentials.
  http.setAuthorization(api_key.c_str(), api_secret.c_str());

  // Construct device key by concatenating "LSPL_" with the chip ID.
  String device_key = "LSPL_" + chipId;
  // Build JSON body with flags indicating the start of OTA update.
  String requestBody = "{\"device_key\":\"" + device_key + "\",\"is_ota\":1,\"OACK\":0}";
  Serial.println("[BOOT] Requesting firmware URL...");
  Serial.println("Request Body: " + requestBody);

  // Send POST request to obtain the firmware URL.
  int httpCode = http.POST(requestBody);
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[ERROR] Firmware URL request failed, HTTP code: %d\n", httpCode);
    http.end();
    return false;
  }

  // Retrieve and log the firmware URL response.
  String payload = http.getString();
  Serial.println("Firmware URL Response: " + payload);
  http.end();

  // Parse JSON response.
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("[ERROR] JSON parse failed: ");
    Serial.println(error.c_str());
    return false;
  }

  // Extract the 'message' object.
  JsonObject message = doc["message"];
  if (message.isNull()) {
    Serial.println("[ERROR] 'message' field not found in firmware URL response.");
    return false;
  }

  // Extract the firmware URL from the "ota_file_url" field.
  firmware_url = message["ota_file_url"].as<String>();
  if (firmware_url == "") {
    Serial.println("[ERROR] Firmware URL is empty in the response.");
    return false;
  }

  Serial.println("[BOOT] Firmware URL obtained.");
  return true;
}


/******************************************************************************************************************************
**  Function Prototype: bool notifyFirmwareUpdateStatus(const String &chipId, const String &api_key, const String &api_secret) 
**  Passed Parameter  : chipId     - The unique chip ID of the device.
//                      api_key    - API key for HTTP Basic Authentication.
//                      api_secret - API secret for HTTP Basic Authentication.
**  Returned Parameter: true if notification is successfully sent, false otherwise
**  Date of Creation  : 30/04/2025
**  Discription       : Notify the server that the firmware update process has been completed.
*********************************************************************************************************************************/
bool notifyFirmwareUpdateStatus(const String &chipId, const String &api_key, const String &api_secret) 
{
  HTTPClient http;
  const char* url = "https://iotweet.io/api/method/beetwin_iot.beetwin_iot.api.device_config.process_new_config_handle_request";
  
  // Begin HTTP connection and set headers.
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  // Use HTTP Basic Authentication.
  http.setAuthorization(api_key.c_str(), api_secret.c_str());

  // Construct device key.
  String device_key = "LSPL_" + chipId;
  // Build JSON body with flags indicating OTA update completion.
  String requestBody = "{\"device_key\":\"" + device_key + "\",\"is_ota\":0,\"OACK\":1}";
  Serial.println("[BOOT] Notifying firmware update status...");
  Serial.println("Request Body: " + requestBody);

  // Send POST request to notify the server.
  int httpCode = http.POST(requestBody);
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[ERROR] Firmware update status notification failed, HTTP code: %d\n", httpCode);
    http.end();
    return false;
  }
  
  // Retrieve and log the server's response.
  String payload = http.getString();
  Serial.println("Firmware Update Status Response: " + payload);
  http.end();
  return true;
}

/************************************************************************************************************************************************
**  Function Prototype: void performOTAUpdate(const String &firmware_url, const String &chipId, const String &api_key, const String &api_secret) 
**  Passed Parameter  : Firmware download url, Chip id(device pcb no), api key, secret kay
**  Returned Parameter: Batch num or Cycle Entry status.
**  Date of Creation  : 30/04/2025
**  Discription       : This Function connects to server with apikey and secret kay and download the firmware from given url
*************************************************************************************************************************************************/

void performOTAUpdate(const String &firmware_url, const String &chipId, const String &api_key, const String &api_secret) 
{
  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.println("[ERROR] No Wi-Fi connection!");
    return;
  }

  Serial.println("[BOOT] Downloading firmware...");

  HTTPClient http;
  http.begin(firmware_url);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[ERROR] HTTP Request Failed! Error code: %d\n", httpCode);
    http.end();
    return;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0) {
    Serial.println("[ERROR] Invalid firmware file size!");
    http.end();
    return;
  }

  WiFiClient* stream = http.getStreamPtr();

  Serial.printf("[BOOT] Firmware size: %d bytes\n", contentLength);

  // Check if enough space is available
  if (!Update.begin(contentLength)) {
    Serial.println("[ERROR] Not enough space for OTA update!");
    http.end();
    return;
  }

  size_t written = 0;
  uint8_t buff[128] = { 0 };  // small buffer for better control
  unsigned long lastProgressTime = millis();

  while (http.connected() && (written < contentLength)) {
    size_t available = stream->available();
    if (available) {
      int bytesRead = stream->readBytes(buff, ((available > sizeof(buff)) ? sizeof(buff) : available));
      if (Update.write(buff, bytesRead) != bytesRead) {
        Serial.println("[ERROR] Write failed during OTA!");
        Update.abort();
        http.end();
        return;
      }
      written += bytesRead;

      // Optional: Progress output
      if (millis() - lastProgressTime > 1000) {
        Serial.printf("[BOOT] OTA Progress: %d/%d bytes\n", written, contentLength);
        lastProgressTime = millis();
      }
    }
    delay(1);  // yield to WiFi stack
  }

  if (written == contentLength) {
    Serial.println("[BOOT] Firmware written successfully!");
    if (Update.end(true)) {
      Serial.println("[BOOT] OTA Update successful.");
      
      if (notifyFirmwareUpdateStatus(chipId, api_key, api_secret)) {
        Serial.println("[BOOT] Firmware update status notified successfully.");
      } else {
        Serial.println("[ERROR] Firmware update status notification failed.");
      }
      
      Serial.println("[BOOT] Restarting...");
      ESP.restart();
    } else {
      Serial.printf("[ERROR] OTA end failed: %s\n", Update.errorString());
    }
  } else {
    Serial.printf("[ERROR] OTA Incomplete: Written %d bytes, Expected %d bytes\n", written, contentLength);
    Update.abort();
  }

  http.end();
}


