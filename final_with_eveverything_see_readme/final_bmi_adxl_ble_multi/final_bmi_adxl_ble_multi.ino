#include <nrf_soc.h>
#include "lp_BLESerial.h"
#include "mysht40.h"
#include <SPI.h>
#include <Wire.h>
#include "bmi2xx.h"
#include "myadxl362.h"
#include "Arduino.h"
#include <SD.h>


int gyr_detecter = 0;
long motion_sensor_time_interval = 80;
#define adxl_slaveSelectPin 15
long interval_T1 = 5;
long interval_T3 = 5000;
long interval_T4 = motion_sensor_time_interval;
// print brightness of LED3 every 5 seconds
bool bmiflag = 0;

unsigned long prevTime_T1 = millis();
unsigned long prevTime_T3 = millis();
unsigned long prevTime_T4 = millis();
bool shtflag = 0;

int automatical_time_interval_counter = 0;
bool automatical_time_interval_function_switch = 1;
bool t_and_h_output_switch = 0;

int offset = 0;
File my_sd_File;
MYADXL362 adxl;
MYSHT40 sht;
int16_t adxl_XData = 0, adxl_YData = 0, adxl_ZData = 0, adxl_Temperature = 0;

// int config_runtime_flag = 0;
uint8_t bmi_pwr_control;  // 0

uint8_t adxl_pwr_control;  // 1

uint8_t bmi_acc_config;  // 2
uint8_t bmi_gyr_config;

uint8_t adxl_hz_config;  // 3


uint8_t te_hu_control;     // 4
uint8_t infrared_control;  // 5

int config_from_ble_counter = 0;  //因为收到的数字我需要++ ，总不能加到7 8 9把，每次到+到6的时候就会清零。从头开始储存参数

uint8_t received_config_from_ble[6];



lp_BLESerial blePeripheral;




BLEService sensorConfigService = BLEService("19B10000-E8F2-537E-4F6C-D104768A1214");

// create switch characteristic
BLECharCharacteristic sensors_config_Characteristic = BLECharCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


BLEService Start_Service = BLEService("f2783635-3a07-4a51-9696-459ae784363e");  // 我发现只要一个lp_BLESerial blePeripheral就够了。我两个ttribute，到时候收到消息可以分辨是哪里收到了消息


BLECharCharacteristic Start_Characteristic = BLECharCharacteristic("f2783636-3a07-4a51-9696-459ae784363e", BLERead | BLEWrite);


String sd_filename = "where_is_my_name.CSV";
unsigned int BMI_chip_ID;
int file_count;
int config_size;
int BMI_init_status;
int BMI_power_mode;
BMI2xx bmi2xx;


void setup() {
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  // put your setup code here, to run once:
  Serial.begin(115200);
  NRFSetup_Callback();
  BLESetup_Callback();
  delay(50);
  Serial.println("setup finished");
}
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();

  // Task 1 : Blink LED1 (T1)
  if (currentTime - prevTime_T1 > interval_T1) {
    BLECentral central = blePeripheral.central();
    if (central) {
      if (blePeripheral.available()) {
        digitalWrite(13, LOW);
        String receivedMessage = blePeripheral.readString();
        Serial.print("Received message: ");
        Serial.println(receivedMessage);
        receivedMessage.remove(receivedMessage.length() - 1);  // 这里必去去掉最后一个元素，我从mit2app收到的最后一个元素不知道是什么，但是由于这个元素，导致我储存的数据csv前两行会有便宜。应该是最后这个休止符导致的
        sd_filename = receivedMessage;
        offset = 0;  // 原来bug在这里，当我更改文件名的时候，需要把offset清零，不然是从上次断点开始计数，比如从200开始存数据，那么前面两百可能就有坏数据
        //每次改名字的时候打头，，汇报暂时不加
        //   SD.begin(27);

        // Serial.println("rename");
        // my_sd_File = SD.open(sd_filename, FILE_WRITE);

        // if (my_sd_File) {
        //   String csv_file_name = "stamp,bmi_acc_x,bmi_acc_y,bmi_acc_z,bmi_gyr_x,bmi_gyr_y,bmi_gyr_z,adxl_x,adxl_y,adxl_z\n";
        //   my_sd_File.write(csv_file_name.c_str(), csv_file_name.length());


        //   my_sd_File.close();  //

        // }






        // if (receivedMessage[0] == 'F') {  // 这里居然只能单引号

        //   Serial.print("File name is: ");
        //   Serial.println(receivedMessage.substring(2));
        //   sd_filename = receivedMessage.substring(2).c_str();
        //   sd_filename_store_outside_loop = sd_filename;
        //   Serial.print("varialble is: ");
        //   Serial.println(sd_filename);
        // }
        digitalWrite(13, HIGH);
      }

      // central still connected to peripheral

      if (sensors_config_Characteristic.written()) {
        digitalWrite(13, LOW);
        received_config_from_ble[config_from_ble_counter] = sensors_config_Characteristic.value();
        config_from_ble_counter++;
        if (config_from_ble_counter == 6) {
          config_from_ble_counter = 0;
          Serial.print("Current config:  ");
          for (int i = 0; i < 6; i++) {
            Serial.print(received_config_from_ble[i], HEX);
            Serial.print(" ");
          }
          Serial.println(" ");
          config_all_sensors();
          digitalWrite(13, HIGH);  // 放这里最合适，因为每次点了config device收到数据就直接配置了，不用去action开头判断 配置不配置
        }
      }

      if (Start_Characteristic.written()) {
        // Serial.print(Start_Characteristic.value(), HEX);
        // Serial.println(" ");
        if (Start_Characteristic.value() == 0x01) {



          bmiflag = 1;

          // Serial.println("sht.on_or_off_status");
          // Serial.println(sht.on_or_off_status);
          // if (sht.on_or_off_status == 1) {
          //   runner.addTask(Sht40_Action);        // if等以后添加。因为这里添加后会失去一个功能，读取数据的时候不能配置device。
          //   Sht40_Action.enable();
          // }

          shtflag = 1;
        }
        if (Start_Characteristic.value() == 0x00) {

          bmiflag = 0;


          shtflag = 0;

          // config_runtime_flag = 0;


          //                       pinMode(24, OUTPUT);
          // digitalWrite(24, LOW);
          //               pinMode(9, OUTPUT);
          // digitalWrite(9, LOW);
          //       pinMode(10, OUTPUT);
          // digitalWrite(10, LOW);
          //  Wire.end();
        }
      }
      // Send a message over BLE
      // bleSerial.println("Hello from NRF52832!");
      //dummy, ble is high frequency, set up other dummy tasks

      // delay(1);
    }

    prevTime_T1 = currentTime;
  }

  // Task 2 : Glow LED2 when BTN is pressed
  if (((currentTime - prevTime_T3) > interval_T3) && (shtflag == 1)) {
    if (t_and_h_output_switch == 1) {
      Sht40_Action_Callback();
      prevTime_T3 = currentTime;
    }
  }

  // Task 3 : Read input from serial monitor (0-255) and then write to LED3


  // Task 4 : print the brightness of LED3 in the serial monitor after every 5 seconds
  if (((currentTime - prevTime_T4) > interval_T4) && (bmiflag == 1)) {
    unsigned long offset1 = 0;  // 时间偏移量
    unsigned long currentTime = millis();
    if (currentTime > 60000) {
      offset1 = currentTime;
    }
    uint16_t actualTime = currentTime - offset1;

    unsigned long startTime = micros();  // 记录开始时间
    // 执行一些代码




    // pinMode(22, OUTPUT);  //打开IC开关
    // digitalWrite(22, LOW);

    // if (config_runtime_flag == 0) {
    //   config_all_sensors();  //我发现放在BLEaction也不行，以为可能还没执行到这个函数，BMI_ADXL_Action就开始执行了，所以导致没有输出，我必须保证在读取前执行这个配置所以只有一种办法，
    //                          //使用BMI_ADXL_Action的时候调用这个一次，只能一次，不然一直配置，然后用flag表示执行过，当我按了stop，flag为0，下次执行BLE ACTIOND的时候又会仅仅配置一次
    //   config_runtime_flag = 1;
    // }


    SPI.setDataMode(SPI_MODE3);







    int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
    uint8_t lsb_acc_x, msb_acc_x, lsb_acc_y, msb_acc_y, lsb_acc_z, msb_acc_z;  //比如从both更改为acc之后，gyr有前一次的值，我想变为0，我把这里所有给了初始值，但是我发现还是不等于0
    uint8_t lsb_gyr_x, msb_gyr_x, lsb_gyr_y, msb_gyr_y, lsb_gyr_z, msb_gyr_z;
    // Burst data read from ACC_x_0_7 to Gyr_8_13
    byte dataToSend = accel_gyr_addr | READ;

    digitalWrite(14, LOW);
    SPI.transfer(dataToSend);  //Send register location
    SPI.transfer(0x00);
    lsb_acc_x = SPI.transfer(0x00);
    msb_acc_x = SPI.transfer(0x00);
    lsb_acc_y = SPI.transfer(0x00);
    msb_acc_y = SPI.transfer(0x00);
    lsb_acc_z = SPI.transfer(0x00);
    msb_acc_z = SPI.transfer(0x00);

    lsb_gyr_x = SPI.transfer(0x00);
    msb_gyr_x = SPI.transfer(0x00);
    lsb_gyr_y = SPI.transfer(0x00);
    msb_gyr_y = SPI.transfer(0x00);
    lsb_gyr_z = SPI.transfer(0x00);
    msb_gyr_z = SPI.transfer(0x00);
    digitalWrite(14, HIGH);




    SPI.setDataMode(SPI_MODE0);



    digitalWrite(adxl_slaveSelectPin, LOW);
    SPI.transfer(0x0B);  // read instruction
    SPI.transfer(0x0E);  // Start at XData Reg
    adxl_XData = SPI.transfer(0x00);
    adxl_XData = adxl_XData + (SPI.transfer(0x00) << 8);
    adxl_YData = SPI.transfer(0x00);
    adxl_YData = adxl_YData + (SPI.transfer(0x00) << 8);
    adxl_ZData = SPI.transfer(0x00);
    adxl_ZData = adxl_ZData + (SPI.transfer(0x00) << 8);
    adxl_Temperature = SPI.transfer(0x00);
    adxl_Temperature = adxl_Temperature + (SPI.transfer(0x00) << 8);



    digitalWrite(adxl_slaveSelectPin, HIGH);
    acc_x = ((int16_t)msb_acc_x << 8) | (int16_t)lsb_acc_x;
    acc_y = ((int16_t)msb_acc_y << 8) | (int16_t)lsb_acc_y;
    acc_z = ((int16_t)msb_acc_z << 8) | (int16_t)lsb_acc_z;

    gyr_x = ((int16_t)msb_gyr_x << 8) | (int16_t)lsb_gyr_x;
    gyr_y = ((int16_t)msb_gyr_y << 8) | (int16_t)lsb_gyr_y;
    gyr_z = ((int16_t)msb_gyr_z << 8) | (int16_t)lsb_gyr_z;

    Serial.print("acc_x = ");
    Serial.print(acc_x);
    Serial.print("\t");

    Serial.print("acc_y = ");
    Serial.print(acc_y);
    Serial.print("\t");

    Serial.print("acc_z = ");
    Serial.print(acc_z);
    Serial.print("\t");

    Serial.print("gyr_x = ");
    Serial.print(gyr_x);
    Serial.print("\t");

    Serial.print("gyr_y = ");
    Serial.print(gyr_y);
    Serial.print("\t");

    Serial.print("gyr_z = ");
    Serial.print(gyr_z);
    Serial.print("\t");

    unsigned long endTime = micros();  // 记录结束时间

    unsigned long elapsedTime = endTime - startTime;  // 计算运行时间

    //  Serial.print("Time elapsed: ");
    //  Serial.print(elapsedTime);
    // Serial.print("     ");

    // acc_x |= ((uint16_t)msb_acc_x << 8) | (uint16_t)lsb_acc_x;

    // Serial.print("acc_x = ");
    // if ((acc_x >> 15) & 0x01 == 1) {
    //   Serial.print("-");
    //   acc_x = (~acc_x) + 1;
    // }
    // Serial.print(acc_x);
    // Serial.print("\t");

    // acc_y |= ((uint16_t)msb_acc_y << 8) | (uint16_t)lsb_acc_y;
    // Serial.print("acc_y = ");
    // if ((acc_y >> 15) & 0x01 == 1) {
    //   Serial.print("-");
    //   acc_y = (~acc_y) + 1;
    // }
    // Serial.print(acc_y);
    // Serial.print("\t");

    // acc_z |= ((uint16_t)msb_acc_z << 8) | (uint16_t)lsb_acc_z;
    // Serial.print("acc_z = ");
    // if ((acc_z >> 15) & 0x01 == 1) {
    //   Serial.print("-");
    //   acc_z = (~acc_z) + 1;
    // }
    // Serial.print(acc_z);
    // Serial.print("\t");



    // gyr_x |= ((uint16_t)msb_gyr_x << 8) | (uint16_t)lsb_gyr_x;
    // Serial.print("gyr_x = ");
    // if ((gyr_x >> 15) & 0x01 == 1) {
    //   Serial.print("-");
    //   gyr_x = (~gyr_x) + 1;
    // }
    // Serial.print(gyr_x);
    // Serial.print("\t");

    // gyr_y |= ((uint16_t)msb_gyr_y << 8) | (uint16_t)lsb_gyr_y;
    // Serial.print("gyr_y = ");
    // if ((gyr_y >> 15) & 0x01 == 1) {
    //   Serial.print("-");
    //   gyr_y = (~gyr_y) + 1;
    // }
    // Serial.print(gyr_y);
    // Serial.print("\t");

    // gyr_z |= ((uint16_t)msb_gyr_z << 8) | (uint16_t)lsb_gyr_z;
    // Serial.print("gyr_z = ");
    // if ((gyr_z >> 15) & 0x01 == 1) {
    //   Serial.print("-");
    //   gyr_z = (~gyr_z) + 1;
    // }
    // Serial.print(gyr_z);

    Serial.print("\t");
    //Serial.println("\t");


    // Serial.print("BMI ST:");
    // BMI_init_status = bmi2xx.readRegister(INT_STATUS, 2);
    // Serial.println(BMI_init_status);




    //  Serial.print("sdname=");
    //Serial.print(sd_filename);


    Serial.print("XVALUE=");
    Serial.print(adxl_XData);
    Serial.print("\tYVALUE=");
    Serial.print(adxl_YData);
    Serial.print("\tZVALUE=");
    Serial.print(adxl_ZData);
    Serial.print("\tTEMPERATURE=");
    Serial.println(adxl_Temperature);

    int16_t input_data[9] = { acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, adxl_XData, adxl_YData, adxl_ZData };

    char buffer[20000];  //必须很大，不然会出现乱码

    offset += sprintf(buffer + offset, "%lu", currentTime);
    offset += sprintf(buffer + offset, ",");

    for (int i = 0; i < 9; i++) {  //这里不能是sizeof input，不然数据是16个
      offset += sprintf(buffer + offset, "%d", input_data[i]);

      // Add comma delimiter for all elements except the last one
      if (i < 8) {
        offset += sprintf(buffer + offset, ",");
      }
    }
    Serial.println("offset.");
    Serial.println(offset);
    buffer[offset++] = '\n';  //最后添加换行
                              // Serial.println("offst...");
                              //   Serial.println(offset);
    if (offset > 5000) {
      // delay(10);
      //
      //不能在开头初始化，不然会影响spi传输，
      SD.begin(27);
      // delay(10);
      //my_sd_File = SD.open("test.csv", FILE_WRITE);
      // int sd_file_length = strlen(sd_filename_store_outside_loop);
      // char sd_name_buffer[sd_file_length + 1];
      // strcpy(sd_name_buffer, sd_filename);
      Serial.println("write successfully...");
      Serial.println(sd_filename);
      my_sd_File = SD.open(sd_filename, FILE_WRITE);

      if (my_sd_File) {



        // Serial.print("Writing to data.csv...");
        my_sd_File.write(buffer, offset);
        offset = 0;
        my_sd_File.close();  //
        // Serial.println("sd_filename wonr?");
        // Serial.println(sd_filename);
      }
    }

    prevTime_T4 = currentTime;

    if (automatical_time_interval_function_switch == 1) {
      automatical_time_interval_counter++;
      gyr_detecter = gyr_detecter + abs(gyr_z);
      if (automatical_time_interval_counter == 10) {
        gyr_detecter = gyr_detecter / 10;  // 玄学，如果这里没有除法，写入文件不会出错
        gyr_detecter = (int)gyr_detecter;
        automatical_time_interval(gyr_detecter);
        automatical_time_interval_counter = 0;
        gyr_detecter = 0;
      }
    }
  }
}

void Sht40_Setup() {
  pinMode(24, OUTPUT);
  digitalWrite(24, HIGH);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);

  Wire.begin();

  sht.customized_mode_sht40(0xF6);

  delay(10);
  Serial.println("sht40 initialization setup finished...");
}
void Sht40_Action_Callback() {
  float* sht_data = sht.get_data_float();  // 获取数据，返回一个指向包含温度和湿度值的浮点数数组的指针, cpp必须使用static，不然读出来全是0
  Serial.print("Temperature: ");
  Serial.print(sht_data[0]);
  Serial.print(", Humidity: ");
  Serial.print(sht_data[1]);
  Serial.println("%");
}
void NRFSetup_Callback() {
  GPIO_Initiation();
  pinMode(13, OUTPUT);  // LED输出

  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(27, OUTPUT);

  pinMode(22, OUTPUT);  //打开IC开关
  digitalWrite(22, HIGH);

  pinMode(23, OUTPUT);  //打开IC开关
  digitalWrite(23, HIGH);

  pinMode(24, OUTPUT);  //打开IC开关
  digitalWrite(24, HIGH);





  BMI_ADXL_Setup();


  Sht40_Setup();
}

void BMI_ADXL_Setup() {
  // if (BMI_init_status != 1) { // 我发现BMI初始化放在这里最好，因为修改模式和初始化完全无关，我刚开始是放在BMI_ADXL_Action里面的，这导致第一次使用的时候很慢，要等待初始化完成，但是放在这里的话
  //                         // 在我配置手机蓝牙的时候，它自己就开始初始化了，我点START之后立马开始运行了，这样好得多
  // 绝了 ，这里配置不信，导致BMI只会输出acc数据，gyr一直为0，但是放在setup_callback就没事。  我最后妥协了，在nrfsetup callback里面呼叫了这个函数，变成顺序执行
  //   while (1) {
  //     BMISetup();
  //     if (BMI_init_status == 1) {
  //       break;
  //     }
  //   }
  // }

  // adxl.Initialization(adxl_slaveSelectPin);
  adxl.Initialization(adxl_slaveSelectPin);
  Serial.println("adxl initialization set up finished...");
  if (BMI_init_status != 1) {  // 我发现BMI初始化放在这里最好，因为修改模式和初始化完全无关，我刚开始是放在BMI_ADXL_Action里面的，这导致第一次使用的时候很慢，要等待初始化完成，但是放在这里的话
                               // 在我配置手机蓝牙的时候，它自己就开始初始化了，我点START之后立马开始运行了，这样好得多
    while (1) {
      BMI_8kb_upload();
      if (BMI_init_status == 1) {
        break;
      }
    }
  }
  digitalWrite(13, HIGH);
  Serial.println("BMI initialization set up finished...");
}


void GPIO_Initiation() {
  //SPI 上拉电阻
  // pinMode(6, INPUT_PULLUP);
  // pinMode(7, INPUT_PULLUP);
  // pinMode(8, INPUT_PULLUP);
  // pinMode(14, INPUT_PULLUP);  //CS BMI
  // pinMode(15, INPUT_PULLUP);  //CS ADXL
  // pinMode(27, INPUT_PULLUP);
  //I2C 上拉电阻
  //pinMode(9, INPUT_PULLUP);
  //pinMode(10, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(10, OUTPUT);  //我发现不用吧i2c的时候必须要给低电平，不然i2c会导致sht40的shtvcc引脚有3V的电压
  digitalWrite(10, LOW);


  //DISABLE IC SWITCH
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  pinMode(24, OUTPUT);
  digitalWrite(24, LOW);
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);

  //DISABLE BMI INT
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);


  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);
  pinMode(28, OUTPUT);
  digitalWrite(28, LOW);
  pinMode(31, OUTPUT);  //IR OUT
  digitalWrite(31, LOW);
}

void BLESetup_Callback() {



  // create service

  blePeripheral.setLocalName("My NRF Device");
  blePeripheral.setAdvertisedServiceUuid(sensorConfigService.uuid());

  // add service and characteristic
  blePeripheral.addAttribute(sensorConfigService);
  blePeripheral.addAttribute(sensors_config_Characteristic);

  blePeripheral.addAttribute(Start_Service);
  blePeripheral.addAttribute(Start_Characteristic);
  // begin initialization


  blePeripheral.begin();


  Serial.println(F("BLE waits to be connected..."));


  delay(5);
}




void BMI_8kb_upload() {
  SPI.begin();

  SPI.setDataMode(SPI_MODE3);
  //SPI.setBitOrder(MSBFIRST);
  delay(50);
  // SPI 4MHz
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  //Dummy data
  SPI.transfer(0xFF);
  //Software Reset
  bmi2xx.writeRegister(CMD, 0xB6);
  delay(100);

  // Initialization Phase A.
  // Dummy
  bmi2xx.readRegister(0x00, 2);
  // BMI_chip_ID = bmi2xx.readRegister(CHIPID, 2);
  // Serial.print("BMI_chip_ID  = ");
  // Serial.println(BMI_chip_ID);

  BMI_power_mode = bmi2xx.readRegister(PWR_CONF, 2);
  // Serial.print("BMI_power_mode  = ");
  // Serial.println(BMI_power_mode);

  // Initialization Phase B.
  // disalbe POWER save mode
  // it was supposed to be 0x00
  bmi2xx.writeRegister(PWR_CONF, 0x0);
  //wait > 450us
  delay(1);

  //prepare config load INIT_CTRL = 0x00
  bmi2xx.writeRegister(INIT_CTRL, 0x00);

  //load config file and save as array
  config_size = sizeof(bmi2xx_config_file);

  if (config_size != CONFIG_SIZE) {
    Serial.println("Please put proper config file");
    while (1)
      ;
  }

  file_count = 0;

  byte* filepos = (byte*)bmi2xx_config_file;

  while (config_size > 0) {
    bmi2xx.Upload_file(config_size, file_count, filepos);

    filepos += 32;
    file_count++;
    config_size -= 32;
    delay(1);
  }

  //complete config load INIT_CTRL = 0x01
  bmi2xx.writeRegister(INIT_CTRL, 0x01);

  //Wait >= 140
  delay(150);

  // Initialization Phase C - Checking the correct initialization status
  BMI_init_status = bmi2xx.readRegister(INT_STATUS, 2);
  // Should be 1
  Serial.print("BMI_init_status  = ");
  Serial.println(BMI_init_status);
  //最原始代码有lidumode的时候，更改模式是放在这个最后位置的，我也放这里。 我发现放这里不行，以为运行中就无法改变了，以为这个BMISETUP只运行一次。
}

void config_all_sensors() {

  bmi_pwr_control = received_config_from_ble[0];

  adxl_pwr_control = received_config_from_ble[1];

  bmi_acc_config = received_config_from_ble[2];
  bmi_gyr_config = received_config_from_ble[2];
  motion_sensor_inteval_set(bmi_acc_config);  //  不同的采样率要对应不同的数据速率



  adxl_hz_config = received_config_from_ble[3];

  te_hu_control = received_config_from_ble[4];

  infrared_control = received_config_from_ble[5];

  bmi2xx.customized_mode_bmi(bmi_pwr_control, bmi_acc_config, bmi_gyr_config);
  adxl.customized_mode_adxl(adxl_pwr_control, adxl_hz_config);
  sht.customized_mode_sht40(te_hu_control);
}
void motion_sensor_inteval_set(uint8_t bmi_hz_configure) {
  if (bmi_hz_configure == 0x05) {
    motion_sensor_time_interval = 79;
  }
  if (bmi_hz_configure == 0x06) {
    motion_sensor_time_interval = 39;
  }
  if (bmi_hz_configure == 0x07) {
    motion_sensor_time_interval = 19;
  }
  if (bmi_hz_configure == 0x08) {
    motion_sensor_time_interval = 9;
  }
  if (bmi_hz_configure == 0x09) {
    motion_sensor_time_interval = 4;
  }
  if (bmi_hz_configure == 0x0a) {
    motion_sensor_time_interval = 2;  // should be 2.5, but egal, not important
  }
  interval_T4 = motion_sensor_time_interval;
  Serial.println("interval");
  Serial.println(motion_sensor_time_interval);
}

// void automatical_time_interval(int16_t x, int16_t y, int16_t z) {

void automatical_time_interval(int x) {
  int testnumer = x;
  //int testnumer = abs(x) + abs(y) + abs(z);
  Serial.print("testnumer:");
  Serial.print(testnumer);
  // Serial.print("\t");
  if (testnumer <= 100) {
    motion_sensor_time_interval = 99;
  }
  if (testnumer >= 101 && testnumer <= 200) {
    motion_sensor_time_interval = 79;
  }
  if (testnumer >= 201 && testnumer <= 400) {
    motion_sensor_time_interval = 59;
  }
  if (testnumer >= 401 && testnumer <= 600) {
    motion_sensor_time_interval = 39;
  }
  if (testnumer >= 601 && testnumer <= 800) {
    motion_sensor_time_interval = 19;
  }
  if (testnumer >= 801) {
    motion_sensor_time_interval = 9;
  }
  interval_T4 = (long)motion_sensor_time_interval;
}
