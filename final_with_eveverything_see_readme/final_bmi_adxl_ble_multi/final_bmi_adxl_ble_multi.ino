#include <TaskScheduler.h>
#include "lp_BLESerial.h"
#include "mysht40.h"
#include <SPI.h>
#include <Wire.h>
#include "bmi2xx.h"
#include "myadxl362.h"
#include "Arduino.h"

#define adxl_slaveSelectPin 15
MYADXL362 adxl;
MYSHT40 sht;
int16_t adxl_XData = 0, adxl_YData = 0, adxl_ZData = 0, adxl_Temperature = 0;

int config_runtime_flag = 0;
uint8_t bmi_pwr_control;  // 0

uint8_t adxl_pwr_control;  // 1

uint8_t bmi_acc_config;  // 2
uint8_t bmi_gyr_config;

uint8_t adxl_hz_config;  // 3


uint8_t te_hu_control;     // 4
uint8_t infrared_control;  // 5

int config_from_ble_counter = 0;

uint8_t received_config_from_ble[6];



lp_BLESerial blePeripheral;




BLEService sensorConfigService = BLEService("19B10000-E8F2-537E-4F6C-D104768A1214");

// create switch characteristic
BLECharCharacteristic sensors_config_Characteristic = BLECharCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


BLEService Start_Service = BLEService("f2783635-3a07-4a51-9696-459ae784363e");  // 我发现只要一个lp_BLESerial blePeripheral就够了。我两个ttribute，到时候收到消息可以分辨是哪里收到了消息


BLECharCharacteristic Start_Characteristic = BLECharCharacteristic("f2783636-3a07-4a51-9696-459ae784363e", BLERead | BLEWrite);


const char* sd_filename = "";
unsigned int BMI_chip_ID;
int file_count;
int config_size;
int BMI_init_status;
int BMI_power_mode;
BMI2xx bmi2xx;

Scheduler runner;







void NRFSetup_Callback();
void BLESetup_Callback();
void BMI_ADXL_Action_Callback();  // 从BLEACTION收到消息后，add这个task给scheduler，它不带任何接收BLE，只负责读取数据
                                  // 它两分开不好，因为不知道SPI会不会冲突，所以就在一个人物里面，BMI先用，adxl再用，保证不会冲突，或者bmi读两次，adxl读一次的情况
void BLE_Action_Callback();
void Sht40_Action_Callback();

//setup（）初始化GPIO和初始化,包含了BMISETUP和adxlsetup， addtask在后面原生arduino函数里面，adxl启动过程太复杂，我直接封装了

Task NRFSetup(TASK_IMMEDIATE, 1, &NRFSetup_Callback);
Task BLESetup(TASK_IMMEDIATE, 1, &BLESetup_Callback);



Task BMI_ADXL_Action(50, TASK_FOREVER, &BMI_ADXL_Action_Callback);  // 没有BMISETUP是因为，要让BMI_init_status=1的次数是不确定了，有时候一次成功，有时候4 5 次，但是要分配次数，我该分配几次呢？所以没有这个
Task BLE_Action(10, TASK_FOREVER, &BLE_Action_Callback);

Task Sht40_Action(200, TASK_FOREVER, &Sht40_Action_Callback);

// BMISetup还是不是任务类型比较好，因为后续逻辑不好写，要么是每次初始化，要么是ini=1就一直删除任务，浪费资源
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
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);  //CS BMI
  pinMode(15, INPUT_PULLUP);  //CS ADXL

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
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);


  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);
  pinMode(27, OUTPUT);
  digitalWrite(27, LOW);
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
  runner.addTask(BLE_Action);
  BLE_Action.enable();
}




void BMI_8kb_upload() {
  SPI.begin();

  SPI.setDataMode(SPI_MODE3);
  //SPI.setBitOrder(MSBFIRST);
  delay(50);
  // SPI 4MHz
  SPI.setClockDivider(SPI_CLOCK_DIV4);

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



void BLE_Action_Callback() {

  BLECentral central = blePeripheral.central();
  if (central) {
    if (blePeripheral.available()) {
      digitalWrite(13, LOW);
      String receivedMessage = blePeripheral.readString();
      Serial.print("Received message: ");
      Serial.println(receivedMessage);

      if (receivedMessage[0] == 'F') {  // 这里居然只能单引号

        Serial.print("File name is: ");
        Serial.println(receivedMessage.substring(2));
        sd_filename = receivedMessage.substring(2).c_str();
        Serial.print("varialble is: ");
        Serial.println(sd_filename);
      }
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




        runner.addTask(BMI_ADXL_Action);
        BMI_ADXL_Action.enable();

        // Serial.println("sht.on_or_off_status");
        // Serial.println(sht.on_or_off_status);
        // if (sht.on_or_off_status == 1) {
        //   runner.addTask(Sht40_Action);        // if等以后添加。因为这里添加后会失去一个功能，读取数据的时候不能配置device。
        //   Sht40_Action.enable();
        // }
              
          runner.addTask(Sht40_Action);
          Sht40_Action.enable();
        

      }
      if (Start_Characteristic.value() == 0x00) {

        BMI_ADXL_Action.disable();
        runner.deleteTask(BMI_ADXL_Action);


        Sht40_Action.disable();
        runner.deleteTask(Sht40_Action);

        config_runtime_flag = 0;


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

    delay(1);
  }
}

void config_all_sensors() {

  bmi_pwr_control = received_config_from_ble[0];

  adxl_pwr_control = received_config_from_ble[1];

  bmi_acc_config = received_config_from_ble[2];
  bmi_gyr_config = received_config_from_ble[2];

  adxl_hz_config = received_config_from_ble[3];

  te_hu_control = received_config_from_ble[4];

  infrared_control = received_config_from_ble[5];

  bmi2xx.customized_mode_bmi(bmi_pwr_control, bmi_acc_config, bmi_gyr_config);
  adxl.customized_mode_adxl(adxl_pwr_control, adxl_hz_config);
  sht.customized_mode_sht40(te_hu_control);
}



void BMI_ADXL_Action_Callback() {

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


  delay(1);




  uint16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
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

  delay(1);

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



  unsigned long endTime = micros();  // 记录结束时间

  unsigned long elapsedTime = endTime - startTime;  // 计算运行时间

  Serial.print("Time elapsed: ");
  Serial.print(elapsedTime);
  Serial.print("     ");

  acc_x |= ((uint16_t)msb_acc_x << 8) | (uint16_t)lsb_acc_x;
  Serial.print("acc_x = ");
  if ((acc_x >> 15) & 0x01 == 1) {
    Serial.print("-");
    acc_x = (~acc_x) + 1;
  }
  Serial.print(acc_x);
  Serial.print("\t");

  acc_y |= ((uint16_t)msb_acc_y << 8) | (uint16_t)lsb_acc_y;
  Serial.print("acc_y = ");
  if ((acc_y >> 15) & 0x01 == 1) {
    Serial.print("-");
    acc_y = (~acc_y) + 1;
  }
  Serial.print(acc_y);
  Serial.print("\t");

  acc_z |= ((uint16_t)msb_acc_z << 8) | (uint16_t)lsb_acc_z;
  Serial.print("acc_z = ");
  if ((acc_z >> 15) & 0x01 == 1) {
    Serial.print("-");
    acc_z = (~acc_z) + 1;
  }
  Serial.print(acc_z);
  Serial.print("\t");



  gyr_x |= ((uint16_t)msb_gyr_x << 8) | (uint16_t)lsb_gyr_x;
  Serial.print("gyr_x = ");
  if ((gyr_x >> 15) & 0x01 == 1) {
    Serial.print("-");
    gyr_x = (~gyr_x) + 1;
  }
  Serial.print(gyr_x);
  Serial.print("\t");

  gyr_y |= ((uint16_t)msb_gyr_y << 8) | (uint16_t)lsb_gyr_y;
  Serial.print("gyr_y = ");
  if ((gyr_y >> 15) & 0x01 == 1) {
    Serial.print("-");
    gyr_y = (~gyr_y) + 1;
  }
  Serial.print(gyr_y);
  Serial.print("\t");

  gyr_z |= ((uint16_t)msb_gyr_z << 8) | (uint16_t)lsb_gyr_z;
  Serial.print("gyr_z = ");
  if ((gyr_z >> 15) & 0x01 == 1) {
    Serial.print("-");
    gyr_z = (~gyr_z) + 1;
  }
  Serial.print(gyr_z);

  Serial.print("\t");
  //Serial.println("\t");


  // Serial.print("BMI ST:");
  // BMI_init_status = bmi2xx.readRegister(INT_STATUS, 2);
  // Serial.println(BMI_init_status);




  Serial.print("XVALUE=");
  Serial.print(adxl_XData);
  Serial.print("\tYVALUE=");
  Serial.print(adxl_YData);
  Serial.print("\tZVALUE=");
  Serial.print(adxl_ZData);
  Serial.print("\tTEMPERATURE=");
  Serial.println(adxl_Temperature);
}


void setup() {
  Serial.begin(115200);
  runner.init();
  runner.addTask(NRFSetup);
  NRFSetup.enable();
  runner.addTask(BLESetup);
  BLESetup.enable();

  delay(50);
  Serial.println("setup finished");
}

void loop() {
  runner.execute();  // put your main code here, to run repeatedly:
}
