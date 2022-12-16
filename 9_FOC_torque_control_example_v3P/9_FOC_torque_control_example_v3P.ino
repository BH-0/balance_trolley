  /**
Deng's FOC 双电机 FOC电流控制例程 测试库：SimpleFOC 2.2.1 测试硬件：灯哥开源FOC V3.0
在串口窗口中输入：A+电流控制M0，B+电流控制M1，电流单位为A
setup()中可取消注释设置电压限制与电流限制
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(14) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 16.8V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是 GB6010 ，使用自己的电机需要修改PID参数，才能实现更好效果
*/

#include <SimpleFOC.h>
#include <WiFi.h>
# include <stdlib.h>
# include <string.h>

const char* id="BH_TX";              //常量定义
const char* psw="qoqoqo1003155077";

//标志位
uint8_t StartStop_bit = 0;  //小车使能

//定时器
hw_timer_t *timer = NULL;
uint32_t flag = 0;
static void IRAM_ATTR Timer0_CallBack(void);
float sensor1AngleBuf = 0.0f;
float sensor2AngleBuf = 0.0f;

//电机实例
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);

//编码器实例
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);


// 在线电流检测实例
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01, 50.0, 35, 34);

// commander通信实例
Commander command = Commander(Serial);
void doMotor1(char* cmd){ command.motor(&motor1, cmd); }
void doMotor2(char* cmd){ command.motor(&motor2, cmd); }

IPAddress connect(){             //连接到指定wifi
  WiFi.begin(id,psw);
   while (WiFi.status()!= WL_CONNECTED) {
    delay(1000);
    Serial.println("正在尝试连接该wifi...");
  }
  Serial.println("连接成功！");
  Serial.println("IP address set: "); 
  IPAddress localip = WiFi.localIP();
  Serial.println(localip); //print LAN IP
  return localip;
}

void setup() {
  // 编码器设置
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  

  // 驱动器设置
  driver1.voltage_power_supply = 12.6;
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 12.6;
  driver2.init();
  motor2.linkDriver(&driver2);

  // 电流限制
//    motor1.current_limit = 2;
//    motor2.current_limit = 2;
    // 电压限制
//    motor1.voltage_limit = 12;
//    motor2.voltage_limit = 12;


  // 电流检测
  current_sense1.init();
  current_sense1.gain_b *= -1;
  current_sense1.gain_a *= -1;
 // current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);
  // current sense init and linking
  current_sense2.init();
  current_sense2.gain_b *= -1;
  current_sense2.gain_a *= -1;
//  current_sense2.skip_align = true;
  motor2.linkCurrentSense(&current_sense2);

  // 控制环
  // 其他模式 TorqueControlType::voltage TorqueControlType::dc_current 
  motor1.torque_controller = TorqueControlType::foc_current; 
  motor1.controller = MotionControlType::torque;
  motor2.torque_controller = TorqueControlType::foc_current; 
  motor2.controller = MotionControlType::torque;

  // FOC电流控制PID参数
   motor1.PID_current_q.P = 5;
   motor1.PID_current_q.I= 1000;
   motor1.PID_current_d.P= 5;
   motor1.PID_current_d.I = 1000;
   motor1.LPF_current_q.Tf = 0.002; // 1ms default
   motor1.LPF_current_d.Tf = 0.002; // 1ms default

   motor2.PID_current_q.P = 5;
   motor2.PID_current_q.I= 1000;
   motor2.PID_current_d.P= 5;
   motor2.PID_current_d.I = 1000;
   motor2.LPF_current_q.Tf = 0.002; // 1ms default
   motor2.LPF_current_d.Tf = 0.002; // 1ms default

    // 速度环PID参数
    motor1.PID_velocity.P = 0.1;
    motor1.PID_velocity.I = 0.12;
    motor1.PID_velocity.D = 0;

    motor2.PID_velocity.P = 0.1;
    motor2.PID_velocity.I = 0.12;
    motor2.PID_velocity.D = 0;
    // default voltage_power_supply 
  
    // 速度限制
    motor1.velocity_limit = 15;
    motor2.velocity_limit = 20;


  // monitor接口设置

  Serial.begin(115200);
  Serial2.begin(921600);

  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // monitor相关设置
  motor1.monitor_downsample = 0;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL |  _MON_CURR_Q;
  motor2.monitor_downsample = 0;
  motor2.monitor_variables = _MON_TARGET | _MON_VEL |  _MON_CURR_Q;
  


  //电机初始化
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC(); 
  
  motor2.init();
  // align encoder and start FOC
  motor2.initFOC(); 

  // 初始目标值
  motor1.target = 0.05;
  motor2.target = -0.05;

  // 映射电机到commander
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");

  //定时器初始化
  timer = timerBegin(0, 80, true);//设置定时器0,80分频，向上计数
  timerAttachInterrupt(timer, Timer0_CallBack, true);
  timerAlarmWrite(timer, 100000, true);//设置定时器，定时0.1s 单位us
  timerAlarmEnable(timer);  //开始计数

  //创建wifi连接
  //connect();
  
  //初始化完成
  Serial.println(F("Double motor sketch ready."));
  Serial2.println(F("OK."));  //不知道为啥第一次发送是乱码
  Serial2.println(F("OK."));
  Serial2.println(F("OK."));
  Serial2.println(F("L0R0"));
  Serial2.println(F("L0R0"));
  Serial2.println(F("L0R0"));
  _delay(1000);
}


void loop() {
  static uint8_t sum = 0;
  if(StartStop_bit == 1)  //电机使能
  {
    // iterative setting FOC phase voltage
    motor1.loopFOC();
    motor2.loopFOC();

    // iterative function setting the outter loop target
    motor1.move();
    motor2.move();

    // user communication
    command.run();
    motor1.monitor();
    motor2.monitor();
  }

  //读取串口2数据
  //命令格式 "A10 B-20\r\n"
  while(Serial2.available())
  {
    char inChar = (char)Serial2.read();
    char inBuff[16] = {0};
    int targetNum = 0;
    float targetBuf = 0.0;
    char i = 0;

    if(StartStop_bit == 1)  //电机使能
    {
      if(inChar == 'A')
      {
        i = 0;
        memset(inBuff, 0, 16);
        Serial2.print(F("A:"));
        while(Serial2.available())
        {
          inChar = Serial2.read();
          if(((inChar >= '0' && inChar <= '9') || inChar == '-') && i<4)
          {  
            //读取数值
            //Serial2.print(inChar);
            inBuff[i] = inChar;
            i++;
          }
          else
            break;
        }
        if(inBuff[0] == '-')  //正负判断
          targetNum = atoi(inBuff+1)*-1;
        else
          targetNum = atoi(inBuff);
        if(targetNum >= -100 && targetNum <= 100)
        {
          targetBuf = (float)targetNum*2/100;  //电流计算
          char buf[8] = {0};
          sprintf(buf, "%.2f", targetBuf);
          Serial2.println(buf);  //打印浮点数
          motor1.target = targetBuf;
        }
        //Serial2.print(F("\r\n"));
      }

      if(inChar == 'B')
      {
        i = 0;
        memset(inBuff, 0, 16);
        Serial2.print(F("B:"));
        while(Serial2.available())
        {
          inChar = Serial2.read();
          if(((inChar >= '0' && inChar <= '9') || inChar == '-') && i<4)
          {  
            //读取数值
            //Serial2.print(inChar);
            inBuff[i] = inChar;
            i++;
          }
          else
            break;
        }
        if(inBuff[0] == '-')  //正负判断
          targetNum = atoi(inBuff+1)*-1;
        else
          targetNum = atoi(inBuff);
        if(targetNum >= -100 && targetNum <= 100)
        {
          targetBuf = (float)targetNum*2/100;  //电流计算
          char buf[8] = {0};
          sprintf(buf, "%.2f", targetBuf);
          Serial2.println(buf);  //打印浮点数
          motor2.target = targetBuf * -1;
        }
        //Serial2.print(F("\r\n"));
      }
    }

    if(inChar == '#')//指令接收
    {
      i = 0;
      memset(inBuff, 0, 16);
      while(Serial2.available())
      {
        inChar = Serial2.read();
        if((inChar != '#') && i<15)
        {  
          //读取数值
          //Serial2.print(inChar);
          inBuff[i] = inChar;
          i++;
        }
        else
          break;
      }
      //指令处理
      if(strcmp(inBuff,"START") == 0)
      {
        StartStop_bit = 1;
        //WiFi.mode(WIFI_OFF);          //断开当前wifi网络的连接
      }
      else if(strcmp(inBuff,"STOP") == 0)
      {
        StartStop_bit = 0;
        Serial2.println(F("L0R0"));
        Serial2.println(F("L0R0"));
        Serial2.println(F("L0R0"));
        //创建wifi连接
        //connect();
      }
    }

    if(inChar == 'C')
    {
      StartStop_bit = 0;
      I2Ctwo.end();
      I2Cone.end();
      motor1.disable(); //失能电机
      motor2.disable();
      if(sum>=2)
      {
        while(Serial2.available())//清空多余数据再进入升级
        {
          inChar = Serial2.read();
        }
        update(); //进入升级阶段，升级完成后直接重启
      }
      else
        sum++;
    }
    else
      sum = 0;
  }
  
  if(flag == 1 && StartStop_bit == 1) //每100ms回传一次数据
  {
      flag = 0;
      float buf = 0.0f;

      buf = sensor1.getAngle();
      Serial2.print('L');
      Serial2.print(buf - sensor1AngleBuf);  //打印电机角度
      sensor1AngleBuf = buf;

      buf = sensor2.getAngle();
      Serial2.print('R');
      Serial2.println(buf - sensor2AngleBuf);  //打印电机角度
      sensor2AngleBuf = buf;
      
  }
}

//定时器回调函数
static void IRAM_ATTR Timer0_CallBack(void)
{
  flag = 1;
}

static void update(void)
{
  WiFiClient client;
  uint8_t inChar = 0;
  uint8_t cmdBit = 0;

  Serial2.println(F("#UP1"));  //返回升级标志
  //创建wifi连接
  const char* ipBufs = connect().toString().c_str();
  uint8_t ipBuf1,ipBuf2,ipBuf3;
  char ipBuf[16] = {0};
  sscanf(ipBufs,"%u.%u.%u.",&ipBuf1,&ipBuf2,&ipBuf3);
  sprintf(ipBuf,"%u.%u.%u.1",ipBuf1,ipBuf2,ipBuf3);//默认192.168.xxx.1为TCP客户端地址
  Serial.println("IP server:");
  Serial.println(ipBuf);
  Serial2.println(F("#UP2"));  //返回升级标志
  while (client.connect(ipBuf, 8888) == 0)  //TCP连接
  {
    Serial.println("连接TCP服务端中...");
    delay(1000);
  }
  Serial2.println(F("#UP3"));  //返回升级标志
  Serial.println("升级中...");
  while(Serial2.available())//清空多余数据
    inChar = Serial2.read();
  inChar = 0;
  //client.write_P("C",1);
  while(1)  //建立透传
  {
    char Serial2_but = 0;
    char inBuff[16] = {0};
    uint8_t sum = 0;

    if (Serial2.available()) //串口发往TCP服务器
    {
      Serial2_but = Serial2.read();
      client.write_P(&Serial2_but,1);
      //Serial.print(String(Serial2_but));

      if(Serial2_but == '#')  //收到指令
      {
        sum = 0;
        while(Serial2.available())
        {
          Serial2_but = Serial2.read();
          client.write_P(&Serial2_but,1);
          //Serial.print(String(Serial2_but));
          
          inBuff[sum] = Serial2_but;
          if(Serial2_but == '#')
          {
            inBuff[sum] = '\0';
            if(strcmp(inBuff,"RST") == 0) //复位
            {
              Serial.println("复位");
              Serial.println("复位");
              abort();
            }
            sum = 0;
            break;
          }
          
          if(sum < 15)
            sum++;
          else
          {
            sum = 0;
            break;
          }

        }
        sum = 0;
      }  
    }
    else if (client.available()) //TCP服务器发往串口
    {
      Serial2.write(client.read());
    }

  }

}
