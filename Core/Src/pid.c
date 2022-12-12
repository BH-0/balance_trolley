#include "pid.h"

PID pid = {0};
extern uint16_t Tof_Value; //距离

void PID_Init()
{
    /*平衡PID环控制参数初始化*/
    pid.Sv = 0;	  //用户设定平衡位置值
    pid.Kp = 4.30f;  //2.50//MPU6050 1.50f; //平衡比例项系数 + 1.47
    RF2G4_Send_Data[5] = pid.Kp*10;
    pid.Kd = 0.45f;  //0.26//MPU60500.085f; //平衡微分项系数 + 0.075
    RF2G4_Send_Data[6] = pid.Kd*100;

    /*速度PID环控制参数初始化*/
    pid.Kp_speed = 300;	  //速度环比例项系数 + 95
    RF2G4_Send_Data[7] = pid.Kp_speed/10;
    pid.Ki_speed = 1.5f; //速度环积分项系数 (一般为Kp/200) 0.375
    pid.EK_speed = 0;	  //速度偏差
    pid.SEK_speed = 0;	  //历史偏差之和
    pid.Set_Speed = 0;	  //目标速度

    /*转向PID环控制参数初始化*/
    pid.Kp_turn = -1.6f; //转向环比例项系数 -  85
    RF2G4_Send_Data[8] = pid.Kp_turn*-100;
    pid.Kd_turn = -0.25f; //转向环微分项系数 - -0.16f
    RF2G4_Send_Data[9] = pid.Kd_turn*-100;
    pid.Angle_turn = 0;	 //目标转向角度


    pid.Flag_Left = 0;  //转向标志
    pid.Flag_Right = 0;
    pid.Turn_Amplitude = 2; //转速

    uint8_t brake = 1;//刹车标志
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
float myabs(float a)
{
    float temp;
    if(a<0)  temp=-a;
    else temp=a;
    return temp;
}

/*小车平衡环部分，微分+比例控制
微分变量为直接读取的加速度*/

int balance(float Angle)
{
    float Bias;
    int balance;
    Bias = (Angle - pid.Sv); //+0.3;
    balance = pid.Kp * Bias + gyroy * pid.Kd;
    return balance;
}

/*小车速度环部分， 积分+比例控制*/
int velocity(float encoder_left, float encoder_right)
{
    int velocity;
    if((RF2G4_Send_Data[13]&0x7F) == 2)    //跟随模式
    {
        if(Tof_Value>80 && Tof_Value <= 400)    //响应范围
        {
            pid.Set_Speed = (float)(Tof_Value-240)/1250/-1;
        }
    }
    else if ((RF2G4_Send_Data[13]&0x7F) == 1)  //避障模式
    {
        if(Tof_Value>80 && Tof_Value <= 400)    //响应范围
        {
            if(pid.Set_Speed < 0)   //前进
            {
                pid.Set_Speed *= (float)(Tof_Value-240)/160 ;
                if(Tof_Value<300)
                    pid.SEK_speed = 0;
            }
        }
    }
    pid.EK_speed *= 0.7f;
    pid.EK_speed = pid.EK_speed + ((encoder_left + encoder_right) / 10 - 0) * 0.3f; //===一阶低通滤波器
    pid.SEK_speed = pid.SEK_speed + pid.EK_speed + pid.Set_Speed * 4;			   //===积分出位移

    if (pid.SEK_speed > 100)
        pid.SEK_speed = 100;
    //===积分限幅
    if (pid.SEK_speed < -100)
        pid.SEK_speed = -100;
    //===积分限幅
//    if(pid.brake == 0)//刹车标志
//        velocity = pid.Kp_speed * pid.EK_speed + (pid.Ki_speed * pid.SEK_speed);
//    else
    velocity = pid.Kp_speed * pid.EK_speed + (pid.Ki_speed* pid.SEK_speed);
    if (pitch < -40 || pitch > 40)
        pid.SEK_speed = 0;

        //限幅
        if (velocity > 30)
            velocity = 30;
        if (velocity < -30)
            velocity = -30;

    return velocity;
}

/*小车转向环部分，比例控制*/
int turn(float gyro) //转向控制
{
    int Turn;
    float Bias;

    Bias = (gyro - pid.Angle_turn);
    if(Bias >= 180) {
        Bias = -(360 - Bias);
        //RTT_printf(1,":%f\r\n",Bias);
    }
    else if(Bias <= -180)
    {
        Bias = 360 + Bias;
    }
    Turn = (int)(Bias * pid.Kp_turn) + pid.Kd_turn * gyroz;
    /*进行转向速度的单独限幅*/
    if (Turn >= 30)
        Turn = 30;
    else if (Turn <= -30)
        Turn = -30;

    return Turn;
}

/**************************************************************************
函数功能：转向控制  修改转向速度，请修改Turn_Amplitude即可
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn_x(float encoder_left,float encoder_right,float gyro)
{
    static float Turn_Target, Turn, Encoder_temp, Turn_Convert=0.9f, Turn_Count, Kd_turn_buf = 0;
//    float Turn_Amplitude = 2;  //速度控制
//    float Kp=42,Kd=0;
    //=============遥控左右旋转部分=======================//
    if(1==pid.Flag_Left||1==pid.Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
    {
        if(++Turn_Count==1)
            Encoder_temp=myabs(encoder_left+encoder_right);
        Turn_Convert=50/Encoder_temp;
        if(Turn_Convert<0.6f)Turn_Convert=0.6f;
        if(Turn_Convert>3)Turn_Convert=3;
    }
    else
    {
        Turn_Convert    = 0.9f;
        Turn_Count      = 0;
        Encoder_temp    = 0;
    }
    if(1 == pid.Flag_Left)	           Turn_Target += Turn_Convert;
    else if(1 == pid.Flag_Right)	     Turn_Target -= Turn_Convert;
    else Turn_Target = 0;
    if(Turn_Target >  pid.Turn_Amplitude)  Turn_Target=pid.Turn_Amplitude;    //===转向速度限幅
    if(Turn_Target < -pid.Turn_Amplitude) Turn_Target=-pid.Turn_Amplitude;

    if(pid.Flag_Left !=0 || pid.Flag_Right != 0)
    {
        Kd_turn_buf = 0;    //左右旋转时取消阻尼
    }
    else
    {
        Kd_turn_buf = pid.Kd_turn;
    }
    Turn = -Turn_Target * pid.Kp_turn + Kd_turn_buf * gyroz;//===结合Z轴陀螺仪进行PD控制
    /*进行转向速度的单独限幅*/
    if (Turn >= 30)
        Turn = 30;
    else if (Turn <= -30)
        Turn = -30;
    return Turn;
}


//函数功能：限幅函数
void Xianfu_Pwm(int *Moto_Left, int *Moto_Right)
{
    static int Moto_Left_last = 0;
    static int Moto_Right_last = 0;
    if(RF2G4_Send_Data[0] == 1)
    {
        int Amplitude = 100; //最大100 最小-100
        if (*Moto_Left < -Amplitude)
            *Moto_Left = -Amplitude;
        if (*Moto_Left > Amplitude)
            *Moto_Left = Amplitude;
        if (*Moto_Right < -Amplitude)
            *Moto_Right = -Amplitude;
        if (*Moto_Right > Amplitude)
            *Moto_Right = Amplitude;

        //异常转速变化
        if((Moto_Left_last-*Moto_Left)>50 || (Moto_Left_last-*Moto_Left)<-50)
            *Moto_Left = Moto_Left_last;
        else
            Moto_Left_last = *Moto_Left;

        if((Moto_Right_last-*Moto_Right)>50 || (Moto_Right_last-*Moto_Right)<-50)
            *Moto_Right = Moto_Right_last;
        else
            Moto_Right_last = *Moto_Right;
    }
    else
        Moto_Left_last = Moto_Right_last = 0;
}
