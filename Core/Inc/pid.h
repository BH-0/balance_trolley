#ifndef __PID_H
#define __PID_H
#include "sys.h"
#include "usart.h"

//PID结构体
typedef struct
{
    float Sv; //用户设定平衡位置值（目标值）
    float Pv;   //（当前值）
    /*平衡环参数设定 */
    float Kd;	   //平衡微分项系数
    float Kp; //平衡比例项系数

    /*速度环参数设定 */
    float Set_Speed; //目标左速度
    float Ki_speed;		  //速度环积分项系数
    float Kp_speed;		  //速度环比例项系数
    float EK_speed;		  //速度偏差
    float SEK_speed;		  //历史偏差之和

    /*转向环参数设定 */
    float Kp_turn;	  //转向环比例项系数
    float Kd_turn;	  //转向环微分项系数
    float Angle_turn; //目标转向角度


    uint8_t Flag_Left;  //左转
    uint8_t Flag_Right; //右转
    float Turn_Amplitude;   //速度
} PID;


extern PID pid;
void PID_Init(void);							   //PID数据初始化
int balance(float Angle);						   //平衡环PID
int velocity(float encoder_left, float encoder_right); //速度环
int turn(float gyro); //转向控制
int turn_x(float encoder_left,float encoder_right,float gyro);//转向控制
//函数功能：限幅函数
void Xianfu_Pwm(int *Moto_Left, int *Moto_Right);

#endif	/* __PID_H */
