
#ifndef ADAFRUIT_NEOPIXEL_H
#define ADAFRUIT_NEOPIXEL_H

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "cmsis_os.h"

typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
}RGBColor_TypeDef;

extern const uint16_t index_wave[];
extern uint16_t index_wave_num;

extern const RGBColor_TypeDef RED ;
extern const RGBColor_TypeDef GREEN;
extern const RGBColor_TypeDef BLUE;
extern const RGBColor_TypeDef SKY;
extern const RGBColor_TypeDef MAGENTA ;
extern const RGBColor_TypeDef YELLOW ;
extern const RGBColor_TypeDef ORANGE;
extern const RGBColor_TypeDef BLACK;
extern const RGBColor_TypeDef WHITE;
extern const RGBColor_TypeDef PURPLE;
extern const RGBColor_TypeDef PINK;

extern float RGB_cmd_bit;   //亮度控制
extern uint8_t RGB_mode_bit;
//----------------------------------------------------------------
//模式列表
//0x80：此前缀为普通模式（表示小车可根据实际情况改变RGB模式）
//0x01:远光
//0x02:警灯
//----------------------------------------------------------------

void Send_8bits(uint8_t dat);
void Send_2811_24bits(uint8_t GData,uint8_t RData,uint8_t BData);
void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b);
void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void SetPixelColor(uint16_t n, uint32_t c);
void PixelUpdate(void);
uint32_t Wheel(uint8_t WheelPos);
uint32_t Color(uint8_t r, uint8_t g, uint8_t b);//合并颜色
void rainbow(float Brightness, uint8_t wait);
void rainbowCycle(uint8_t wait);
void theaterChase(uint32_t c, uint8_t wait);
void theaterChaseRainbow(uint8_t wait) ;
void colorWipe(uint32_t c, uint8_t wait);

void WS2812B_Init(void);
void WS2812B_Test(void);

void double_rotating(RGBColor_TypeDef color1, RGBColor_TypeDef color2, uint8_t wait);
void progress_bar(uint8_t level, RGBColor_TypeDef color1, RGBColor_TypeDef color2,uint8_t wait);
#endif // ADAFRUIT_NEOPIXEL_H
