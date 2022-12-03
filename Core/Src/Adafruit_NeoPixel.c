#include "Adafruit_NeoPixel.h"

#define PIXEL_MAX 8

float RGB_cmd_bit = 0.05f;    //亮度控制
uint8_t RGB_mode_bit = 0;   //RGB模式


// Some Static Colors
const RGBColor_TypeDef RED      = {255,0,0};
const RGBColor_TypeDef GREEN    = {0,255,0};
const RGBColor_TypeDef BLUE     = {0,0,255};
const RGBColor_TypeDef SKY      = {0,255,255};
const RGBColor_TypeDef MAGENTA  = {255,0,255};
const RGBColor_TypeDef YELLOW   = {255,255,0};
const RGBColor_TypeDef ORANGE   = {127,106,0};
const RGBColor_TypeDef BLACK    = {0,0,0};
const RGBColor_TypeDef WHITE    = {255,255,255};
const RGBColor_TypeDef PURPLE   = {65,105,225};


void Send_8bits(uint8_t dat) 
{
  uint8_t i=0;
  static uint8_t CodeOne=0x3C;//7c,3e
  static uint8_t CodeZero=0x30;//70,38
  
  for (i=0;i<8;i++)
  {
    if((dat & 0x80)==0x80)
    {
      HAL_SPI_Transmit_DMA(&hspi2, &CodeOne, 1);
    }
    else
    {
      HAL_SPI_Transmit_DMA(&hspi2, &CodeZero, 1); 
    }
    dat=dat<<1;
  }
}
//G--R--B
//MSB first	
void Send_2811_24bits(uint8_t RData,uint8_t GData,uint8_t BData)
{
  Send_8bits(GData*RGB_cmd_bit);
  Send_8bits(RData*RGB_cmd_bit);
  Send_8bits(BData*RGB_cmd_bit);
} 

//像素刷新
void PixelUpdate(void)//should >24us
{
  uint8_t rst[24]={0};
  HAL_SPI_Transmit_DMA(&hspi2, rst, 24);
}

void WS2812B_Init(void)//should >50us
{
  uint8_t ResCode[50]={0};
  HAL_SPI_Transmit_DMA(&hspi2, ResCode, 50);
  setAllPixelColor(0, 0, 0);
  osDelay(50);
  setAllPixelColor(0, 0, 0);
  osDelay(50);
}


uint8_t rBuffer[PIXEL_MAX]={0};
uint8_t gBuffer[PIXEL_MAX]={0};
uint8_t bBuffer[PIXEL_MAX]={0};
void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b)
{ 
  uint8_t i=0;
  for(i=0;i<PIXEL_MAX;i++)
  {
    rBuffer[i]=0;
    gBuffer[i]=0;
    bBuffer[i]=0;
  }
  for(i=0;i<PIXEL_MAX;i++)
  {
    rBuffer[i]=r;
    gBuffer[i]=g;
    bBuffer[i]=b;
  }

  for(i=0;i<PIXEL_MAX;i++)
  {							  
    Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
  }
	PixelUpdate();
}
//设置某像素点的颜色
void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{	 
  uint8_t i=0;

  for(i=0;i<PIXEL_MAX;i++)
  {
    rBuffer[i]=0;
    gBuffer[i]=0;
    bBuffer[i]=0;
  }
  rBuffer[n]=r;
  gBuffer[n]=g;
  bBuffer[n]=b;
  for(i=0;i<PIXEL_MAX;i++)
  {							  
    Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
  }
  PixelUpdate();
}
void SetPixelColor(uint16_t n, uint32_t c)
{	 
  uint8_t i=0;
	  
  rBuffer[n]=(uint8_t)(c>>16);
  gBuffer[n]=(uint8_t)(c>>8);
  bBuffer[n]=(uint8_t)c;

  for(i=0;i<PIXEL_MAX;i++)
  {							  
    Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
  }
   PixelUpdate();
}
//合并颜色
uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}
//颜色转换
uint32_t Wheel(uint8_t WheelPos)
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) 
  {
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
//彩虹
void rainbow(uint8_t wait)
{
  uint16_t i, j;
  
  for(j=0; j<256; j++) 
  {
    for(i=0; i<PIXEL_MAX; i++)
    {
      SetPixelColor(i, Wheel((i+j) & 255));
    }
    PixelUpdate();
    osDelay(wait);
  }
}
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) 
{
  uint16_t i = 0;
  static uint16_t j = 0;
  //单次圈数
  float transition = 1;  // 5 cycles of all colors on wheel

//  for(j=0; j<256*transition; j++)
//  {
    for(i=0; i< PIXEL_MAX; i++) 
    {
      SetPixelColor(i, Wheel(((i * 256 / PIXEL_MAX) + j) & 255));
    }
    PixelUpdate();

    if(j+1 >= 256*transition)
        j = 0;
    else
        j++;
    osDelay(wait);
//  }
}
//Theatre-style crawling lights.o???????
void theaterChase(uint32_t c, uint8_t wait) 
{
  for (int j=0; j<10; j++) 
  {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) 
    {
      for (uint16_t i=0; i < PIXEL_MAX; i=i+1)//turn every one pixel on
      {
        SetPixelColor(i+q, c);    
      }
      PixelUpdate();
      osDelay(wait);
      
      for (uint16_t i=0; i < PIXEL_MAX; i=i+1) //turn every one pixel off
      {
        SetPixelColor(i+q, 0);        
      }
      PixelUpdate();
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) 
{
  for (int j=0; j < 256; j++) 
  {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++)
    {
      for (uint16_t i=0; i < PIXEL_MAX; i=i+1) //turn every one pixel on
      {
        SetPixelColor(i+q, Wheel( (i+j) % 255));    
      }
      PixelUpdate();
      
      osDelay(wait);
      
      for (uint16_t i=0; i < PIXEL_MAX; i=i+1)//turn every one pixel off
      {
        SetPixelColor(i+q, 0);        
      }
      PixelUpdate();
    }
  }
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) 
{
  uint16_t i=0;
  for( i=0; i<PIXEL_MAX; i++) 
  {
    SetPixelColor(i, c);
    PixelUpdate();
    osDelay(wait);
  }
}



void WS2812B_Test(void)
{
  setAllPixelColor(255, 0, 0);
  osDelay(500);
  setAllPixelColor(0, 255, 0);
  osDelay(500);
  setAllPixelColor(0, 0, 255);
  osDelay(500);
  
  setAllPixelColor(0, 0, 0);
  osDelay(500);
  
  setPixelColor(0, 0, 255, 0);
  osDelay(500);
  setPixelColor(2, 0, 0, 255);
  osDelay(500); 
  setPixelColor(4, 255, 0, 0);
  osDelay(500);
  setPixelColor(6, 125, 125, 125);
  osDelay(500);    
  setPixelColor(5, 0, 255, 0);
  osDelay(500);
  setPixelColor(3, 0, 0, 255);
  osDelay(500); 
  setPixelColor(1, 255, 0, 0);
  osDelay(500);
  setAllPixelColor(0, 0, 0);
  osDelay(50);
//  while(1)
//  {
//    // Some example procedures showing how to display to the pixels:
//    colorWipe(Color(255, 0, 0), 500); // Red
//    colorWipe(Color(0, 255, 0), 500); // Green
//    colorWipe(Color(0, 0, 255), 500); // Blue
//    // Send a theater pixel chase in...
//    theaterChase(Color(127, 127, 127), 50); // White
//    theaterChase(Color(127, 0, 0), 50); // Red
//    theaterChase(Color(0, 127, 0), 50); // Green   
//    theaterChase(Color(0, 0, 127), 50); // Blue   
//    rainbow(20);//2??o?
//    rainbowCycle(20);//?-???
//    theaterChaseRainbow(100);//o???????
//    //test code over
//  }
}

//双旋转
void double_rotating(RGBColor_TypeDef color1, RGBColor_TypeDef color2, uint8_t wait)
{
    static uint16_t j = 0;

    SetPixelColor((j+7)%8, Color(color1.R, color1.G, color1.B));
    SetPixelColor((j+6)%8, Color(color1.R*0.4f, color1.G*0.4f, color1.B*0.4f));
    SetPixelColor((j+5)%8, Color(color1.R*0.16f, color1.G*0.16f, color1.B*0.16f));
    //SetPixelColor((j+4)%8, Color(color1.R*0.08f, color1.G*0.08f, color1.B*0.08f));

    SetPixelColor((j+4)%8, Color(color2.R, color2.G, color2.B));
    SetPixelColor((j+3)%8, Color(color2.R*0.4f, color2.G*0.4f, color2.B*0.4f));
    SetPixelColor((j+2)%8, Color(color2.R*0.16f, color2.G*0.16f, color2.B*0.16f));
    SetPixelColor((j+1)%8, Color(color2.R*0.08f, color2.G*0.08f, color2.B*0.08f));
    if(j+1 > 7)
        j = 0;
    else
        j++;
    osDelay(wait);
}






