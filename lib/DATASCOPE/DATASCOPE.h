
#ifndef _DATASCOPE_H__
#define _DATASCOPE_H__

/**
 * @brief       此库负责和上位机通讯显示波形
 *
 */

extern unsigned char DataScope_OutPut_Buffer[42];

class DATASCOPE {
  private:
    unsigned char Send_Count;

  public:
    DATASCOPE();
    void Float2Byte(float* target, unsigned char* buf, unsigned char beg);
    void DataScope_Get_Channel_Data(float Data, unsigned char Channel);
    unsigned char DataScope_Data_Generate(unsigned char Channel_Number);
    void Display(float data1); // 用于记录并显示数据
    void Display(float data1, float data2);
    void Display(float data1, float data2, float data3);              // 显示3个数据
    void Display(float data1, float data2, float data3, float data4); // 显示4个数据
    // 添加更多的声明
    void Display(float data1, float data2, float data3, float data4, float data5);
    void Display(float data1, float data2, float data3, float data4, float data5, float data6);
    void Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7);
    void Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8);
    void Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9);
    void Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10);
};

#endif