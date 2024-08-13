
#include "DATASCOPE.h"
#include "Arduino.h"
unsigned char DataScope_OutPut_Buffer[42] = {0};

DATASCOPE::DATASCOPE() {
}

void DATASCOPE::Float2Byte(float* target, unsigned char* buf, unsigned char beg) {
    unsigned char* point;
    point = (unsigned char*)target;
    buf[beg + 1] = point[1];
    buf[beg + 2] = point[2];
    buf[beg + 3] = point[3];
}
void DATASCOPE::DataScope_Get_Channel_Data(float Data, unsigned char Channel) {
    if ((Channel > 10) || (Channel == 0))
        return;
    else {
        switch (Channel) {
            case 1:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 1);
                break;
            case 2:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 5);
                break;
            case 3:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 9);
                break;
            case 4:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 13);
                break;
            case 5:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 17);
                break;
            case 6:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 21);
                break;
            case 7:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 25);
                break;
            case 8:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 29);
                break;
            case 9:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 33);
                break;
            case 10:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 37);
                break;
        }
    }
}

unsigned char DATASCOPE::DataScope_Data_Generate(unsigned char Channel_Number) {
    if ((Channel_Number > 10) || (Channel_Number == 0)) {
        return 0;
    } else {
        DataScope_OutPut_Buffer[0] = '$';

        switch (Channel_Number) {
            case 1:
                DataScope_OutPut_Buffer[5] = 5;
                return 6;
            case 2:
                DataScope_OutPut_Buffer[9] = 9;
                return 10;
            case 3:
                DataScope_OutPut_Buffer[13] = 13;
                return 14;
            case 4:
                DataScope_OutPut_Buffer[17] = 17;
                return 18;
            case 5:
                DataScope_OutPut_Buffer[21] = 21;
                return 22;
            case 6:
                DataScope_OutPut_Buffer[25] = 25;
                return 26;
            case 7:
                DataScope_OutPut_Buffer[29] = 29;
                return 30;
            case 8:
                DataScope_OutPut_Buffer[33] = 33;
                return 34;
            case 9:
                DataScope_OutPut_Buffer[37] = 37;
                return 38;
            case 10:
                DataScope_OutPut_Buffer[41] = 41;
                return 42;
        }
    }
    return 0;
}

void DATASCOPE::Display(float data1) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    Send_Count = DataScope_Data_Generate(1);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    Send_Count = DataScope_Data_Generate(2);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    DataScope_Get_Channel_Data(data3, 3);
    Send_Count = DataScope_Data_Generate(3);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3, float data4) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    DataScope_Get_Channel_Data(data3, 3);
    DataScope_Get_Channel_Data(data4, 4);
    Send_Count = DataScope_Data_Generate(4);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3, float data4, float data5) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    DataScope_Get_Channel_Data(data3, 3);
    DataScope_Get_Channel_Data(data4, 4);
    DataScope_Get_Channel_Data(data5, 5);
    Send_Count = DataScope_Data_Generate(5);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3, float data4, float data5, float data6) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    DataScope_Get_Channel_Data(data3, 3);
    DataScope_Get_Channel_Data(data4, 4);
    DataScope_Get_Channel_Data(data5, 5);
    DataScope_Get_Channel_Data(data6, 6);
    Send_Count = DataScope_Data_Generate(6);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    DataScope_Get_Channel_Data(data3, 3);
    DataScope_Get_Channel_Data(data4, 4);
    DataScope_Get_Channel_Data(data5, 5);
    DataScope_Get_Channel_Data(data6, 6);
    DataScope_Get_Channel_Data(data7, 7);
    Send_Count = DataScope_Data_Generate(7);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    DataScope_Get_Channel_Data(data3, 3);
    DataScope_Get_Channel_Data(data4, 4);
    DataScope_Get_Channel_Data(data5, 5);
    DataScope_Get_Channel_Data(data6, 6);
    DataScope_Get_Channel_Data(data7, 7);
    DataScope_Get_Channel_Data(data8, 8);
    Send_Count = DataScope_Data_Generate(8);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    DataScope_Get_Channel_Data(data3, 3);
    DataScope_Get_Channel_Data(data4, 4);
    DataScope_Get_Channel_Data(data5, 5);
    DataScope_Get_Channel_Data(data6, 6);
    DataScope_Get_Channel_Data(data7, 7);
    DataScope_Get_Channel_Data(data8, 8);
    DataScope_Get_Channel_Data(data9, 9);
    Send_Count = DataScope_Data_Generate(9);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}

void DATASCOPE::Display(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10) {
    int i;
    DataScope_Get_Channel_Data(data1, 1);
    DataScope_Get_Channel_Data(data2, 2);
    // 重复上述模式，为每个数据通道赋值
    DataScope_Get_Channel_Data(data3, 3);
    DataScope_Get_Channel_Data(data4, 4);
    DataScope_Get_Channel_Data(data5, 5);
    DataScope_Get_Channel_Data(data6, 6);
    DataScope_Get_Channel_Data(data7, 7);
    DataScope_Get_Channel_Data(data8, 8);
    DataScope_Get_Channel_Data(data9, 9);
    DataScope_Get_Channel_Data(data10, 10);
    Send_Count = DataScope_Data_Generate(10);
    for (i = 0; i < Send_Count; i++) {
        Serial.write(DataScope_OutPut_Buffer[i]);
    }
}
