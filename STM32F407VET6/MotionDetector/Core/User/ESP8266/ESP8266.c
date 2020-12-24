//
// Created by Y on 2020/12/17.
//

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "ESP8266.h"
#include "usart.h"

uint8_t rx_byte;
uint32_t rx_index = 0;
uint8_t rx_buf[RX_LEN];
uint8_t tx_buf[TX_LEN];

void Clear_rx_buf(void)
{
    rx_index = 0;
    memset(rx_buf, 0x00, RX_LEN);
}

//向ESP8266发送定长数据
void ESP8266_ATSendBuf(uint8_t* buf,uint16_t len)
{
    Clear_rx_buf();
    //定长发送
    HAL_UART_Transmit(&huart6, (uint8_t *)buf, len, 0xFFFF);
}

//向ESP8266发送字符串
void ESP8266_ATSendString(char *str)
{
    Clear_rx_buf();
    HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 0xFFFF);
}

//查找字符串中是否包含另一个字符串
uint8_t FindStr(char *dest, char *src, uint16_t retry_nms)
{
    retry_nms /= 10;                           //超时时间

    while(strstr(dest,src)==0 && retry_nms)//等待串口接收完毕或超时退出
    {
        HAL_Delay(10);
        retry_nms--;
    }

    if(retry_nms) {
        return 1;  //在响应时间内,则返回1
    }

    return 0;
}

//退出透传
void ESP8266_ExitUnvarnishedTrans(void)
{
    ESP8266_ATSendString("+++");
    HAL_Delay(50);
    ESP8266_ATSendString("+++");
    HAL_Delay(50);
}

//开启透传模式
static uint8_t ESP8266_OpenTransmission(void)
{
    uint8_t cnt=2;
    while(cnt--)
    {
        ESP8266_ATSendString("AT+CIPMODE=1\r\n");
        if(FindStr((char*)rx_buf,"OK",200)!=0)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * 功能：检查ESP8266是否正常
 * 参数：None
 * 返回值：ESP8266返回状态
 *        非0 ESP8266正常
 *        0 ESP8266有问题
 */
static uint8_t ESP8266_Check(void)
{
    uint8_t check_cnt = 5;
    while(check_cnt--)
    {
        ESP8266_ATSendString("AT\r\n");         //发送AT握手指令

        if(FindStr((char*)rx_buf,"OK",200) != 0)//返回OK
        {
            return 1;
        }
    }

    return 0;
}

/**
 * 功能：初始化ESP8266
 * 参数：None
 * 返回值：初始化结果，非0为初始化成功,0为失败
 */
uint8_t ESP8266_Init(void)
{
    ESP8266_ExitUnvarnishedTrans();		//退出透传
    HAL_Delay(500);
    ESP8266_ATSendString("AT+RST\r\n");
    HAL_Delay(800);
    if(!ESP8266_Check()){             //使用AT指令检查ESP8266是否存在
        return 0;
    }

    ESP8266_ATSendString("ATE0\r\n");     	//关闭回显

    if(FindStr((char*)rx_buf,"OK",500) == 0)  //设置不成功
    {
        return 0;
    }

    return 1;
}

/**
 * 功能：恢复出厂设置
 * 参数：None
 * 返回值：None
 * 说明:此时ESP8266中的用户设置将全部丢失回复成出厂状态
 */
void ESP8266_Restore(void)
{
    ESP8266_ExitUnvarnishedTrans();          	//退出透传
    HAL_Delay(500);
    ESP8266_ATSendString("AT+RESTORE\r\n");		//恢复出厂
}

/**
 * 功能：连接热点
 * 参数：
 *         ssid:热点名
 *         pswd:热点密码
 * 返回值：
 *         连接结果,非0连接成功,0连接失败
 * 说明：
 *         失败的原因有以下几种(UART通信和ESP8266正常情况下)
 *         1. WIFI名和密码不正确
 *         2. 路由器连接设备太多,未能给ESP8266分配IP
 */
uint8_t ESP8266_ConnectAP(char *ssid,char *pswd)
{
    uint8_t cnt = 5;
    while(cnt--)
    {
        ESP8266_ATSendString("AT+CWMODE_CUR=1\r\n");              //设置为STATION模式
        if(FindStr((char*)rx_buf,"OK",200) != 0)
        {
            break;
        }
    }
    if(cnt == 0){
        return 0;
    }

    cnt = 2;
    while(cnt--)
    {
        memset(tx_buf, 0, sizeof(tx_buf));//清空发送缓冲
        sprintf((char*)tx_buf, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", ssid, pswd);//连接目标AP
        ESP8266_ATSendString((char*)tx_buf);
        if(FindStr((char*)rx_buf,"OK",8000) != 0)    //连接成功且分配到IP
        {
            return 1;
        }
    }
    return 0;
}

/**
 * 功能：使用指定协议(TCP/UDP)连接到服务器
 * 参数：
 *         mode:协议类型 "TCP","UDP"
 *         ip:目标服务器IP
 *         port:目标是服务器端口号
 * 返回值：
 *         连接结果,非0连接成功,0连接失败
 * 说明：
 *         失败的原因有以下几种(UART通信和ESP8266正常情况下)
 *         1. 远程服务器IP和端口号有误
 *         2. 未连接AP
 *         3. 服务器端禁止添加(一般不会发生)
 */
uint8_t ESP8266_ConnectServer(char *mode, char *ip, uint16_t port)
{
    uint8_t cnt;

    ESP8266_ExitUnvarnishedTrans();                   //多次连接需退出透传
    HAL_Delay(500);

    //连接服务器
    cnt = 2;
    while(cnt--)
    {
        memset(tx_buf, 0, sizeof(tx_buf));//清空发送缓冲
        sprintf((char*)tx_buf, "AT+CIPSTART=\"%s\",\"%s\",%d\r\n", mode, ip, port);
        ESP8266_ATSendString((char*)tx_buf);
        if(FindStr((char*)rx_buf,"OK",8000) != 0)
        {
            break;
        }
    }
    if(cnt == 0) {
        return 0;
    }

    //设置透传模式
    if(ESP8266_OpenTransmission() == 0) return 0;

    //开启发送状态
    cnt = 2;
    while(cnt--)
    {
        ESP8266_ATSendString("AT+CIPSEND\r\n");//开始处于透传发送状态
        if(FindStr((char*)rx_buf,">",200) != 0)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * 功能：主动和服务器断开连接
 * 参数：None
 * 返回值：
 *         连接结果,非0断开成功,0断开失败
 */
uint8_t DisconnectServer(void)
{
    uint8_t cnt = 5;

    ESP8266_ExitUnvarnishedTrans();	//退出透传
    HAL_Delay(500);

    while(cnt--)
    {
        ESP8266_ATSendString("AT+CIPCLOSE\r\n");//关闭链接

        if(FindStr((char*)rx_buf,"CLOSED",200) != 0)//操作成功,和服务器成功断开
        {
            break;
        }
    }
    if(cnt) return 1;

    return 0;
}

/**
 * 功能：AP模式,UDP传输
 * 参数：
 *         ssid:WIFI名
 *         pswd:WIFI密码
 * 返回值：
 *         连接结果,非0连接成功,0连接失败
 */
uint8_t ESP8266_AP(char *ssid,char *pswd)
{
    uint8_t cnt = 2;
    while(cnt--)
    {
        ESP8266_ATSendString("AT+CWMODE=2\r\n");              //设置为AP模式
        if(FindStr((char*)rx_buf,"OK",200) != 0)
        {
            break;
        }
    }
    if(cnt == 0){
        return 0;
    }
    HAL_Delay(100);

    ESP8266_ATSendString("AT+RST\r\n");
    HAL_Delay(800);

    cnt = 2;
    while(cnt--)
    {
        memset(tx_buf, 0, sizeof(tx_buf));//清空发送缓冲
        sprintf((char*)tx_buf, "AT+CWSAP=\"%s\",\"%s\",1,%d\r\n", ssid, pswd, 3);//创建AP
        ESP8266_ATSendString((char*)tx_buf);
        if(FindStr((char*)rx_buf,"OK",8000) != 0)    //创建成功
        {
            break;
        }
    }
    if(cnt == 0){
        return 0;
    }
    HAL_Delay(100);

    cnt = 2;
    while(cnt--)
    {
        ESP8266_ATSendString("AT+CIPMUX=0\r\n");
        if(FindStr((char*)rx_buf,"OK",8000) != 0)    //创建成功
        {
            break;
        }
    }
    if(cnt == 0){
        return 0;
    }
    HAL_Delay(100);

    return 1;
}

uint8_t ESP8266_UDP_Connect(char *ip,int port)
{
    uint8_t cnt = 2;
    while(cnt--)
    {
        memset(tx_buf, 0, sizeof(tx_buf));//清空发送缓冲
        sprintf((char*)tx_buf, "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n", ip, port);//建立UDP连接
        ESP8266_ATSendString((char*)tx_buf);
        if(FindStr((char*)rx_buf,"OK",8000) != 0)    //创建成功
        {
            break;
        }
    }
    if(cnt == 0){
        return 0;
    }

    return 1;
}

uint8_t ESP8266_UDP_Send(char *data)
{
    uint8_t cnt = 2;
    while(cnt--)
    {
        memset(tx_buf, 0, sizeof(tx_buf));//清空发送缓冲
        sprintf((char*)tx_buf, "AT+CIPSEND=%d\r\n", strlen(data));//数据长度
        ESP8266_ATSendString((char*)tx_buf);
        if(FindStr((char*)rx_buf,"OK",20) != 0)
        {
            break;
        }
    }
    if(cnt == 0){
        return 0;
    }

    cnt = 2;
    while(cnt--)
    {
        ESP8266_ATSendString(data);
        if(FindStr((char*)rx_buf,"OK",20) != 0)
        {
            break;
        }
    }
    if(cnt == 0){
        return 0;
    }

    return 1;
}