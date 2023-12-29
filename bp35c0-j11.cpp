/********************************************************************************
  bp35c0-j11.cpp
  Copyright (c) 2019 ROHM Co.,Ltd.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
modified by A  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
改変する  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*********************************************************************************/
#include <Arduino.h>
#include "bp35c0-j11.h"

unsigned const char uni_req[4] = {0xD0 , 0xEA , 0x83 , 0xFC};
unsigned const char uni_res[4] = {0xD0 , 0xF9 , 0xEE , 0x5D};

unsigned char ini_data[4] = {0x03 , 0x00 , 0x05 , 0x00};       // エンドデバイス/Sleep 非対応/922.9MHz/20mW出力unsigned char _pair_id[8] = {0x00 , 0x1D , 0x12 , 0x91 , 0x00 , 0x00 , 0x05 , 0xA7};   // 接続先MACアドレス
unsigned char _pair_id[8] = {0x00 , 0x1D , 0x12 , 0x91 , 0x00 , 0x00 , 0x05 , 0xA7};   // 接続先MACアドレス
unsigned char _pan_id[2] = {0x00 , 0x01};   // 自身のPan ID (for PAN Coodinator)
unsigned char _mac_adr[16] = {0xFE , 0x80 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x02 , 0x1D , 0x12 , 0x91 , 0x00 , 0x00 , 0x05 , 0xE7}; // 接続先IPv6アドレス
unsigned char _my_port[2] = { 0x01 , 0x23 };     // オープンするUDPポート
unsigned char _dist_port[2] = { 0x0E , 0x1A };   // 送信先UDPポート
unsigned char _password[16] = { '1' , '1' , '1' , '1' , '2', '2' , '2' , '2' , '3' , '3' , '3' , '3' , '4' , '4' , '4' , '4' };    // PANA認証時のパスワード
unsigned char radiodata[MAX_WISUN_DATA_SIZE] = {0};

CMD_FORMAT cmd_format;

BP35C0J11::BP35C0J11(void)
{
  
}

/********************************************************************************
*   Name     : begin
*   Function : initial setting bp35c0-j11
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::begin(void)
{
  j11_init();
  return wait_msg();

}

/********************************************************************************
*   Name     : init
*   Function : Mode setting bp35c0-j11
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::init(uint8_t mode, uint8_t sleep, uint8_t channel, uint8_t power)
{
  device_mode = mode;

  ini_data[0] = mode;
  ini_data[1] = sleep;
  ini_data[2] = channel;
  ini_data[3] = power;

  cmd_send(CMD_INI);
  return wait_msg();

}

/********************************************************************************
*   Name     : set_auth
*   Function : Authentication information setting
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::set_auth(const char* addr, const char* pw)
{
  if((device_mode == COORDINATOR) | (device_mode == END_DEVICE) ){
    return false;
  }

  memcpy(_password, pw, sizeof(_password));
  memcpy(_pair_id, addr, sizeof(_pair_id));

  cmd_send(CMD_PANA_SET);
  return false;
}

/********************************************************************************
*   Name     : set_auth
*   Function : Authentication information setting
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::set_auth(const char* pw)
{
  if((device_mode == PAN_COORDINATOR) | (device_mode == DUAL_MODE) ){
    return false;
  }

  memcpy(_password, pw, sizeof(_password));

  cmd_send(CMD_PANA_SET);
  return wait_msg();

}

/********************************************************************************
*   Name     : scan
*   Function : Scan target
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::scan(void)
{
  boolean rc = false;

  while(!rc){
    cmd_send(CMD_SCAN);
    rc = wait_msg();
    sleep(5);
  }

  return rc;

}

/********************************************************************************
*   Name     : start_auth
*   Function : Start Authen
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::start_han(const char* id)
{
  boolean rc = false;

  if((device_mode == PAN_COORDINATOR) | (device_mode == DUAL_MODE) ){
    memcpy(_pan_id, id, sizeof(_pan_id));
  }else{
    memcpy(_pair_id, id, sizeof(_pair_id));
  }

  while(!rc){
    cmd_send(CMD_HAN);
    rc = wait_msg();
    sleep(5);
  }

  while(!rc){
    cmd_send(CMD_PANA);
    rc = wait_msg();
    if(rc) {
      rc = wait_msg();
    }
    sleep(5);
  }

   return rc;

}

/********************************************************************************
*   Name     : start_udp
*   Function : Open UDP port
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::start_udp(const char* mac, const char* my, const char* dist)
{
  boolean rc = false;

  memcpy(_mac_adr, mac, sizeof(_mac_adr));
  memcpy(_my_port, my, sizeof(_my_port));
  memcpy(_dist_port, dist, sizeof(_dist_port));

  while(!rc){
    cmd_send(CMD_PORTOPEN);
    rc = wait_msg();
    sleep(5);
  }

  return rc;

}

/********************************************************************************
*   Name     : send_data
*   Function : Send data
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::send_data(const char* data)
{
  memcpy(radiodata, data, sizeof(radiodata));

  cmd_send(CMD_UDPSEND);
  return wait_msg();

}

/********************************************************************************
*   Name     : j11_init
*   Function : initial setting bp35c0-j11
*   input    : -
*   return   : -
*********************************************************************************/
void BP35C0J11::j11_init(void) {

  // configure output D20/D21
  pinMode(PIN_ENABLE, OUTPUT);      
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);

  delay(1000);

  // Serial port initial 
  Serial2.begin(115200);
  Serial.begin(115200);
  Serial.write("RESET");
  Serial.println("");

  digitalWrite(PIN_RESET, LOW);     // reset
  delay(500);
  digitalWrite(PIN_RESET, HIGH);

}

/********************************************************************************
*   Name     : wait_msg
*   Function : wait for response from bp35c0-j11
*   input    : -
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::wait_msg(void)
{
  unsigned long start_time;
  unsigned long current_time;
  unsigned char rcvdata[128] = {0} ;
  unsigned char cnt = 0 ;
  start_time = millis();
  while (Serial2.available() == 0)
  {
    current_time = millis();
    if ((current_time - start_time) > TIMEOUT) {
      Serial.println("receive timeout");
      return false;
    }

  }
  while (Serial2.available() > 0 ) {
    delay(5);
    rcvdata[cnt] = Serial2.read();
#ifdef DEBUG
    Serial.print(rcvdata[cnt] , HEX);
#endif
    cnt++;
    if (cnt >= 128) {
      Serial.println("receive data over flow");
      return false;
    }
  }
  if (rcvdata[0] == uni_res[0] && rcvdata[1] == uni_res[1] &&
      rcvdata[2] == uni_res[2] && rcvdata[3] == uni_res[3]) {     // RESPONSE/NORTIFICATION
    switch (rcvdata[4] << 8 | rcvdata[5]) {
      case (NORT_WAKE):

        break;
      case (RES_INI):
        if (rcvdata[12] == 0x01) {
          Serial.println("Init Success");
        } else {
          Serial.println("Init Error");
          return false;
        }
        break;
      case (RES_PANA_SET):
        if (rcvdata[12] == 0x01) {
          Serial.println("PANA Password set Success");
        } else {
          Serial.println("PANA Password set Error");
          return false;
        }
        break;
      case (RES_SCAN):

        break;
      case (NORT_SCAN):
        break;
      case (RES_HAN):
        if (rcvdata[12] == 0x01) {
          Serial.println("HAN Act Success");
        } else {
          Serial.println("HAN Act Error");
          return false;
        }
        break;
      case (RES_PANA):
        if (rcvdata[12] == 0x01) {
          Serial.println("PANA Act Success");
        } else {
          Serial.println("PANA Act Error");
          return false;
        }
        break;
      case (NORT_PANA):
        if (rcvdata[12] == 0x01) {
          Serial.println("PANA Connect Success");
        } else {
          Serial.println("PANA Connecgt Error");
          return false;
        }
        break;
      case (RES_CON_SET):
        if (rcvdata[12] == 0x01) {
          Serial.println("Normal connect mode");
        } else {
          Serial.println("connect mode change error");
          return false;
        }
        break;
      case (RES_PORTOPEN):
        if (rcvdata[12] == 0x01) {
          Serial.println("UDP port open Success");
        } else {
          Serial.println("UDP port open Error");
          return false;
        }
        break;
      case (RES_UDPSEND):
        if (rcvdata[12] == 0x01) {
          Serial.println("UDP send Success");
        } else {
          Serial.println("UDP send Error");
          return false;
        }
        break;
      case (0x2FFF):
        Serial.println("checksum error");
        return false;
        break;
      default:
        Serial.println("uni code error");
        return false;
        break;
    }

  } else {
    Serial.println("recv data error");
    return false;
  }

  return true;
}



/********************************************************************************
*   Name     : cmd_send
*   Function : REQUEST command to bp35c0-j11
*   input    : cmd  - REQUEST command 
*   return   : true/false
*********************************************************************************/
boolean BP35C0J11::cmd_send(unsigned short cmd) {
  unsigned short dat_chksum = 0 ;
  unsigned short msg_length = 0 ;
  unsigned short dat_length = 0 ;
  unsigned short send_dat_size = 0 ;
  unsigned char data[128] = {0};

  unsigned char send_data[128] = {0} ;
  unsigned char cnt = 0 ;

  switch (cmd) {
    case (CMD_RESET):
      dat_length = 0;
      break;
    case (CMD_INI):
      dat_length = (unsigned short)4;
      for (cnt = 0 ; cnt < dat_length ; cnt++ ) {
        data[cnt] = ini_data[cnt] ;
      }
      break;
    case (CMD_PANA_SET):
      if((device_mode == PAN_COORDINATOR) | (device_mode == DUAL_MODE) ){
        dat_length = (unsigned short)(16 + 8);
        for (cnt = 0 ; cnt < sizeof(_pair_id) ; cnt++ ) {
          data[cnt] = _pair_id[cnt] ;
        }
        for (cnt = 0 ; cnt < sizeof(_password) ; cnt++ ) {
          data[sizeof(_pair_id)+cnt] = _password[cnt] ;
        }
      }else{
        dat_length = (unsigned short)16 ;
        for (cnt = 0 ; cnt < dat_length ; cnt++ ) {
          data[cnt] = _password[cnt] ;
        }
      }
      break;
    case (CMD_SCAN):
      break;
    case (CMD_HAN):
      dat_length = (unsigned short)8 ;
      for (cnt = 0 ; cnt < dat_length ; cnt++ ) {
        data[cnt] = _pair_id[cnt] ;
      }
      break;
    case (CMD_PANA):
      dat_length = 0;
      break;
    case (CMD_CON_SET):
      dat_length = 1;
      data[0] = 0x02 ;
      break;
    case (CMD_PORTOPEN):
      dat_length = 2;
      for (cnt = 0 ; cnt < dat_length ; cnt++ ) {
        data[cnt] = _my_port[cnt] ;
      }
      break;
    case (CMD_UDPSEND):
      send_dat_size = sizeof(radiodata) ;
      dat_length = 22 + send_dat_size ;
      for (cnt = 0 ; cnt < 16 ; cnt++ ) {
        data[cnt] = _mac_adr[cnt] ;
      }
      data[16] = _my_port[0] ;
      data[17] = _my_port[1] ;          // 送信元UDPポート　：0x0123
      data[18] = _dist_port[0] ;
      data[19] = _dist_port[1] ;        // 送信先UDPポート：0x0E1A
      data[20] = (unsigned char)(send_dat_size >> 8);
      data[21] = (unsigned char)(send_dat_size & 0xFF); // send data length
      for (cnt = 0 ; cnt < send_dat_size ; cnt++) {
        data[22 + cnt] = radiodata[cnt];              // data
      }
      break;
    default:
      return false;
  }

  msg_length = (unsigned short)(4 + dat_length);
  for (cnt = 0 ; cnt < dat_length ; cnt++) {
    dat_chksum += data[cnt];
  }
  msg_create(cmd , msg_length , dat_chksum, data , send_data );
  Serial2.write(send_data, msg_length + CMD_HDR_LEN);
#ifdef DEBUG
  debugmsg( msg_length + CMD_HDR_LEN , send_data);
#endif

  return true;
}

/********************************************************************************
*   Name     : msg_create
*   Function : create Request command format
*   input    : cmd - Request command
*              msg_length - message data length
               dat_chksum - data checksum
               *pdada     - wireless data
               *psend_data- request command format data
*   return   : -
*********************************************************************************/
void static BP35C0J11::msg_create(unsigned short cmd, unsigned short msg_length,  unsigned short dat_chksum, unsigned char *pdata, unsigned char *psend_data)
{
  unsigned char cnt = 0 ;
  unsigned short hdr_chksum = uni_req[0] + uni_req[1] + uni_req[2] + uni_req[3] ;

  hdr_chksum += cmd + msg_length;

  for (cnt = 0 ; cnt < 4 ; cnt++) {
    psend_data[cnt] = uni_req[cnt];
  }
  psend_data[4] = (unsigned char)((cmd & 0xFF00) >> 8);
  psend_data[5] = (unsigned char)(cmd & 0xFF);
  psend_data[6] = (unsigned char)((msg_length & 0xFF00) >> 8);
  psend_data[7] = (unsigned char)(msg_length & 0xFF);
  psend_data[8] = (unsigned char)((hdr_chksum & 0xFF00) >> 8);
  psend_data[9] = (unsigned char)(hdr_chksum & 0xFF);
  psend_data[10] = (unsigned char)((dat_chksum & 0xFF00) >> 8);
  psend_data[11] = (unsigned char)(dat_chksum & 0xFF);

  if (msg_length > 4) {
    for (cnt = 0 ; cnt < msg_length - 4 ; cnt++)
    {
      psend_data[12 + cnt] = pdata[cnt];
    }
  }
}



/********************************************************************************
*   Name     : debugmsg
*   Function : output serial console for debug
*   input    : datalength - output data lengh
               psend_data - output data pointer
*   return   : -
*********************************************************************************/
void BP35C0J11::debugmsg(unsigned short datalength , unsigned char* psend_data) {
  unsigned char cnt = 0 ;

  for ( cnt = 0 ; cnt < datalength ; cnt++) {
    Serial.print(psend_data[cnt] , HEX);
    Serial.print(" ");
  }
  Serial.println("");
}



