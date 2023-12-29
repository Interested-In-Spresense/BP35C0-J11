/*
 enddvice_sample.ino
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
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/
#include "bp35c0-j11.h"

unsigned const char password[16] = { '1' , '1' , '1' , '1' , '2', '2' , '2' , '2' , '3' , '3' , '3' , '3' , '4' , '4' , '4' , '4' };    // PANA認証時のパスワード
unsigned const char pair_id[8] = {0x00 , 0x1D , 0x12 , 0x91 , 0x00 , 0x01 , 0x68 , 0xBD};   // 接続先MACアドレス
unsigned const char mac_adr[16] = {0xFE , 0x80 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x02 , 0x1D , 0x12 , 0x91 , 0x00 , 0x01 , 0x68 , 0xBD}; // 接続先IPv6アドレス
unsigned const char my_port[2] = { 0x01 , 0x23 };     // オープンするUDPポート
unsigned const char dist_port[2] = { 0x0E , 0x1A };   // 送信先UDPポート

BP35C0J11 bp35c0j11;

void setup() {

  if(!bp35c0j11.begin()){
    puts("Device error");
    exit(1);
  }

  if(!bp35c0j11.init(END_DEVICE, SLEEP_DISABLE, 0x05, 0x00)){
    puts("Init error");
    exit(1);
  }

  if(!bp35c0j11.set_auth(password)){
    puts("set_auth error");
    exit(1);
  }

  if(!bp35c0j11.start_han(pair_id)){
    puts("start_han error");
    exit(1);
  }

  if(!bp35c0j11.start_udp(mac_adr, my_port, dist_port)){
    puts("start_udp error");
    exit(1);
  }
}

void loop() {

   static unsigned char data[MAX_WISUN_DATA_SIZE] = { 'T' , 'E' , 'S' , 'T', 'A' , 'B' , 'C' , 'D', 'E' , 'F' , 'G' , 'H', 'I' , 'J' , 'K' , 'L'};

  if(!bp35c0j11.send_data(data)){
    puts("start_auth error");
  }

  for(int i=0; i<sizeof(data); i++){
    data[i] = data[(i+1)%sizeof(data)];
  }

  sleep(5);

}
