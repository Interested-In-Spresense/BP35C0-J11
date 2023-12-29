BP35C0-J11 (WiSUN HAN Module)用 Arduinoライブラリ

-------------------------

このライブラリは、***ローム株式会社*** が作成の ***Arduinoライブラリ*** がベースになっており、
これを改変したものになります。
ライセンス等は、下記ライブラリに準じます。

https://github.com/RohmSemiconductor/Arduino


## BP35C0-J11とは

***BP35C0-J11*** とは、ローム株式会社 から発売の Wi-SUN Enhanced HAN対応 無線モジュールです。

BP35C0-J11 : https://www.rohm.co.jp/products/wireless-communication/specified-low-power-radio-modules/bp35c0-j11-product

こちらのモジュールを Spresense の ***Wi-SUN Add-onボード （SPRESENSE-WiSUN-EVK-701）*** として販売されています。

SPRESENSE-WiSUN-EVK-701 : https://www.rohm.co.jp/support/spresense-add-on-board

ボードの詳細は、商品サイトを参照してください。

-------------------------

### API

#### begin

```
  引数： なし

  戻り値： なし

  説明：
	ライブラリの初期化とHWの起動を行います。
```

#### init

```
  引数： uint8_t mode : デバイスの動作モードを指定します。（PAN_COORDINATOR / COORDINATOR / END_DEVICE / DUAL_MODE）
         uint8_t sleep : sleepモードを指定します。（LEEP_DISABLE / SLEEP_ENABLE）
         uint8_t channel : scanするチャンネルを指定します。（0x04～0x11）
         uint8_t power ; 送信電力を指定します。（0x00：20mW / 0x01：10mW / 0x02：1mW）

  戻り値： boolean : 成功 / 失敗 が返ります。

  説明：
        パラメータに合わせてデバイスのの初期化をします。
```

#### set_auth

```
  引数： const char* addr : MAC アドレスを指定します。
		 const char* pw : パスワードを指定します。（16 文字の ASCII 文字）

  戻り値： boolean : 成功 / 失敗 が返ります。

  説明：
        HAN の PANA 認証を行います。
```

#### scan

```
  引数： なし

  戻り値： boolean : 成功 / 失敗 が返ります。

  説明：
        指定したチャネルに対してスキャンを行います。
        コーディネーターの場合はアクティブスキャン、エンドデバイスの場合はEDスキャンをします。
```

#### start_han

```
  引数： const char* id : Paring ID（PAN コーディネータの MAC アドレス）を指定します。

  戻り値： boolean : 成功 / 失敗 が返ります。

  説明：
        HAN 動作を開始し、成功した場合、運用状態に遷移します。
```

#### start_udp

```
  引数： const char* mac : 送信先の IPv6 アドレスを指定します。
         const char* my : 自身（送信元）の UDP ポート番号を指定します。
         const char* dist : 送信先の UDP ポート番号を指定します。

  戻り値： boolean : 成功 / 失敗 が返ります。

  説明：
        UDP 受信に利用する指定した値の UDP ポートを OPEN します。
```

#### send_data

```
  引数： const char* data : 送りたいデータを指定します。

  戻り値： boolean : 成功 / 失敗 が返ります。

  説明：
        UDPでデータを送信します。
```

### 使い方

	以下のサンプルコードを参考にしてください。
	
	examples/enddevice_sample.ino

