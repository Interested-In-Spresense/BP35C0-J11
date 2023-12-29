BP35C0-J11 (WiSUN HAN Module)�p Arduino���C�u����

-------------------------

���̃��C�u�����́A***���[���������*** ���쐬�� ***Arduino���C�u����*** ���x�[�X�ɂȂ��Ă���A
��������ς������̂ɂȂ�܂��B
���C�Z���X���́A���L���C�u�����ɏ����܂��B

https://github.com/RohmSemiconductor/Arduino


## BP35C0-J11�Ƃ�

***BP35C0-J11*** �Ƃ́A���[��������� ���甭���� Wi-SUN Enhanced HAN�Ή� �������W���[���ł��B

BP35C0-J11 : https://www.rohm.co.jp/products/wireless-communication/specified-low-power-radio-modules/bp35c0-j11-product

������̃��W���[���� Spresense �� ***Wi-SUN Add-on�{�[�h �iSPRESENSE-WiSUN-EVK-701�j*** �Ƃ��Ĕ̔�����Ă��܂��B

SPRESENSE-WiSUN-EVK-701 : https://www.rohm.co.jp/support/spresense-add-on-board

�{�[�h�̏ڍׂ́A���i�T�C�g���Q�Ƃ��Ă��������B

-------------------------

### API

#### begin

```
  �����F �Ȃ�

  �߂�l�F �Ȃ�

  �����F
	���C�u�����̏�������HW�̋N�����s���܂��B
```

#### init

```
  �����F uint8_t mode : �f�o�C�X�̓��샂�[�h���w�肵�܂��B�iPAN_COORDINATOR / COORDINATOR / END_DEVICE / DUAL_MODE�j
         uint8_t sleep : sleep���[�h���w�肵�܂��B�iLEEP_DISABLE / SLEEP_ENABLE�j
         uint8_t channel : scan����`�����l�����w�肵�܂��B�i0x04�`0x11�j
         uint8_t power ; ���M�d�͂��w�肵�܂��B�i0x00�F20mW / 0x01�F10mW / 0x02�F1mW�j

  �߂�l�F boolean : ���� / ���s ���Ԃ�܂��B

  �����F
        �p�����[�^�ɍ��킹�ăf�o�C�X�̂̏����������܂��B
```

#### set_auth

```
  �����F const char* addr : MAC �A�h���X���w�肵�܂��B
		 const char* pw : �p�X���[�h���w�肵�܂��B�i16 ������ ASCII �����j

  �߂�l�F boolean : ���� / ���s ���Ԃ�܂��B

  �����F
        HAN �� PANA �F�؂��s���܂��B
```

#### scan

```
  �����F �Ȃ�

  �߂�l�F boolean : ���� / ���s ���Ԃ�܂��B

  �����F
        �w�肵���`���l���ɑ΂��ăX�L�������s���܂��B
        �R�[�f�B�l�[�^�[�̏ꍇ�̓A�N�e�B�u�X�L�����A�G���h�f�o�C�X�̏ꍇ��ED�X�L���������܂��B
```

#### start_han

```
  �����F const char* id : Paring ID�iPAN �R�[�f�B�l�[�^�� MAC �A�h���X�j���w�肵�܂��B

  �߂�l�F boolean : ���� / ���s ���Ԃ�܂��B

  �����F
        HAN ������J�n���A���������ꍇ�A�^�p��ԂɑJ�ڂ��܂��B
```

#### start_udp

```
  �����F const char* mac : ���M��� IPv6 �A�h���X���w�肵�܂��B
         const char* my : ���g�i���M���j�� UDP �|�[�g�ԍ����w�肵�܂��B
         const char* dist : ���M��� UDP �|�[�g�ԍ����w�肵�܂��B

  �߂�l�F boolean : ���� / ���s ���Ԃ�܂��B

  �����F
        UDP ��M�ɗ��p����w�肵���l�� UDP �|�[�g�� OPEN ���܂��B
```

#### send_data

```
  �����F const char* data : ���肽���f�[�^���w�肵�܂��B

  �߂�l�F boolean : ���� / ���s ���Ԃ�܂��B

  �����F
        UDP�Ńf�[�^�𑗐M���܂��B
```

### �g����

	�ȉ��̃T���v���R�[�h���Q�l�ɂ��Ă��������B
	
	examples/enddevice_sample.ino

