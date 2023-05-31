# sco-ltem1wlm_nrf9160

## Getting start

### About
このソフトウェアはさくらのセキュアモバイルコネクトを利用して水位計の計測値をサーバに送信するサンプルコードです。
同梱しているPCBファイルで製造した基板上で動作します。(PCB/SCO-LTEM1WLM_NRF9160-B)

水位計からクラウドサーバへの上り通信のみを行います。通信プロトコルはUDPで再送しません。
2分に1回の送信で平均消費電流は500uA(3.6V)程度です。消費電流は電波状況によって変動します。

開発環境はnRF Connect SDK v2.3.0です。付属のbashで下記のコマンドでビルドできます。
ビルドのみ：./build.sh production
ビルド後書き込み：./flash.sh production

### Install nRF Connect SDK

See [nRF Connect SDK Getting started](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html).  
If you want to install the development environment quickly, see [Installing the nRF Connect SDK through nRF Connect for Desktop](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_assistant.html#gs-assistant).

Using nRF Connect SDK v2.3.0

### Clone this repository

```
git clone https://github.com/sakura-internet/sco-ltem1wlm_nrf9160.git
cd sco-ltem1wlm_nrf9160
```

### Clean

```
rm -rf build
```

### Build

Use `build.sh` for build.

```
./build.sh [target] [board]
```

target

- develop (default)
- staging
- production
- local

board

- scm-ltem1nrf_nrf9160ns on SCO-LTEM1WLM-B

For develop / SCM-LTEM1NRF
```
./build.sh
```

For production / SCM-LTEM1NRF
```
./build.sh production
```

For local only
```
cp -n prj.conf.develop prj.conf.local
vi prj.conf.local
./build.sh local
```

### Flash

`nrfjprog` is required.

For develop
```
./flash.sh
```

For production
```
./flash.sh production
```

OR

Write the HEX image file 'build/{ENV}/zephyr/merged.hex' using nRF Connect `Programmer' application.

---
Please refer to the [Wiki(Japanese)](https://github.com/sakura-internet/sipf-std-client_nrf9160/wiki) for specifications.
