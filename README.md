# NSL-SDK
NSL2206-SDK와 NSL3140-SDK library입니다.

## WINDOWS 컴파일 방법
- Visual studio 2019에서 테스트 되었습니다.
- PCL-1.8.1-AllInOne-msvc2017-win64.exe 및 opencv 를 먼저 설치 후 사용 하십시오.
- NSL3140의 기본 아이피는 192.168.0.220 입니다. 변경 시 WINDOWS Application을 사용하여 변경 가능합니다.

## USB 인식용 rules 정의
```
$ sudo vi /etc/udev/rules.d/defined_lidar.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyLidar2206"
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777",SYMLINK+="ttyLidar3140"

$ service udev reload
$ service udev restart
```

## LINUX 컴파일 방법
```
$ cd NSL-SDK/NSL2206-SDK/SDK_SAMPLE or cd NSL-SDK/NSL3140-SDK/SDK_SAMPLE
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./nslApp
```

