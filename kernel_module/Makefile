#빌드할 모듈의 이름
obj-m += stm32_adc_driver.o

#현재 라즈베리파이 커널 빌드 경로를 자동으로 찾음
KDIR := /lib/modules/$(shell uname -r)/build

#현재 디렉터리의 위치
PWD := $(shell pwd)

#make 명령어를 치면 실행될 타겟
all:
	make -C $(KDIR) M=$(PWD) modules

#make clean 명령어를 치면 실행될 타겟
clean:
	make -C $(KDIR) M=$(PWD) clean