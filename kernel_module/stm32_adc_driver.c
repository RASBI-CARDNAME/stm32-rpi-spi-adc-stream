#include <linux/init.h> // 모듈 시작 / 정지 헤더
#include <linux/module.h> //커널 모듈 필수 헤더
#include <linux/kernel.h> //커널 함수를 정의한 헤더
#include <linux/fs.h> // 파일 시스템 구조체 (file_operations 등)
#include <linux/cdev.h> // 문자 디바이스 구조체
#include <linux/uaccess.h> //유저 공간 메모리 접근 (copy_to_user)
#include <linux/device.h> //디바이스 클래스 생성(udev 연동)
#include <linux/err.h> //에러처리 (IS_ERR, PTR_ERR)
#include <linux/spi/spi.h> //spi 헤더 추가
#include <linux/slab.h> //kmalloc 메모리 할당 용

//모듈 정보 정의
MODULE_LICENSE("GPL");
MODULE_AUTHOR("RASBI");
MODULE_DESCRIPTION("STM32 ADC High-Speed Driver");
MODULE_VERSION("0.6");

//매크로
#define DEVICE_NAME "stm32_adc"
#define CLASS_NAME "stm32_cls"
#define SPI_BUF_SIZE 4096 //핑퐁버퍼이므로 절반. 4KB

//전역 변수들
static int major_number; //주번호
static struct class* stm32_adc_class = NULL; //디바이스 클래스 구조체
// static struct device* stm32_adc_device = NULL; // 이제는 probe 안에서 관리함.
static struct cdev stm32_cdev; //문자 디바이스 구조체
static struct spi_device *stm32_spi_device = NULL;
static u8 *kbuf; //메모리 공간은 probe에서 할당할 예정. 일단은 주소를 담는 변수만 선언.


static int dev_open(struct inode *inodep, struct file *filep) {
    printk(KERN_INFO "STM32_ADC: Device has been opened\n");
    return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    int ret;

    // 1. 유저가 요청한 크기가 너무 작으면 거절 (안전장치)
    if (len < SPI_BUF_SIZE) {
        printk(KERN_INFO "STM32_ADC: User Buffer too small (%zu < %d)\n", len, SPI_BUF_SIZE); //%zu는 size_t를 출력하기 위한 포맷 지정자 

        return -EINVAL; //유효 하지 않은 인자 오류 코드
    }

    //장치 할당이 되지 않았다면
    if (!stm32_spi_device) return -EFAULT;

    // 2. SPI 데이터 읽기 (4096바이트를 한번에 읽어온다)
    //이 함수는 cs를 내리고 4096번 클럭을 쏠 때 까지 리턴하지 않는다 (블록킹)
    //커널 api: spi_read(장치 포인터, 버퍼, 바이트 단위 길이)
    ret = spi_read(stm32_spi_device, kbuf, SPI_BUF_SIZE);

    if (ret < 0) {
        printk(KERN_ALERT "STM32_ADC: SPI Read Failed! %d\n",ret);
        return ret;
    }

    // 3.유저 공간으로 복사
    if (copy_to_user(buffer, kbuf, SPI_BUF_SIZE)) {
        return -EFAULT;
    }

    // 4.읽은 바이트 수 리턴
    return SPI_BUF_SIZE;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
    int ret;
    u8 *tx_buf;

    //장치 할당 확인
    if (!stm32_spi_device) return -EFAULT;

    //쓰기 길이 확인
    if (len == 0) return 0;

    //메모리 할당
    tx_buf = kmalloc(len, GFP_KERNEL); //blocking
    if (!tx_buf) return -ENOMEM; //요청한 메모리를 할당할 수 없음.

    //유저 데이터 가져오기
    //전부 성공시 0, 실패하면 실패한 바이트만큼의 리턴값이 나옴.
    if (copy_from_user(tx_buf, buffer, len)) {
        kfree(tx_buf);
        return -EFAULT;
    }

    //spi 전송 (blocking)
    ret = spi_write(stm32_spi_device, tx_buf, len);
    kfree(tx_buf);

    if (ret <0) {
        printk(KERN_ALERT "STM32_ADC: SPI Write Failed! %d\n", ret);
        return ret;
    }

    return len;
}


static int dev_release(struct inode *inodep, struct file *filep) {
    printk(KERN_INFO "STM32_ADC: Device successfully closed\n");
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};

// ================================================================
// [NEW] 여기서부터 SPI 드라이버 관련 코드 (기존 init/exit 대체)
// ================================================================

/*
probe 함수: 커널이 디바이스 트리에서 "rasbi, stm32_adc"를 발견하고,
이 드라이버와 매칭이 되면 호출되는 함수. 즉 하드웨어 발견시 실행 됨.
*/

static int stm32_adc_probe(struct spi_device *spi) {
    printk(KERN_INFO "STM32_ADC: Probe called! SPI device found.\n");

    //1. spi 장치 정보 저장 (인자로 받은 주소 저장)
    stm32_spi_device = spi;

    // 커널 메모리 할당
    //GFP_KERNEL get free page플래그로 메모리 여유가 없다면 커널이 공간 할당할때 까지 기다림(커널이 잠재움). 그동안 커널은 스왑하거나 하여 공간 확보 함. 
    kbuf = kmalloc(SPI_BUF_SIZE, GFP_KERNEL);
    if (!kbuf) {
        printk(KERN_ALERT "STM32_ADC: Failed to allocate memory\n");
        return -ENOMEM;
    }

    //spi 설정
    spi -> mode = SPI_MODE_0;
    spi -> bits_per_word = 8;
    spi_setup(spi);

    //2. 문자 디바이스 등록 (기존 init에 있던 내용이 여기로 오게 된다.) 
    dev_t dev_num;
    int ret;

    //번호 할당
    ret = alloc_chrdev_region(&dev_num,0,1, DEVICE_NAME);
    if (ret <0) return ret;
    major_number = MAJOR(dev_num);

    //cdev 등록
    cdev_init(&stm32_cdev, &fops);
    stm32_cdev.owner = THIS_MODULE;
    ret = cdev_add(&stm32_cdev, dev_num, 1);
    if (ret <0) goto add_err;

    //클래스 생성
    stm32_adc_class = class_create(CLASS_NAME);
    if (IS_ERR(stm32_adc_class)) {
        ret = PTR_ERR(stm32_adc_class);
        goto class_err;
    }

    //디바이스 파일 생성 (/dev/stm32_adc)
    struct device *dev_ret = device_create(stm32_adc_class,&spi->dev, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(dev_ret)) {
        ret = PTR_ERR(dev_ret);
        goto device_err;
    }

    printk(KERN_INFO "STM32_ADC: Probe success. Your Major number:%d\n",major_number);
    return 0;

device_err:
    class_destroy(stm32_adc_class);
class_err:
    cdev_del(&stm32_cdev);
add_err:
    unregister_chrdev_region(dev_num,1);
    return ret;
}

/*
remove 함수: 드라이버가 제거되거나 장치가 뽑힐 때 호출
*/
static void stm32_adc_remove(struct spi_device *spi) {
    printk(KERN_INFO "STM32_ADC: Remove called.\n");

    kfree(kbuf); //메모리 반납
    device_destroy(stm32_adc_class, MKDEV(major_number,0));
    class_destroy(stm32_adc_class);
    cdev_del(&stm32_cdev);
    unregister_chrdev_region(MKDEV(major_number,0),1);
}

//매칭 테이블: 디바이스 트리의 "compatible"과 값이 일치해야 함
static const struct of_device_id stm32_adc_dt_ids[] = {
    {.compatible = "rasbi,stm32_adc",},
    {}
};
MODULE_DEVICE_TABLE(of, stm32_adc_dt_ids);

// SPI 드라이버 구조체 정의
static struct spi_driver stm32_adc_driver = {
    .driver = {
        .name = "stm32_adc_driver",
        .owner = THIS_MODULE,
        .of_match_table = stm32_adc_dt_ids,
    },
    .probe = stm32_adc_probe, //장치 발견시 실행할 함수
    .remove = stm32_adc_remove, // 장치 제거 시 실행할 함수
};

//모듈 초기화 / 해제 (이제 spi_register_driver만 호출함)
static int __init stm32_adc_init(void) {
    printk(KERN_INFO "STM32_ADC: Registering SPI Driver...\n");
    return spi_register_driver(&stm32_adc_driver);
}

static void __exit stm32_adc_exit(void) {
    printk(KERN_INFO "STM32_ADC: Unregistering SPI Driver...\n");
    spi_unregister_driver(&stm32_adc_driver);
}

module_init(stm32_adc_init);
module_exit(stm32_adc_exit);