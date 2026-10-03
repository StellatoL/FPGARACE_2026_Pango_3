#include "hardware/pango_fun.h"
#include "hardware/pcie_frame_status.h"
#include "ui_mainwindow.h"
#include <unistd.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <pthread.h>
#include <sys/mman.h>
#include <cstring>
#define PAGE_ROUND_DOWN(x)				((x) & ~(getpagesize() - 1))
#define PAGE_ROUND_UP(x)				(PAGE_ROUND_DOWN((x) + getpagesize() - 1))

#define BAR0_MAX                       0x500
void* pango_fun::map_mem(unsigned long addr, unsigned int size, DEV_MEM *map_mem)
{
    int fd = -1;
    void *vaddr = NULL;
    off_t offset;
    size_t len;

    offset = PAGE_ROUND_DOWN(addr);
    len    = PAGE_ROUND_UP(size);

    // 若已映射到同一 BAR0 区域，直接复用现有映射。
    // 该检查必须在下面的 busy 保护之前：否则关闭再打开设备时会命中
    // busy 分支返回 NULL，导致 mem_pio_addr 变为空、后续 pio_write 解引用空指针崩溃。
    if ((map_mem->vaddr != NULL) && (offset == map_mem->offset) &&
        (len == map_mem->len))
        return map_mem->vaddr;

    if(map_mem->vaddr != NULL)
    {
        //printf("Bar0 is busy, please release it first\n");
        qDebug()<<"Bar0 is busy, please release it first\n";
        return vaddr;
    }

    fd = open(MEM_FILE_PATH, O_RDWR | O_SYNC);
    if(fd < 0)
    {
        //printf("open '%s' failed !!!\n", MEM_FILE_PATH);
        qDebug()<<"open failed !!!"<<MEM_FILE_PATH;
    }
    else
    {
        vaddr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
        close(fd);
    }
    if ((vaddr == NULL) || (vaddr == MAP_FAILED))
    {
        //printf("mmap '%s' failed !!!\n", MEM_FILE_PATH);
        qDebug()<<"mmap failed !!!"<<MEM_FILE_PATH;
    }
    else
    {
        map_mem->len 	= len;
        map_mem->offset = offset;
        map_mem->vaddr 	= vaddr;
    }
    //qDebug()<<"bar0 vir addr is at"<<vaddr;
    qDebug() << "bar0 vir addr is at" << vaddr;
    return vaddr;
}
//initial
pango_fun::pango_fun()
{
    std::memset(&dma_operation, 0, sizeof(dma_operation));
    dev_fd  =   -1;


}

/*
int pango_fun::nano_delay(long delay)
{
    struct timespec req, rem;
    long nano_delay = delay;
    int ret = 0;
    while(nano_delay > 0)
    {
        rem.tv_sec = 0;
        rem.tv_nsec = 0;
        req.tv_sec = 0;
        req.tv_nsec = nano_delay;
        if(ret = (nanosleep(&req, &rem) == -1))
        {
       //     printf_error("nanosleep failed !!!\n");
        }
        nano_delay = rem.tv_nsec;
    };

    return ret;
}

*/


void pango_fun::pio_write(unsigned int  write_data,int pio_bse_addr,int user_reg_addr)
{

    *((unsigned int *)mem_pio_addr + pio_bse_addr + user_reg_addr ) = write_data  ;


}

void pango_fun::pio_read(unsigned char* read_buf,int pio_addr)
{

    *read_buf  =  *((unsigned char *)mem_pio_addr + pio_addr) ;
    //qDebug()<<"read data is"<<;


}

void pango_fun::dma_write_test()
{
    dma_oper->current_len=(4>>2);
    memset(dma_oper->data.write_buf,0,DMA_MAX_PACKET_SIZE);
    dma_oper->data.write_buf[0]=0x31;
    ioctl(dev_fd,PCI_MAP_ADDR_CMD,dma_oper);
    ioctl(dev_fd,PCI_WRITE_TO_KERNEL_CMD,dma_oper);//write data to kernel
    ioctl(dev_fd,PCI_DMA_READ_CMD,dma_oper);//DMA read data to device
    for(int i =0; i<=1250;i++)
    {}
    ioctl(dev_fd,PCI_UMAP_ADDR_CMD,dma_oper);


}



void pango_fun::dma_map(bool flag)
{
    if(flag)
    {
        ioctl(dev_fd,PCI_MAP_ADDR_CMD,dma_oper);
    }
    else
    {
        ioctl(dev_fd,PCI_UMAP_ADDR_CMD,dma_oper);
    }
}


void pango_fun::dma_set_config(void)
{
    ioctl(dev_fd,PCI_SET_CONFIG,dma_oper);

}




void pango_fun::dma_write_data(unsigned char cmd_in)
{
    ioctl(dev_fd,PCI_WRITE_TO_KERNEL_CMD,dma_oper);//write data to kernel
    for(int i =0; i<=1250;i++)
    {}
    ioctl(dev_fd,PCI_DMA_READ_CMD,dma_oper);//DMA read data to device

}



void pango_fun::write_dw_data(uchar* send_buf)
{
    dma_oper->current_len=(16>>2);
    memset(dma_oper->data.write_buf,0,DMA_MAX_PACKET_SIZE);
    dma_oper->data.write_buf[0]=send_buf[0];
    dma_oper->data.write_buf[1]=send_buf[1];
    dma_oper->data.write_buf[2]=send_buf[2];
    dma_oper->data.write_buf[3]=send_buf[3];
    ioctl(dev_fd,PCI_MAP_ADDR_CMD,dma_oper);
    ioctl(dev_fd,PCI_WRITE_TO_KERNEL_CMD,dma_oper);//write data to kernel
    ioctl(dev_fd,PCI_DMA_READ_CMD,dma_oper);//DMA read data to device
    for(int i =0; i<=1250;i++)
    {}
    ioctl(dev_fd,PCI_UMAP_ADDR_CMD,dma_oper);
}


int  pango_fun::dma_read(unsigned int len,unsigned char* img_buf)
{
    //读取数据
    int rd_status;
    rd_status = read(dev_fd,img_buf,len);
    return rd_status;
}



void pango_fun::dma_read_test()
{
    dma_oper->current_len=(4>>2);
    memset(dma_oper->data.read_buf,0,DMA_MAX_PACKET_SIZE);
    ioctl(dev_fd,PCI_MAP_ADDR_CMD,dma_oper);
    ioctl(dev_fd,PCI_DMA_WRITE_CMD,dma_oper);//write data to kernel from device
    //nano_delay(8000);
    ioctl(dev_fd,PCI_READ_FROM_KERNEL_CMD,dma_oper);//read data to usr from kernel
    ioctl(dev_fd,PCI_UMAP_ADDR_CMD,dma_oper);
    for(int i=0;i<=3;i++)
    {
        qDebug()<<"data is "<<dma_oper->data.read_buf[i];
    }
}




COMMAND_OPERATION cmd_op ={};

//get device
int pango_fun::getDevice()
{
    ns_sleep.tv_nsec = 1;//1ns
    dev_fd  =   open(PCIE_DRIVER_FILE_PATH, O_RDWR);
    if(dev_fd < 0)
    {
        return  -1  ;
    }
    else
    {
        //read(dev_fd, &command_operation, sizeof(command_operation));
        ioctl(dev_fd,PCI_READ_DATA_CMD,&command_operation);
        // 打印 vendor ID

        qDebug().nospace() << "vendor_id:"
                           << QStringLiteral("0x%1").arg(
                                  command_operation.get_pci_dev_info.vendor_id,
                                  4, 16, QLatin1Char('0'));

        // 打印 device ID
        qDebug().nospace() << "device_id:"
                           << QStringLiteral("0x%1").arg(
                                  command_operation.get_pci_dev_info.device_id,
                                  4, 16, QLatin1Char('0'));

        //打印 link speed and width
        qDebug().nospace()
            << "link up: gen"
            << command_operation.get_pci_dev_info.link_speed
            << " width x"
            << command_operation.get_pci_dev_info.link_width;
        //打印 max payload
        qDebug().nospace()
            << "max payload:"
            << command_operation.get_pci_dev_info.mps;


        dma_set_config();
        ioctl(dev_fd, PCI_MAP_BAR0_CMD, &cmd_op);						/*获取 bar0 地址映射 */
        mem_pio_addr = map_mem(cmd_op.get_pci_dev_info.bar[0].bar_base , BAR0_MAX , &map_dev_mem);


    }
    return  dev_fd  ;

}

//config dma
void pango_fun::dma_config_init(unsigned int len)
{
     dma_oper->current_len=len>>2   ;   //DW
     dma_oper->offset_addr = 0      ;
}

// 线程接收
// 与 rk_pcie_video_ 对齐：始终对 img_buf 基址调用 read()，
// 驱动/DMA 根据 FPGA 帧序号将数据写入正确的槽位偏移。
// send_wr_done 传递 FPGA 帧序号 (0-3)，GUI 据此定位数据。
void pango_fun::run()
{

    while(!stopped)
    {
        if (dev_fd < 0)
        {
            QThread::msleep(10);
            continue;
        }

        rd_len = dma_read(1280*720*2, img_buf);

        if (pango_frame_status_ready(rd_len))
        {
            index = pango_frame_status_index(rd_len);
            if(index!=index_old)
            {
                emit send_wr_done(index);
                index_old = index;
            }

        }
        rd_len = 0;

    }


}

// ============================================================
// I2C 视频源切换（/dev/i2c-0 → FPGA 视频复用器 0x66）
// ============================================================

// I2C slave 地址和寄存器定义
static constexpr int  kI2cAddr       = 0x66;
static constexpr int  kI2cRegSelect  = 0x01;  // 写: 选择视频源
static constexpr int  kI2cRegStatus  = 0x02;  // 读: 当前状态
static constexpr char kI2cDevice[]   = "/dev/i2c-0";

bool pango_fun::i2c_open() {
    if (i2c_fd >= 0) return true; // 已打开
    i2c_fd = open(kI2cDevice, O_RDWR);
    if (i2c_fd < 0) {
        qDebug() << "[I2C] open" << kI2cDevice << "failed:" << strerror(errno);
        return false;
    }
    // 设置从设备地址 (使用 ioctl 方式, 兼容性最好)
    if (ioctl(i2c_fd, 0x0703 /*I2C_SLAVE*/, kI2cAddr) < 0) {
        qDebug() << "[I2C] ioctl I2C_SLAVE failed:" << strerror(errno);
        close(i2c_fd); i2c_fd = -1;
        return false;
    }
    qDebug() << "[I2C] opened" << kI2cDevice << "addr=0x66";
    return true;
}

void pango_fun::i2c_close() {
    if (i2c_fd >= 0) { close(i2c_fd); i2c_fd = -1; }
}

bool pango_fun::i2c_switch_to_camera() {
    if (!i2c_open()) return false;
    // 写 0x00 到寄存器 0x01 → 切换到摄像头
    unsigned char buf[2] = { (unsigned char)kI2cRegSelect, 0x00 };
    if (write(i2c_fd, buf, 2) != 2) {
        qDebug() << "[I2C] switch to camera failed:" << strerror(errno);
        return false;
    }
    qDebug() << "[I2C] switched to CAMERA";
    return true;
}

bool pango_fun::i2c_switch_to_hdmi() {
    if (!i2c_open()) return false;
    // 写 0x01 到寄存器 0x01 → 切换到 HDMI
    unsigned char buf[2] = { (unsigned char)kI2cRegSelect, 0x01 };
    if (write(i2c_fd, buf, 2) != 2) {
        qDebug() << "[I2C] switch to HDMI failed:" << strerror(errno);
        return false;
    }
    qDebug() << "[I2C] switched to HDMI";
    return true;
}

int pango_fun::i2c_query_source() {
    if (!i2c_open()) return -1;
    // 写寄存器地址 0x02, 再读 1 字节
    unsigned char reg = (unsigned char)kI2cRegStatus;
    if (write(i2c_fd, &reg, 1) != 1) {
        qDebug() << "[I2C] query source write failed:" << strerror(errno);
        return -1;
    }
    unsigned char val = 0;
    if (read(i2c_fd, &val, 1) != 1) {
        qDebug() << "[I2C] query source read failed:" << strerror(errno);
        return -1;
    }
    int src = (int)val;
    qDebug() << "[I2C] current source:" << src << (src == 0 ? "(CAMERA)" : src == 1 ? "(HDMI)" : "(?)");
    return (src == 0 || src == 1) ? src : -1;
}

const char* pango_fun::i2c_source_name() {
    int src = i2c_query_source();
    if (src == 0) return u8"摄像头";
    if (src == 1) return "HDMI";
    return u8"未知";
}

// ============================================================
// I2C FPGA 预处理控制
// ============================================================

bool pango_fun::i2c_write_reg(int reg, bool on) {
    if (!i2c_open()) return false;
    unsigned char buf[2] = { (unsigned char)reg, on ? (unsigned char)0x01 : (unsigned char)0x00 };
    if (write(i2c_fd, buf, 2) != 2) {
        fprintf(stderr, "[I2C] write reg 0x%02x failed: %s\n", reg, strerror(errno));
        return false;
    }
    printf("[I2C] reg 0x%02x = %s\n", reg, on ? "ON" : "OFF");
    return true;
}

int pango_fun::i2c_read_reg(int reg) {
    if (!i2c_open()) return -1;
    unsigned char r = (unsigned char)reg;
    if (write(i2c_fd, &r, 1) != 1) { fprintf(stderr, "[I2C] read reg write failed\n"); return -1; }
    unsigned char val = 0;
    if (read(i2c_fd, &val, 1) != 1) { fprintf(stderr, "[I2C] read reg read failed\n"); return -1; }
    return (int)val;
}

bool pango_fun::i2c_set_preprocess(bool on)   { return i2c_write_reg(0x03, on); }
bool pango_fun::i2c_set_plate_overlay(bool on) { return i2c_write_reg(0x04, on); }
bool pango_fun::i2c_set_white_balance(bool on) { return i2c_write_reg(0x05, on); }
bool pango_fun::i2c_set_dehaze(bool on)        { return i2c_write_reg(0x06, on); }
