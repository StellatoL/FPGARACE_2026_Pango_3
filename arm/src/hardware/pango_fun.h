#ifndef PANGO_FUN_H
#define PANGO_FUN_H



#include <QtWidgets>

#define BMP_HEADER_SIZE 54

#define PCIE_DRIVER_FILE_PATH 			"/dev/pango_pci_driver"			/* pcie驱动文件目录 */
#define MEM_FILE_PATH 					"/dev/mem"						/* mem驱动文件目录 */
#define VEISION 						"Pango PCIe Test v1.0"			/* 软件测试版本信息 */

#define TYPE 							'S'
#define PCI_READ_DATA_CMD    			_IOWR(TYPE, 0, int)				/* 读数据指令 */
#define PCI_WRITE_DATA_CMD    			_IOWR(TYPE, 1, int)				/* 写数据指令 */
#define PCI_MAP_ADDR_CMD    			_IOWR(TYPE, 2, int)				/* DMA总线地址映射 */
#define PCI_WRITE_TO_KERNEL_CMD    		_IOWR(TYPE, 3, int)				/* 写内核数据操作 */
#define PCI_DMA_READ_CMD    			_IOWR(TYPE, 4, int)				/* DMA读操作 */
#define PCI_DMA_WRITE_CMD    			_IOWR(TYPE, 5, int)				/* DMA写操作 */
#define PCI_READ_FROM_KERNEL_CMD    	_IOWR(TYPE, 6, int)				/* 读取内核数据操作 */
#define PCI_UMAP_ADDR_CMD    			_IOWR(TYPE, 7, int)				/* 释放映射地址 */
#define PCI_PERFORMANCE_START_CMD		_IOWR(TYPE, 8, int)				/* 性能测试开始操作 */
#define PCI_PERFORMANCE_END_CMD			_IOWR(TYPE, 9, int)				/* 性能测试结束操作 */
#define PCI_MAP_BAR0_CMD				_IOWR(TYPE,10, int)							/* 获取bar0信息*/
#define PCI_SET_CONFIG					_IOWR(TYPE,11, int)							/* 配置DMA控制*/


#define H_IMAGE 540
#define W_IMAGE 960
#define COLOR_NUM 3
#define ONE_DMA_RD_BYTES_LEN 1920    //
#define ONE_FRAME_NEED_TIMES (H_IMAGE*H_IMAGE*2)/ONE_DMA_RD_BYTES_LEN

//#include <mainwindow.h>

#define MAX_BLOCK_SIZE   				1024							/* 位流分块缓存区最大值 */
#define DMA_MAX_PACKET_SIZE   			4096							/* DMA包最大值 */
#define DMA_MIN_PACKET_SIZE   			4								/* DMA包最小值 */
//open pcie devicer
int open_pci_driver(void)   ;





//--------------struct---------------------------
typedef struct _BAR_INFO_
{
    unsigned long bar_base;
    unsigned long bar_len;
}BAR_BASE_INFO;

typedef struct _CAP_INFO_
{
    unsigned char flag;
    unsigned char id;
    unsigned char addr_offset;
    unsigned char next_offset;
}CAP_INFO;

typedef struct _CAP_LIST_
{
    unsigned char cap_status;
    unsigned char cap_error;
    CAP_INFO cap_buf[256];
}CAP_LIST;

typedef struct _PCI_INFO_
{
    unsigned int vendor_id;
    unsigned int device_id;
    unsigned int cmd_reg;
    unsigned int status_reg;
    unsigned int revision_id;
    unsigned int class_prog;
    unsigned int class_device;
    BAR_BASE_INFO bar[6];
    unsigned int min_gnt;
    unsigned int max_lat;
    unsigned int link_speed;
    unsigned int link_width;
    unsigned int mps;
    unsigned int mrrs;
    unsigned int data[1024];
}PCI_DEVICE_INFO;

typedef struct _LOAD_DATA_
{
    unsigned int num_words;
    unsigned int block_words[MAX_BLOCK_SIZE];
}LOAD_DATA_INFO;

typedef struct _PCI_LOAD_
{
    unsigned char link_status;
    unsigned int  crc;
    unsigned char axi_direction;
    unsigned char load_status;
    unsigned int total_num_words;
    LOAD_DATA_INFO data_block;
}PCI_LOAD_INFO;



typedef struct _COMMAND_
{
    unsigned char w_r;
    unsigned char step;
    unsigned int addr;
    unsigned int data;
    unsigned int cnt;
    unsigned int delay;
    PCI_DEVICE_INFO get_pci_dev_info;
    CAP_LIST cap_info;
    PCI_LOAD_INFO load_info;
}COMMAND_OPERATION;


typedef struct _DMA_DATA_
{
    unsigned char read_buf[DMA_MAX_PACKET_SIZE];
    unsigned char write_buf[DMA_MAX_PACKET_SIZE];
}DMA_DATA;

typedef struct _DMA_OPERATION_
{
    unsigned int current_len;
    unsigned int offset_addr;
    unsigned int cmd;
    DMA_DATA data;
}DMA_OPERATION;

typedef struct _DEV_MEM_
{
    off_t offset;
    size_t len;
    void *vaddr;
} DEV_MEM;

typedef struct _PERFORMANCE_OPERATION_
{
    unsigned int current_len;
    unsigned int cmd;
    unsigned char cmp_flag;
}PERFORMANCE_OPERATION;







class pango_fun : public QThread
{
    Q_OBJECT

public:
    pango_fun();
    int getDevice(void);
    void dma_config_init(unsigned int len);
    int dma_read(unsigned int len,unsigned char* img_buf);
    void dma_write_test();
    void dma_write_data(unsigned char cmd_in);
    void dma_read_test();
    void write_dw_data(uchar* send_buf);
    void dma_map(bool flag);
    void dma_set_config(void);
    DMA_OPERATION dma_operation;
    DMA_OPERATION *dma_oper=&dma_operation;
    COMMAND_OPERATION command_operation;
    void pio_write(unsigned int write_data,int pio_bse_addr,int user_reg_addr);
    void pio_read(unsigned char* read_buf,int pio_addr);

    // === I2C 视频源切换 ===
    // 通过 /dev/i2c-0 控制 FPGA 端的视频复用器 (地址 0x66)
    // 寄存器 0x01: 0x00=摄像头, 0x01=HDMI
    // 寄存器 0x02: 只读, 当前源 (0x00/0x01)
    bool i2c_open();
    void i2c_close();
    bool i2c_switch_to_camera();    // → 摄像头
    bool i2c_switch_to_hdmi();      // → HDMI
    int  i2c_query_source();        // 返回 0=摄像头, 1=HDMI, -1=错误
    const char* i2c_source_name();  // 返回 "摄像头" / "HDMI" / "未知"

    // === I2C FPGA 预处理控制 ===
    // 寄存器 0x03: FPGA 整图预处理开关
    // 寄存器 0x04: FPGA 车牌定位+画框
    // 寄存器 0x05: 白平衡
    // 寄存器 0x06: 暗通道去雾
    // 写 0x01=开启, 0x00=关闭
    bool i2c_write_reg(int reg, bool on);      // 通用单寄存器写
    int  i2c_read_reg(int reg);                // 通用单寄存器读 (-1=错误)
    bool i2c_set_preprocess(bool on);          // → 0x03
    bool i2c_set_plate_overlay(bool on);       // → 0x04
    bool i2c_set_white_balance(bool on);       // → 0x05
    bool i2c_set_dehaze(bool on);              // → 0x06

    // ARM 侧接收缓冲：4 帧环形缓冲，与 FPGA 的 4 槽位一一对应。
    // 驱动/DMA 根据 FPGA 帧序号将数据固定写入 img_buf + index*IMG_SIZE，
    // 因此 ARM 侧不轮转，始终对基址 read()，由驱动决定数据落点。
    unsigned char *img_buf   = new uchar[1280*720*2*4];
    void *map_mem(unsigned long addr, unsigned int size, DEV_MEM *map_mem);
    bool stopped = true ;
    void *mem_pio_addr =NULL;//bar0 vir adddr
    DEV_MEM map_dev_mem = {0, 0, NULL};
    int dev_fd;
    int rd_len=0 ;
    int index=0;
    int index_old=0;
protected:
    void run(void);

private:
    struct timespec ns_sleep;
    int speedRuning;
    int i2c_fd = -1; // /dev/i2c-0 文件描述符


Q_SIGNALS:
    void send_wr_done(int index);
};






#endif // PANGO_FUN_H
