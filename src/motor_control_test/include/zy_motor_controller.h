#include <stdio.h>

#include "pci_errno.h"
#include "pci_zecm.h"
#include "pci_dbg.h"

#include <string>
#include <memory>

#include "stdlib.h"
#include <chrono>
#include <functional>
#include <sstream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <iostream>
#include "motor_controller.h"
#include <rclcpp/rclcpp.hpp>

#ifdef VXWORKS
#define main SYNC_CONTROL_TEST
#endif

#define RETURN_IF_FAILED(expr)      { int ret = (expr); if (ret != 0) { _ERR_(#expr " failed: err=%d", ret); return -1; } else { _INF_(#expr " succeeded"); } }

/**************************motor config*************************/

#define NUM_SERVOS  2           // 总电机数

typedef  uint32_t   U32;
typedef  uint16_t   U16;
typedef  uint8_t    U8;
typedef  int32_t    I32;
typedef  int16_t    I16;

enum {
    OP_MODE_CSP = 8,
    OP_MODE_CSV = 9,
    OP_MODE_CST = 10,
    OP_MODE_CSV_polygon = 11,
};

static U32 g_op_mode = OP_MODE_CSP;
static U32 g_run_task = 1;

////////////////////////////////////////////////////////////////////////////////
// 过程映像  控制字 联合体union 共用内存
typedef union {
    struct {
        U16 switch_on: 1;
        U16 enable_voltage: 1;
        U16 quick_stop: 1;
        U16 enable_operation: 1;
        U16 pad0: 3;
        U16 fault_reset: 1;
        U16 halt: 1;
        U16 pad1: 1;
        U16 pad2: 6;
    } b;
    U16 v;
} servo_ctrl_word;

//状态字  联合体union 共用内存
typedef union {
    struct {
        U16 ready_to_switch_on: 1;
        U16 switched_on: 1;
        U16 operation_enabled: 1;
        U16 fault: 1;
        U16 voltage_enabled: 1;
        U16 quick_stop: 1;
        U16 switch_on_disabled: 1;
        U16 warning: 1;
        U16 pad0: 1;
        U16 remote: 1;
        U16 target_reached: 1;
        U16 internal_limit_active: 1;
        U16 pad1: 4;
    } b;
    U16 v;
} servo_stat_word;

typedef struct {
    uint8_t             *mode;
    servo_ctrl_word     *ctrl;
    I32                 *p;
    I32                 *v;
    U32                 *t_max;
} servo_ctrl;

typedef struct {
    uint8_t             *mode;
    servo_stat_word     *stat;
    I32                 *p;
    I32                 *v;
    U32                 *t;
} servo_stat;

typedef struct {
    servo_ctrl servos[NUM_SERVOS];
} proc_img_o;

typedef struct {
    servo_stat servos[NUM_SERVOS];
} proc_img_i;

typedef struct {
    int32_t             po_mode_offset;
    int32_t             po_ctrl_word_offset;
    int32_t             po_p_offset;
    int32_t             po_v_offset;
    int32_t             po_t_max_offset;
    int32_t             pi_mode_offset;
    int32_t             pi_stat_offset;
    int32_t             pi_p_offset;
    int32_t             pi_v_offset;
    int32_t             pi_t_offset;
} pi_offset_config;

static pi_offset_config  __pi_config[] =
{
    {
        .po_mode_offset = 2,
        .po_ctrl_word_offset = 0,
        .po_p_offset = 5,
        .po_v_offset = 11,
        .po_t_max_offset = 3,
        .pi_mode_offset = 4,
        .pi_stat_offset = 2,
        .pi_p_offset = 5,
        .pi_v_offset = 9,
        .pi_t_offset = 13
    },
    {
        .po_mode_offset = 27,
        .po_ctrl_word_offset = 25 ,
        .po_p_offset = 30,
        .po_v_offset = 36,
        .po_t_max_offset = 28,
        .pi_mode_offset = 29,
        .pi_stat_offset = 27,
        .pi_p_offset = 30,
        .pi_v_offset = 34,
        .pi_t_offset = 38
    },
	{
		.po_mode_offset = -1,
		.po_ctrl_word_offset = 20 + 0,
		.po_p_offset = 20 + 2,
		.pi_mode_offset = -1,
		.pi_stat_offset = 38 + 2,
		.pi_p_offset = 38 + 5,
	},
};

typedef struct {
    servo_ctrl_word sc;    // 新控制字
    servo_stat_word ss;    // 旧状态字
    int run_delay;         // 延时启动
    int fault_reset_delay; // 延时再错误恢复
    float zero_p;          // 零点位置
    float offset_p;        // 偏移输出位置
    float cmd_v;           //输出速度命令
    I32 out_p;             // 输出位置
    I32 out_v;             // 输出转速
    I16 out_t;             // 输出扭矩
    bool reverse;          // 当前方向
}servos_t;

static servos_t __servos[NUM_SERVOS] = {0};


typedef struct { 
    uint16_t slave; 
    uint16_t index; 
    uint8_t subindex; 
    uint32_t len; 
    uint32_t val;
} EcatSdoItem;

//运动加载参数
typedef struct { 
    float vel_max ;  //最大速度
    float acc_max ;  //最大加速度 mm/s^2 用线速度表示
    float jert_acc_max ; //最大加加速或减加速  mm/s^3
    float dcc_max ; //最大减速度 mm/s^2  用线速度表示 
    float jert_dcc_max ; //最大减减速或加减速 mm/s^3   
    float followError_speed ; //跟随误差
    float sync_time ;  //同步周期时间 单位 秒s
    bool IsEmpty() const {
    return vel_max == 0 && acc_max == 0.0 && jert_acc_max == 0.0 && 
    dcc_max == 0.0 && jert_dcc_max == 0.0 && followError_speed == 0.0 && 
    sync_time == 0.0;
    }
} OptionParam;


#ifdef _WIN32
#include <windows.h>
void usleep(int64_t usec){
    HANDLE timer;
    LARGE_INTEGER ft;
    ft.QuadPart = -(10*usec);
    timer = CreateWaitableTimer(NULL, TRUE, NULL);
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
    WaitForSingleObject(timer, INFINITE);
    CloseHandle(timer);
}
#else
#include <unistd.h>
#endif


class zy_motor_controller : public motor_controller
{
    public:
    //构造函数 输入卡号索引index(为适配多张卡，输入卡的旋转开关号）,端口号channel（NET0为0，NET1为1），
    //ENI文件地址ENI_path_ecat,运动参数OptionParam的加载
    //PCIe-2E channel只有一个通道,PCIe-4E有两个通道
    zy_motor_controller(std::int16_t index_ecat, std::int16_t ecat_channel,
            std::string ENI_path_ecat,OptionParam option,rclcpp::NodeOptions node_options);
    
    //初始配置，包括打开开设备,加载ENI文件并启动ecat总线运行
    //切换主站控制状态，COE参数配置，映射地址配置，使能PI镜像缓存跟总线进行数据交换
    int config();

    //目标速度赋值
    void test_set_vel_target(float val0,float val1);

    //速度同步周期模式， 指定轴编号及运动参数
    int sync_control();

    private:
    //coe参数写入
    int32_t ecat_coe_download(ECAT_HANDLE hHandle);

    //控制字操作
    static bool motor_control(int axis, servo_stat_word *s0, 
            const servo_stat_word *s1, servo_ctrl_word *c, bool serverOn, bool quickStop);

    //PI模式的输入数据获取
    proc_img_o *pi_get_output(const pi_offset_config *config, void *output);

    //输入映像地址 绑定数据结构类型
    proc_img_i *pi_get_input(const pi_offset_config *config, const void *output);

    uint32_t index = 0;     //卡号索引index
    uint32_t channel = 0;   //端口号channel（NET0为0，NET1为1）
    ECAT_HANDLE ecat_dev = nullptr;   //ecat卡的引用
    std::string ENI_path;   //ENI文件地址
    std::string param_path_motor;  //运动参数路径
    
    uint8_t *input_data;    //ecat输入数据映射指针
    uint8_t *output_data;   //ecat输出数据映射指针

    proc_img_o	__pi_o;     //过程映像 输出数据结构类型
    proc_img_i	__pi_i;     //过程映像 输入数据结构类型

    
    
};

