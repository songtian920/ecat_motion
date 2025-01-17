#include <stdio.h>

#include "pci_errno.h"
#include "pci_zecm.h"
#include "pci_dbg.h"

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

static U32 g_op_mode = OP_MODE_CSV;
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
    float offset_v;        //偏移输出速度
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



/************************ motor config end ***************************/
static bool motor_control(int axis, servo_stat_word *s0, const servo_stat_word *s1, servo_ctrl_word *c, bool show)
{
    servos_t *servo = &__servos[axis];
    do {
        c->v = 0;
        c->b.enable_voltage = 1;
        c->b.quick_stop = 1;
        c->b.enable_operation = s1->b.switched_on || s1->b.operation_enabled;
        c->b.switch_on = s1->b.ready_to_switch_on;

        if (s1->b.fault && !servo->fault_reset_delay) 
        {
            // 延时1秒再尝试错误恢复，防止伺服在错误和恢复中反复快速切换损坏电路
            _DBG_("axis %d: delay fault reset, ctrl:%d, stat:%x", axis, c->v, (int)s1->v);
            servo->fault_reset_delay = 2000; // 2s(即2000个周期)
            servo->run_delay = 2000; // 当错误恢复成功伺服出力后，延时2s再开始加速(根据抱闸动作时间适当调整)
            break;
        }
        // 伺服发生错误，且在延时等待处理中
        if (servo->fault_reset_delay) 
        {
            servo->fault_reset_delay--;
            if (!servo->fault_reset_delay) 
            {
                // 延时减到0，尝试进行错误恢复
                _DBG_("axis %d: start fault reset", axis);
                c->b.fault_reset = 1;
            }
            break;
        }

    } while (0);
    
    s0->v = s1->v;

    return s1->b.operation_enabled;
}

proc_img_o	__pi_o;
proc_img_i	__pi_i;

static proc_img_o * pi_get_output(const pi_offset_config *config, void *output)
{
	for (uint32_t i = 0; i < NUM_SERVOS; i++)
	{
		servo_ctrl *ctrl = &__pi_o.servos[i];
		const pi_offset_config *curr = &config[i];
        //c指针的偏移操作 
		ctrl->mode = (uint8_t *)((curr->po_mode_offset != -1) ? (char *)output + curr->po_mode_offset : 0);
		ctrl->ctrl = (servo_ctrl_word *)((curr->po_ctrl_word_offset != -1) ? (char *)output + curr->po_ctrl_word_offset : 0);
		ctrl->p = (I32 *)((curr->po_p_offset != -1) ? (char *)output + curr->po_p_offset : 0);
        ctrl->v = (I32 *)((curr->po_v_offset != -1) ? (char *)output + curr->po_v_offset : 0);
        ctrl->t_max = (U32 *)((curr->po_t_max_offset != -1) ? (char *)output + curr->po_t_max_offset : 0);
	}
    return &__pi_o;
}

static proc_img_i * pi_get_input(const pi_offset_config *config, const void *output)
{
	for (uint32_t i = 0; i < NUM_SERVOS; i++)
	{
		servo_stat *stat = &__pi_i.servos[i];
		const pi_offset_config *curr = &config[i];
        //c指针的偏移操作 
		stat->mode = (uint8_t *)((curr->pi_mode_offset != -1) ? (char *)output + curr->pi_mode_offset : 0);
		stat->stat = (servo_stat_word *)((curr->pi_stat_offset != -1) ? (char *)output + curr->pi_stat_offset : 0);
		stat->p = (I32 *)((curr->pi_p_offset != -1) ? (char *)output + curr->pi_p_offset : 0);
        stat->v = (I32 *)((curr->pi_v_offset != -1) ? (char *)output + curr->pi_v_offset : 0);
        stat->t = (U32 *)((curr->pi_t_offset != -1) ? (char *)output + curr->pi_t_offset : 0);
	}
    return &__pi_i;
}

static int motor_mode_init(ECAT_HANDLE ecat_dev,uint32_t op_mode)
{
    int ret;
    uint32_t mode = 0;
    uint32_t len;
    int fail_try = 10;
    int i;

    for(i = 0; i < NUM_SERVOS && fail_try > 0; )
    {
        len = 1;
        ret=EcatCoeSDODownload(ecat_dev,i,0x6060,0x0,1,&op_mode,100);
        usleep(1000*100);
        
        ret=EcatCoeSDOUpload(ecat_dev,i,0x6060,0x0,&len,&mode,100);
        if(mode != op_mode)
        {
            _INF_("request is: %d, curr is: %d", op_mode,mode);    
            usleep(1000*100);
            fail_try --;
            ret = -1;
            continue;
        }    
        
        fail_try = 10;
        i++;
    }

    return ret;
}

//coe参数写入
int32_t ecat_coe_download(ECAT_HANDLE hHandle)
{ 
    uint32_t i, c,rd_len;  //结构数据 行，列，数据长度
    uint32_t rd_val = 0;
    EcatSdoItem items[2][8] ={
        //0号slave参数
        {
            { 0, 0x6091, 0x01, 4, 1 },  //电子齿轮比 分子
            { 0, 0x6091, 0x02, 4, 1 },  //电子齿轮比 分母
            { 0, 0x6092, 0x01, 4, 131072 },  //进给量 分子
            { 0, 0x6092, 0x02, 4, 1 },  //进给量 分母
            { 0, 0x605A, 0x00, 2, 2 },  //快速停机方式，值2，按6085h减速停机，停机后保持自由
            { 0, 0x605D, 0x00, 2, 2 },  //暂停方式，值2，按6085h减速停机，停机后保持位置锁定
            { 0, 0x6085, 0x00, 4, 131072 },  //快速停止减速度
            { 0, 0x607E, 0x00, 1, 0 },  //指令极性 0为正， 224为负,首次设定需要伺服断电重启
        },
        //1号slave参数
        {
            { 1, 0x6091, 0x01, 4, 1 },  //电子齿轮比 分子
            { 1, 0x6091, 0x02, 4, 1 },  //电子齿轮比 分母
            { 1, 0x6092, 0x01, 4, 131072 },  //进给量 分子
            { 1, 0x6092, 0x02, 4, 1 },  //进给量 分母
            { 1, 0x605A, 0x00, 2, 2 },  //快速停机方式，值2，按6085h减速停机，停机后保持自由
            { 1, 0x605D, 0x00, 2, 2 },  //暂停方式，值2，按6085h减速停机，停机后保持位置锁定
            { 1, 0x6085, 0x00, 4, 131072 },  //快速停止减速度
            { 1, 0x607E, 0x00, 1, 0 },  //指令极性 0为正， 224为负，首次设定需要伺服断电重启
        },
    }
    ;

    for (i = 0; i < 2; i++)
    {
        for(c = 0; c < 8;c++)
        {
            EcatSdoItem *item = &items[i][c];
            EXIT_IF_FAIL(EcatCoeSDODownload(hHandle, item->slave, item->index, item->subindex, item->len, &item->val, 5000));

            rd_len = item->len;
            EXIT_IF_FAIL(EcatCoeSDOUpload(hHandle, item->slave, item->index, item->subindex, &rd_len, &rd_val, 5000));

            if (rd_val != item->val)
            {
                _ERR_("need val:%u, but is:%u", item->val, rd_val); 
                //return -1;
            }
        }
    }
    return 0;
}

//速度二次多项式平滑加减速
int vel_quad_polygon()
{
    //先判断目标值是否超出范围
    printf("usage: ./program encoder_id chnnal\r\n");
    return 0;
}


int main(int argc, char* argv[])
{
    char index_buff[10];
    uint32_t index = 0;
    uint64_t loops = 0;
    uint32_t channel = 0;
    ECAT_HANDLE ecat_dev;

    if (argc != 3)
    {
        printf("usage: ./program encoder_id chnnal\r\n");
        index=1;
        channel=0;
        //return 0;
    }
    else
    {
        index = atoi(argv[1]);
        channel = atoi(argv[2]);
        
    }

    if(channel > 1) channel = 1;
  
    //PCIe-2E 只有一个通道,PCIe-4E有两个通道
    //根据通道号打开设备，获得句柄
    RETURN_IF_FAILED(EcatOpen(&ecat_dev, BOARD_ALIAS(index_buff, index), channel));
    //EcatBusRun() 启动并传入ENI文件
    RETURN_IF_FAILED(EcatBusRun(ecat_dev, "hcfa_D3E_1602_1A02_ENI.xml"));

    // EcatRequestMasterState请求切换状态
    /**
        @brief 请求切换主站状态
        @param hHandle ECAT 控制句柄
        @param u8State 切换到主站的状态，参考 EcatStateP、EcatStateO 等枚举值
        @return 返回 ECAT_S_OK(0)表示成功，非 0 表示失败
    */
    uint8_t state;
    RETURN_IF_FAILED(EcatRequestMasterState(ecat_dev, EcatStateO));
    for (uint32_t i = 0; ; i++)
    {
        //EcatGetMasterReqState
        /**
            @brief  获取当前主站请求的状态
            @param  hHandle               ECAT控制句柄
            @param[OUT]  pu8State         主站当前的状态，@see EcatState
            @return  返回ECAT_S_OK(0)表示成功，非0表示失败
        */
        RETURN_IF_FAILED(EcatGetMasterState(ecat_dev, &state));
        _INF_("request is: 8, curr is: %d", state);
        usleep(500000);
        if (state == EcatStateO) break;
    }

    //电子齿轮比 等参数写入
    ecat_coe_download(ecat_dev);
    
    // 清空并预计填入数据
    //将输入输出数据绑定本地指针
    uint8_t *input_data;
    uint8_t *output_data;
    //EcatPINMap申请本地输入输出缓冲
    /**
        @brief  获取镜像缓存的内存映射地址，可用于高速访问，非线程安全
        @param  hHandle               ECAT控制句柄
        @param  u32PiArea             镜像的区域索引，支持PI_AREA_LOCAL_OUTPUT、PI_AREA_LOCAL_INPUT
        @param[OUT]  ppvData          返回PI镜像的地址
        @return  返回ECAT_S_OK(0)表示成功，非0表示失败
    */
    RETURN_IF_FAILED(EcatPINMap(ecat_dev, PI_AREA_LOCAL_INPUT, (void **)&input_data));
    RETURN_IF_FAILED(EcatPINMap(ecat_dev, PI_AREA_LOCAL_OUTPUT, (void **)&output_data));
    printf("input_data:   %p\r\n", input_data);
    printf("output_data:  %p\r\n", output_data);

    for (uint32_t i = 0; i < 2; i++)
    {
        //过程数据队列入队
        /**
            @brief 拷贝镜像输出缓存到输出队列的尾部，队列的有效个数加 1，非线程安全
            @param hHandle ECAT 控制句柄
            @param bReqClearQueue true: 快速输出当前帧,并在输出帧后清空之前的队列数据 false：无特殊操作
            @param u32TimeOutMs 超时时间，以毫秒为单位
            @return 返回
        */    
        EcatPIOutputQueuePush(ecat_dev, false, 100);
    }

    //使能PI镜像缓存跟总线进行数据交换
    RETURN_IF_FAILED(EcatPIEnable(ecat_dev));

    //计算参数
    float vel_max = 200000.0f;  //最大速度
    float acc_max = 100.0f;  //最大加速度 mm/s^2 用线速度表示
    float jert_acc_max = 100.0f; //最大加加速或减加速  mm/s^3
    float dcc_max = 100.0f; //最大减速度 mm/s^2  用线速度表示 
    float jert_dcc_max = 100.0f; //最大减减速或加减速 mm/s^3   
    float followError_speed =  1000.0; //跟随误差
    float sync_time = 0.001;  //同步周期时间 单位 秒s

    float vel_target[] = {131072.0f,131072.0f}; //目标速度 mm/s
    float vel_targey_last[] = {0.0,0.0}; //目标速度上一个旧值
    
    

    //Todo 加入看门口

    while (1)
    {
        /**
         * EcatPIInputQueuePop
            @brief 拷贝输入队列的头部到镜像输入缓存，队列的有效个数减 1，非线程安全
            @param hHandle ECAT 控制句柄
            @param bReqClearRemain true: 获取队列中最新的数据，然后清空残留的队列数据 false：无特殊操作
            @param u32TimeOutMs 超时时间，以毫秒为单位
            @return 返回 ECAT_S_OK(0)表示成功，非 0 表示失败
        */
        while (EcatPIInputQueuePop(ecat_dev, false, 100) != 0);

        //输入内存块input_data，部分有效地址绑定绑定到输入映像上 通过pi指针获取返回状态数据
        proc_img_i *pi = pi_get_input(__pi_config, input_data);
        //输出内存块output_data，部分有效地址绑定到输出映像上，通过po指针输出数据
        proc_img_o *po = pi_get_output(__pi_config, output_data);
        
        loops++;
        for (int i = 0; i < NUM_SERVOS; i++)
        {
            servos_t *servo = &__servos[i];
            
            if (po->servos[i].mode) 
            {
                //模式指针不为空，则赋值模式
                *po->servos[i].mode = g_op_mode;
            }
                
            //控制字逻辑 
            bool run = motor_control(i, &servo->ss, pi->servos[i].stat, &servo->sc, false);

            if (loops % 1000 == 0) 
            {
                _INF_("servo[%d]: p=%10d, ctrl:%08x stat=%08x", i, *pi->servos[i].p, po->servos[i].ctrl->v, pi->servos[i].stat->v);
            }

            // 伺服未出力，或已出力但延时未结束，应停止转动
            if (!run || servo->run_delay != 0) {
                servo->out_p = *pi->servos[i].p;
                servo->zero_p = (float)servo->out_p;
                servo->offset_p = 0;  //位置偏移值清零
                servo->offset_v = 0;  //速度偏移值清零
                servo->out_v = 0;
                servo->out_t = 0;
                servo->reverse = false;
            }

            // 伺服已出力，延时计数递减
            if (run && servo->run_delay)
                servo->run_delay--;

            // 伺服已出力，延时结束，进行正常控制
            if (run && !servo->run_delay) {

                switch (g_op_mode)
                {
                    case OP_MODE_CSP:
                        // 不同轴单位可能不一样，下表定义各轴位置范围及增量
                        I32 range_min[] = { -131072, -10000, -500 };
                        I32 range_max[] = {  131072, 10000, 1500 };
                        float step_size[] = { 5.4f, 5.4f, 5.2f };
                        if (servo->offset_p < range_min[i]) servo->reverse = false;
                        if (servo->offset_p > range_max[i]) servo->reverse = true;
                        servo->offset_p += (servo->reverse ? -1 : 1) * step_size[i];
                        servo->out_p = (I32)(servo->zero_p + servo->offset_p);
                        break;

                    case OP_MODE_CSV:
                        
                        //中间变量
                        float vel_pre = 0.0f;  //速度预计算值

                        //Todo先判断当前速度和命令速度滞后性是否超出
                        if(abs(*pi->servos[i].v-servo->out_v)>followError_speed)
                        {
                            printf("%s\n", "当前速度跟随命令速度超出范围值！");
                        }

                        //判读目标速度是在当前命令速度上,还是当前速度下
                        if(servo->out_v<vel_target[i])  
                        {
                            //先判断当前速度是否接近目标速度
                            if(servo->out_v<vel_target[i]-acc_max)
                            {   
                                printf("%s\n", "当前命令速度小于目标速度");
                                //速度输出不等于 目标速度
                                if(servo->offset_v!=vel_target[i])
                                {
                                    printf("%s\n", "加速中");
                                    servo->offset_v += acc_max; //速度偏移值加速
                                    servo->out_v= (U32)servo->offset_v;  //速度指令输出
                                }
                                else
                                {
                                    printf("%s\n", "加速中，目标速度已经下发到命令");
                                }
                            }
                            //速度在接近目标速度范围
                            else
                            {
                                servo->offset_v = vel_target[i]; 
                                servo->out_v= (U32)servo->offset_v;  //速度指令输出

                                printf("%s\n", "当前命令速度接近目标速度！，直接给目标速度值");
                            }
                        }
                        //当前速度大于目标速度
                        else  
                        {
                            printf("%s\n", "当前命令速度大于目标速度");
                            //速度在接近目标速度范围
                            if(servo->out_v>vel_target[i]+acc_max)
                            {   
                                //速度输出不等于 目标速度
                                if(servo->offset_v!=vel_target[i])
                                {
                                    printf("%s\n", "加速中");
                                    servo->offset_v -= dcc_max; //速度偏移值加速
                                    servo->out_v= (U32)servo->offset_v;  //速度指令输出
                                }
                                else
                                {
                                    printf("%s\n", "减速中，目标速度已经下发到命令");
                                }
                            }
                            //速度在接近目标速度范围
                            else
                            {
                                servo->offset_v = vel_target[i]; 
                                servo->out_v= (U32)servo->offset_v;  //速度指令输出
                                printf("%s\n", "当前命令速度接近目标速度！，直接给目标速度值");
                               
                            }
                        }
                        break;
                    //多项式速度控制
                    case OP_MODE_CSV_polygon:



                        break;
                }
            }

            // 更新输出变量
            *po->servos[i].ctrl = servo->sc;
            switch (g_op_mode)
            {
                case OP_MODE_CSP: 
                    *po->servos[i].v = 0;
                    *po->servos[i].t_max = 5000;
                    *po->servos[i].p = servo->out_p;                     
                    break;   

                case OP_MODE_CSV:
                    *po->servos[i].v = servo->out_v;
                    *po->servos[i].t_max = 5000;
                    *po->servos[i].p = 0;                     
                    break;   
            }
        }
        //更新输出内存映像
        if (EcatPIOutputQueuePush(ecat_dev, 0, 100) != 0)
        {
            _ERR_("push pi output error");
            break;
        }
        
    }
	return 0;
}
