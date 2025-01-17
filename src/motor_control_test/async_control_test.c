#include <stdio.h>
#include <stdlib.h>

#include "pci_errno.h"
#include "pci_zecm.h"
#include "pci_dbg.h"

#ifdef VXWORKS
#define main ASYNC_CONTROL_TEST
#endif

#define RETURN_IF_FAILED(expr)      { int ret = (expr); if (ret != 0) { _ERR_(#expr " failed: err=%d", ret); return -1; } else { _INF_(#expr " succeeded"); } }

/**************************motor config*************************/
#define NUM_SERVOS  3           // 总电机数

typedef  uint32_t   U32;
typedef  uint16_t   U16;
typedef  uint8_t    U8;
typedef  int32_t    I32;
typedef  int16_t    I16;

#define MIN_LIMIT(T,L) ((T) < (L) ? (L) : (T))
#define MAX_LIMIT(T,L) ((T) > (L) ? (L) : (T))

enum {
    OP_MODE_PP = 1,      
    OP_MODE_PV = 3,    
    OP_MODE_PT = 4,      
    OP_MODE_CSP = 8,
    OP_MODE_CSV = 9,
    OP_MODE_CST = 10,
};
static uint32_t g_op_mode = OP_MODE_PP;
static U32 g_run_task = 1;

////////////////////////////////////////////////////////////////////////////////
// 过程映像
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
} servo_ctrl;

typedef struct {
    uint8_t             *mode;
    servo_stat_word     *stat;
    I32                 *p;
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
    int32_t             pi_mode_offset;
    int32_t             pi_stat_offset;
    int32_t             pi_p_offset;
} pi_offset_config;

static pi_offset_config  __pi_config[] =
{
    {
        .po_mode_offset = -1,
        .po_ctrl_word_offset = 0,
        .po_p_offset = 2,
        .pi_mode_offset = 4,
        .pi_stat_offset = 2,
        .pi_p_offset = 5,
    },
    {
        .po_mode_offset = -1,
        .po_ctrl_word_offset = 8 + 0,
        .po_p_offset = 8 + 2,
        .pi_mode_offset = -1,
        .pi_stat_offset = 19 + 2,
        .pi_p_offset = 19 + 4,
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
    I32 out_p;             // 输出位置
    I32 out_v;             // 输出转速
    I16 out_t;             // 输出扭矩
    bool reverse;          // 当前方向
}servos_t;

static servos_t __servos[NUM_SERVOS] = {0};

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
static bool state_control(int axis, servo_stat_word *s0, const servo_stat_word *s1, servo_ctrl_word *c, bool show)
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

static  proc_img_o	__pi_o;
static  proc_img_i	__pi_i;
static  proc_img_i *pi = NULL;
static  proc_img_o *po = NULL;
static uint8_t *input_buff = NULL;
static uint8_t *output_buff = NULL;

static proc_img_o *pi_get_output(const pi_offset_config *config, void *output)
{
	for (uint32_t i = 0; i < NUM_SERVOS; i++)
	{
		servo_ctrl *ctrl = &__pi_o.servos[i];
		const pi_offset_config *curr = &config[i];

		ctrl->mode = (uint8_t *)((curr->po_mode_offset != -1) ? (char *)output + curr->po_mode_offset : 0);
		ctrl->ctrl = (servo_ctrl_word *)((curr->po_ctrl_word_offset != -1) ? (char *)output + curr->po_ctrl_word_offset : 0);
		ctrl->p = (I32 *)((curr->po_p_offset != -1) ? (char *)output + curr->po_p_offset : 0);
	}
    return &__pi_o;
}

static proc_img_i *pi_get_input(const pi_offset_config *config, const void *output)
{
	for (uint32_t i = 0; i < NUM_SERVOS; i++)
	{
		servo_stat *stat = &__pi_i.servos[i];
		const pi_offset_config *curr = &config[i];
		stat->mode = (uint8_t *)((curr->pi_mode_offset != -1) ? (char *)output + curr->pi_mode_offset : 0);
		stat->stat = (servo_stat_word *)((curr->pi_stat_offset != -1) ? (char *)output + curr->pi_stat_offset : 0);
		stat->p = (I32 *)((curr->pi_p_offset != -1) ? (char *)output + curr->pi_p_offset : 0);
	}
    return &__pi_i;
}


static int motor_mode_init(ECAT_HANDLE ecat_dev,uint32_t op_mode)
{
    int ret = -1;
    uint32_t mode = 0;
    uint32_t len;
    int fail_try = 100;
    int i;

    for(i = 0; i < NUM_SERVOS && fail_try>0; )
    {
        len = 1;
        ret=EcatCoeSDODownload(ecat_dev,i,0x6060,0x0,1,&op_mode,100);
        usleep(1000*100);
        
        ret=EcatCoeSDOUpload(ecat_dev,i,0x6060,0x0,&len,&mode,100);
        printf("motor[%d] set_mode:%d mode:%d\r\n",i,op_mode,mode);

        if(mode != op_mode)
        {
            usleep(1000*00);
            fail_try --;
            continue;
        }    
        fail_try = 100;
        i++;
    }

    return ret;
}

static int motor_state_init(ECAT_HANDLE ecat_dev,ECAT_ENI_INFO ptInfo)
{    
    int  i = 0;
    int fail_try = 1000;
    bool run = false;
    uint32_t max_speed = 5000;


    for (i = 0; i < NUM_SERVOS && fail_try > 0;)
    {
        servos_t *servo = &__servos[i];

        if(EcatPIRead(ecat_dev,PI_AREA_SHADOW_INPUT,0,ptInfo.u32PIInputByteSize,input_buff))
        {
            printf("EcatPIRead Fail\r\n");
        }

        run = state_control(i, &servo->ss, pi->servos[i].stat, &servo->sc, false);
        usleep(1000);

        // 更新输出变量
        *po->servos[i].ctrl = servo->sc;
        *po->servos[i].p = 0;

        if (EcatPIWrite(ecat_dev,PI_AREA_FAST_OUTPUT,0,ptInfo.u32PIOutputByteSize,output_buff) != 0)
        {
            printf("EcatPIWrite Fail\r\n");
        }
  
        if(!run)
        {
            fail_try--;
            continue;
        }

        //初始位置初始化
        servo->out_p = *pi->servos[i].p;
        servo->zero_p = (float)servo->out_p;
        servo->offset_p = 0;
        servo->out_v = 0;
        servo->out_t = 0;
        servo->reverse = false;
        
        //初始话最高速度
        EcatCoeSDODownload(ecat_dev,i,0x6081,0x0,4,&max_speed,100);

        fail_try = 1000;
        i++;
    }
    
    return run? 0 : -1;
}

int main(int argc, char* argv[])
{
    int ret = 0;
    char index_buff[10];
    uint32_t index = 0;
    uint64_t loops = 0;
    uint32_t channel = 0;

    ECAT_HANDLE ecat_dev;
    ECAT_ENI_INFO ptInfo;

    if (argc != 3)
    {
        printf("usage: ./program encoder_id chnnal\r\n");
        return 0;
    }

    index = atoi(argv[1]);
    channel = atoi(argv[2]);
    if(channel > 1) channel = 1;
    //PCIe-2E 只有一个通道,PCIe-4E有两个通道
    RETURN_IF_FAILED(EcatOpen(&ecat_dev, BOARD_ALIAS(index_buff, index), channel));
    
    RETURN_IF_FAILED(EcatBusRun(ecat_dev, "protal3.xml"));

    // 请求切换状态
    uint8_t state;
    RETURN_IF_FAILED(EcatRequestMasterState(ecat_dev, EcatStateO));
    for (uint32_t i = 0; ; i++)
    {
        RETURN_IF_FAILED(EcatGetMasterState(ecat_dev, &state));
        _INF_("request is: 8, curr is: %d", state);
        usleep(500000);
        if (state == EcatStateO) break;
    }

    RETURN_IF_FAILED(motor_mode_init(ecat_dev,OP_MODE_PP));

    EcatGetEniInfo(ecat_dev,&ptInfo);
    printf("InputByteSize:%d\r\n",ptInfo.u32PIInputByteSize);
    printf("OutputByteSize:%d\r\n",ptInfo.u32PIOutputByteSize);

    input_buff  = malloc(1024);
    output_buff = malloc(1024);

    if(input_buff == NULL || output_buff == NULL)
    {
        printf("malloc fail\r\n");
        return 0;
    }

    pi = pi_get_input(__pi_config,input_buff);
    po = pi_get_output(__pi_config,output_buff);

    RETURN_IF_FAILED(EcatPIEnable(ecat_dev));

    while(motor_state_init(ecat_dev,ptInfo))
    {
        printf("wait motor state init...\r\n");      

        usleep(1000*500);
    }
    
    printf("mootor init ok, test start!\r\n");

    while (1)
    {
        loops++;
        if(!EcatPIRead(ecat_dev,PI_AREA_SHADOW_INPUT,0,ptInfo.u32PIInputByteSize,input_buff))
        {
            for (int i = 0; i < NUM_SERVOS; i++)
            {
                servos_t *servo = &__servos[i];

                if (loops % 10 == 0) 
                {
                    _INF_("IN  servo[%d]: p=%10d, ctrl:%08x stat=%08x", i, *pi->servos[i].p, po->servos[i].ctrl->v, pi->servos[i].stat->v);
                    _INF_("OUT servo[%d]: p=%10d", i,  *po->servos[i].p);
                }

                // 伺服已出力，延时结束，进行正常控制
                // 不同轴单位可能不一样，下表定义各轴位置范围及增量
                I32 range_min[] = { -500, -500, -1500 };
                I32 range_max[] = {  500, 500, 1500 };
                float step_size[] = { 1, 1, 10 };
                if (servo->offset_p < range_min[i]) servo->reverse = false;
                if (servo->offset_p > range_max[i]) servo->reverse = true;
                servo->offset_p += (servo->reverse ? -1 : 1) * step_size[i];
                servo->out_p = (I32)(servo->zero_p + servo->offset_p);

                servo->out_p = MIN_LIMIT(servo->out_p ,range_min[i]);
                servo->out_p = MAX_LIMIT(servo->out_p ,range_max[i]);

                // 更新输出变量
                po->servos[i].ctrl->v = 0x2F;
                *po->servos[i].p = servo->out_p;
            }
            
            if (EcatPIWrite(ecat_dev,PI_AREA_FAST_OUTPUT,0,ptInfo.u32PIOutputByteSize,output_buff) != 0)
            {
                _ERR_("write PI_AREA_FAST_OUTPUT error");
                break;
            }
            usleep(1000*2);

            //pp mode need 0x6040: 0x2f -> 0x3f
            for (int i = 0; i < NUM_SERVOS; i++)
            {
                po->servos[i].ctrl->v = 0x3F;
                servos_t *servo = &__servos[i];
            }
            if (EcatPIWrite(ecat_dev,PI_AREA_FAST_OUTPUT,0,ptInfo.u32PIOutputByteSize,output_buff) != 0)
            {
                _ERR_("write PI_AREA_FAST_OUTPUT error");
                break;
            }
        }

        usleep(1000);
    }
	return 0;
}
