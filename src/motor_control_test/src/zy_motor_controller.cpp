#include "zy_motor_controller.h"
#include <cstdlib>


zy_motor_controller::zy_motor_controller(std::int16_t index_ecat, std::int16_t ecat_channel,
            std::string ENI_path_ecat,OptionParam option,rclcpp::NodeOptions node_options ) : motor_controller(node_options)
{
    index = index_ecat;  //卡号索引index 赋值
    channel = ecat_channel;  //端口号channel赋值（NET0为0，NET1为1）
    ENI_path = ENI_path_ecat;  //ENI文件地址赋值
    

    //参数赋值 如参数指针为空则使用默认参数，不为空则传参给成员
    if(!option.IsEmpty())
    {
        //参数赋值 判断一下参数是否有效，有效参数赋值，无效保持默认值
        vel_max = (option.vel_max!=NULL) ? option.vel_max : vel_max;   //最大速度
        acc_max = (option.acc_max!=NULL) ? option.acc_max : acc_max;   //最大加速度 mm/s^2 用线速度表示
        jert_acc_max = (option.jert_acc_max!=NULL) ? option.jert_acc_max : jert_acc_max;  //最大加加速或减加速  mm/s^3
        dcc_max = (option.dcc_max!=NULL) ? option.dcc_max : dcc_max;   //最大减速度 mm/s^2  用线速度表示
        jert_dcc_max = (option.jert_dcc_max!=NULL) ? option.jert_dcc_max : jert_dcc_max;  //最大减减速或加减速 mm/s^3
        followError_speed = (option.followError_speed!=NULL) ? option.followError_speed : followError_speed; //跟随误差
        sync_time = (option.sync_time!=NULL) ? option.sync_time : sync_time;                   //同步周期时间 单位 秒s
    }
}

int zy_motor_controller::config()
{
    //根据通道号打开设备，获得句柄
    //PCIe-2E 只有一个通道,PCIe-4E有两个通道
    char index_buff[10];
    RETURN_IF_FAILED(EcatOpen(&ecat_dev, BOARD_ALIAS(index_buff, index), channel)) 
        

    //启动并传入ENI文件
    RETURN_IF_FAILED(EcatBusRun(ecat_dev, "hcfa_D3E_1602_1A02_ENI.xml"));

    //切换主站控制状态  
    uint8_t state;
    RETURN_IF_FAILED(EcatRequestMasterState(ecat_dev, EcatStateO));
    for (uint32_t i = 0; ; i++)
    {
        //获取主站状态
        RETURN_IF_FAILED(EcatGetMasterState(ecat_dev, &state));
        _INF_("request is: 8, curr is: %d", state);
        usleep(500000);
        //判断主站状态为控制状态，则退出循环
        if (state == EcatStateO) break;
        //切换控制状态失败失败
        if (i > 10)
        {
            _ERR_("failed to switch to control state");
            return 2; // 10s 后没有主站切换到控制状态
        }    
    }

    //COE参数配置
    ecat_coe_download(ecat_dev);

    // 清空并预计填入数据
    //将输入输出数据绑定本地指针
    RETURN_IF_FAILED(EcatPINMap(ecat_dev, PI_AREA_LOCAL_INPUT, (void **)&input_data));
    RETURN_IF_FAILED(EcatPINMap(ecat_dev, PI_AREA_LOCAL_OUTPUT, (void **)&output_data));
    for (uint32_t i = 0; i < 2; i++)
    {
        //过程数据队列入队
        EcatPIOutputQueuePush(ecat_dev, false, 100);
    }
    //使能PI镜像缓存跟总线进行数据交换
    RETURN_IF_FAILED(EcatPIEnable(ecat_dev));


    return 0;
}

int zy_motor_controller::ecat_coe_download(ECAT_HANDLE hHandle)
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

//测试目标速度赋值
void zy_motor_controller::test_set_vel_target(float val0,float val1)
{
    vel_target[0] = val0;
    vel_target[1] = val1;
}

//控制字 
bool zy_motor_controller::motor_control(int axis, servo_stat_word *s0, 
    const servo_stat_word *s1, servo_ctrl_word *c, bool serverOn, bool quickStop)
{
    servos_t *servo = &__servos[axis];
    do {
        if(serverOn)  //打开使能
        {
            c->v = 0;  //控制字先清空
            c->b.enable_voltage = 1;    //伺服上电
            c->b.quick_stop = quickStop;        //快速停止关闭 0是快速停止，1是打开，servo_on操作quick_stop应为1
            c->b.enable_operation = s1->b.switched_on || s1->b.operation_enabled;  //开关打开或已经使能，则控制字使能
            c->b.switch_on = s1->b.ready_to_switch_on;   //状态字准备开关状态完成，则控制字打开开关
        }
        else  //关闭使能
        {
            c->v = 0;   //控制字先清空
            c->b.enable_voltage = 1;   //伺服上电
            c->b.quick_stop = 1;    //快速停止关闭 
            c->b.enable_operation = 0;
            c->b.switch_on = s1->b.ready_to_switch_on;
        }
        
        //故障复位
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


//输出映像地址 绑定数据结构类型
proc_img_o *zy_motor_controller::pi_get_output(const pi_offset_config *config, void *output)
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

//输入映像地址 绑定数据结构类型
proc_img_i *zy_motor_controller::pi_get_input(const pi_offset_config *config, const void *output)
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

//打开周期同步
int zy_motor_controller::sync_control()
{
    uint64_t loops = 0;
    while(1)
    {   
        time_count++;  //同步时间计数，用于取时间戳计算，单位1ms
        //第一步,获取主站输入数据映像，并绑定输入输出映像
        while (EcatPIInputQueuePop(ecat_dev, false, 100) != 0);
        //输入内存块input_data，部分有效地址绑定绑定到输入映像上 通过pi指针获取返回状态数据
        proc_img_i *pi = pi_get_input(__pi_config, input_data);
        //输出内存块output_data，部分有效地址绑定到输出映像上，通过po指针输出数据
        proc_img_o *po = pi_get_output(__pi_config, output_data);

        //第二步，for循环 控制字赋值， 判断状态 给出运行结果run
        //轴run状态
        //todo目前最多是四个轴，后续要改为多轴的
        bool run[4]= {false, false, false, false};
        loops++;
        for (int i = 0; i < NUM_SERVOS; i++)
        {   
            //遍历每个轴
            servos_t *servo = &__servos[i];
            
            //控制字逻辑 servo_on， quick_stop, 故障复位
            bool run_axis = motor_control(i, &servo->ss, pi->servos[i].stat, &servo->sc, _server_on,_quick_stop);

            if (loops % 1000 == 0) 
            {
                //1s输出一次信息
                _INF_("servo[%d]: p=%10d, ctrl:%08x stat=%08x", i, *pi->servos[i].p, po->servos[i].ctrl->v, pi->servos[i].stat->v);
            }

            // 伺服未出力，或已出力但延时未结束，应停止转动
            if (!run_axis || servo->run_delay != 0) {
                servo->out_p = *pi->servos[i].p;
                servo->zero_p = (float)servo->out_p;
                servo->offset_p = 0;  //位置偏移值清零
                servo->cmd_v = 0;  //速度偏移值清零
                servo->out_v = 0;
                servo->out_t = 0;
                servo->reverse = false;
            }

            // 伺服已出力，延时计数递减
            if (run_axis && servo->run_delay){
                servo->run_delay--;
            }                
            else if(run_axis && !servo->run_delay){
                run[i] = true;
            }

            // 更新输出变量控制字
            *po->servos[i].ctrl = servo->sc;
        }

        //Todo第三步 判断运动是否开启，开启后判断目标指令是否更新，更新则调用重新计算曲线函数，计算出当前时间的目标值
        //判断运动开启状态
        if(!motion_start_ctrl)
        {
            //未开启，则初始状态值
            //todo未运动，则可以把中间参数初始值
            vel_target[0] = 0.0 ;
            vel_target[1] = 0.0 ;
            vel_target[2] = 0.0 ;
            vel_target[3] = 0.0 ;
            vel_target_last[0] = 0.0 ;
            vel_target_last[1] = 0.0 ;
            vel_target_last[2] = 0.0 ;
            vel_target_last[3] = 0.0 ;
            vel_cmd_update = false; //轴目标速度更新标记
            time_count = 0; //同步执行的时间计数
            time_span_s_curve = 0; //曲线存活的有效时间跨度
            motion_start_state = false;  //运动开始状态返回
        }
        else
        {
            motion_start_state = true;  //运动开始状态返回
        }

        //第四步 如果是每个轴是run且运动标志start状态，则遍历每个轴，将第三步计算的目标值，赋给ecat输出映像
        //todo测试用两个轴，后续换四个轴
        if(run[0] && run[1])
        {
            for (int i = 0; i < NUM_SERVOS; i++)
            {
                servos_t *servo = &__servos[i];
                
                if (po->servos[i].mode) 
                {
                    //模式指针不为空，则赋值模式
                    *po->servos[i].mode = g_op_mode;
                }

                //目标值赋值
                if(g_op_mode == OP_MODE_CSP)
                {
                    // 不同轴单位可能不一样，下表定义各轴位置范围及增量
                    I32 range_min[] = { -131072, -130000, -500 };
                    I32 range_max[] = {  131072, 130000, 1500 };
                    float step_size[] = { 10.4f, 10.4f, 5.2f };
                    if (servo->offset_p < range_min[i]) servo->reverse = false;
                    if (servo->offset_p > range_max[i]) servo->reverse = true;
                    servo->offset_p += (servo->reverse ? -1 : 1) * step_size[i];
                    servo->out_p = (I32)(servo->zero_p + servo->offset_p);
                }
                else if(g_op_mode == OP_MODE_CSV) 
                {
                    //中间变量
                    float vel_pre = 0.0f;  //速度预计算值

                    //Todo先判断当前速度和命令速度滞后性是否超出范围
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
                            if(servo->cmd_v!=vel_target[i])
                            {
                                printf("%s\n", "加速中");
                                servo->cmd_v += acc_max; //速度偏移值加速
                                servo->out_v= (U32)servo->cmd_v;  //速度指令输出
                            }
                            else
                            {
                                printf("%s\n", "加速中，目标速度已经下发到命令");
                            }
                        }
                        //速度在接近目标速度范围
                        else
                        {
                            servo->cmd_v = vel_target[i]; 
                            servo->out_v= (U32)servo->cmd_v;  //速度指令输出

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
                            if(servo->cmd_v!=vel_target[i])
                            {
                                printf("%s\n", "加速中");
                                servo->cmd_v -= dcc_max; //速度偏移值加速
                                servo->out_v= (U32)servo->cmd_v;  //速度指令输出
                            }
                            else
                            {
                                printf("%s\n", "减速中，目标速度已经下发到命令");
                            }
                        }
                        //速度在接近目标速度范围
                        else
                        {
                            servo->cmd_v = vel_target[i]; 
                            servo->out_v= (U32)servo->cmd_v;  //速度指令输出
                            printf("%s\n", "当前命令速度接近目标速度！，直接给目标速度值");
                        }
                    }
                    
                }
                else if(g_op_mode == OP_MODE_CSV_polygon)
                {
                    //速度保护判读 速度安全执行命令，不安全返回错误值
                    bool safe_speed = speed_safety(vel_target[i],vel_target_last[i],
                            *pi->servos[i].v,vel_max,sync_time);  
                    if(safe_speed)
                    {
                        servo->cmd_v = vel_target[i]; 
                        servo->out_v= (U32)servo->cmd_v;  //速度指令输出
                    }
                    else
                    {
                        //todo 速度不安全，命令速度改为0， 减速停止，并报警，注：急刹有风险，只有急停有急刹

                    }
                    
                }
                
                //更新输出映像 目标值变量
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
        }
        
        //第五步，更新输出内存映像
        if (EcatPIOutputQueuePush(ecat_dev, 0, 100) != 0)
        {
            //Todo: error这里有返回值
            _ERR_("push pi output error");
            break;
        }
    }

    return 0;
}


