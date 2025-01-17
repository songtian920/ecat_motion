#include "motor_controller.h"
#include <cmath> 

motor_controller::motor_controller(rclcpp::NodeOptions node_options) : Node("motor_controller", node_options)
{
    
}

//伺服激磁
int motor_controller::server_on()
{
    _server_on = true; 
    _quick_stop = true; 
    
    return 0;
}

//伺服消磁
int motor_controller::server_off()
{
    _server_on = false; 
    _quick_stop = false;
    return 0;
}

//打开 运动开始控制
int motor_controller::motion_start()
{
    motion_start_ctrl = true;
    usleep(2000);  //2ms
    uint16_t cout_timeout = 0;
    while (!motion_start_state)
    {
        cout_timeout++;
        if(cout_timeout>100)
            return -1; //开启运动失败，超时500ms未开启运动状态
        usleep(5000);  //5ms
    }
    return 0; //开启成功
}

// 运动关闭控制
int motor_controller::motion_stop()
{
    motion_start_ctrl = false;
    usleep(2000);  //2ms
    uint16_t cout_timeout = 0;
    while (motion_start_state)
    {
        cout_timeout++;
        if(cout_timeout>100)
            return -1; //关闭运动失败，超时500ms未关闭运动状态
        usleep(5000);  //5ms
    }
    return 0; //关闭成功
}

//速度保护
bool motor_controller::speed_safety(float vel_target,float cmd_v_last,
        float v_current,float v_max,float _sync_time)
{
    //飞车保护
    //先判断当前命令加速度是否超过最大值的20%
    float vel_stamp = abs(vel_target)-abs(cmd_v_last);
    float acc_or_dcc = vel_stamp/(_sync_time*1000.0f);
    bool acc_dcc_safety = false;  //加减速安全标记
    bool vel_safety = false;  //速度保护值
    if(acc_or_dcc > 0.0f){
        acc_dcc_safety = acc_or_dcc > acc_max*1.2;
    }
    else{
        acc_dcc_safety = abs(acc_or_dcc) > dcc_max*1.2;    
    }
    //速度保护 判断目标速度值，及当前速度值是否超出最大值的20%
    if(v_max*1.2 - abs(vel_target) > 0 && v_max*1.2 - abs(v_current) > 0)
        vel_safety = true;
    //返回值
    if(vel_safety&&acc_dcc_safety)
        return true;
    else
        return false;
}

//计算s型速度曲线 vel_current需要给当前的命令值，不要读取当前轴速，有波动会导致震颤
int motor_controller::vel_s_curve(float vel_current[],float vel_out[])
{   
    float vel_spans[servo_num];  //各轴速度差跨度       
    int span_max_axis = -1; //速度差最大的轴编号
    float span_max = 0; //最大速度差
    // 若速度更新标记为true，则重新计算曲线
    if(vel_cmd_update)
    {
        //同步时间计数清零，用于取时间戳计算，单位1ms
        //当重新计算曲线时，将时间计数清零
        time_count=0;  

        

        //判读速度是否在保护值范围,并找出速度变化跨度最大的轴
        for(int i=0;i < servo_num ;i++)
        {
            if(abs(vel_cmd[i]) > vel_max)
            {
                break;
                return -1; //返回错误 速度超最大限制
            }
            //各轴目标速度和当前速度差
            vel_spans[i] = vel_cmd[i]-vel_current[i];
            //!保存计算曲线的初始速度
            vel_init_0[i] = vel_current[i];
            //排序比较
            if(i==0)
            {   
                //0号轴
                span_max_axis = 0;
                span_max = vel_spans[0];
            }
            else
            {
                //如果当前速度差绝对值大于 当前绝对值最大值，则更新最大
                if(abs(vel_spans[i]) > abs(span_max))
                    span_max_axis = i;  //更新为当前轴号
                    span_max = vel_spans[i];  //更新为当前值
            }
        }

        //根据最大速度差选出的主轴，用主轴计算出曲线总时间及时间分区
        if(span_max>0)  //加速度
        {
            
            //把差速度分1/2，v_diff_m = v_diff*0.5，求出这个点的加速度a_tm
            //v_diff_m = ∫(0-tm) a = ∫(0-tm) (jert*t) 则 v_diff_m = 0.5*jert*t^2
            //推出 tm = (v_diff_m*2)^(1/2)
            float tm = sqrt(span_max*0.5*2/jert_acc_max);  //计算出加速时区 中分时间点
            float a_m = jert_acc_max*tm;  //计算出中分时间点的加速度
            //判断中分时间点加速度是否小于加速度最大值
            if(a_m < acc_max)
            {
                //此种是 情况1 ，先加加速，到时域中分区再减加速 到达目标速度
                condition_s_curve = false;
                //计算出中分时域时间
                t_aa_s = 0; //加加速时间起点0
                t_aa_e = tm;  //加加速时间终点tm
                t_da_s = tm; //减加速时间开始tm
                t_da_e = tm*2; //减加速时间结束tm*2
                time_span_s_curve = t_da_e; //时间跨度最大值
            }
            else
            {
                //此种是 情况2 ，先加加速，到加速度最大值，然后匀加速，再减加速 到达目标速度
                condition_s_curve = true;
                t_aa_s = 0; //加加速时间起点0
                t_aa_e = acc_max/jert_acc_max;  //加加速时间终点
                float t_aa = t_aa_e-t_aa_s;  //加加速时间段
                float t_da = acc_max/jert_acc_max; //减加速时间和 加加速时间段相等
                float v_aa = 0.5*jert_acc_max*t_aa*t_aa;  // 加加速增加的速度值
                float v_da = jert_acc_max*t_da-0.5*jert_acc_max*t_da*t_da;  //减加速增加的速度值
                float v_uniform_acc = span_max-v_aa-v_da;  //匀加速部分 速度分段
                float t_uniform_acc = v_uniform_acc/jert_acc_max;  //匀加速时间段

                t_a_s = t_aa_e; //匀加速开始时刻
                t_a_e = t_a_s + t_uniform_acc; //匀加速结束时间
                t_da_e = t_a_e;  //减加速开始时间
                t_da_e = t_da_e + t_da;  //减加速结束时间
                time_span_s_curve = t_da_e;  //最大时间跨度
            }
        }
        else  //减速度
        {
            //把差速度分1/2，v_diff_m = v_diff*0.5，求出这个点的减速度da_tm
            //v_diff_m = ∫(0-tm) a = ∫(0-tm) (jert*t) 则 v_diff_m = 0.5*jert*t^2
            //推出 tm = (v_diff_m*2)^(1/2)
            float tm = sqrt(span_max*0.5*2/jert_acc_max);  //计算出加速时区 中分时间点
            float d_m = jert_acc_max*tm;  //计算出中分时间点的加速度
            //判断中分时间点加速度是否小于加速度最大值
            if(abs(d_m) < abs(dcc_max))
            {
                //此种是 情况1 ，先加加速，到时域中分区再减加速 到达目标速度
                condition_s_curve = false;
                //计算出中分时域时间
                t_ad_s = 0; //加加速时间起点0
                t_ad_e = tm;  //加加速时间终点tm
                t_dd_s = tm; //减加速时间开始tm
                t_dd_e = tm*2; //减加速时间结束tm*2
                time_span_s_curve = t_dd_e; //时间跨度最大值
            }
            else
            {
                //此种是 情况2 ，先加减速，到减速度最大值，然后匀减速，再减减速 到达目标速度
                condition_s_curve = true;
                t_ad_s = 0; //加加速时间起点0
                t_ad_e = dcc_max/jert_dcc_max;  //加减速时间终点
                float t_ad = t_ad_e-t_ad_s;  //加减速时间段
                float t_dd = dcc_max/jert_dcc_max; //减减速时间和 加减速时间段相等
                float v_ad = 0.5*jert_dcc_max*t_ad*t_ad;  // 加减速减少的速度值
                float v_dd = jert_dcc_max*t_dd - 0.5*jert_dcc_max*t_dd*t_dd;  //减减速减少的速度值
                float v_uniform_dcc = span_max-v_ad-v_dd;  //匀减速部分 速度分段
                float t_uniform_dcc = v_uniform_dcc/jert_dcc_max;  //匀加速时间段

                t_d_s = t_ad_e; //匀减速开始时刻
                t_d_e = t_d_s + t_uniform_dcc; //匀减速结束时间
                t_dd_e = t_d_e;  //减减速开始时间
                t_dd_e = t_dd_e + t_dd;  //减减速结束时间
                time_span_s_curve = t_dd_e;  //最大时间跨度
            }
        }
    }

    //四轴同步，则根具各轴速度差比例求出各轴 加加速度，及减减速度
    for(int i = 0;i < servo_num ;i++)
    {
        if(i!= span_max_axis)  //当前不是找到速度差最大轴
        {
            //按照当前轴速度差和最大速度差比例分配加加速度值
            jert_acc_real[i] = (abs(vel_spans[i])/abs(span_max))*jert_acc_max; 
            //按照当前轴速度差和最大速度差比例分配减减速度值
            jert_dcc_real[i] = (abs(vel_spans[i])/abs(span_max))*jert_acc_max; 
        }
        else  //当前是找到速度差最大的轴
        {
            //将最大加速度给到真实使用值
            jert_acc_real[i] = jert_acc_max;  //将最大加加速度值赋给速度差最大轴
            jert_dcc_real[i] = jert_dcc_max;  //将最大减加速度值赋给速度差最大轴
        }
    }

    //按照曲线计算出给定速度目标值
    //判断曲线时域跨度是否失效，失效则保持上个速度命令，未失效则计算新速度
    //当时间跨度计数超过 曲线有效时间跨度 则保持原来速度
    if(time_count < time_span_s_curve)
    {
        //计算每个轴的速度
        for(int i = 0;i < servo_num ;i++)
        {
            if(vel_spans[i] > 0)   //加速阶段
            {   
                if(!condition_s_curve)  //情况1 分两端速 加加速和减加速
                {
                    //加速曲线
                    if(time_count <= t_aa_e)
                    {
                        //加加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vel_out[i] = 0.5*jert_acc_real[i]*(t-t_aa_s)*(t-t_aa_s) + vel_init_0[i];  // 加速时域速度曲线
                    }
                    else if(time_count > t_da_s && time_count < t_da_e)
                    {
                        //减加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vel_out[i] = jert_acc_real[i]*(t_aa_e-t_aa_s)*(t-t_da_s)-0.5*jert_acc_max*(t-t_da_s)*(t-t_da_s) +
                            0.5*jert_acc_real[i]*(t_aa_e-t_aa_s)*(t_aa_e-t_aa_s) + vel_init_0[i];  
                    }
                    else
                    {
                        //错误
                        printf("%s\n","时间未在分段时域内，检查时域分段计算值")
                        return -1;
                    }
                }
                else  //情况2 分三段速 加加速 匀加速 减加速
                {
                    if(time_count <= t_aa_e)
                    {
                        //加加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vel_out[i] = 0.5*jert_acc_real[i]*(t-t_aa_s)*(t-t_aa_s) + vel_init_0[i];  // 加速时域速度曲线
                    }
                    else if(time_count > t_a_s && time_count <= t_a_e)
                    {
                        //匀加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vle_out[i] = jert_acc_real[i]*(t_aa_e-t_aa_s)*(t-t_a_s) + 
                            0.5*jert_acc_real[i]*(t_aa_e-t_aa_s)*(t_aa_e-t_aa_s) + vel_init_0[i];
                    }
                    else if(time_count > t_da_s && time_count <= t_da_e)
                    {
                        //减加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vle_out[i] = -0.5*jert_acc_real[i]*(t-t_da_s)*(t-t_da_s) + jert_acc_real[i]*(t_aa_e-t_aa_s)*(t-t_da_s)+
                            jert_acc_real[i]*(t_a_e-t_a_s) + 0.5*jert_acc_real[i]*(t_aa_e-t_aa_s)*(t_aa_e-t_aa_s) + vel_init_0[i];
                    }
                    else
                    {
                        //错误
                        printf("%s\n","时间未在分段时域内，检查时域分段计算值")
                        return -1;
                    }
                }
            }
            else   //减速阶段
            {
                if(!condition_s_curve)  //情况1 分两端速 加加速和减加速
                {
                    //加速曲线
                    if(time_count <= t_ad_e)
                    {
                        //加加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vel_out[i] = 0.5*jert_dcc_real[i]*(t-t_ad_s)*(t-t_ad_s) + vel_init_0[i];  // 加速时域速度曲线
                    }
                    else if(time_count > t_dd_s && time_count < t_dd_e)
                    {
                        //减加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vel_out[i] = jert_dcc_real[i]*(t_ad_e-t_ad_s)*(t-t_dd_s)-0.5*jert_dcc_max*(t-t_dd_s)*(t-t_dd_s) +
                            0.5*jert_dcc_real[i]*(t_ad_e-t_ad_s)*(t_ad_e-t_ad_s) + vel_init_0[i];  
                    }
                    else
                    {
                        //错误
                        printf("%s\n","时间未在分段时域内，检查时域分段计算值")
                        return -1;
                    }
                }
                else  //情况2 分三段速 加加速 匀加速 减加速
                {
                    if(time_count <= t_ad_e)
                    {
                        //加加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vel_out[i] = 0.5*jert_dcc_real[i]*(t-t_ad_s)*(t-t_ad_s) + vel_init_0[i];  // 减速时域速度曲线
                    }
                    else if(time_count > t_d_s && time_count <= t_d_e)
                    {
                        //匀加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vle_out[i] = jert_dcc_real[i]*(t_ad_e-t_ad_s)*(t-t_d_s) + 
                            0.5*jert_dcc_real[i]*(t_ad_e-t_ad_s)*(t_ad_e-t_ad_s) + vel_init_0[i];
                    }
                    else if(time_count > t_dd_s && time_count <= t_dd_e)
                    {
                        //减加速阶段
                        float t = (float)time_count/1000.0f;  // 单位 s
                        vle_out[i] = -0.5*jert_dcc_real[i]*(t-t_dd_s)*(t-t_dd_s) + jert_dcc_real[i]*(t_ad_e-t_ad_s)*(t-t_dd_s)+
                            jert_dcc_real[i]*(t_d_e-t_d_s) + 0.5*jert_dcc_real[i]*(t_ad_e-t_ad_s)*(t_ad_e-t_ad_s) + vel_init_0[i];
                    }
                    else
                    {
                        //错误
                        printf("%s\n","时间未在分段时域内，检查时域分段计算值")
                        return -1;
                    }
                }
            }
        }
    }
    else
    {
        //超过曲线有效时间跨度则返回当前命令速度

    }
    return 0;
}

int motor_controller::config()
{
    return 0;
}
