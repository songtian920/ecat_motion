#include <stdio.h>

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
#include <rclcpp/rclcpp.hpp>


#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H


class motor_controller : public rclcpp::Node
{
    public:

    //默认构造函数
    motor_controller(rclcpp::NodeOptions node_options);
    
    //速度保护
    bool speed_safety(float vel_target,float cmd_v_last,
        float v_current,float v_max,float _sync_time);

    //初始配置
    int config();

    //伺服激磁
    int server_on();

    //伺服消磁
    int server_off();

    //打开 运动开始控制
    int motion_start();

    //运动关闭控制
    int motion_stop();

    //速度同步周期模式， 指定轴编号及运动参数
    int sync_control();

    //计算s型速度曲线
    int vel_s_curve(float vel_current[],float vel_out[]);
    
    protected:

    //轴计算静态参数
    float vel_max = 200000.0f;  //最大速度
    float acc_max = 100.0f;  //最大加速度 mm/s^2 用线速度表示
    float jert_acc_max = 100.0f; //最大加加速或减加速  mm/s^3
    float dcc_max = -100.0f; //最大减速度 mm/s^2  用线速度表示 值为负数
    float jert_dcc_max = -100.0f; //最大减减速或加减速 mm/s^3   值为负数
    float followError_speed =  1000.0; //跟随误差
    float sync_time = 0.001;  //同步周期时间 单位 秒s
    uint8_t servo_num =2;  //todo轴数量为2 临时
    float jert_acc_real[4] = {0.0f, 0.0f, 0.0f, 0.0f};  //真实的使用加加速度值
    float jert_dcc_real[4] = {0.0f, 0.0f, 0.0f, 0.0f};  //真实的使用减减速度值

    //轴计算临时参数
    float vel_cmd[4] = {0.0,0.0,0.0,0.0} ; //todo 目前按照四个轴设置 上位机发送的命令速度 mm/s
    float vel_cmd_last[4] = {0.0,0.0,0.0,0.0} ; //todo目前按照四个轴设置 上位机发送的命令速度旧值 mm/s 
    bool vel_cmd_update = false; //轴上位发送命令速度更新标记
    uint64_t time_count = 0; //同步执行的时间计数
    

    //速度s曲线保存参数   
    //情况1 为假，是两段速，(加加速)(减加速)或(加减速)(减减速)
    //情况2 为真,(加加速)(匀加速)(减加速)或(加减速)(匀加速)(减减速)
    bool condition_s_curve = false; 
    uint32_t time_span_s_curve = 0; //曲线存活的有效时间跨度
    float vel_init_0[4]; //计算曲线的初始速度
    float t_aa_s; //加加速时间起点
    float t_aa_e; //加加速时间终点
    float t_a_s;  //匀加速时间起点
    float t_a_e;  //匀加速时间终点
    float t_da_s; //减加速时间起点
    float t_da_e; //减加速时间终点

    float t_ad_s; //加减速时间起点
    float t_ad_e; //加减速时间终点
    float t_d_s;  //匀减速时间起点
    float t_d_e;  //匀减速时间终点
    float t_dd_s; //减减速时间起点
    float t_dd_e; //减减速时间终点

    //控制字操作
    bool _server_on = false;  //伺服上电
    bool _quick_stop = false; //快速停止
    bool motion_start_ctrl = false; //运动开始控制标志
    bool motion_start_state = false; //运动开始返回标记

    float vel_target[4] = {0.0,0.0,0.0,0.0} ; //最终输出给slave目标速度 mm/s
    float vel_target_last[4] = {0.0,0.0,0.0,0.0} ; //todo目前按照四个轴设置 目标速度上一个旧值
};

#endif