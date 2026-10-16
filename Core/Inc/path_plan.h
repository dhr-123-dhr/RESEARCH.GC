/**
  ******************************************************************************
  * @file    path_plan.h
  * @brief   路网节点图 A* 路径规划(工创赛麦轮 AGV) —— 对外接口
  * ----------------------------------------------------------------------------
  * 设计要点(与上位机 agv_roadnet_planner.py 1:1 对齐):
  *   1) 只做"井字路网 9 节点 + 12 条边"的【节点图 A*】, 绝不使用栅格 A*;
  *   2) 地图尺寸 2400mm x 2400mm, 所有坐标单位 mm, 全部整型运算, 无浮点;
  *   3) 黑柱(= 现场临时障碍物)不是几何避障, 而是【直接封锁整条路网边】;
  *   4) 4 个黄色方块是固定墙体, 在本代码中只作地图记录, 不参与 A*(见 .c 注释);
  *   5) 上位机画布上的红色顶点编号是 1~9, 本代码数组下标是 0~8,
  *      对外显示(串口屏 t1)统一使用 1~9, 与上位机画布完全一致。
  ******************************************************************************
  */
#ifndef __PATH_PLAN_H
#define __PATH_PLAN_H

#include "main.h"
#include <stdint.h>

/* ===================== 场地与路网规模 ===================== */
#define FIELD_MAX_MM    2400     /* 场地边长 mm(坐标范围 0..2400) */
#define RN_NODE_NUM     9        /* 路网节点数(上位机 STANDARD_VERTICES) */
#define RN_EDGE_NUM     12       /* 路网边数(上位机 STANDARD_LINKS) */

/* ===================== 任务参数(写死, 与上位机一致) ===================== */
/* 加工轮数: 每轮 = 原料区取料 -> 粗加工区加工 -> 暂存区放料 */
#define TASK_ROUNDS     2
/* 任务步数上限 = 起点启停区(1) + 读取二维码(1) + 每轮3步 + 返回终点启停区(1)
 *             = 3 + 3 * TASK_ROUNDS   (TASK_ROUNDS=2 时为 9 步) */
#define STEP_MAX        (3 + 3 * TASK_ROUNDS)

/* ===================== 动作码(与上位机 rn_action_code 一致) ===================== */
#define ACT_START       0        /* 出发/待命: 起点启停区 */
#define ACT_QR          1        /* 读取二维码 */
#define ACT_RAW         2        /* 原料区取料 */
#define ACT_TMP         3        /* 暂存区放料 */
#define ACT_ROUGH       4        /* 粗加工区加工 */
#define ACT_BACK        5        /* 返回/结束: 终点启停区 */

/* ===================== 淘晶驰串口屏 ===================== */
#define HMI_T1          "t1"     /* 文本控件名 */
/* 数据包结束符: 必须与屏工程(工创串口屏.HMI)里的"结束符"设置一致!
 * 该屏工程设的是 FF FF FF -> 这里用 "\xFF\xFF\xFF"
 * (若以后把屏工程改成 \r\n, 这里改回 "\r\n" 即可) */
#define HMI_END         "\xFF\xFF\xFF"
#define HMI_T1_BUF_LEN  200      /* t1 文本缓冲上限(字节); 全路径最多约 81 个号 ≈ 170 字符 */

/* ===================== 通信协议参数 ===================== */
#define PKT_LINE_BUF    64       /* 单条消息缓冲(字节); 单条最长约 17 字节, 留足余量 */
#define PKT_IDLE_MS     500      /* 包边界: 最后一字节后 500ms 无新数据 = 整包结束
                                     (放宽到 500ms, 是为了容忍屏把长串分片发送) */
#define OBS_MAX         12       /* 一包内最多缓存几条 OBS_ADD(屏上障碍按钮数量上限) */
#define OBS_TOL_MM      150      /* 黑柱"命中某条边"的容差 mm(点的是圆柱实体位置) */
                                 /* 未摆放的柱: 屏可以直接【不发那行 OBS_ADD】;
                                    发 "OBS_ADD:0,0" 也同样合法(不封锁任何边) */

/* ===================== 单步与本段路点上限 ===================== */
#define STEP_POINT_MAX  16       /* 单步本段路点数上限(最多9个节点 + 首尾接入段) */
#define FRAME_BUF_LEN   200      /* $STEP 帧缓冲上限 */

/* ===================== 手动测试开关 ===================== */
/* 1 = main 上电后先跑一次"手填起终点 + 手填黑柱"的测试(不依赖串口屏), 结果发到屏 t1 */
#define PATHPLAN_MANUAL_TEST  0

/* ===================== 单步数据结构 ===================== */
typedef struct
{
    uint8_t  action;                    /* 动作码(0~5) */
    uint8_t  stop_node;                 /* 本步停靠路网节点(数组下标 0~8) */
    int16_t  count;                     /* 本步"本段"路点数 */
    int16_t  x[STEP_POINT_MAX];         /* 本段路点 X(mm) */
    int16_t  y[STEP_POINT_MAX];         /* 本段路点 Y(mm) */
} PathPlanStep;

/* ===================== 对外函数 ===================== */
void    PathPlan_Init(void);
void    PathPlan_OnRxByte(uint8_t b);                   /* 在 USART1 接收中断里调用本函数 */
void    PathPlan_Tick(void);                            /* 在 while(1) 里调用(200ms 包边界判定) */

uint8_t PathPlan_SetStart(int16_t x, int16_t y);        /* 手动测试: 设置起点启停区中心 */
uint8_t PathPlan_SetGoal(int16_t x, int16_t y);         /* 手动测试: 设置终点启停区中心 */
uint8_t PathPlan_ObsAdd(int16_t x, int16_t y);          /* 封锁该点所在的路网边;(0,0)=未摆放 */
void    PathPlan_ClearObs(void);                        /* 清空全部动态封锁(黑柱) */
uint8_t PathPlan_RunTask(void);                         /* 生成整任务, 并把停靠节点号串发到屏 t1 */
uint8_t PathPlan_RunOneLeg(void);                       /* 手动测试: 起点->终点 单段 A* */

int16_t PathPlan_GetStepCount(void);
uint8_t PathPlan_GetStepAction(int16_t i);
uint8_t PathPlan_GetStepStopNode(int16_t i);            /* 返回数组下标 0~8(显示时 +1) */
int16_t PathPlan_GetStepPointCount(int16_t i);
int16_t PathPlan_GetStepPointX(int16_t i, int16_t k);
int16_t PathPlan_GetStepPointY(int16_t i, int16_t k);
const char* PathPlan_GetStepFrame(int16_t i);           /* $STEP,动作码,点数,x1,y1,...# */
const char* PathPlan_GetStopText(void);                 /* 屏 t1 内容: 全路径节点号(1~9,"-"连接) + 总长 */

void    PathPlan_SendText(const char *txt);             /* 向串口屏发一条 t1.txt="..." */

#endif /* __PATH_PLAN_H */
