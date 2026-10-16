/**
  ******************************************************************************
  * @file    path_plan.c
  * @brief   路网节点图 A*(工创赛麦轮 AGV) —— 地图 / 黑柱封边 / 任务序列 / A* / 串口屏
  * ----------------------------------------------------------------------------
  * 【核心设计: A* 只允许在"井字路网 9 节点 + 12 条边"上寻路, 没有任何栅格】
  *   1) 搜索节点就是 9 个路网节点本身, 4 邻域(沿边走), 曼哈顿距离启发式;
  *   2) 黑柱(现场临时障碍物)不是几何避障, 而是【直接封锁整条被挡住的路网边】:
  *      · 坐标由淘晶驰串口屏按键发送(点的通常是圆柱实体位置, 允许偏离中心线);
  *      · 命中判定容差 OBS_TOL_MM = 150mm, 命中几条边就封几条边;
  *      · 点落在交叉口上 -> 封锁该点相连的所有边; 重复发送同一条边 = 幂等;
  *      · "OBS_ADD:0,0" = 该柱未摆放, 合法且不封锁任何边;
  *   3) 4 个黄色方块(障碍物11/12/21/22)是固定墙体。按上位机 A* 的判定
  *      (车宽300mm -> 膨胀半径160mm), 它们离最近的路网中心线都还有 200mm,
  *      一条边都封不到(上位机实测 blocked=0), 所以本文件只把它们记录成常量;
  *   4) 任务序列与上位机 rebuild_task_sequence 完全一致:
  *        起点启停区 -> 读取二维码 -> 每轮(原料区取料->粗加工区加工->暂存区放料)
  *        -> 返回终点启停区   (2 + 3*TASK_ROUNDS = 9 步)
  *   5) 每步"本段" = 上一步停靠点 -> 本步停靠点 的 A* 路点(保留共线中间节点);
  *      任务首点前补"起点启停区中心", 任务末点后补"终点启停区中心"(麦轮可斜走);
  *   6) 某一段无解 -> 该步整步删除, 其余步骤继续; 起点/终点启停区无解 -> 整任务作废;
  *   7) 输出: 串口屏 t1 显示【全路径节点号串】(编号 1~9, 与上位机画布一致) + 路网总长mm,
 *      与上位机"路网点顺序"那行的口径完全相同(含直线穿过的中间节点);
  *      "$STEP,动作码,点数,x1,y1,...,xn,yn#" 帧留在内部, 供底盘逐步执行;
  *   8) 全整型运算, 无浮点, 不使用 printf/sprintf。
  ******************************************************************************
  */

#include "path_plan.h"
#include "usart.h"      /* 使用 huart1(USART1 = 淘晶驰串口屏, 115200 8N1) 发送数据 */
#include <string.h>

/* USART1_RX 的 DMA 句柄: CubeMX 在 usart.c 里定义(DMA1_Channel5, 循环模式) */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* ==========================================================================
 * 一、地图常量(与上位机 agv_roadnet_planner.py 完全一致, 单位 mm)
 * ========================================================================== */

/* 9 个路网节点坐标(数组下标 0~8; 上位机画布红字编号 = 下标 + 1 = 1~9)
 *         上位机画布编号:        本代码下标:
 *   上排 y=2100 :  7 ── 8 ── 9        6 ── 7 ── 8
 *   中排 y=1200 :  4 ── 5 ── 6        3 ── 4 ── 5
 *   下排 y=300  :  1 ── 2 ── 3        0 ── 1 ── 2
 * 注意: 编号 1 = 右下角 (2100,300), 编号 9 = 左上角 (150,2100)
 */
static const int16_t NODE_X[RN_NODE_NUM] = {2100, 1200, 150, 2100, 1200, 150, 2100, 1200, 150};
static const int16_t NODE_Y[RN_NODE_NUM] = { 300,  300, 300, 1200, 1200, 1200, 2100, 2100, 2100};

/* 12 条路网边(顺序与上位机 STANDARD_LINKS 完全一致 E0~E11)
 *   E0 (0,1) 900     E1 (1,2) 1050    E2 (3,4) 900     E3 (4,5) 1050
 *   E4 (6,7) 900     E5 (7,8) 1050    E6 (0,3) 900     E7 (3,6) 900
 *   E8 (1,4) 900     E9 (4,7) 900     E10(2,5) 900     E11(5,8) 900
 */
static const uint8_t EDGE_A[RN_EDGE_NUM]   = { 0, 1, 3, 4, 6, 7, 0, 3, 1, 4, 2, 5};
static const uint8_t EDGE_B[RN_EDGE_NUM]   = { 1, 2, 4, 5, 7, 8, 3, 6, 4, 7, 5, 8};
static const int16_t EDGE_LEN[RN_EDGE_NUM] = {900, 1050, 900, 1050, 900, 1050, 900, 900,
                                               900, 900, 900, 900};

/* 4 个黄色方块障碍物(固定墙体, 仅作地图记录, 不参与 A*)
 *   障碍物11 (775,1625)   障碍物12 (775, 775)
 *   障碍物21 (1625,1625)  障碍物22 (1625, 775)
 *   (每个 450x450; 离最近路网中心线 200mm, 车宽300mm 时封不到任何边) */
static const int16_t YELLOW_BLOCK[4][2] = {{775, 1625}, {775, 775}, {1625, 1625}, {1625, 775}};

/* 工位坐标(写死; 与上位机 STANDARD_ELEMENTS 的【代码常量】一致) */
#define QR_X      2400      /* 二维码板 */
#define QR_Y      1200
#define RAW_X     1110      /* 原料区   */
#define RAW_Y     2315
#define ROUGH_X   1200      /* 粗加工区 */
#define ROUGH_Y     75
#define TMP_X       75      /* 暂存区   */
#define TMP_Y     1200

/* 启停区默认中心(仅供手动测试; 正式运行由串口屏 START/GOAL 发送) */
#define STOP1_X   2250      /* 启停区1 (上位机画布右上角方块) */
#define STOP1_Y   2250
#define STOP2_X   2250      /* 启停区2 (上位机画布右下角方块) */
#define STOP2_Y    150

/* ==========================================================================
 * 二、运行状态
 * ========================================================================== */
#define DIST_INF  0x7FFFFFFF            /* g/f 的"无穷大" */

static uint8_t  edge_blocked[RN_EDGE_NUM];   /* 1 = 该边被黑柱封锁 */
static int16_t  start_x, start_y;            /* 起点启停区中心(mm) */
static int16_t  goal_x,  goal_y;             /* 终点启停区中心(mm) */
static uint8_t  have_start, have_goal;       /* 是否已收到起点/终点 */

static PathPlanStep steps[STEP_MAX];         /* 任务步骤表 */
static int16_t  step_count;                  /* 实际步数(无解的步已被删除) */

static char     stop_text[HMI_T1_BUF_LEN];   /* 屏 t1 内容: 全路径节点号串 + 路网总长 */
static char     frame_buf[FRAME_BUF_LEN];    /* $STEP 帧缓冲 */

/* 全路径节点序列: 整个任务依次经过的所有路网节点(相邻重复已去重)
 * 口径与上位机 rn_path_vertex_seq() 一致 —— 含"直线穿过的中间节点" */
static uint8_t  path_seq[STEP_MAX * RN_NODE_NUM];
static int16_t  path_seq_len;

/* ---- 串口收包状态机 ---- */
static char     line_buf[PKT_LINE_BUF];      /* 当前消息缓冲(单条) */
static uint8_t  line_len;                    /* 当前消息长度 */
static uint8_t  pkt_err;                     /* 本包出现过非法行(整包作废) */
static uint8_t  pkt_new;                     /* 本包已有内容, 正在等待包边界 */
static uint32_t last_tick;                   /* 最后一个字节的到达时刻(HAL_GetTick) */
static uint8_t  goal_fallback;               /* 1 = 屏还没发过 GOAL 行, 终点暂时跟随起点 */
static int32_t  task_len;                    /* 本次任务的【路网总长】mm(不含启停区接入段) */

/* ---- 本包缓存: 整个包先缓存, 等"包结束"再统一处理(因此与屏上的发送顺序无关) ---- */
static uint8_t  pkt_have_start;              /* 本包出现过 START 行 */
static uint8_t  pkt_have_goal;               /* 本包出现过 GOAL 行 */
static int16_t  pkt_start_x, pkt_start_y;    /* 本包 START 的坐标 */
static int16_t  pkt_goal_x,  pkt_goal_y;     /* 本包 GOAL 的坐标 */
static int16_t  pkt_obs[OBS_MAX][2];         /* 本包缓存的障碍坐标 */
static uint8_t  pkt_obs_n;                   /* 本包已缓存的障碍条数 */

/* ---- 串口接收 DMA(USART1_RX -> DMA1_Channel5, 循环模式) ---- */
static uint8_t  rx_dma_buf[RX_DMA_BUF_SIZE]; /* DMA 环形接收缓冲 */
static uint16_t rx_dma_last;                 /* 上次已取出到哪个位置(索引) */

/* ==========================================================================
 * 三、小工具函数(全整型, 不用 sprintf, 省 Flash)
 * ========================================================================== */

/* 把整数按十进制写入缓冲区, 返回写入后的位置 */
static char* put_int(char *p, int32_t v)
{
    char     tmp[12];                       /* int32 最多 10 位数字 + 符号, 留足余量 */
    uint8_t  k = 0;
    uint32_t u;

    if (v < 0) { *p++ = '-'; u = (uint32_t)(-v); }
    else       { u = (uint32_t)v; }

    if (u == 0)
    {
        *p++ = '0';
        return p;
    }
    while (u > 0)
    {
        tmp[k++] = (char)('0' + (u % 10));
        u /= 10;
    }
    while (k > 0)
    {
        *p++ = tmp[--k];
    }
    return p;
}

/* 把字符串写入缓冲区, 返回写入后的位置 */
static char* put_str(char *p, const char *s)
{
    while (*s != '\0')
    {
        *p++ = *s++;
    }
    return p;
}

/* s 是否以 pre 开头 */
static uint8_t starts_with(const char *s, const char *pre)
{
    while (*pre != '\0')
    {
        if (*s++ != *pre++)
        {
            return 0;
        }
    }
    return 1;
}

/* 跳过空格与制表符 */
static const char* skip_space(const char *s)
{
    while (*s == ' ' || *s == '\t')
    {
        s++;
    }
    return s;
}

/* 解析 "x,y"(允许前后空格, 必须恰好两个十进制整数), 成功返回 1 */
static uint8_t atoi_xy(const char *s, int16_t *px, int16_t *py)
{
    const char *p = skip_space(s);
    int32_t     v;
    uint8_t     i, neg, digits;

    for (i = 0; i < 2; i++)
    {
        v = 0; neg = 0; digits = 0;
        if (*p == '-') { neg = 1; p++; }
        while (*p >= '0' && *p <= '9')
        {
            if (digits >= 5) return 0;       /* 单个整数最多 5 位, 防 int32 解析溢出 */
            v = v * 10 + (*p - '0');
            p++;
            digits++;
        }
        if (!digits) return 0;
        if (neg) v = -v;

        if (i == 0)                       /* 第一个数: 后面必须是逗号 */
        {
            *px = (int16_t)v;
            p = skip_space(p);
            if (*p != ',') return 0;
            p++;
        }
        else                              /* 第二个数 */
        {
            *py = (int16_t)v;
        }
    }
    p = skip_space(p);
    return (*p == '\0') ? 1 : 0;          /* 不允许有多余字符 */
}

/* ==========================================================================
 * 四、路网查询 / 黑柱封边 / 坐标吸附
 * ========================================================================== */

/* 节点 n 是否可用: 至少有一条未被封锁的边与它相连 */
static uint8_t node_open(uint8_t n)
{
    uint8_t e;
    for (e = 0; e < RN_EDGE_NUM; e++)
    {
        if (edge_blocked[e]) continue;
        if (EDGE_A[e] == n || EDGE_B[e] == n) return 1;
    }
    return 0;
}

/* 清空全部动态封锁(收到 START 行时调用 = 每包全量刷新) */
static void obs_clear(void)
{
    memset(edge_blocked, 0, sizeof(edge_blocked));
}

/* 封锁坐标 (x,y) 命中的全部路网边(容差 OBS_TOL_MM)
 * 返回: 1 = 合法(含"未摆放" 0,0 ); 0 = 非法(越界 / 容差内没命中任何边) */
static uint8_t obs_add(int16_t x, int16_t y)
{
    uint8_t  e, hit = 0;
    int16_t  d, lo, hi;

    if (x == 0 && y == 0) return 1;                              /* 该柱未摆放 */
    if (x < 0 || x > FIELD_MAX_MM || y < 0 || y > FIELD_MAX_MM) return 0;

    for (e = 0; e < RN_EDGE_NUM; e++)
    {
        if (NODE_X[EDGE_A[e]] == NODE_X[EDGE_B[e]])               /* 竖边: x 固定 */
        {
            d = (int16_t)(x - NODE_X[EDGE_A[e]]);
            if (d < 0) d = (int16_t)(-d);
            lo = (NODE_Y[EDGE_A[e]] < NODE_Y[EDGE_B[e]]) ? NODE_Y[EDGE_A[e]] : NODE_Y[EDGE_B[e]];
            hi = (NODE_Y[EDGE_A[e]] > NODE_Y[EDGE_B[e]]) ? NODE_Y[EDGE_A[e]] : NODE_Y[EDGE_B[e]];
            if (d <= OBS_TOL_MM && y >= lo && y <= hi)
            {
                edge_blocked[e] = 1;                              /* 整条边封锁 */
                hit++;
            }
        }
        else                                                      /* 横边: y 固定 */
        {
            d = (int16_t)(y - NODE_Y[EDGE_A[e]]);
            if (d < 0) d = (int16_t)(-d);
            lo = (NODE_X[EDGE_A[e]] < NODE_X[EDGE_B[e]]) ? NODE_X[EDGE_A[e]] : NODE_X[EDGE_B[e]];
            hi = (NODE_X[EDGE_A[e]] > NODE_X[EDGE_B[e]]) ? NODE_X[EDGE_A[e]] : NODE_X[EDGE_B[e]];
            if (d <= OBS_TOL_MM && x >= lo && x <= hi)
            {
                edge_blocked[e] = 1;                              /* 整条边封锁 */
                hit++;
            }
        }
    }
    return (hit > 0) ? 1 : 0;
}

/* 把坐标吸附到最近的可利用路网节点(平方距离比较, 全整型; 平局取小编号)
 * 返回: 1 = 成功; 0 = 没有任何可用节点 */
static uint8_t snap_node(int16_t x, int16_t y, uint8_t *out)
{
    uint8_t i, best = 0, found = 0;
    int32_t dx, dy, d, best_d = 0;

    for (i = 0; i < RN_NODE_NUM; i++)
    {
        if (!node_open(i)) continue;
        dx = (int32_t)x - (int32_t)NODE_X[i];
        dy = (int32_t)y - (int32_t)NODE_Y[i];
        d  = dx * dx + dy * dy;
        if (!found || d < best_d)
        {
            found  = 1;
            best_d = d;
            best   = i;
        }
    }
    if (found) *out = best;
    return found;
}

/* 曼哈顿距离(与边权同量纲, 可采纳启发式) */
static int32_t manhattan(uint8_t a, uint8_t b)
{
    int32_t dx = (int32_t)NODE_X[a] - (int32_t)NODE_X[b];
    int32_t dy = (int32_t)NODE_Y[a] - (int32_t)NODE_Y[b];
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    return dx + dy;
}

/* ==========================================================================
 * 五、A*(节点图: 4 邻域沿边 + 曼哈顿启发式 + 数组扫描取最小 f, 全整型)
 * ========================================================================== */
/* 计算 s -> g 的最短路(只走未封锁的边)
 *   path    : 输出节点序列(数组下标 0~8, 含首尾, 保留共线中间节点)
 *   path_len: 输出节点个数
 *   返回    : 1 = 有解; 0 = 无解(路网不连通 / 被黑柱切断) */
static uint8_t astar(uint8_t s, uint8_t g, uint8_t *path, int16_t *path_len)
{
    int32_t  gs[RN_NODE_NUM];       /* 起点到各节点的实际代价 g */
    int32_t  fs[RN_NODE_NUM];       /* f = g + h */
    int8_t   par[RN_NODE_NUM];      /* 父节点(-1 = 无) */
    uint8_t  open_[RN_NODE_NUM];    /* 1 = 在待扩展表里 */
    uint8_t  closed[RN_NODE_NUM];   /* 1 = 已扩展 */
    uint8_t  i, e, cur, nb, best;
    int32_t  bestf, ng;
    int16_t  k, m;
    uint8_t  tmp;

    for (i = 0; i < RN_NODE_NUM; i++)
    {
        gs[i] = DIST_INF;  fs[i] = DIST_INF;  par[i] = -1;
        open_[i] = 0;      closed[i] = 0;
    }
    gs[s] = 0;
    fs[s] = manhattan(s, g);
    open_[s] = 1;

    for (;;)
    {
        /* 线性扫描取 f 最小者; f 相等时靠"扫描顺序 + 严格小于"天然取节点号小者,
           与上位机实测的平局选择一致(已用同一场景逐点对拍验证) */
        best  = 0xFF;
        bestf = DIST_INF;
        for (i = 0; i < RN_NODE_NUM; i++)
        {
            if (open_[i] && !closed[i] && fs[i] < bestf)
            {
                bestf = fs[i];
                best  = i;
            }
        }
        if (best == 0xFF) return 0;             /* 待扩展表空 -> 无通路 */

        cur        = best;
        open_[cur] = 0;
        closed[cur] = 1;
        if (cur == g) break;                    /* 到达终点 */

        /* 按边表顺序 E0..E11 扩展邻居(等长平局的处理顺序与对拍结论一致) */
        for (e = 0; e < RN_EDGE_NUM; e++)
        {
            if (edge_blocked[e]) continue;
            if (EDGE_A[e] == cur)      { nb = EDGE_B[e]; }
            else if (EDGE_B[e] == cur) { nb = EDGE_A[e]; }
            else                       { continue; }
            if (closed[nb]) continue;

            ng = gs[cur] + (int32_t)EDGE_LEN[e];
            if (ng < gs[nb])                    /* 严格更小才更新(等长不改父节点) */
            {
                gs[nb]    = ng;
                par[nb]   = (int8_t)cur;
                fs[nb]    = ng + manhattan(nb, g);
                open_[nb] = 1;
            }
        }
    }

    /* 回溯: g -> ... -> s */
    k = 0;
    cur = g;
    path[k] = g;
    k++;
    while (cur != s)
    {
        cur = (uint8_t)par[cur];
        path[k] = cur;
        k++;
    }
    /* 反转为 s -> g */
    for (m = 0; m < (k / 2); m++)
    {
        tmp             = path[m];
        path[m]         = path[k - 1 - m];
        path[k - 1 - m] = tmp;
    }
    *path_len = k;
    return 1;
}

/* ==========================================================================
 * 六、任务序列(与上位机 rebuild_task_sequence 完全一致)
 * ========================================================================== */

/* 追加一步
 *   action    : 动作码
 *   from_node : 上一步停靠节点(本段起点)
 *   to_node   : 本步停靠节点(本段终点)
 *   with_head : 非0 -> 在本段最前面插入"起点启停区中心"(接入段)
 *   with_tail : 非0 -> 在本段最后面追加"终点启停区中心"(接入段)
 * 返回: 1 = 追加成功; 0 = 本段无解 或 步骤表已满(调用方据此删除本步) */
static uint8_t add_step(uint8_t action, uint8_t from_node, uint8_t to_node,
                        uint8_t with_head, uint8_t with_tail)
{
    uint8_t       path[RN_NODE_NUM];
    int16_t       n = 0;
    int16_t       k;
    PathPlanStep *st;

    if (step_count >= STEP_MAX) return 0;
    if (!astar(from_node, to_node, path, &n)) return 0;     /* 该段无解 -> 删除本步 */

    for (k = 1; k < n; k++)                                 /* 累加本段路网长度(不含接入段) */
    {
        task_len += manhattan(path[k - 1], path[k]);
    }
    for (k = 0; k < n; k++)                                 /* 记录全路径节点(相邻重复去重) */
    {
        if (path_seq_len > 0 && path_seq[path_seq_len - 1] == path[k]) continue;
        if (path_seq_len < (int16_t)(STEP_MAX * RN_NODE_NUM))
        {
            path_seq[path_seq_len] = path[k];
            path_seq_len++;
        }
    }

    st            = &steps[step_count];
    st->action    = action;
    st->stop_node = to_node;
    st->count     = 0;

    if (with_head)                                          /* 接入段: 起点中心 -> 路网点 */
    {
        st->x[st->count] = start_x;
        st->y[st->count] = start_y;
        st->count++;
    }
    for (k = 0; k < n; k++)                                 /* A* 节点序列(保留共线点) */
    {
        if (st->count >= STEP_POINT_MAX) break;
        st->x[st->count] = NODE_X[path[k]];
        st->y[st->count] = NODE_Y[path[k]];
        st->count++;
    }
    if (with_tail)                                          /* 接入段: 路网点 -> 终点中心 */
    {
        if (st->count < STEP_POINT_MAX)
        {
            st->x[st->count] = goal_x;
            st->y[st->count] = goal_y;
            st->count++;
        }
    }
    step_count++;
    return 1;
}

/* 生成整任务(2 + 3*TASK_ROUNDS 步); 返回 1 = 成功, 0 = 起点/终点不可用 */
static uint8_t build_task(void)
{
    uint8_t cur_node;                       /* 上一步"保留成功"的停靠节点 */
    uint8_t node;
    uint8_t r;

    step_count   = 0;
    task_len     = 0;
    path_seq_len = 0;
    if (!have_start) return 0;                  /* 连起点都没收到 -> 无法计算 */
    /* 说明: 若屏幕从来没发过 GOAL 行, 终点已由 START 行兜底为"起点坐标"(见 handle_line) */
    if (!have_goal) return 0;

    /* 第 0 步: 起点启停区(本段 = 起点中心 + 起始节点, 带起点接入段) */
    if (!snap_node(start_x, start_y, &cur_node)) return 0;   /* 起点无可用节点 -> 整任务作废 */
    steps[0].action    = ACT_START;
    steps[0].stop_node = cur_node;
    steps[0].count     = 0;
    if (!(start_x == NODE_X[cur_node] && start_y == NODE_Y[cur_node]))
    {
        steps[0].x[steps[0].count] = start_x;               /* 接入段首点 = 车的当前位置 */
        steps[0].y[steps[0].count] = start_y;
        steps[0].count++;
    }
    steps[0].x[steps[0].count] = NODE_X[cur_node];
    steps[0].y[steps[0].count] = NODE_Y[cur_node];
    steps[0].count++;
    step_count = 1;
    path_seq[path_seq_len] = cur_node;          /* 全路径的第 1 个节点 = 起点节点 */
    path_seq_len++;

    /* 第 1 步: 读取二维码板 */
    if (snap_node(QR_X, QR_Y, &node) && add_step(ACT_QR, cur_node, node, 0, 0))
    {
        cur_node = node;
    }
    /* 之后每轮: 原料区取料 -> 粗加工区加工 -> 暂存区放料 */
    for (r = 0; r < TASK_ROUNDS; r++)
    {
        if (snap_node(RAW_X, RAW_Y, &node) && add_step(ACT_RAW, cur_node, node, 0, 0))
        {
            cur_node = node;
        }
        if (snap_node(ROUGH_X, ROUGH_Y, &node) && add_step(ACT_ROUGH, cur_node, node, 0, 0))
        {
            cur_node = node;
        }
        if (snap_node(TMP_X, TMP_Y, &node) && add_step(ACT_TMP, cur_node, node, 0, 0))
        {
            cur_node = node;
        }
    }
    /* 最后一步: 返回终点启停区(带终点接入段 = 车要停进启停区中心) */
    if (snap_node(goal_x, goal_y, &node) && add_step(ACT_BACK, cur_node, node, 0, 1))
    {
        cur_node = node;
    }
    (void)cur_node;                                          /* 末次赋值不再使用, 消除告警 */
    return (step_count >= 1) ? 1 : 0;
}

/* ==========================================================================
 * 七、输出: 屏 t1 文本 / $STEP 帧
 * ========================================================================== */

/* 生成屏 t1 内容: 【全路径】依次经过的所有路网节点号(1~9)用 '-' 连接, 末尾追加路网总长(mm)
 * 口径与上位机"路网点顺序"那行完全一致(含直线穿过的中间节点, 相邻重复已去重)
 * 例(标准地图, 起点=终点=启停区1, 两根黑柱):
 *     "7-8-5-4-5-8-5-2-3-6-5-8-5-2-3-6-5-8-7,16800" */
static void build_stop_text(void)
{
    char    *p = stop_text;
    int16_t  i;

    if (step_count <= 0 || path_seq_len <= 0)
    {
        p = put_str(p, "ERR");
        *p = '\0';
        return;
    }
    for (i = 0; i < path_seq_len; i++)                      /* 全路径节点号(显示用 1~9) */
    {
        if (i > 0)
        {
            *p++ = '-';
        }
        p = put_int(p, (int32_t)path_seq[i] + 1);
    }
    *p++ = ',';                                             /* 追加路网总长(mm) */
    p = put_int(p, task_len);
    *p = '\0';
}

/* 组一步的串口帧: $STEP,动作码,点数,x1,y1,...,xn,yn#
 * (与上位机导出格式逐字节一致; 本段最后一点 = 停靠点,
 *  最后一步的最后一点 = 终点启停区中心)
 * 注意: 返回内部静态缓冲, 下一次调用会被覆盖 */
const char* PathPlan_GetStepFrame(int16_t i)
{
    char               *p = frame_buf;
    int16_t             k;
    const PathPlanStep *st;

    if (i < 0 || i >= step_count)
    {
        frame_buf[0] = '\0';
        return frame_buf;
    }
    st = &steps[i];
    p = put_str(p, "$STEP,");
    p = put_int(p, (int32_t)st->action);
    *p++ = ',';
    p = put_int(p, (int32_t)st->count);
    for (k = 0; k < st->count; k++)
    {
        *p++ = ',';
        p = put_int(p, st->x[k]);
        *p++ = ',';
        p = put_int(p, st->y[k]);
    }
    *p++ = '#';
    *p   = '\0';
    return frame_buf;
}

/* 向淘晶驰串口屏发一条: t1.txt="xxx" + 结束符 */
void PathPlan_SendText(const char *txt)
{
    char  out[HMI_T1_BUF_LEN + 24];
    char *p = out;

    p = put_str(p, HMI_T1);          /* "t1" */
    p = put_str(p, ".txt=\"");
    p = put_str(p, txt);
    p = put_str(p, "\"");
    p = put_str(p, HMI_END);         /* 默认 "\r\n" */
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)out, (uint16_t)(p - out), 100);
}

/* ==========================================================================
 * 八、对外运行入口
 * ========================================================================== */

/* 生成整任务, 并把"停靠节点号串"发到屏 t1; 返回 1 = 成功 */
uint8_t PathPlan_RunTask(void)
{
    if (!build_task())
    {
        PathPlan_SendText("ERR");               /* 起点/终点不可用 -> 整任务作废 */
        return 0;
    }
    build_stop_text();
    PathPlan_SendText(stop_text);
    return 1;
}

/* 手动测试用: 起点 -> 终点 单段 A*(同样带两端接入段), 结果发到屏 t1 */
uint8_t PathPlan_RunOneLeg(void)
{
    uint8_t s, g;

    if (!have_start || !have_goal)
    {
        PathPlan_SendText("ERR");
        return 0;
    }
    if (!snap_node(start_x, start_y, &s) || !snap_node(goal_x, goal_y, &g))
    {
        PathPlan_SendText("ERR");
        return 0;
    }
    step_count   = 0;
    task_len     = 0;
    path_seq_len = 0;
    if (!add_step(ACT_START, s, g, 1, 1))
    {
        step_count = 0;
        PathPlan_SendText("NO_PATH");           /* 该段无通路 */
        return 0;
    }
    build_stop_text();
    PathPlan_SendText(stop_text);
    return 1;
}

/* ==========================================================================
 * 九、串口收包解析(淘晶驰串口屏 -> STM32)
 * 【消息切分规则】屏幕是把攒好的内容"一次性整串"发来的, 可能一个结束符都不带,
 *                 所以本解析按【关键字】切分消息:
 *   1) 字节流里出现 START: / GOAL: / OBS_ADD: => 视为"一条新消息开始",
 *      上一条消息就在关键字出现处结束(所以你 t0 里几行之间有没有 \r\n 都行);
 *   2) 收到 \n 也作为一条消息结束(兼容以后给屏加结束符);
 *   3) 最后一条消息由"500ms 无新字节"收尾 -> 随后整包统一处理;
 * 【★包尾统一处理(与屏上的发送顺序完全无关)】:
 *   · 本包出现过 START -> 先清空动态封锁表(每包全量刷新), 再逐条应用本包障碍;
 *   · 本包没有 START   -> 只追加本包障碍(一次按一个障碍按钮的用法照旧有效);
 *   · 再更新起点/终点(若从未收到过 GOAL, 终点自动跟随起点), 最后算路+刷屏;
 *   认识的内容:
 *     START:x,y      起点启停区中心(通常一个任务只发一次)
 *     GOAL:x,y       终点启停区中心(可以不发)
 *     OBS_ADD:x,y    黑柱坐标(命中哪条边就封哪条边); 0,0 = 该柱未摆放
 *   非法: 坐标越界 / 容差内没命中任何边 / 数值格式错 / 单条过长 / 一包障碍超过 OBS_MAX
 *         -> 整包作废, 屏显示 ERR
 * ========================================================================== */

/* 处理一整行(不含 \r\n) */
static void handle_line(const char *ln)
{
    int16_t x, y;

    if (ln[0] == '\0') return;                       /* 空行: 由调用处当作包边界 */

    if (starts_with(ln, "START:"))
    {
        if (atoi_xy(ln + 6, &x, &y))
        {
            pkt_start_x    = x;                      /* 只入缓存, 等包结束统一处理 */
            pkt_start_y    = y;
            pkt_have_start = 1;
            pkt_new        = 1;
        }
        else
        {
            pkt_err = 1;
        }
    }
    else if (starts_with(ln, "GOAL:"))
    {
        if (atoi_xy(ln + 5, &x, &y))
        {
            pkt_goal_x    = x;
            pkt_goal_y    = y;
            pkt_have_goal = 1;
            pkt_new       = 1;
        }
        else
        {
            pkt_err = 1;
        }
    }
    else if (starts_with(ln, "OBS_ADD:"))
    {
        if (!atoi_xy(ln + 8, &x, &y))
        {
            pkt_err = 1;                             /* 数值格式错 */
        }
        else if (x == 0 && y == 0)
        {
            pkt_new = 1;                             /* 0,0 = 该柱未摆放: 合法, 不入缓存 */
        }
        else if (x < 0 || x > FIELD_MAX_MM || y < 0 || y > FIELD_MAX_MM)
        {
            pkt_err = 1;                             /* 坐标越界 */
        }
        else if (pkt_obs_n >= OBS_MAX)
        {
            pkt_err = 1;                             /* 一包障碍条数超上限 */
        }
        else
        {
            pkt_obs[pkt_obs_n][0] = x;               /* 入缓存, 包结束再统一封边 */
            pkt_obs[pkt_obs_n][1] = y;
            pkt_obs_n++;
            pkt_new = 1;
        }
    }
    /* 其它行: 直接忽略(不参与协议, 也不报错) */
}

/* ★整包结束: 统一处理本包缓存(与屏上的发送顺序完全无关)
 *   ① 本包出现过 START -> 先清空动态封锁表(每包全量刷新)
 *   ② 再按顺序逐条应用本包的障碍(封整条边; 越界/150mm 内没命中 -> 整包作废)
 *   ③ 再更新起点/终点(从未收到过 GOAL 时, 终点自动跟随起点)
 *   ④ 最后重算整任务并刷新屏 t1 */
static void packet_finish(void)
{
    uint8_t i;
    uint8_t ok = 1;

    if (!pkt_new && !pkt_err)
    {
        return;                                      /* 空包: 什么都不做 */
    }

    if (!pkt_err)
    {
        if (pkt_have_start)
        {
            obs_clear();                             /* ① 本包带 START -> 全量刷新 */
        }
        for (i = 0; i < pkt_obs_n; i++)              /* ② 逐条应用本包障碍 */
        {
            if (!obs_add(pkt_obs[i][0], pkt_obs[i][1]))
            {
                ok = 0;
                break;
            }
        }
        if (ok && pkt_have_start)                    /* ③ 起点(+终点兜底) */
        {
            start_x    = pkt_start_x;
            start_y    = pkt_start_y;
            have_start = 1;
            if (goal_fallback || !have_goal)
            {
                goal_x        = start_x;
                goal_y        = start_y;
                have_goal     = 1;
                goal_fallback = 1;
            }
        }
        if (ok && pkt_have_goal)                     /* ③ 真实终点 */
        {
            goal_x        = pkt_goal_x;
            goal_y        = pkt_goal_y;
            have_goal     = 1;
            goal_fallback = 0;
        }
    }

    if (pkt_err || !ok)
    {
        PathPlan_SendText("ERR");                    /* 整包作废 */
    }
    else
    {
        (void)PathPlan_RunTask();                    /* ④ 重算整任务并刷新屏 t1 */
    }

    /* 清本包状态, 准备下一包 */
    pkt_err        = 0;
    pkt_new        = 0;
    line_len       = 0;
    pkt_obs_n      = 0;
    pkt_have_start = 0;
    pkt_have_goal  = 0;
}

/* 缓冲区末尾是否刚好是一个"新消息关键字"(OBS_ADD: / GOAL: / START:)
 * 若是, 且关键字前面还有内容, 返回该关键字长度(用于把上一条消息切出来), 否则返回 0 */
static uint8_t tail_keyword_len(void)
{
    if (line_len > 8 && starts_with(&line_buf[line_len - 8], "OBS_ADD:")) return 8;
    if (line_len > 5 && starts_with(&line_buf[line_len - 5], "GOAL:"))    return 5;
    if (line_len > 6 && starts_with(&line_buf[line_len - 6], "START:"))   return 6;
    return 0;
}

/* 逐字节喂入(在 USART1 接收中断里调用本函数即可, 本文件不碰中断底层) */
void PathPlan_OnRxByte(uint8_t b)
{
    last_tick = HAL_GetTick();

    if (b == '\r') return;                           /* 回车: 直接丢弃 */
    if (b == 0xFF) return;                           /* 0xFF: 丢弃(屏的结束符字节, 不算内容) */
    if (b == 0x00) return;                           /* 0x00: 丢弃(屏若用 prints x,N 补零) */
    if (b == '\n')                                   /* 换行 = 一行结束 */
    {
        if (line_len == 0)
        {
            if (pkt_new || pkt_err)
            {
                packet_finish();                     /* 空行 = 包边界 */
            }
        }
        else
        {
            line_buf[line_len] = '\0';
            handle_line(line_buf);
            line_len = 0;
        }
        return;
    }
    if (line_len < (PKT_LINE_BUF - 1))
    {
        uint8_t keep;

        line_buf[line_len] = (char)b;
        line_len++;

        keep = tail_keyword_len();
        if (keep > 0)                                /* 缓冲区末尾出现了新消息的关键字 */
        {
            line_buf[line_len - keep] = '\0';
            handle_line(line_buf);                   /* 关键字之前的内容 = 上一条消息 */
            memmove(line_buf, &line_buf[line_len - keep], keep);
            line_len = keep;                         /* 关键字留在缓冲区开头, 继续攒新消息 */
        }
    }
    else
    {
        line_len = 0;                                /* 单条过长: 丢弃并报非法 */
        pkt_err  = 1;
    }
}

/* 在 while(1) 里调用: 200ms 空闲同样算一个包边界 */
void PathPlan_Tick(void)
{
    /* 注意: 屏可能一个结束符都不发, 所以这里不能只看 pkt_new/pkt_err,
       必须把"缓冲区里还攒着没处理的字节"(line_len > 0) 也算作待处理内容 */
    if (line_len == 0 && !pkt_new && !pkt_err) return;
    if ((HAL_GetTick() - last_tick) < PKT_IDLE_MS) return;

    if (line_len > 0)                                /* 末尾残留半行: 也按一行处理 */
    {
        line_buf[line_len] = '\0';
        handle_line(line_buf);
        line_len = 0;
    }
    packet_finish();
}

/* ==========================================================================
 * 十、串口接收(USART1_RX 由 DMA1_Channel5 循环搬运到 rx_dma_buf)
 *   · PathPlan_StartRxDma(): HAL_UART_Receive_DMA(循环模式, 一次启动永久有效)
 *   · PathPlan_PollRxDma():  主循环里调用, 按 DMA 剩余计数算出"新到哪了",
 *                            把新字节逐个喂给 PathPlan_OnRxByte()
 *   ★这样 CPU 再忙(算路径 / 发 t1 / 以后的底盘代码)也不会丢字节
 * ========================================================================== */

/* 启动 DMA 循环接收(PathPlan_Init() 里已调用; 重复调用也没关系) */
void PathPlan_StartRxDma(void)
{
    rx_dma_last = 0;
    (void)HAL_UART_Receive_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
}

/* 在 while(1) 里调用: 取出 DMA 已经收到、但还没交给解析器的字节 */
void PathPlan_PollRxDma(void)
{
    uint16_t pos;

    /* DMA 剩余待搬运字节数 -> 反算出"当前写指针位置" */
    pos = (uint16_t)(RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));

    while (rx_dma_last != pos)                   /* 逐个取走新到的字节 */
    {
        PathPlan_OnRxByte(rx_dma_buf[rx_dma_last]);
        rx_dma_last++;
        if (rx_dma_last >= RX_DMA_BUF_SIZE)
        {
            rx_dma_last = 0;                     /* 环形绕回 */
        }
    }
}

/* ==========================================================================
 * 十一、初始化与查询接口
 * ========================================================================== */
void PathPlan_Init(void)
{
    memset(edge_blocked, 0, sizeof(edge_blocked));
    memset(steps, 0, sizeof(steps));

    start_x = STOP1_X;                       /* 默认起点: 启停区1 中心(手动测试用) */
    start_y = STOP1_Y;
    goal_x  = STOP2_X;                       /* 默认终点: 启停区2 中心(手动测试用) */
    goal_y  = STOP2_Y;
    have_start = 0;                          /* 正式运行必须等屏发 START/GOAL 行 */
    have_goal  = 0;

    step_count = 0;
    line_len   = 0;
    pkt_err    = 0;
    pkt_new    = 0;
    last_tick  = HAL_GetTick();

    pkt_have_start = 0;
    pkt_have_goal  = 0;
    pkt_obs_n      = 0;

    stop_text[0] = '\0';
    frame_buf[0] = '\0';

    (void)YELLOW_BLOCK;                      /* 4 个黄块仅作地图记录, 不参与 A* */

    PathPlan_StartRxDma();                   /* 启动 USART1 的 DMA 循环接收 */
}

uint8_t PathPlan_SetStart(int16_t x, int16_t y)
{
    start_x    = x;
    start_y    = y;
    have_start = 1;
    return 1;
}

uint8_t PathPlan_SetGoal(int16_t x, int16_t y)
{
    goal_x    = x;
    goal_y    = y;
    have_goal = 1;
    return 1;
}

uint8_t PathPlan_ObsAdd(int16_t x, int16_t y)
{
    return obs_add(x, y);
}

void PathPlan_ClearObs(void)
{
    obs_clear();
}

int16_t PathPlan_GetStepCount(void)
{
    return step_count;
}

uint8_t PathPlan_GetStepAction(int16_t i)
{
    if (i < 0 || i >= step_count) return 0xFF;
    return steps[i].action;
}

uint8_t PathPlan_GetStepStopNode(int16_t i)
{
    if (i < 0 || i >= step_count) return 0xFF;
    return steps[i].stop_node;               /* 数组下标 0~8, 对外显示时 +1 */
}

int16_t PathPlan_GetStepPointCount(int16_t i)
{
    if (i < 0 || i >= step_count) return 0;
    return steps[i].count;
}

int16_t PathPlan_GetStepPointX(int16_t i, int16_t k)
{
    if (i < 0 || i >= step_count) return 0;
    if (k < 0 || k >= steps[i].count) return 0;
    return steps[i].x[k];
}

int16_t PathPlan_GetStepPointY(int16_t i, int16_t k)
{
    if (i < 0 || i >= step_count) return 0;
    if (k < 0 || k >= steps[i].count) return 0;
    return steps[i].y[k];
}

const char* PathPlan_GetStopText(void)
{
    return stop_text;
}

/* ==========================================================================
 * 【验收对照表】标准地图 + 起点 = 终点 = 启停区1(2250,2250)
 * ----------------------------------------------------------------------------
 *  屏 t1 格式: "全路径节点号串,路网总长mm"  (= 上位机"路网点顺序"那行 + 总长)
 *   ① 无黑柱(整包只有 START):
 *        t1 = 7-4-7-8-5-2-3-6-5-8-5-2-3-6-5-8-7,15000
 *        各步本段(1~9) = 7-4 / 4-7-8 / 8-5-2 / 2-3-6 / 6-5-8 / 8-5-2 / 2-3-6 / 6-5-8-7
 *   ② 黑柱 (2100,700) + (2100,1600):
 *        t1 = 7-8-5-4-5-8-5-2-3-6-5-8-5-2-3-6-5-8-7,16800   <-- 绕开右列, 总长变大
 *        各步本段(1~9) = 7-8-5-4 / 4-5-8 / 8-5-2 / 2-3-6 / 6-5-8 / 8-5-2 / 2-3-6 / 6-5-8-7
 *  说明:
 *    · 黑柱(2100,700)  -> 封锁边 E6 (0,3) 即 x=2100, y 300..1200
 *    · 黑柱(2100,1600) -> 封锁边 E7 (3,6) 即 x=2100, y 1200..2100
 *    · 第 0 步本段首点 = 起点启停区中心(2250,2250)
 *      最后一步本段末点 = 终点启停区中心(与起点同点)
 *    · 参考: 各步"停靠节点" = 7-4-8-2-6-8-2-6-7 (两种情况都一样),
 *      所以要靠"全路径串 + 总长"来核对黑柱封边有没有生效
 *    · 以上与上位机(Python)实测输出【逐点一致】
 * ========================================================================== */






