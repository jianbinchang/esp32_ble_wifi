/*!
///////////////////////////////////////////////////////////
 *  \file      jcc_ellipse.c
 *  \brief     jcc ellipse
 *  \details   Implementation ellipse fit
 *  \author    张永强
 *  \version   0.1
 *  \date      23-09-2016 14:57
///////////////////////////////////////////////////////////
*/
#include <string.h>
#include <math.h>


#define ROW   5
#define COL   (ROW+1)

typedef float jccfloat_t;

static jccfloat_t _g_matrix[ROW][COL];

static int _equal(jccfloat_t a, jccfloat_t b)
{
    return fabs(a - b) < 1e-9;
}

void ellipse_reset(void)
{
    int row, column;
    for(row = 0 ; row < ROW ; row++)
    {
        for(column = 0 ; column < COL ; column++)
            _g_matrix[row][column] = 0.0f;
    }
}

void ellipse_rawdata(float x, float y)
{
    jccfloat_t V[ROW];
    int row, column;
    V[0] = x * x;
    V[1] = y * y;
    V[2] = x;
    V[3] = y;
    V[4] = 1.0;
    ///累加V x Vt矩阵
    for(row = 0 ; row < ROW ; row++)
    {
        for(column = 0 ; column < ROW; column++)
        {
            _g_matrix[row][column] += V[row] * V[column];
        }
        _g_matrix[row][column] += 1;
    }
}



///交换行
static void _matrix_swaprow(int row1, int row2)
{
    int column;
    jccfloat_t tmp;
    for(column = 0 ; column < COL ; column++)
    {
        tmp = _g_matrix[row1][column];
        _g_matrix[row1][column] = _g_matrix[row2][column];
        _g_matrix[row2][column] = tmp;
    }
}

///每次找第row行及以下，col列中元素绝对值最大的列与第row行交换。
static void _matrix_movebiggestelement2top(int s_row, int s_column)
{
    int row;
    for(row = s_row + 1 ; row < ROW ; row++)
    {
        if( fabs(_g_matrix[s_row][s_column]) < fabs(_g_matrix[row][s_column]))
        {
            _matrix_swaprow(s_row, row);
        }
    }
}

///高斯消元法，求行阶梯型矩阵
static int _matrix_gausselimination(void)
{
    int row, column, i, j;
    jccfloat_t tmp;

    for(row = 0, column = 0 ; row < ROW - 1 && column < COL - 1; row++, column++)
    {
        _matrix_movebiggestelement2top(row, column);
        if(_equal(_g_matrix[row][column], 0.0f)) ///如果col列中的元素全为0，那么则处理col + 1列，row不变。
        {
            row--;
            continue;
        }
        for(i = row + 1 ; i < ROW ; i++)
        {
            if(_equal(_g_matrix[i][column], 0.0f)) continue;
            tmp = _g_matrix[i][column] / _g_matrix[row][column];
            for(j = column ; j < COL ; j++)
            {
                _g_matrix[i][j] -= _g_matrix[row][j] * tmp;
            }
        }
    }
    return 1;
}

///求行最简型矩阵
static int _matrix_rowsimplify(void)
{
    int c = COL;
    int row, column, k, s, t;
    jccfloat_t tmp;
    for(row = 0, column = 0; row < ROW && column < COL; row++, column++)
    {
        if(_equal(_g_matrix[row][column], 0))
        {
            row--;
            continue;
        }
        c--;
        tmp = 1 / _g_matrix[row][column];
        for(k = column; k < COL; k++) _g_matrix[row][k] *= tmp;
        for(s = 0; s < row; s++)
        {
            if(_equal(_g_matrix[s][column], 0)) continue;
            tmp = _g_matrix[s][column] / _g_matrix[row][column];
            for(t = column; t < COL; t++) _g_matrix[s][t] -= _g_matrix[row][t] * tmp;
        }
    }
    return c;
}

///矩阵求解
static void _matrix_solve(jccfloat_t* C, jccfloat_t* sol)
{
    int row, column, i;
    int any_sol[ROW];
    //memset(any_sol, 0, ROW);
    memset(any_sol, 0, ROW * sizeof(int));
    for(row = 0, column = 0 ; row < ROW && column < COL ; row++, column++)
    {
        if(_equal(_g_matrix[row][column], 0.0f))
        {
            any_sol[column] = 1;
            row--;
        }
    }
    row = 0;
    for(column = 0 ; column < COL - 1 ; column++)
    {
        if(any_sol[column] == 1)
        {
            sol[column] = C[column];
        }
        else
        {
            sol[column] = _g_matrix[row][COL - 1];
            for(i = column + 1 ; i < COL - 1 ; i++)
            {
                if(any_sol[i] == 1 && !_equal(_g_matrix[row][i], 0.0f))
                {
                    sol[column] -= _g_matrix[row][i] * C[i];
                }
            }
            row++;
        }
    }
}
static int _notnan(jccfloat_t d)
{
    return (d == d);              /* IEEE: only NaN is not equal to itself */
}

void ellipse_process(float radius, float *sx, float *sy, float *ox, float *oy)
{
    jccfloat_t C[ROW] = { -1.0f, -1.0f, 0, 0, 0};
    jccfloat_t Res[ROW];
    jccfloat_t k, f;

    ///任意解
    C[4] = radius * radius;

    _matrix_gausselimination();
    _matrix_rowsimplify();
    _matrix_solve(C, Res);

    k = (Res[2] * Res[2] / Res[0] + Res[3] * Res[3] / Res[1] - 4 * Res[4]) / (4 * radius * radius);

    f = sqrt(Res[0] / k);
    if(_notnan(f)) *sx = f;
    f = sqrt(Res[1] / k);
    if(_notnan(f))  *sy = f;

    f = Res[2] / (2 * Res[0]);
    if(_notnan(f))  *ox = f;
    f = Res[3] / (2 * Res[1]);
    if(_notnan(f)) *oy = f;
}




///测试代码
#if 0
#include <stdio.h>
//模拟磁场测量数据 最佳偏移量为 -15 15 -25左右
static const  jccfloat_t testdata2[]
=
{
//-------yaw-----
    38.66, 24.6055, 0.391088,
    31.5841, 28.9861, 0.473838,
    23.929, 31.939, 0.719569,
    15.9273, 33.3745, 1.12082,
    7.82219, 33.249, 1.6654,
    -0.140175, 31.5664, 2.33675,
    -7.7178, 28.3777, 3.11449,
    -14.6804, 23.7799, 3.97498,
    -20.8166, 17.9126, 4.89207,
    -25.9397, 10.9541, 5.83789,
    -29.8942, 3.11583, 6.78372,
    -32.5599, -5.36401, 7.70081,
    -33.8558, -14.2278, 8.5613,
    -33.7426, -23.2062, 9.33904,
    -32.2236, -32.0264, 10.0104,
    -29.345, -40.4204, 10.555,
    -25.1944, -48.1332, 10.9562,
    -19.8977, -54.9304, 11.202,
    -13.616, -60.6055, 11.2847,
    -6.54009, -64.9861, 11.202,
    1.115, -67.939, 10.9562,
    9.11669, -69.3745, 10.555,
    17.2219, -69.249, 10.0104,
    25.1842, -67.5664, 9.33904,
    32.7618, -64.3777, 8.5613,
    39.7245, -59.7799, 7.70081,
    45.8606, -53.9126, 6.78372,
    50.9837, -46.9541, 5.83789,
    54.9382, -39.1158, 4.89207,
    57.6039, -30.636, 3.97498,
    58.8999, -21.7722, 3.11449,
    58.7866, -12.7938, 2.33676,
    57.2676, -3.97361, 1.6654,
    54.3891, 4.42041, 1.12082,
    50.2384, 12.1332, 0.719571,
    44.9417, 18.9304, 0.473835,
    38.66, 24.6055, 0.391088,
//-------roll-----
    38.66, 24.6055, 0.391088,
    38.66, 29.105, 8.8938,
    38.66, 32.1732, 18.1138,
    38.66, 33.717, 27.7709,
    38.66, 33.6893, 37.5716,
    38.66, 32.0911, 47.2183,
    38.66, 28.9709, 56.4178,
    38.66, 24.4235, 64.8905,
    38.66, 18.5871, 72.3791,
    38.66, 11.639, 78.656,
    38.66, 3.79034, 83.5304,
    38.66, -4.7204, 86.8543,
    38.66, -13.6346, 88.5267,
    38.66, -22.6815, 88.4967,
    38.66, -31.5861, 86.7653,
    38.66, -40.078, 83.3851,
    38.66, -47.899, 78.4588,
    38.66, -54.8115, 72.136,
    38.66, -60.6055, 64.6089,
    38.66, -65.105, 56.1062,
    38.66, -68.1732, 46.8862,
    38.66, -69.717, 37.2291,
    38.66, -69.6893, 27.4284,
    38.66, -68.0911, 17.7817,
    38.66, -64.9709, 8.5822,
    38.66, -60.4235, 0.109455,
    38.66, -54.5871, -7.37912,
    38.66, -47.639, -13.656,
    38.66, -39.7903, -18.5304,
    38.66, -31.2796, -21.8543,
    38.66, -22.3654, -23.5267,
    38.66, -13.3185, -23.4967,
    38.66, -4.41387, -21.7653,
    38.66, 4.07796, -18.3851,
    38.66, 11.899, -13.4588,
    38.66, 18.8115, -7.136,
    38.66, 24.6055, 0.391095,
};


//实际磁场测量数据 最佳偏移量为 -11 -7 -12左右
static const  jccfloat_t testdata1[]
=
{
    24.1284,    35.5046,    -20.8257,
    24.2202,    35.5046,    -20.7339,
    24.1284,    35.6881,    -20.6422,
    23.7615,    35.7798,    -20.2752,
    23.8532,    35.6881,    -20.367,
    24.1284,    35.6881,    -20.5505,
    23.945, 35.5963,    -20.8257,
    23.8532,    35.6881,    -20.6422,
    23.8532,    35.8716,    -20.2752,
    23.945, 35.7798,    -20.2752,
    24.1284,    35.8716,    -20.1835,
    27.4312,    36.789, -18.9908,
    29.5413,    36.6055,    -18.4404,
    36.789, 31.9266,    -17.8899,
    41.8349,    27.6147,    -15.7798,
    42.7523,    27.156, -15.0459,
    43.3945,    24.0367,    -15.8716,
    46.789, 26.6055,    -11.2844,
    52.7523,    18.9908,    -8.62385,
    52.3853,    15.5963,    -10.2752,
    51.6514,    13.578, -11.4679,
    47.5229,    4.12844,    -15.2294,
    42.7523,    0,  -18.1651,
    33.8532,    1.46789,    -23.3945,
    30.2752,    6.69725,    -25.5046,
    17.2477,    5.04587,    -27.9817,
    1.83486,    0,  -26.8807,
    -10.0917,   1.10092,    -23.4862,
    -9.72477,   3.02752,    -23.8532,
    -11.4679,   10.0917,    -23.3028,
    -7.52294,   16.2385,    -24.1284,
    -4.86239,   21.7431,    -23.4862,
    5.41284,    30.2752,    -22.1101,
    11.1009,    34.4037,    -20.367,
    22.3853,    36.6055,    -17.7982,
    26.055, 35.5963,    -17.7982,
    34.1284,    30.1835,    -18.2569,
    40.7339,    26.6055,    -16.6055,
    40.367, 24.1284,    -18.0734,
    39.8165,    24.1284,    -18.4404,
    38.8073,    28.4404,    -16.5138,
    34.1284,    44.8624,    -1.65138,
    32.2018,    46.6972,    19.0826,
    32.844, 43.4862,    25.8716,
    34.1284,    21.2844,    44.5872,
    37.0642,    -6.14679,   45.3211,
    38.2569,    -18.8073,   39.0826,
    36.5138,    -28.9908,   29.4495,
    36.9725,    -30.8257,   25.1376,
    37.4312,    -32.5688,   10.367,
    40.6422,    -21.7431,   -8.16514,
    41.4679,    -10.9174,   -15.5046,
    35.2294,    3.94495,    -23.3945,
    30.1835,    12.0183,    -25.0459,
    30.6422,    17.9817,    -24.1284,
    41.3761,    40.7339,    -1.55963,
    42.5688,    42.0183,    5.59633,
    40.8257,    42.9358,    5.22936,
    32.2018,    42.7523,    -7.70642,
    22.4771,    41.1009,    -14.2202,
    25.3211,    33.1193,    -20.7339,
    27.6147,    28.6239,    -22.6606,
    21.1009,    23.7615,    -27.2477,
    16.6055,    23.211, -28.6239,
    13.1193,    25.5046,    -28.8073,
    8.80734,    22.2018,    -29.2661,
    4.6789, 20, -29.2661,
    4.58716,    18.8073,    -29.4495,
    5.50459,    25.2294,    -28.0734,
    8.53211,    39.4495,    -20.2752,
    14.8624,    48.7156,    -9.54128,
    12.2018,    52.3853,    -1.19266,
    14.3119,    50, -7.33945,
    10.6422,    47.6147,    -11.6514,
    4.95413,    42.5688,    -17.5229,
    -0.0917431, 31.4679,    -24.7706,
    0.825688,   31.3761,    -24.9541,
    3.21101,    32.4771,    -24.7706,
    6.23853,    34.2202,    -23.8532,
    7.15596,    35.4128,    -23.6697,
    8.62385,    35.8716,    -23.578,
    8.7156, 35.1376,    -24.1284,
    8.07339,    31.4679,    -25.9633,
    10.8257,    32.0183,    -25.9633,
    11.5596,    32.844, -25.3211,
    12.844, 34.6789,    -24.2202,
    19.3578,    37.3395,    -22.2018,
    20.5505,    37.0642,    -22.4771,
    20.9174,    36.9725,    -22.4771,
    20.9174,    36.9725,    -22.4771
};

#define tstdat    testdata2

void magnfittest(void)
{
    int datasize, i;
    float sx, sy, ox, oy;
    sx = 0;
    sy = 0;
    ox = 0;
    oy = 0;
    ellipse_reset();
    datasize = sizeof(tstdat) / sizeof(tstdat[0]) / 3;
    for(i = 0; i < datasize; i++)
    {
        ellipse_rawdata(tstdat[3 * i + 0], tstdat[3 * i + 1]); //仅使用xy轴数据
    }
    ellipse_process(55, &sx, &sy, &ox, &oy);
    printf("%g,%g,%g,%g", sx, sy, ox, oy);
}

#endif // 1
