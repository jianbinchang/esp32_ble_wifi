/*!
///////////////////////////////////////////////////////////
 *  \file      jcc_ellipsoid.c
 *  \brief     jcc ellipsoid
 *  \details   Implementation ellipsoid fit
 *  \author    张永强
 *  \version   0.1
 *  \date      23-09-2016 14:57
///////////////////////////////////////////////////////////
*/
#include <string.h>
#include <math.h>


#define ROW   7
#define COL   (ROW+1)

typedef float jccfloat_t;

static jccfloat_t _g_matrix[ROW][COL];

static int _equal(jccfloat_t a, jccfloat_t b)
{
    return fabs(a - b) < 1e-9;
}

void ellipsoid_reset(void)
{
    int row , column;
    for(row = 0 ; row < ROW ; row++)
    {
        for(column = 0 ; column < COL ; column++)
            _g_matrix[row][column] = 0.0f;
    }
}

void ellipsoid_rawdata(float x , float y , float z)
{
    jccfloat_t V[ROW];
    int row , column;
    V[0] = x * x;
    V[1] = y * y;
    V[2] = z * z;
    V[3] = x;
    V[4] = y;
    V[5] = z;
    V[6] = 1.0;
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
static void _matrix_swaprow(int row1 , int row2)
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
static void _matrix_movebiggestelement2top(int s_row , int s_column)
{
    int row;
    for(row = s_row + 1 ; row < ROW ; row++)
    {
        if( fabs(_g_matrix[s_row][s_column]) < fabs(_g_matrix[row][s_column]))
        {
            _matrix_swaprow(s_row , row);
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
        _matrix_movebiggestelement2top(row , column);
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
static void _matrix_solve(jccfloat_t* C , jccfloat_t* sol)
{
    int row, column, i;
    int any_sol[ROW];
    // memset(any_sol , 0 , ROW);
    memset(any_sol, 0, ROW * sizeof(int));
    for(row = 0, column = 0 ; row < ROW && column < COL ; row++, column++)
    {
        if(_equal(_g_matrix[row][column] , 0.0f))
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

void ellipsoid_process(float radius, float *sx, float *sy, float *sz, float *ox, float *oy, float *oz)
{
    jccfloat_t C[ROW] = { -1.0f, -1.0f, -1.0f, 0, 0, 0, 0};
    jccfloat_t Res[ROW];
    jccfloat_t k, f;

    ///任意解
    C[6] = radius * radius;

    _matrix_gausselimination();
    _matrix_rowsimplify();
    _matrix_solve(C , Res);
    k = (Res[3] * Res[3] / Res[0] + Res[4] * Res[4] / Res[1] + Res[5] * Res[5] / Res[2] - 4 * Res[6]) / (4 * radius * radius);
    f = sqrt(Res[0] / k);
    if(_notnan(f)) *sx = f;
    f = sqrt(Res[1] / k);
    if(_notnan(f))  *sy = f;
    f = sqrt(Res[2] / k);
    if(_notnan(f)) *sz = f;
    f = Res[3] / (2 * Res[0]);
    if(_notnan(f))  *ox = f;
    f = Res[4] / (2 * Res[1]);
    if(_notnan(f)) *oy = f;
    f = Res[5] / (2 * Res[2]);
    if(_notnan(f)) *oz = f;
}






///测试代码
#if 0
#include <stdio.h>

//实际磁场测量数据 最佳偏移量为 -11 -7 -12左右
static const  jccfloat_t testdata[]
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


void magnfittest(void)
{
    int datasize, i;
    float sx, sy, sz, ox, oy, oz;
    ellipsoid_reset();
    datasize = sizeof(testdata) / sizeof(testdata[0]) / 3;
    for(i = 0; i < datasize; i++)
    {
        ellipsoid_rawdata(testdata[3 * i + 0] , testdata[3 * i + 1] , testdata[3 * i + 2]);
    }
    ellipsoid_process(55, &sx, &sy, &sz, &ox, &oy, &oz);
    printf("%g,%g,%g,%g,%g,%g", sx, sy, sz, ox, oy, oz);
}

#endif // 1
