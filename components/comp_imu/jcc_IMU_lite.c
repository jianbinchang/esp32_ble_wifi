/*!
///////////////////////////////////////////////////////////
 *  \file      jcc_IMU.c
 *  \brief     jcc IMU lite version
 *  \details   为江豚IV裁剪
 *  \author
 *  \version   0.1 lite
 *  \date      21-10-2016 15:00
///////////////////////////////////////////////////////////
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
//#include "drv_uart1.h"

//#include "product_config.h"
///======================================================================================
///类型声明
///======================================================================================
#include "jcc_IMU.h"
#if  ellipoid_cal
#include "jcc_ellipsoid.h"
#elif  ellipse_cal
#include "jcc_ellipse.h"
#endif
#define  ellipse_cal      1
#define PI               3.1415926535897932
#define G                9.80665

#define MAGN_GM  55.0f
#define MAGN_U   100.0f
#define MAGN_L   25.0f

#define REF_TRR  0.1  //0.5

typedef struct _lhfilter_ST
{
    double a, a1;
    double fd;
} lhf_t;

typedef struct _vect_ST
{
    float x, y, z, a; ///前进方向、经度为x，侧向、纬度为y 海拔、垂直为z  可信度为a
} vect_t;

typedef struct _fmatrix3x3_ST
{
    double v[3][3];
} f3x3_t;

typedef struct _g3d_sensor_T
{
    float x, y, z;
    float timeout;
    unsigned long lasttime;
    int sm;
    int en;
    int califlag;
} g3dsensor_t;

#define magn_cali_size 128
typedef struct _magn_scali_ctx_ST
{
    int   index;
    int   type;
    float timer;
    float roll;
    float yaw;
    float x[magn_cali_size];
    float y[magn_cali_size];
    float z[magn_cali_size];
    void (*fcb)(void);
} magncalictx_t;

typedef struct _gyro_scali_ctx_ST
{
    int   index;
    int   type;
    float timer;
    float x;
    float y;
    float z;
    void (*fcb)(void);
} gyrocalictx_t;


typedef struct _dvl_sensor_T
{
    float btx, bty, btz;
    float wtx, wty, wtz;
    float va1, va2, va3, va4;
    float timeout;
    unsigned long lasttime;
    int sm;
    int en;
} dvlsensor_t;

typedef struct _pres_sensor_T
{
    float z;
    float timeout;
    unsigned long lasttime;
    int sm;
    int en;
} presssensor_t;

typedef struct _gpsposi_data_T
{
    float gpgga_latitude, gpgga_longitude, gpgga_altitude, gpgga_time;
    float timeout;
    unsigned long lasttime;
    int sm;
    int en;
} gpsposi_t;


typedef struct _gpsdir_data_T
{
    float gpvtg_dir, gpvtg_magn, gpvtg_speed;
    float timeout;
    unsigned long lasttime;
    int sm;
    int en;
} gpsdir_t;


typedef struct _gpsspeed_data_T
{
    float gpvbw_longitudinal, gpvbw_transverse;
    float timeout;
    unsigned long lasttime;
    int sm;
    int en;
} gpsspeed_t;


typedef struct _atitudesensor_T
{
    float z;
    float timeout;
    unsigned long lasttime;
    int sm;
    int en;
} atitudesensor_t;


typedef struct _sensors_T
{
    g3dsensor_t *acel;            ///AMG 传感器
    g3dsensor_t *gyro;            ///AMG 传感器
    g3dsensor_t *magn;            ///AMG 传感器
    g3dsensor_t *gyro2;           ///单轴光纤传感器
} sensors_t;


typedef struct _Imu_Ctx_ST
{
///会话
    int sm;
    float timeout;
    double q[4];
    f3x3_t rotmat;
    lhf_t mx, my, mz;
    double egainx, egainy, egainz;
///输入
    sensors_t sensors;
///输出
    vect_t attitude;             //欧拉角
    vect_t acceleration;         //加速度
    vect_t velocity;             //速度
    vect_t watervelocity;        //水流速度
    vect_t position;             //位置
    vect_t geoposition;          //地理位置
} Imu_Ctx_t;

///======================================================================================
///私有变量
///======================================================================================
static Imu_Ctx_t        imuctx;
static g3dsensor_t      acel;
static g3dsensor_t      magn;
static magncali_t      magn_cali;
static magncalictx_t   magncalictx;
static gyrocali_t      gyro_scali;
static gyrocalictx_t   gyrocalictx;
static g3dsensor_t      gyro;
static g3dsensor_t      gyro2;


///======================================================================================
///私有函数
///======================================================================================


static int lhf_init(lhf_t *filter, float fs, float f0)
{
    if(fs <= 0) filter->a = 1;
    else filter->a = 2 * PI * f0 / fs;
    if(filter->a > 1) filter->a = 1;
    filter->a1 = 1 - filter->a;
    filter->fd = 0;
    return 0;
}

static int lf_process(lhf_t *filter, float lfin, float *lfout)
{
    filter->fd = filter->a * lfin + filter->a1 * filter->fd;
    if(lfout) *lfout = filter->fd;
    return 0;
}


///计算旋转矩阵
static void imu_UpdateRotationMatrix(Imu_Ctx_t *imu)
{
    double q1q1 = imu->q[1] * imu->q[1];
    double q2q2 = imu->q[2] * imu->q[2];
    double q3q3 = imu->q[3] * imu->q[3];
    double q0q1 = imu->q[0] * imu->q[1];
    double q0q2 = imu->q[0] * imu->q[2];
    double q0q3 = imu->q[0] * imu->q[3];
    double q1q2 = imu->q[1] * imu->q[2];
    double q1q3 = imu->q[1] * imu->q[3];
    double q2q3 = imu->q[2] * imu->q[3];
    imu->rotmat.v[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    imu->rotmat.v[0][1] = 2.0f * (q1q2 - q0q3);
    imu->rotmat.v[0][2] = 2.0f * (q1q3 + q0q2);
    imu->rotmat.v[1][0] = 2.0f * (q1q2 + q0q3);
    imu->rotmat.v[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    imu->rotmat.v[1][2] = 2.0f * (q2q3 - q0q1);
    imu->rotmat.v[2][0] = 2.0f * (q1q3 - q0q2);
    imu->rotmat.v[2][1] = 2.0f * (q2q3 + q0q1);
    imu->rotmat.v[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

//#define G_ACC
#if defined G_ACC
static double baseZacc = 1.0f;  /*机体坐标系下Z方向向量*/
unsigned char isCalibrated = 0;
_glb_acc glb_acc;
double vecxZ = 0, vecyZ = 0, veczZ = 0, Zacc = 0; /*机体坐标系下Z方向向量*/
//  static float R11,R21;       /*¾ØÕó(1,1),(2,1)Ïî*/
#endif
//float AC_Azimuth( ax,s16 ay, s16 az, s16 mx, s16 my, s16 mz)
//{
//	float accVals[3], magVals[3];
//	float ftemp;
//	
//	accVals[0] = ax;
//	accVals[1] = az;
//	accVals[2] = ay;
//	magVals[0] = mx;
//	magVals[1] = mz;
//	magVals[2] = my;
//			
//	ftemp = getYaw(accVals, magVals) * 180.0f / 3.141593f;
//	if(ftemp > 0) ftemp = -180.0f + (ftemp - 180.0f) ;
//	ftemp = 0.0f - ftemp;
//	ftemp += 90.0f;
//	ftemp += -2.0f;	//补偿磁偏角，不同地区会不一样
//	if(ftemp > 360.0f) ftemp -= 360.0f;
//	return ftemp;	

///更新姿态
static void imu_AHRSupdate(Imu_Ctx_t *imu, sensors_t *sensors, float dt)
{
    double dnorm;
    double hx, hy, bx;
    double ex = 0, ey = 0, ez = 0;
    double qa, qb, qc, qd;
    double gx = 0, gy = 0, gz = 0;
#if defined G_ACC
    double q0s = 0, q1s = 0, q2s = 0, q3s = 0; /*四元数的平方*/

#endif
   //printf("%d %d %d \r\n",sensors->gyro->sm ,sensors->acel->sm ,sensors->magn->sm );
   //printf("%d %d %d \r\n",sensors->gyro->en ,sensors->acel->en ,sensors->magn->en );
    if (sensors->magn && sensors->magn->en && (sensors->magn->sm == SENSOR_OK))
    {
        double ez_ef, mx, my, mz;
        dnorm = 1 / sqrt(sensors->magn->x * sensors->magn->x + sensors->magn->y * sensors->magn->y + sensors->magn->z * sensors->magn->z);
        mx = sensors->magn->x * dnorm;
        my = sensors->magn->y * dnorm;
        mz = sensors->magn->z * dnorm;
        hx = imu->rotmat.v[0][0] * mx + imu->rotmat.v[0][1] * my + imu->rotmat.v[0][2] * mz;
        hy = imu->rotmat.v[1][0] * mx + imu->rotmat.v[1][1] * my + imu->rotmat.v[1][2] * mz;
        bx = sqrt(hx * hx + hy * hy);
        ///惯性坐标系下的预测北极与磁场北的偏差角的正弦值
        ez_ef = -(hy * bx);
        ///误差投影到物体坐标系
        ex += imu->rotmat.v[2][0] * ez_ef;
        ey += imu->rotmat.v[2][1] * ez_ef;
        ez += imu->rotmat.v[2][2] * ez_ef;
    }
    if (sensors->acel && sensors->acel->en && (sensors->acel->sm == SENSOR_OK))
    {
        double ax, ay, az;
        dnorm = 1 / sqrt(sensors->acel->x * sensors->acel->x + sensors->acel->y * sensors->acel->y + sensors->acel->z * sensors->acel->z);
        ax = sensors->acel->x * dnorm;
        ay = sensors->acel->y * dnorm;
        az = sensors->acel->z * dnorm;
        ///偏差为预测方向与测量方向的叉乘
        ex += (ay * imu->rotmat.v[2][2] - az * imu->rotmat.v[2][1]);
        ey += (az * imu->rotmat.v[2][0] - ax * imu->rotmat.v[2][2]);
        ez += (ax * imu->rotmat.v[2][1] - ay * imu->rotmat.v[2][0]);
    }
    if (sensors->gyro && sensors->gyro->en && (sensors->gyro->sm == SENSOR_OK))
    {
        gx = sensors->gyro->x;
        gy = sensors->gyro->y;
        if (sensors->gyro2 && sensors->gyro2->en && (sensors->gyro2->sm == SENSOR_OK))
        {
            gz = sensors->gyro2->z;
            imu->egainz = 0.1;
        }
        else
        {
            gz = sensors->gyro->z;
            if(imu->sm == IMU_STATUP)
            {
                imu->egainz = 10;
            }
            else if(imu->sm == IMU_RUN)
            {
                imu->egainz = 10.5;
            }
   #if 0
            gz = sensors->gyro->z;
            if(imu->sm == IMU_STATUP)
            {
                imu->egainz = 10;
            }
            else if(imu->sm == IMU_RUN)
            {
                imu->egainz = 0.5;
            }

            //carll add230803
            gx = sensors->gyro->x;
            if(imu->sm == IMU_STATUP)
            {
                imu->egainx = 10;
            }
            else if(imu->sm == IMU_RUN)
            {
                imu->egainx = 0.5;
            }

            gy = sensors->gyro->y;
            if(imu->sm == IMU_STATUP)
            {
                imu->egainy = 10;
            }
            else if(imu->sm == IMU_RUN)
            {
                imu->egainy = 0.5;
            }
            //carll add230803
#endif
        }
    }
    else
    {
        gx = 0;
        gz = 0;
        gy = 0;
    }
    ///按比例调整误差
    gx += (imu->egainx * ex);
    gy += (imu->egainy * ey);
    gz += (imu->egainz * ez);
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    {
        double q0, q1, q2, q3;
        qa = imu->q[0];
        qb = imu->q[1];
        qc = imu->q[2];
        qd = imu->q[3];
        q0 = qa;
        q1 = qb;
        q2 = qc;
        q3 = qd;
        q0 += (-qb * gx - qc * gy - qd * gz);
        q1 += (qa * gx + qc * gz - qd * gy);
        q2 += (qa * gy - qb * gz + qd * gx);
        q3 += (qa * gz + qb * gy - qc * gx);
        dnorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        imu->q[0] = q0 * dnorm;
        imu->q[1] = q1 * dnorm;
        imu->q[2] = q2 * dnorm;
        imu->q[3] = q3 * dnorm;

#if defined G_ACC

        q0s = q0 * q0 * dnorm * dnorm;
        q1s = q1 * q1 * dnorm * dnorm;
        q2s = q2 * q2 * dnorm * dnorm;
        q3s = q3 * q3 * dnorm * dnorm;

//           R11 = q0s + q1s - q2s - q3s;   /*矩阵(1,1)项*/
//       R21 = 2 * (q1 * q2 + q0 * q3); /*矩阵(2,1)项*/

        vecxZ = 2 * (q1 * dnorm * q3 * dnorm - q0 * dnorm * q2 * dnorm);
        vecyZ = 2 * (q0 * dnorm * q1 * dnorm + q2 * dnorm * q3 * dnorm);
        veczZ = q0s - q1s - q2s + q3s;

#endif
    }
#if defined G_ACC

    if (vecxZ > 1) vecxZ = 1;
    if (vecxZ < -1) vecxZ = -1;
    if (sensors->acel && sensors->acel->en && (sensors->acel->sm == SENSOR_OK) && (isCalibrated == 0))   /*校准*/
    {
        baseZacc = (sensors->acel->x * vecxZ + sensors->acel->y * vecyZ + sensors->acel->z * veczZ);
        isCalibrated = 1;
    }
    if(sensors->acel && sensors->acel->en && (sensors->acel->sm == SENSOR_OK) && (isCalibrated == 1))
    {
        Zacc = (sensors->acel->x * vecxZ + sensors->acel->y * vecyZ + sensors->acel->z * veczZ);
        glb_acc.z = Zacc + baseZacc;   /*Z轴去掉重力加速度*/
    }
#endif
    ///更新旋转矩阵
    imu_UpdateRotationMatrix(imu);
}


///姿态转换为欧拉角
static void imu_UpdateEulerAngles(Imu_Ctx_t *imu)
{
    imu->attitude.x = atan2(imu->rotmat.v[2][1], imu->rotmat.v[2][2]);
    imu->attitude.y = -asin(-imu->rotmat.v[2][0]);
    imu->attitude.z = -atan2(imu->rotmat.v[1][0], imu->rotmat.v[0][0]);
    if (imu->attitude.z < 0)  imu->attitude.z += (2 * PI);
}


///用磁场、加速度传感器设置imu初始值
static void imu_AHRReset(Imu_Ctx_t *imu)
{
    double p, h, r;
    double sx, sy, sz, cx, cy, cz;
    r = imu->attitude.x;
    p = imu->attitude.y;
    h = imu->attitude.z;
    if(acel.en)
    {
        double n, ax, ay, az;
        n = 1 / sqrt(acel.x * acel.x + acel.y * acel.y + acel.z * acel.z);
        ax = acel.x * n, ay = acel.y * n, az = acel.z * n;
        p =  asin(ax);
        r =  atan2(ay, az);
        if(magn.en)
        {
            double mx, my, mz, mtx, mty;
            n = 1 / sqrt(magn.x * magn.x + magn.y * magn.y + magn.z * magn.z);
            mx = magn.x * n, my = magn.y * n, mz = magn.z * n;
            ///mt=m-mii=m-(a.m)*a
            n = ax * mx + ay * my + az * mz;
            mtx = mx - n * ax;
            mty = my - n * ay;
            h = atan2(mty, mtx);
            if(h < 0) h += 2 * PI;
        }
    }
    sx = sin(r / 2), sy = sin(p / 2), sz = sin(h / 2);
    cx = cos(r / 2), cy = cos(p / 2), cz = cos(h / 2);
    imu->q[0] =  cx * cy * cz + sx * sy * sz;
    imu->q[1] =  sx * cy * cz - cx * sy * sz;
    imu->q[2] = -cx * sy * cz - sx * cy * sz;
    imu->q[3] = -cx * cy * sz + sx * sy * cz;
    imu_UpdateRotationMatrix(imu);
    imu_UpdateEulerAngles(imu);
}


///估计姿态
static void imu_EstimatedAttitude(Imu_Ctx_t *imu, sensors_t *sensors, float timestep)
{
    imu_AHRSupdate(imu, sensors, timestep);
    imu_UpdateEulerAngles(imu);
}


///初始化IMU
void imu_Init(float timestep)
{
    float das = 5.0f;
    Imu_Ctx_t *imu = &imuctx;
    imu->sensors.acel = &acel;
    imu->sensors.magn = &magn;
    imu->sensors.gyro = &gyro;
    imu->sensors.gyro2 = &gyro2;
    acel.timeout = 0.2;
    magn.timeout = 0.2;
    gyro.timeout = 0.2;
    gyro2.timeout = 0.2;
    imu->egainx = REF_TRR;
    imu->egainy = REF_TRR;
    imu->egainz = REF_TRR;
    imu->timeout = 1.0f;
    imu->sm = IMU_STATUP;
    lhf_init(&imu->mx, 1 / timestep, das);
    lhf_init(&imu->my, 1 / timestep, das);
    lhf_init(&imu->mz, 1 / timestep, das);
    imu->q[0] = 1.0;
    imu->q[1] = 0;
    imu->q[2] = 0;
    imu->q[3] = 0;
    imu->attitude.x = 0;
    imu->attitude.y = 0;
    imu->attitude.z = 0;
    imu_UpdateRotationMatrix(imu);
}


///IMU数据输入
static void imu_Input(Imu_Ctx_t *imu, sensors_t *sensors, float timestep)
{
    //printf(" imu->sm = %d timestep %.2f\r\n",imu->sm,timestep);
    switch(imu->sm)
    {

        case IMU_INIT:
//    {
//        float timestep=0.1;
//        imu->egainx=5.0;
//        imu->egainy=5.0;
//        imu->egainz=5.0;
//        imu->timeout=3.0;
//        imu_EstimatedAttitude(imu,sensors,timestep);
//        imu->sm=IMU_STATUP;
//    }
            break;
        case IMU_STATUP:
            imu_EstimatedAttitude(imu, sensors, timestep);
            imu->timeout -= timestep;
            //printf(" imu->timeout = %.2f timestep =  %.2f \r\n",imu->timeout,timestep);
            if(imu->timeout < 0.01f)
            {
                imu_AHRReset(imu);
                imu->timeout = 0;
                imu->sm = IMU_RUN;
            }
            break;
        case IMU_RUN:
            imu_EstimatedAttitude(imu, sensors, timestep);

            break;
    }
}

static void acel_sensorfill(g3dsensor_t *sensor, float x, float y, float z, float timestep)
{
    sensor->x = x;
    sensor->y = y;
    sensor->z = z;
    switch(sensor->sm)
    {
        case SENSOR_INIT:
            sensor->sm = SENSOR_STATUP;
            break;
        case SENSOR_STATUP:
            if(timestep > sensor->timeout) sensor->sm = SENSOR_FAILED;
            else sensor->sm = SENSOR_OK;
            break;
        case SENSOR_OK:
            if(timestep > sensor->timeout) sensor->sm = SENSOR_FAILED;
            break;
        case SENSOR_FAILED:
            if(timestep < sensor->timeout)  sensor->sm = SENSOR_OK;
            break;
    }
}


#define GYROSCALICOUNT  1280
static void gyro_sensorfill(g3dsensor_t *sensor, float x, float y, float z, float timestep)
{
    sensor->x = x;
    sensor->y = y;
    sensor->z = z;
    if(gyro_scali.type & SENSOR_CALI_OK)  sensor->x += gyro_scali.x,  sensor->y += gyro_scali.y, sensor->z += gyro_scali.z;

    switch(sensor->sm)
    {
        case SENSOR_INIT:
            sensor->sm = SENSOR_STATUP;
            break;
        case SENSOR_STATUP:
            if(timestep > sensor->timeout) sensor->sm = SENSOR_FAILED;
            else sensor->sm = SENSOR_OK;
            break;
        case SENSOR_OK:
            if(timestep > sensor->timeout) sensor->sm = SENSOR_FAILED;
            break;
        case SENSOR_FAILED:
            if(timestep < sensor->timeout)  sensor->sm = SENSOR_OK;
            break;
    }

    if(magn.califlag)
    {
        if(timestep < sensor->timeout)
        {
            float YL;
            YL = PI * 2 * 0.8f;
            switch(magncalictx.type)
            {
                case 0:
                    magncalictx.yaw += (z * timestep);
                    magncalictx.roll += (x * timestep);
#if  ellipoid_cal
                    if(  ((magncalictx.yaw >= YL) || (magncalictx.yaw < -YL ))
                         && ((magncalictx.roll >= YL) || (magncalictx.roll < -YL ))
                         && (magncalictx.index >= 24)
                      )
#elif  ellipse_cal
                    if(  ((magncalictx.yaw >= YL) || (magncalictx.yaw < -YL ))
                         && (magncalictx.index >= 12)
                      )
#endif
                    {
                        magncalictx.type =  SENSOR_CALI_OK;
                    }
                    break;
                case SENSOR_CALI_OK:
                    if(magncalictx.timer > 1.0f) magncalictx.timer = 1.0f;
                    break;
            }
        }
    }
    if(gyro.califlag)
    {
        if(gyrocalictx.timer < 0)
        {
            gyro.califlag = 0;
            if(gyrocalictx.type == SENSOR_CALI_OK)
            {
                gyro_scali.x = -gyrocalictx.x / GYROSCALICOUNT;
                gyro_scali.y = -gyrocalictx.y / GYROSCALICOUNT;
                gyro_scali.z = -gyrocalictx.z / GYROSCALICOUNT;
                gyro_scali.type = SENSOR_CALI_OK;

                //DEBUG_PRINT("gyro_scali %f,%f,%f\r\n", gyro_scali.x, gyro_scali.y, gyro_scali.z);
                //DEBUG_PRINT(" gyro_scali.type = %d \r\n", gyro_scali.type);

            }
            {
                void(*finishcallback)(void);
                if(gyrocalictx.fcb)
                {
                    finishcallback = gyrocalictx.fcb;
                    gyrocalictx.fcb = NULL;
                    finishcallback();
                }
            }
        }
        else
        {
            float dp = 2 * PI / 20;
            if((x * x + y * y + z * z) < 3.0f * dp * dp) //
            {
                if(gyrocalictx.index)
                {
                    gyrocalictx.x += x;
                    gyrocalictx.y += y;
                    gyrocalictx.z += z;
                }
                else
                {
                    gyrocalictx.x = x;
                    gyrocalictx.y = y;
                    gyrocalictx.z = z;
                }
                gyrocalictx.index++;
                if(gyrocalictx.index >= GYROSCALICOUNT)
                {
                    gyrocalictx.type = SENSOR_CALI_OK;
                    gyrocalictx.timer = 0;
                }
            }
            else
            {
                gyrocalictx.index = 0;
            }
            gyrocalictx.timer -= timestep;
        }
    }
}



///======================================================================
///陀螺仪校准
///======================================================================
void gyro_startcalibration(float sec, void(finishcallback)(void))
{
    gyrocalictx.timer = sec;
    gyrocalictx.index = 0;
    gyrocalictx.type = 0;
    gyrocalictx.fcb = finishcallback;
    gyro.califlag = 1;
}

void gyro_setcalibration(const gyrocali_t *scali)
{
    if(scali->type >= 2)
    {
        memcpy(&gyro_scali, scali, sizeof(gyrocali_t));
        gyro_scali.type |= SENSOR_CALI_OLD;
    }
}

void gyro_getcalibration(gyrocali_t *scali)
{
    memcpy(scali, &gyro_scali, sizeof(gyrocali_t));
}

///======================================================================
///磁场校准
///======================================================================
void magn_getcalibration(magncali_t *scali)
{
    memcpy(scali, &magn_cali, sizeof(magncali_t));
}

void magn_startcalibration(float sec, void(finishcallback)(void))
{
    magncalictx.timer = sec;
    magncalictx.index = 0;
    magncalictx.roll = 0;
    magncalictx.yaw = 0;
    magncalictx.type = 0;
    magncalictx.fcb = finishcallback;
    magn.califlag = 1;
}

void magn_setcalibration(const magncali_t *scali)
{
    if(scali->type >= 2)
    {
        memcpy(&magn_cali, scali, sizeof(magncali_t));
        magn_cali.type |= SENSOR_CALI_OLD;
    }
}


static float magn_mindistance(float x, float y, float z)
{
    float r, t, dx, dy, dz;
    if(magncalictx.index)
    {
        int i;
        dx = magncalictx.x[0] - x;
        dy = magncalictx.y[0] - y;
        dz = magncalictx.z[0] - z;
        t = dx * dx + dy * dy + dz * dz;
        r = t;
        for(i = 1; i < magncalictx.index; i++)
        {
            dx = magncalictx.x[i] - x;
            dy = magncalictx.y[i] - y;
            dz = magncalictx.z[i] - z;
            t = dx * dx + dy * dy + dz * dz;
            if(r > t) r = t;
        }
    }
    else
    {
        r = 1e10;
    }
    return r;
}


static void magn_match(float slb[3], float sub[3], float sstep,
                       float olb[3], float oub[3], float ostep,
                       float *bsx, float *bsy, float *bsz,
                       float *box, float *boy, float *boz,
                       float radius)
{
    float sx, sy, sz, ox, oy, oz;
    float ce, e, x, y, z, e1, e2, e3, e4, e5, e6, spv;
    int i;
    spv = ostep * ostep * magncalictx.index;
    ce = 1e10;
    e1 = 1e10;
    for(sx = slb[0]; sx < sub[0]; sx += sstep)
    {
        e2 = 1e10;
        for(sy = slb[1]; sy < sub[1]; sy += sstep)
        {
            e3 = 1e10;
            for(sz = slb[2]; sz < sub[2]; sz += sstep)
            {
                e4 = 1e10;
                for(ox = olb[0]; ox < oub[0]; ox += ostep)
                {
                    e5 = 1e10;
                    for(oy = olb[1]; oy < oub[1]; oy += ostep)
                    {
                        e6 = 1e10;
                        for(oz = olb[2]; oz < oub[2]; oz += ostep)
                        {
                            e = 0;
                            for(i = 0; i < magncalictx.index; i++)
                            {
                                float k;
                                x = (magncalictx.x[i] + ox) * sx;
                                y = (magncalictx.y[i] + oy) * sy;
                                z = (magncalictx.z[i] + oz) * sz;
                                k = radius * radius - x * x - y * y - z * z;
                                if(k > 0) e += k;
                                else e -= k;
                            }
                            if(e < ce) ce = e, *bsx = sx, *bsy = sy, *bsz = sz, *box = ox, *boy = oy, *boz = oz;
                            if(e < e6)  e6 = e;
                            else if(e - e6 > spv) break;
                        }
                        if(e6 < e5)  e5 = e6;
                        else if(e6 - e5 > spv) break;
                    }
                    if(e5 < e4)  e4 = e5;
                    else if(e5 - e4 > spv) break;
                }
                if(e4 < e3)  e3 = e4;
                else if(e4 - e3 > spv) break;
            }
            if(e3 < e2)  e2 = e3;
            else if(e3 - e2 > spv) break;
        }
        if(e2 < e1)  e1 = e2;
        else if(e2 - e1 > spv) break;
    }
}

void match_process(float radius, float *sx, float *sy, float *sz, float *ox, float *oy, float *oz)
{
    int i;
    float olb[3], oub[3], box, boy, boz, bsx, bsy, bsz, sstep;
    float slb[3] = {0.8f, 0.8f, 0.8f};
    float sub[3] = {1.3f, 1.3f, 1.3f};
    box = 0, boy = 0, boz = 0;
    for(i = 0; i < magncalictx.index; i++) box += magncalictx.x[i], boy += magncalictx.y[i], boz += magncalictx.z[i];
    box /= magncalictx.index, boy /= magncalictx.index, boz /= magncalictx.index;
    olb[0] = box - radius,  olb[1] = boy - radius,  olb[2] = boz - radius;
    oub[0] = box + radius,  oub[1] = boy + radius,  oub[2] = boz + radius;
    sstep = 0.2;  //0.04  0.008 0.0016 0.00032
    for(i = 0; i < 3; i++)
    {
        magn_match(slb, sub, sstep,   olb, oub, sstep * radius,   &bsx, &bsy, &bsz, &box, &boy, &boz, radius);
        slb[0] = bsx - sstep, slb[1] = bsy - sstep, slb[2] = bsz - sstep;
        sub[0] = bsx + sstep, sub[1] = bsy + sstep, sub[2] = bsz + sstep;
        olb[0] = box - sstep * radius, olb[1] = boy - sstep * radius, olb[2] = boz - sstep * radius;
        oub[0] = box + sstep * radius, oub[1] = boy + sstep * radius, oub[2] = boz + sstep * radius;
        sstep = sstep / 5;
    }
    *sx = bsx, *sy = bsy, *sz = bsz;
    *ox = box, *oy = boy, *oz = boz;
}
//extern "C"
//{
 void ellipse_reset(void);
 void ellipse_rawdata(float x, float y);
 void ellipse_process(float radius, float *sx, float *sy, float *ox, float *oy);
//};

static void magn_sensorfill(g3dsensor_t *sensor, float rawx, float rawy, float rawz, float timestep)
{
    float a, x, y, z;
    lf_process(&imuctx.mx, rawx, &x);
    lf_process(&imuctx.my, rawy, &y);
    lf_process(&imuctx.mz, rawz, &z);
    if(magn_cali.type & SENSOR_CALI_OK)
    {
        sensor->x = (x + magn_cali.x) * magn_cali.kx;
        sensor->y = (y + magn_cali.y) * magn_cali.ky;
        sensor->z = (z + magn_cali.z) * magn_cali.kz;
    }
    else
    {
        sensor->x = x;
        sensor->y = y;
        sensor->z = z;
    }
    switch(sensor->sm)
    {
        case SENSOR_INIT:
            sensor->sm = SENSOR_STATUP;
            break;
        case SENSOR_STATUP:
            imuctx.mx.fd = rawx;
            imuctx.my.fd = rawy;
            imuctx.mz.fd = rawz;
            a = sqrt(sensor->x * sensor->x + sensor->y * sensor->y + sensor->z * sensor->z);
            if((timestep > sensor->timeout) || (a < MAGN_L) || (a > MAGN_U )) sensor->sm = SENSOR_FAILED;
            else sensor->sm = SENSOR_OK;
            break;
        case SENSOR_OK:
            a = sqrt(sensor->x * sensor->x + sensor->y * sensor->y + sensor->z * sensor->z);
            if(magn_cali.gm > MAGN_L)
            {
                if((timestep > sensor->timeout) || (a < magn_cali.gm * 0.7f) || (a > magn_cali.gm * 1.4f )) sensor->sm = SENSOR_FAILED;
            }
            else if((timestep > sensor->timeout) || (a < MAGN_L) || (a > MAGN_U )) sensor->sm = SENSOR_FAILED;
            break;
        case SENSOR_FAILED:
            a = sqrt(sensor->x * sensor->x + sensor->y * sensor->y + sensor->z * sensor->z);
            if(timestep < sensor->timeout)
            {
                if(magn_cali.gm > MAGN_L)
                {
                    if((a > magn_cali.gm * 0.7f) && (a < magn_cali.gm * 1.4f)) sensor->sm = SENSOR_OK;
                }
                else if((a > MAGN_L) && (a < MAGN_U )) sensor->sm = SENSOR_OK;
            }
            break;
    }


    if(magn.califlag)
    {
        a = sqrt(sensor->x * sensor->x + sensor->y * sensor->y + sensor->z * sensor->z);
        if(magncalictx.timer <= 0)
        {
            magn.califlag = 0;
            if( magncalictx.type   & SENSOR_CALI_OK)
            {
#if 1
                int i;
#if  ellipoid_cal
                ellipsoid_reset();
                for(i = 0; i < magncalictx.index; i++) ellipsoid_rawdata(magncalictx.x[i], magncalictx.y[i], magncalictx.z[i]);
                magn_cali.gm = 55;
                ellipsoid_process(magn_cali.gm, &magn_cali.kx, &magn_cali.ky, &magn_cali.kz, &magn_cali.x, &magn_cali.y, &magn_cali.z);
#elif  ellipse_cal
                ellipse_reset();
                for(i = 0; i < magncalictx.index; i++) ellipse_rawdata(magncalictx.x[i], magncalictx.y[i]);
                magn_cali.gm = 55;
                ellipse_process(magn_cali.gm, &magn_cali.kx, &magn_cali.ky, &magn_cali.x, &magn_cali.y);

                magn_cali.kz = 0;
                magn_cali.z = 0;
#endif
#else
                match_process(magn_cali.gm, &magn_cali.kx, &magn_cali.ky, &magn_cali.kz, &magn_cali.x, &magn_cali.y, &magn_cali.z);
#endif
            }
            {
                void(*finishcallback)(void);
                if(magncalictx.fcb)
                {
                    finishcallback = magncalictx.fcb;
                    magncalictx.fcb = NULL;
                    finishcallback();
                }
            }
        }
        else
        {
            if(magncalictx.index < magn_cali_size)
            {
                if( (magn.sm > SENSOR_STATUP)
                    && (magn_mindistance(x, y, z) > (10 * 10)) ) //20度左右
                {
                    magncalictx.x[magncalictx.index] = x;
                    magncalictx.y[magncalictx.index] = y;
                    magncalictx.z[magncalictx.index] = z;
                    magncalictx.index++;
                }
            }
            else
            {
                magncalictx.timer = 0;
            }
            magncalictx.timer -= timestep;
        }
    }
}


///======================================================================
///传感器输入
///======================================================================
void amg_sensorinput(amgdata_t *pdata, unsigned long mstick)
{
    Imu_Ctx_t *imu = &imuctx;
    float du;
    g3dsensor_t *sensor;
    sensors_t sensors;
    memset(&sensors, 0, sizeof(sensors));
    if(imu->sensors.acel->sm == SENSOR_INIT)du = 0;
    else du = (mstick - imu->sensors.acel->lasttime) * 0.001f;

    sensor = imu->sensors.acel;
    sensor->lasttime = mstick;
    acel_sensorfill(sensor, pdata->acel_x, pdata->acel_y, pdata->acel_z, du);
    sensors.acel = sensor;

    sensor = imu->sensors.gyro;
    sensor->lasttime = mstick;
    gyro_sensorfill(sensor, pdata->gyro_x, pdata->gyro_y, pdata->gyro_z, du);
    sensors.gyro = sensor;

    sensor = imu->sensors.magn;
    sensor->lasttime = mstick;
    magn_sensorfill(sensor, pdata->magn_x, pdata->magn_y, pdata->magn_z, du);
    sensors.magn = sensor;

    sensor = imu->sensors.gyro2;
    if(sensor) sensors.gyro2 = sensor;

    imu_Input(imu, &sensors, du);

}


void gyro2_sensorinput(float x, float y, float z, unsigned long mstick)
{
    Imu_Ctx_t *imu = &imuctx;
    if(imu->sensors.gyro2 == NULL)
    {
        gyro2.timeout = 0.5;
        imu->sensors.gyro2 = &gyro2;
    }
    if(imu->sensors.gyro2)
    {
        float du;
        g3dsensor_t *sensor = imu->sensors.gyro2;
        sensor->x = x;
        sensor->y = y;
        sensor->z = z;
        switch(sensor->sm)
        {
            case SENSOR_INIT:
                sensor->lasttime = mstick;
                sensor->sm = SENSOR_STATUP;
                break;
            case SENSOR_STATUP:
                du = (mstick - sensor->lasttime) * 0.001f;
                sensor->lasttime = mstick;
                if(du > sensor->timeout) sensor->sm = SENSOR_FAILED;
                else sensor->sm = SENSOR_OK;
                break;
            case SENSOR_OK:
                du = (mstick - sensor->lasttime) * 0.001f;
                sensor->lasttime = mstick;
                if(du > sensor->timeout) sensor->sm = SENSOR_FAILED;
                break;
            case SENSOR_FAILED:
                du = (mstick - sensor->lasttime) * 0.001f;
                sensor->lasttime = mstick;
                if(du < sensor->timeout) sensor->sm = SENSOR_OK;
                break;
        }
    }
}

int imu_getsm(void)
{
    return imuctx.sm;
}

int imu_getattitude(float *roll, float *pitch, float *yaw)
{
    if(roll) *roll = imuctx.attitude.x * (180 / PI);
    if(pitch) *pitch = imuctx.attitude.y * (180 / PI);
    if(yaw) *yaw = imuctx.attitude.z * (180 / PI);
    return imuctx.sm;
}


///获取局部角速度
int imu_getlocal_ang_velo (float *x, float *y, float *z)
{
    if(x) *x = gyro.x;
    if(y) *y = gyro.y;
    if(z) *z = gyro.z;
    return gyro.sm;
}

///获取局部加速度
int imu_getlocal_accel(float *x, float *y, float *z)
{
    if(x) *x = acel.x;
    if(y) *y = acel.y;
    if(z) *z = acel.z;
    return acel.sm;
}


#define RETSENSORSM(SS)   if(strcmp(sensor,#SS)==0){if(imuctx.sensors.SS) return imuctx.sensors.SS->sm;else return SENSOR_EMPTY;}
int sensors_getsm(const char *sensor)
{
    RETSENSORSM(acel);
    RETSENSORSM(magn);
    RETSENSORSM(gyro);
    RETSENSORSM(gyro2);
//    RETSENSORSM(gpsposi);
//    RETSENSORSM(gpsdir);
//    RETSENSORSM(gpsspeed);
//    RETSENSORSM(pres);
//    RETSENSORSM(dvl);
//    RETSENSORSM(atit);
    return SENSOR_EMPTY;
}

#define SETSENSOREN(SS,EN)   if(strcmp(sensor,#SS)==0){ SS.en=EN;  if(imuctx.sensors.SS){ return imuctx.sensors.SS->sm;} else return SENSOR_EMPTY;}
int sensors_seten(const char *sensor, int en)
{
    SETSENSOREN(acel, en);
    SETSENSOREN(magn, en);
    SETSENSOREN(gyro, en);
    SETSENSOREN(gyro2, en);
//    SETSENSOREN(gpsposi,en);
//    SETSENSOREN(gpsdir,en);
//    SETSENSOREN(gpsspeed,en);
//    SETSENSOREN(pres,en);
//    SETSENSOREN(dvl,en);
//    SETSENSOREN(atit,en);
    return SENSOR_EMPTY;
}

