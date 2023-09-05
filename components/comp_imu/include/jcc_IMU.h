/*!
///////////////////////////////////////////////////////////
 *  \file      jcc_IMU.h
 *  \brief     jcc IMU
 *  \details   Implementation of the simple IMU
 *  \author    张永强
 *  \version   0.1
 *  \date      23-09-2016 14:57
///////////////////////////////////////////////////////////
*/

#if !defined(JCCIMU__INCLUDED_)
#define JCCIMU__INCLUDED_
///LITE版本打开下面的宏
#define  LITE
extern float v_acc_z;
///传感器状态
enum _sensorsm_EN
{
    SENSOR_EMPTY=-1,        ///无传感器
    SENSOR_INIT=0,          ///传感器初始化中
    SENSOR_STATUP,          ///传感器启动中
    SENSOR_OK,              ///传感器就绪
    SENSOR_FAILED           ///传感器失效
};
///IMU模块状态
enum _imusm_EN
{
    IMU_INIT=0,             ///imu模块初始化中
    IMU_STATUP,             ///imu模块启动中
    IMU_RUN                 ///imu模块就绪
};

///九轴数据
/*!
 * \attention  jcc IMU 中xyz统一如下定义：
 *                 前进方向、经度、侧倾为x
 *                 侧向、纬度、俯仰为y
 *                 海拔、垂直、首向为z
*/
typedef struct _famgdata_ST
{
    float acel_x,acel_y,acel_z;    ///单位 米每秒方
    float gyro_x,gyro_y,gyro_z;    ///单位 弧度每秒
    float magn_x,magn_y,magn_z;    ///单位 微特
} amgdata_t;

///磁场校准信息
typedef struct _magn_calibration_ST
{
    int type;
    float x,y,z;
    float kx,ky,kz;
    float gm;
} magncali_t;

///陀螺仪校准信息
typedef struct _gyro_calibration_ST
{
    int type;
    float x,y,z;
    float kx,ky,kz;
} gyrocali_t;
typedef struct
{
  float x;
	float y;
	float z;
}	_glb_acc;
enum _sensor_calibrationsm_EN{
    SENSOR_CALI_OLD=1,
    SENSOR_CALI_OK=2
};

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \attention 传感器数据输入需要系统tick，Lite版本精度必须为ms，完整版精度必须为us。
*/

/*!
 * \bref IMU初始化
 * \param[in]  timestep          amg更新周期 单位是秒
*/
void imu_Init(float timestep);


/*!
 * \bref 9轴传感器输入
 * \param[in]  pdata           九轴数据指针
 * \param[in]  ticks          系统tick，精度ms
*/
void amg_sensorinput(amgdata_t *pdata,unsigned long ticks);

/*!
 * \attention  部分磁场传感器需要启动校准或者写入校准数据才能计算出正确的航向！
 *             将传感器东西放置（不用很精确），启动校准后，在指定时间内完成如下步骤：水平转动360度，停顿，然后横滚360度，停顿。
 *             校准结果 magnscal_t::type >= SENSOR_CALI_OK 时满足计算要求。
 */

/*!
 * \bref 读出磁场传感器的校准信息
 * \param[out]  scali          校准数据
 *              如果 scali.type | SENSOR_CALI_OLD  则校准信息为以前写入的值
*/
void magn_getcalibration(magncali_t *scali);


/*!
 * \bref 启动磁场校准
 * \param[in]  sec          校准用时
*/
void magn_startcalibration(float sec,void(*finishcallback)(void));

/*!
 * \bref 写入曾经读出的磁场校准信息
 * \param[in]  scali          校准数据
*/
void magn_setcalibration(const magncali_t *scali);

/*!
 * \attention  本接口只校准陀螺仪零漂，本接口不校准陀螺仪的比例因子！
 *             将传感器静止后，启动校准。
 *             校准结果 magnscal_t::type == SENSOR_CALI_OK 时成功。
 */

/*!
 * \bref 启动陀螺仪校准
 * \param[in]  sec          校准用时
*/
void gyro_startcalibration(float sec,void(*finishcallback)(void));

/*!
 * \bref 写入曾经读出的陀螺仪校准信息
 * \param[in]  scali          校准数据
*/
void gyro_setcalibration(const gyrocali_t *scali);

/*!
 * \bref 读出陀螺仪的校准信息
 * \param[out]  scali          校准数据
 *              如果  scali.type | SENSOR_CALI_OLD  则校准信息为以前写入的值
*/
void gyro_getcalibration(gyrocali_t *scali);




/*!
 * \bref 光纤陀螺输入
 * \param[in]  x               x轴数据 江豚IV忽略未用
 * \param[in]  y               y轴数据 江豚IV忽略未用
 * \param[in]  z               z轴数据
 * \param[in]  ticks           系统tick
*/
void gyro2_sensorinput(float x,float y,float z,unsigned long ticks);

/*!
 * \bref DVL输入
 * \param[in]  BTTransverseVelocity             对底横向速度
 * \param[in]  BTLongitudinalVelocity           对底纵向速度
 * \param[in]  BTVerticalVelocity               对底垂直速度
 * \param[in]  WTTransverseVelocity             对水横向速度
 * \param[in]  WTLongitudinalVelocity           对水纵向速度
 * \param[in]  WTVerticalVelocity               对水垂直速度
 * \param[in]  BTVerticalAltitude1              对底高度1
 * \param[in]  BTVerticalAltitude2              对底高度2
 * \param[in]  BTVerticalAltitude3              对底高度3
 * \param[in]  BTVerticalAltitude4              对底高度4
 * \param[in]  ticks                            系统tick
*/
void dvl_sensorinput(float BTTransverseVelocity,float BTLongitudinalVelocity,float BTVerticalVelocity,
                     float WTTransverseVelocity,float WTLongitudinalVelocity,float WTVerticalVelocity,
                     float BTVerticalAltitude1,float BTVerticalAltitude2,float BTVerticalAltitude3,float BTVerticalAltitude4,
                     unsigned long ticks);


/// GPS输入
/*!
 * \bref GPS方向输入
 * \param[in]  gpvtg_dir            方向1
 * \param[in]  gpvtg_magn           方向2
 * \param[in]  gpvtg_speed          速度
 * \param[in]  ticks               系统tick
*/
void gps_gpsdirinput(float gpvtg_dir,float gpvtg_magn,float gpvtg_speed, unsigned long ticks);

/*!
 * \bref GPS地理位置输入
 * \param[in]  gpgga_altitude           高度
 * \param[in]  gpgga_latitude           纬度
 * \param[in]  gpgga_longitude          经度
 * \param[in]  gpgga_time               UTC时间
 * \param[in]  ticks                   系统tick
*/
void gps_gpsposiinput(float gpgga_altitude,float gpgga_latitude,float gpgga_longitude,float gpgga_time,unsigned long ticks);

/*!
 * \bref GPS速度输入
 * \param[in]  gpvbw_longitudinal       横向速度
 * \param[in]  gpvbw_transverse         纵向速度
 * \param[in]  ticks                   系统tick
*/
void gps_gpsspeedinput(float gpvbw_longitudinal,float gpvbw_transverse ,unsigned long ticks);

/*!
 * \bref ///压力传感器输入
 * \param[in]  z                        深度 米
 * \param[in]  ticks                   系统tick
*/
void pres_sensorinput(float z,unsigned long ticks);

/*!
 * \bref 获取IMU状态
 * \return     见 ::_imusm_EN
*/
int imu_getsm(void);


/*!
 * \bref 返回IMU状态和姿态数据
 * \param[out] roll           侧倾
 * \param[out] pitch          俯仰
 * \param[out] yaw            首向
 * \return     见 ::_imusm_EN
*/
int imu_getattitude(float *roll,float *pitch,float *yaw);


///获取局部角速度
int imu_getlocal_ang_velo (float *x,float *y,float *z);
///获取局部加速度
int imu_getlocal_accel(float *x,float *y,float *z);
///获取局部速度
int imu_getlocal_velo(float *x,float *y,float *z);
///获取全局角速度
int imu_getglobal_ang_velo (float *x,float *y,float *z);
///获取全局加速度
int imu_getglobal_accel(float *x,float *y,float *z);
///获取全局速度
int imu_getglobal_velo(float *x,float *y,float *z);



/*!
 * \bref 返回传感器状态
 * \param[in]  sensor          传感器名称，取值可以是
     AMG 传感器       "acel","gyro","magn"
    单轴光纤传感器  "gyro2"
    DVL              "dvl"
    GPS              "gpsposi","gpsdir","gpsspeed",
    压力传感器      "pres"
 * \return     见 ::_sensorsm_EN
*/
int sensors_getsm(const char *sensor);

/*!
 * \bref 启用/停用 传感器参与IMU计算。   如果停用的传感器保持输入，在启用时可以立即参与计算，否则会有一个启动过程。
 * \param[in]  sensor          传感器名称，取值可以是
     AMG 传感器       "acel","gyro","magn"
    单轴光纤传感器  "gyro2"
    DVL              "dvl"
    GPS              "gpsposi","gpsdir","gpsspeed",
    压力传感器      "pres"
 * \param[in]    en         为0代表停用，为1代表启用
 * \return     见 ::_sensorsm_EN
*/
int sensors_seten(const char *sensor,int en);




/*!
   * \code
    //jcc IMU 使用代码示意

    #include "jcc_IMU.h"

    void magn_finished(void)
    {
            magnscali_t  magnscali;
            magn_getcalibration(&magnscali);
            if(magnscali.type == SENSOR_CALI_OK) writeconfigtoNV(&magnscali);
    }

    void mytest(void)
    {
        sensors_seten("acel",1);
        sensors_seten("gyro",1);
        sensors_seten("magn",1);
        sensors_seten("gyro2",1);
        magn_startcalibration(300,magn_finished);
        ...
        while(1)
        {
            ...
            if(AgileLight100A_getdata(&z))   gyro2_sensorinput(0,0,z,GetTickCount());
            if( BNO055_getAMGdata(fd,&amg))  amg_sensorinput(&amg,GetTickCount());
            if(imu_getattitude(&roll,&pitch,&yaw)==IMU_RUN)
            {
                imu_msg(pitch,roll,yaw,0,0,0);
                ...
            }
            Sleep(50);
        }
    }
    * \endcode

*/


#ifdef __cplusplus
 }
#endif

#endif



