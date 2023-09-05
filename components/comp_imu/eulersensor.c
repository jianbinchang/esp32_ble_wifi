#include "eulersensor.h"
#include "jcc_IMU.h"
#include "esp32_i2c.h"
#include <time.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define SIGNAL_PATH_RESET                                                  0x02
#define ACCEL_DATA_X1                                                      0x0b
#define GYRO_DATA_X1                                                       0x11

#define PWR_MGMT0                                                          0x1f
#define GYRO_CONFIG0                                                       0x20
#define ACCEL_CONFIG0                                                      0x21
#define WHO_AM_I                                                           0x75

#define G                9.80665f
#define PI_180           (3.1415926535897932f/180.0f)

// #define X1 -1                                                              //rov 的坐标系是前右下
// #define X2 0
// #define X3 0
// #define Y1 0
// #define Y2 -1                
// #define Y3 0
// #define Z1 0
// #define Z2 0
// #define Z3 1                

/**下面是矩阵俯仰45°后的矩阵*/
#define X1 -0.7071                                                            //rov 的坐标系是前右下
#define X2 0
#define X3 0.7071
#define Y1 0
#define Y2 -1                     //0.707
#define Y3 0
#define Z1 0.7071
#define Z2 0
#define Z3 0.7071                 //1


#define IST_REG_CNTRL3          0x32
#define IST_WHO_AM_I            0x00
#define IST_REG_CNTRL6				                0x35
#define	IST8308_OPMODE_STANDBY_MODE  			 	0x00
#define	IST8308_OPMODE_FORCE_MODE  				 	0x01
#define	IST8308_OPMODE_CONTINUOS_MODE_10HZ 		 	0x02
#define	IST8308_OPMODE_CONTINUOS_MODE_20HZ 		 	0x04
#define IST8308_OPMODE_CONTINUOS_MODE_50HZ 		 	0x06
#define	IST8308_OPMODE_CONTINUOS_MODE_100HZ 	 	0x08
#define	IST8308_OPMODE_CONTINUOS_MODE_200HZ		 	0x0A
#define	IST8308_OPMODE_CONTINUOS_MODE_8HZ 		 	0x0B
#define	IST8308_OPMODE_CONTINUOS_MODE_1HZ 		 	0x0C
#define	IST8308_OPMODE_CONTINUOS_MODE_0_5HZ		 	0x0D
#define	IST8308_OPMODE_SELF_TEST_MODE	 		 	0x10
#define	IST8308_OPMODE_CONTINUOS_MODE_333HZ		 	0x11
#define	IST8308_OPMODE_CONTINUOS_MODE_500HZ		 	0x12
#define	IST8308_OPMODE_CONTINUOS_MODE_NO_SLEEP	 	0x13

#define	IST8308_WMT_1								0x00
#define	IST8308_WMT_2								0x01
#define	IST8308_WMT_3								0x02
#define	IST8308_WMT_4								0x03
#define	IST8308_WMT_5								0x04
#define	IST8308_WMT_6								0x05
#define	IST8308_WMT_7								0x06
#define	IST8308_WMT_8								0x07
#define	IST8308_WMT_9								0x08
#define	IST8308_WMT_10								0x09
#define	IST8308_WMT_11								0x0A
#define	IST8308_WMT_12								0x0B
#define	IST8308_WMT_13								0x0C
#define	IST8308_WMT_14								0x0D
#define	IST8308_WMT_15								0x0E
#define	IST8308_WMT_16								0x0F
#define	IST8308_WMT_17								0x10
#define	IST8308_WMT_18								0x11
#define	IST8308_WMT_19								0x12
#define	IST8308_WMT_20								0x13
#define	IST8308_WMT_21								0x14
#define	IST8308_WMT_22								0x15
#define	IST8308_WMT_23								0x16
#define	IST8308_WMT_24								0x17
#define	IST8308_WMT_25								0x18
#define	IST8308_WMT_26								0x19
#define	IST8308_WMT_27								0x1A
#define	IST8308_WMT_28								0x1B
#define	IST8308_WMT_29								0x1C
#define	IST8308_WMT_30								0x1D
#define	IST8308_WMT_31								0x1E
#define	IST8308_WMT_32								0x1F

//bit5~bit3
#define IST8308_Y_SENSOR_OSR_1			            (0x00<<3)
#define IST8308_Y_SENSOR_OSR_2			            (0x01<<3)
#define IST8308_Y_SENSOR_OSR_4			            (0x02<<3)
#define IST8308_Y_SENSOR_OSR_8			            (0x03<<3)
#define IST8308_Y_SENSOR_OSR_16			            (0x04<<3)
#define IST8308_Y_SENSOR_OSR_32			            (0x05<<3)
//bit2~bit0
#define IST8308_X_Z_SENSOR_OSR_1			        0x00
#define IST8308_X_Z_SENSOR_OSR_2			        0x01
#define IST8308_X_Z_SENSOR_OSR_4			        0x02
#define IST8308_X_Z_SENSOR_OSR_8			        0x03
#define IST8308_X_Z_SENSOR_OSR_16			        0x04
#define IST8308_X_Z_SENSOR_OSR_32			        0x05

typedef enum
{
    INIT = 0,
    PRE_RUN = 1,
    RUN = 2,
    ERR,
} amg_state_e;

typedef enum
{
    SUCCEED_C = 0x00,
    START_C   = 0x01,
    FAILD_C   = 0x02,
    READY_C   = 0x03,
    //GYROOK_C  = 0x04,
    SET_C     = 0x05,
    IDLE_C    = 0x06
}cali_state_e;

typedef struct
{
    unsigned char calistate;
    unsigned char magntype;
    unsigned char gyrotype;

}amg_cali_state_t;
///矩阵方向
typedef struct
{
    float verti1[3];
    float verti2[3];
    float verti3[3];
}matrix_t;

matrix_t matrix = {{X1, X2, X3}, {Y1, Y2, Y3}, {Z1, Z2, Z3}};

static magncali_t magncali_xyz_r = {0};
static gyrocali_t gyrocali_xyz_r = {0};
static amg_cali_state_t bno055_cali;
static cali_state_e calib_imu_state = SUCCEED_C;
static amg_state_e  amg_run_state = INIT;
static  amgdata_t amg_data;

enum gyro_fsr_e {
    FSR_2000DPS = 0,
    FSR_1000DPS,
    FSR_500DPS,
    FSR_250DPS,
    NUM_GYRO_FSR
};

enum accel_fsr_e {
    FSR_16G = 0,
    FSR_8G,
    FSR_4G,
    FSR_2G,
    NUM_ACCEL_FSR
};

static amgdata_t m_amgdata;  //原始值
static float roll;           //姿态角
static float pitch;
static float yaw;
static int imu_mode;         //ble_flag
static float Mag_RawData[3];
static int icm_fd = 0x68;
static int ist_fd = 0x0C;

/////////////////////////////////////////////////////////////////////////////////////////
int init_fd();
int icm_init();
int ist_init();
int icm_read(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
int ist_read(int16_t *xyz);
int ist8308_get_mag_mt(double *magx, double *magy, double *magz);
int read_atti_value();                     //get zitai
int get_power_key();
void is_en_check_gyr_slot();
void is_en_check_amg_slot();

int icm_get_chip_id(uint8_t *chip_id);
int icm_soft_reset();
int icm_gyro_set_fsr(uint8_t fsr);
int icm_acc_set_fsr(uint8_t fsr);
int icm_set_low_noise();
int ist_soft_reset();
int ist_get_chip_id(uint8_t *chip_id);
int ist_set_contionue_read();
int ist_set_operation_mode(uint8_t mode);
int ist_set_water_mark(uint8_t mark);
int ist_set_fifo(int flag);
int ist_set_osr(uint8_t osr);
unsigned char get_imu_amgdata(amgdata_t *amg);
void imu_calib_state();
int read_para_gyrocali(char*, int, char*, int);  
int atti_init(void);
/////////////////////////////////////////////////////////////////////////////

static int set_para_nvs(magncali_t *magncali, gyrocali_t*gyrocali)
{
    esp_err_t err;
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        printf("nvs_open \r\n");
    }

    if(NULL != magncali)
    {
        err = nvs_set_blob(my_handle, "imu_mag", magncali, sizeof(magncali_t));
        if (err != ESP_OK) 
        {
            printf(" mag nvs_set_blob fail\r\n");
        }
    }

    if(NULL != gyrocali)
    {
        err = nvs_set_blob(my_handle, "gyr", gyrocali, sizeof(gyrocali_t));
        if (err != ESP_OK) 
        {
            printf(" gyr nvs_set_blob fail\r\n");
        }
    }

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) 
    {
        printf("nvs_commit \r\n");
    }

    // Close
    nvs_close(my_handle);
    if (err != ESP_OK) 
    {
        printf("nvs_close \r\n");
    }
    return err;
}

static int get_para_nvs(magncali_t *magncali, gyrocali_t*gyrocali)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t len = 0;
    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) 
    {
        printf("nvs_open \r\n");
    }

    if(NULL != magncali)
    {
        len = sizeof(magncali_t);
        err = nvs_get_blob(my_handle, "imu_mag", magncali, &len);
        if (err != ESP_OK) 
        {
            printf(" mag nvs_get_blob fail\r\n");
        }
    }

    if(NULL != gyrocali)
    {
        len = sizeof(gyrocali_t);
        err = nvs_get_blob(my_handle, "gyr", gyrocali, &len);
        if (err != ESP_OK) 
        {
            printf(" gyr nvs_get_blob fail\r\n");
        }
    }

    // Close
    nvs_close(my_handle);
    if (err != ESP_OK) 
    {
        printf("nvs_close \r\n");
    }
    return err;
}


int (*imu_uart_write_bytes)(int uart_num, const void *src, size_t size) = NULL;

void imu_regist_uart_handle(int (*p_fun)(int uart_num, const void *src, size_t size))
{
    imu_uart_write_bytes = p_fun;
}


void imu_get_atti(float* m_roll, float* m_itch, float* m_yaw)
{
    *m_roll = roll;
    *m_itch = pitch;
    *m_yaw = yaw;
}

//i2c硬件平台接口
int imu_i2cdev_data_read(int sensor_add, int reg_add, int len, unsigned char * pdata)
{
	return I2C_ReadData(sensor_add, reg_add, pdata, len);
}

int imu_i2cdev_data_write(int sensor_add, int reg_add, int len, unsigned char * pdata)
{
    return I2C_WriteData(sensor_add, reg_add, pdata, len);
}

static void msleep(unsigned int secs)
{
	vTaskDelay(secs / portTICK_PERIOD_MS);
}

long get_time_tap()
{
    struct timeval time_;
    gettimeofday(&time_, NULL);
    return time_.tv_sec*1000 + time_.tv_usec/1000;
}

void imu_thread_run()
{
    int ret = 0;
    int count = 0;
    do
    {
        ret = atti_init();
        msleep(500);
    } while (0x01 != ret);
    
    // atti_init();
    // msleep(500);

    while(1)
    {
        read_atti_value();
        msleep(10);
        if(count++ > 10)
        {
            count = 0;
            // printf("-------------------------------------\n");
            //printf("roll=%f pitch=%f yaw=%f\n",roll, pitch, yaw);
            // printf("-------------------------------------\n");
            //printf("a_x=%f; a_y=%f; a_z=%f; g_x=%f; g_y=%f; g_z=%f; m_x=%f; m_y=%f; m_z=%f;\n",
                                                                                        //    m_amgdata.acel_x, m_amgdata.acel_y, m_amgdata.acel_z,
                                                                                        //    m_amgdata.gyro_x, m_amgdata.gyro_y, m_amgdata.gyro_z,
                                                                                        //    m_amgdata.magn_x, m_amgdata.magn_y, m_amgdata.magn_z);
        }
    }
}

//初始化六轴
int atti_init(void)
{
    int ret = 0;
    imu_i2c_init();
    ret = icm_init();
    ret |= ist_init();
 
    if(3 == ret)
    {
        return 1;
    }
    return 0;
}

int icm_get_chip_id(uint8_t *chip_id)
{
    int ret = 0;
    ret = imu_i2cdev_data_read(icm_fd, WHO_AM_I, 1, chip_id);

    return ret;
}

int icm_soft_reset()
{
    int ret = 0;
    uint8_t data = 0x10;

    ret = imu_i2cdev_data_write(icm_fd, SIGNAL_PATH_RESET, 1, &data);
    return ret;
}

int icm_gyro_set_fsr(uint8_t fsr)
{
    uint8_t data = 0;
    int ret = 0;
    ret = imu_i2cdev_data_read(icm_fd, GYRO_CONFIG0, 1, &data);
    data &= 0x0F;
    data |= fsr<<5;
    ret = imu_i2cdev_data_write(icm_fd, GYRO_CONFIG0, 1, &data);
    return ret;
}

int icm_acc_set_fsr(uint8_t fsr)
{
    uint8_t data = 0;
    int ret = 0;

    ret = imu_i2cdev_data_read(icm_fd, ACCEL_CONFIG0, 1, &data);
    data &= 0x0F;
    data |= fsr<<5;
    ret = imu_i2cdev_data_write(icm_fd, ACCEL_CONFIG0, 1, &data);

    return ret;
}

int icm_set_low_noise()
{
     uint8_t data;
     int ret = 0;

     ret =  imu_i2cdev_data_read(icm_fd, PWR_MGMT0, 1, &data);
     data &= ~((uint8_t)(0x03<<2 ));
     data |= (0x03<<2);
     data &= ~((uint8_t)(0x03<<0));
     data |= 0x03<<0;
     ret = imu_i2cdev_data_write(icm_fd, PWR_MGMT0, 1, &data);
    return ret;
}

int icm_init()
{
    uint8_t chip_id = 0;
    icm_get_chip_id(&chip_id);
    if(chip_id != 0x67)
    {
        printf("icm init failed\n");
        return -2;
    }
    //printf("icm_init ok %d\n", ret);

    icm_soft_reset();
    msleep(50);
    icm_gyro_set_fsr(FSR_2000DPS);  //2000
    icm_acc_set_fsr(FSR_16G);
    icm_set_low_noise();
    msleep(50);
    imu_Init(0.01);
    sensors_seten("acel", 1);
    sensors_seten("gyro", 1);
    sensors_seten("magn", 1);  //magn switch on off
    msleep(50);
    return 1;
}



int icm_read(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t buffer[12];
    int ret = 0;

    ret = imu_i2cdev_data_read(icm_fd, ACCEL_DATA_X1, 12, buffer);
    *acc_x = (((int16_t) buffer[0]) << 8) | buffer[1];
    *acc_y = (((int16_t) buffer[2]) << 8) | buffer[3];
    *acc_z = (((int16_t) buffer[4]) << 8) | buffer[5];
    *gyro_x = (((int16_t) buffer[6]) << 8) | buffer[7];
    *gyro_y = (((int16_t) buffer[8]) << 8) | buffer[9];
    *gyro_z = (((int16_t) buffer[10]) << 8) | buffer[11];

    return ret;
}


int ist_soft_reset()
{
    int ret = 0;
    uint8_t data;
    ret = imu_i2cdev_data_read(ist_fd, IST_REG_CNTRL3, 1, &data);
    data |= 0x01;
    ret = imu_i2cdev_data_write(ist_fd, IST_REG_CNTRL3, 1, &data);
    return ret;
}

int ist_get_chip_id(uint8_t *chip_id)
{
    int ret = 0;
    ret = imu_i2cdev_data_read(ist_fd, IST_WHO_AM_I, 1, chip_id);

    return ret;
}

int ist_set_contionue_read()
{
    int ret = 0;
    uint8_t data;
    ret = imu_i2cdev_data_read(ist_fd, IST_REG_CNTRL6, 1, &data);
    data &= ~0x01;
    ret = imu_i2cdev_data_write(ist_fd, IST_REG_CNTRL6, 1, &data);
    return ret;
}

#define IST_REG_CNTRL2				            0x31

int ist_set_operation_mode(uint8_t mode)
{
    int ret = 0;
    uint8_t data;

    ret = imu_i2cdev_data_read(ist_fd, IST_REG_CNTRL2, 1, &data);
    data &= ~0x1F;
    data |= mode;
    ret = imu_i2cdev_data_write(ist_fd, IST_REG_CNTRL2, 1, &data);

    return ret;
}

#define IST_REG_CNTRL1				            0x30
int ist_set_water_mark(uint8_t mark)
{
    int ret = 0;
    uint8_t data, data_reg;

    ret = imu_i2cdev_data_read(ist_fd, IST_REG_CNTRL2, 1, &data_reg);
    ist_set_operation_mode(IST8308_OPMODE_STANDBY_MODE);

    ret = imu_i2cdev_data_read(ist_fd, IST_REG_CNTRL1, 1, &data);
    data &= 0xE0;
    data |= mark;
    ret = imu_i2cdev_data_write(ist_fd, IST_REG_CNTRL1, 1, &data);

    ret = imu_i2cdev_data_write(ist_fd, IST_REG_CNTRL2, 1, &data_reg);

    return ret;
}

int ist_set_fifo(int flag)
{
    int ret = 0;
    uint8_t data;

    ret = imu_i2cdev_data_read(ist_fd, IST_REG_CNTRL2, 1, &data);
    if(flag == 1)
    {
        data |= BIT7;
    }
    else
    {
        data &= ~BIT7;
    }
    ret = imu_i2cdev_data_write(ist_fd, IST_REG_CNTRL2, 1, &data);
    return ret;
}

int ist_set_osr(uint8_t osr)
{
    int ret = 0;
    uint8_t data;

    data = osr;
    ret = imu_i2cdev_data_write(ist_fd, IST_REG_CNTRL2, 1, &data);
    return ret;
}

int ist_init(void)
{
    uint8_t chip_id;
    msleep(50);
    ist_soft_reset();
    msleep(50);
    ist_get_chip_id(&chip_id);
    if(0x08 != chip_id)
    {
        printf("The ist chip err:%x\n", chip_id);
        return -2;
    }
    //printf("The chip id %X\n", chip_id);
    ist_set_contionue_read();
    ist_set_operation_mode(IST8308_OPMODE_STANDBY_MODE);
    ist_set_water_mark(IST8308_WMT_5);
    ist_set_fifo(1);
    ist_set_osr(IST8308_Y_SENSOR_OSR_1 | IST8308_X_Z_SENSOR_OSR_1);
    ist_set_operation_mode(IST8308_OPMODE_CONTINUOS_MODE_50HZ);

    return 2;
}

#define IST8308_REG_STATUS1				            0x10
#define STATUS1_DRDY				                0x01
#define IST8308_REG_DATAX				            0x11

int ist_read(int16_t *xyz)
{
    uint8_t buffer[6], status = 0;

    imu_i2cdev_data_read(ist_fd, IST8308_REG_STATUS1, 1, &status);

    if((status & STATUS1_DRDY) == STATUS1_DRDY)
    {
        imu_i2cdev_data_read(ist_fd, IST8308_REG_DATAX, 6, buffer);
        xyz[0] = (((int16_t) buffer[1]) << 8) | buffer[0];
        xyz[1] = (((int16_t) buffer[3]) << 8) | buffer[2];
        xyz[2] = (((int16_t) buffer[5]) << 8) | buffer[4];

        return 1;
    }
    else
    {
        return 0;
    }
}

int ist8308_get_mag_mt(double *magx, double *magy, double *magz)
{
    //Sensitivity
    #define	MAG_LSBTOUT				6.6
    int16_t msensorData[3];
    uint8_t temp = 0;
    uint8_t i = 0;
    temp = ist_read(msensorData);
    if(temp !=  0)
    {
        for(i = 0 ; i < temp ; i += 3)
        {

            Mag_RawData[0] =  (float)msensorData[i] / (float)MAG_LSBTOUT;
            Mag_RawData[1] =  -(float)msensorData[i + 1] / (float)MAG_LSBTOUT;
            Mag_RawData[2] =  (float)msensorData[i + 2] / (float)MAG_LSBTOUT;
            //printf("Mag_RawData[0]= %f mT, Mag_RawData[1] = %f mT, Mag_RawData[0] = %f mT,\r\n", Mag_RawData[0],Mag_RawData[1], Mag_RawData[2]);

        }
    }
    *magx = Mag_RawData[0];
    *magy = -Mag_RawData[1];
    *magz = Mag_RawData[2];

    return 1;                        //bad carll
}

/***************************************************************************
  Function:        get_bno055_cali_state()
  Description:     获取校准状态，CAN总线查询校准状态时调用
  Input:
  Return:
  author - year/m/d - V1.1
  Others:
***************************************************************************/
void get_bno055_cali_state(amg_cali_state_t *calitype)
{
    bno055_cali.calistate = calib_imu_state;
    memcpy(calitype, &bno055_cali, sizeof(amg_cali_state_t));
}

/***************************************************************************
  Function:        set_bno055_cali_state()
  Description:     设置校准状态，CAN总线设置校准状态时调用
  Input:
  Return:
  author - year/m/d - V1.1
  Others:
***************************************************************************/
void set_bno055_cali_state(amg_cali_state_t *calitype)
{
    calib_imu_state = (cali_state_e)(*calitype).calistate;
}


//------------------------------------------------------------------------------------carll-------------------------------------------------------------------------------
/***************************************************************************
  Function:        matrix_change()
  Description:     矩阵转换
  Input:
  Return:
  author - year/m/d - V1.1
  Others:          采用纵向数组计数方法 矩阵格式为
                   v1[0]  V2[0]  V3[0]
                   v1[1]  V2[1]  V3[1]
                   v1[2]  V2[2]  V3[2]
***************************************************************************/
void matrix_change(double *x, double *y, double *z)
{
    double tempx, tempy, tempz;
    tempx = *x;
    tempy = *y;
    tempz = *z;

    *x = tempx * matrix.verti1[0] + tempy * matrix.verti1[1] + tempz * matrix.verti1[2];
    *y = tempx * matrix.verti2[0] + tempy * matrix.verti2[1] + tempz * matrix.verti2[2];
    *z = tempx * matrix.verti3[0] + tempy * matrix.verti3[1] + tempz * matrix.verti3[2];
}

uint8_t get_imu_amgdata(amgdata_t *amg)
{
    short aacx, aacy, aacz;   //加速度传感器原始数据
    short gyrox, gyroy, gyroz; //陀螺仪原始数据
    double ist8308_mx, ist8308_my, ist8308_mz;
    double gx, gy, gz;
    double ax, ay, az;
    double mx, my, mz;

    icm_read(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz);

    ist8308_get_mag_mt(&ist8308_mx, &ist8308_my, &ist8308_mz);

    mx = -ist8308_my;
    my = -ist8308_mx;
    mz = ist8308_mz;

    ax = aacx * G  * 16.0f / 32768.0f;
    ay = aacy * G * 16.0f / 32768.0f;
    az = aacz * G * 16.0f / 32768.0f;

    gx = gyrox * PI_180  * 2000.0f / 32768.0f;
    gy = gyroy * PI_180 * 2000.0f / 32768.0f;
    gz = gyroz * PI_180 * 2000.0f / 32768.0f;


    matrix_change(&gx, &gy, &gz);
    matrix_change(&ax, &ay, &az);
    matrix_change(&mx, &my, &mz);

    amg->magn_x = mx;
    amg->magn_y = my;
    amg->magn_z = mz;

    amg->acel_x = ax;
    amg->acel_y = ay;
    amg->acel_z = az;

    amg->gyro_x = gx;
    amg->gyro_y = gy;
    amg->gyro_z = gz;

    return 0;
}


//这个接口是个任务，需要循环调用，返回姿态角
int read_atti_value()
{
    static int err_cnt = 0;
    float roll_jcc, pitch_jcc, yaw_jcc;
    switch(amg_run_state)
    {
        case INIT:
        {
             get_para_nvs(&magncali_xyz_r, &gyrocali_xyz_r);
             //printf("read_para_gyrocali %d magncali_xyz_r %d  gyrocali_xyz_r %d \r\n", ret, magncali_xyz_r.type, gyrocali_xyz_r.type);

             bno055_cali.magntype = magncali_xyz_r.type;
             bno055_cali.gyrotype = gyrocali_xyz_r.type;


             if((gyrocali_xyz_r.type >= SENSOR_CALI_OK) && (magncali_xyz_r.type >= SENSOR_CALI_OK))
             {
                 magn_setcalibration(&magncali_xyz_r);
                 gyro_setcalibration(&gyrocali_xyz_r);
                 calib_imu_state = IDLE_C;
                 //printf("file init gyrocali_xyz_rtype %d %f %f %f %f %f %f\n",gyrocali_xyz_r.type, gyrocali_xyz_r.x, gyrocali_xyz_r.y,gyrocali_xyz_r.z,gyrocali_xyz_r.kx,gyrocali_xyz_r.ky,gyrocali_xyz_r.kz);
                 //printf("file init magncali_xyz_rtype %d %f %f %f %f %f %f\n",magncali_xyz_r.type, magncali_xyz_r.x, magncali_xyz_r.y,magncali_xyz_r.z,magncali_xyz_r.kx,magncali_xyz_r.ky,magncali_xyz_r.kz);

             }
             else
            {                                                          //去掉flash存储 直接使用设置参数
                magncali_xyz_r.type = 2;
                magncali_xyz_r.x = 1;
                magncali_xyz_r.y = 1;
                magncali_xyz_r.z = 1;
                magncali_xyz_r.kx = 1;
                magncali_xyz_r.ky = 1;
                magncali_xyz_r.kz = 1;

                gyrocali_xyz_r.type = 2;
                gyrocali_xyz_r.x = 0;       //ci pian jiao
                gyrocali_xyz_r.y = 0;
                gyrocali_xyz_r.z = 0;
                gyrocali_xyz_r.kx = 1;
                gyrocali_xyz_r.ky = 1;
                gyrocali_xyz_r.kz = 1;

                set_para_nvs(&magncali_xyz_r, &gyrocali_xyz_r);                           //设置参数
                get_para_nvs(&magncali_xyz_r, &gyrocali_xyz_r); //test
                
                //test carll
                magn_setcalibration(&magncali_xyz_r);
                gyro_setcalibration(&gyrocali_xyz_r);
                //test carll

                calib_imu_state = IDLE_C;

                printf("default para %d,%d\n", magncali_xyz_r.type, gyrocali_xyz_r.type);
            }

            amg_run_state = PRE_RUN;
            break;
        }

        case PRE_RUN:
        {
            if(calib_imu_state == IDLE_C)
            {
                amg_run_state = RUN;
            }
            else
            {
                if(get_imu_amgdata(&amg_data))
                {
                    amg_run_state = ERR;
                }
                else
                {
                    imu_calib_state();
                }
            }
            break;
        }
        case RUN:
        {
            if(get_imu_amgdata(&amg_data))
            {
                amg_run_state = ERR;
            }
            else
            {
                if(calib_imu_state == IDLE_C)
                {
                    amg_sensorinput(&amg_data, get_time_tap());    //time_get
                    m_amgdata = amg_data;
                    imu_getattitude(&roll_jcc, &pitch_jcc, &yaw_jcc);

                    //ci pian jiao
                    // if((sublue_device.magn_declination > 3600) || (sublue_device.magn_declination < -3600))
                    // {
                    //     sublue_device.magn_declination = 0;
                    // }
                    // yaw_jcc = yaw_jcc + ((float)(sublue_device.magn_declination) / 10.0f);

                    if(yaw_jcc > 360)
                    {
                        yaw_jcc -= 360;
                    }

                    else if(yaw_jcc < 0)
                    {
                        yaw_jcc += 360;
                    }


                    roll_jcc += 180;  ///roll_jcc up time 180  fix bug

                    if(roll_jcc > 180)
                    {
                        roll_jcc -= 360;
                    }
                    else if(roll_jcc < -180)
                    {
                        roll_jcc += 360;
                    }

                    //以下给成员变量赋值
                    roll = roll_jcc;
                    pitch = pitch_jcc;
                    yaw = yaw_jcc;
                }
                else
                {
                    amg_run_state = PRE_RUN;
                }
            }
            break;
        }
        case ERR:
        {
            if(get_imu_amgdata(&amg_data))
            {
                err_cnt++;
                if(err_cnt > 5)
                {
                    err_cnt = 0;
                    amg_run_state = INIT;
                }
            }
            else
            {
                amg_sensorinput(&amg_data, get_time_tap());  //time_get
                amg_run_state = RUN;
                err_cnt = 0;
            }
            break;
        }
    }

    return 1;
}

static void magn_finished(void)
{
    magncali_t  magnscali;
    magn_getcalibration(&magnscali);
    if(magnscali.type >= SENSOR_CALI_OK)
    {
        set_para_nvs(&magnscali, NULL);
        calib_imu_state = SUCCEED_C;
        calib_imu_state = SUCCEED_C;
    }
    bno055_cali.magntype = magnscali.type;
}

static void gyro_finished(void)
{
    gyrocali_t  gyroscali;
    gyro_getcalibration(&gyroscali);
    if(gyroscali.type >= SENSOR_CALI_OK)
    {
        set_para_nvs(NULL, &gyroscali);
        calib_imu_state = SUCCEED_C;
    }
    bno055_cali.gyrotype = gyroscali.type;
}

// jiao zhun
void imu_calib_state(void)
{
    amg_sensorinput(&amg_data, get_time_tap());

    switch(calib_imu_state)
    {
        case START_C:                    //蓝牙端使能 进入到 START_C
        {
            if(imu_mode == 1)
            {
                sensors_seten("magn", 1);
                magn_startcalibration(600, magn_finished);
                bno055_cali.magntype = 0;
                //printf("bno055_cali.magntype = %d\r\n", bno055_cali.magntype);
            }
            else if(imu_mode == 0)
            {
                sensors_seten("gyro", 1);
                gyro_startcalibration(600, gyro_finished);
                bno055_cali.gyrotype = 0;
                //printf("bno055_cali.gyrotype = %d\r\n", bno055_cali.gyrotype);
            }
            calib_imu_state = READY_C;
            break;
        }
        case READY_C:
            //printf("READY_C\r\n");
            break;
        case SUCCEED_C:
        {
            get_para_nvs(&magncali_xyz_r, &gyrocali_xyz_r);
            //printf("SUCCEED_C %d \r\n", ret);

            gyro_getcalibration(&gyrocali_xyz_r);
            magn_getcalibration(&magncali_xyz_r);
            if((gyrocali_xyz_r.type >= SENSOR_CALI_OK) && (magncali_xyz_r.type >= SENSOR_CALI_OK))
            {
                magn_setcalibration(&magncali_xyz_r);
                gyro_setcalibration(&gyrocali_xyz_r);
                calib_imu_state = IDLE_C;

                char* buf = "check_ok";
                if(NULL != imu_uart_write_bytes)
                {
                    imu_uart_write_bytes(1, buf, strlen(buf));
                }
            }
            else
            {
                printf("START_C.....\r\n");
                calib_imu_state = START_C;
            }
            break;
        }
        case SET_C:
        {
            calib_imu_state = SUCCEED_C;
            break;
        }
        case IDLE_C:
        {

            break;
        }
        default:
        {
            calib_imu_state = SUCCEED_C;
            break;
        }
    }
}

void is_en_check_gyr_slot(void)
{
    amg_cali_state_t calitype;
    calitype.calistate = 1;
    imu_mode = 0;
    set_bno055_cali_state(&calitype);
}

void is_en_check_amg_slot(void)
{
    amg_cali_state_t calitype;
    calitype.calistate = 1;
    imu_mode = 1;
    set_bno055_cali_state(&calitype);
}