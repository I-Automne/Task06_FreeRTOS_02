/*
 * MPU6050.c
 *
 *
 * 
 */

#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
/**************************自检函数********************************************
													用于消除零漂
 *******************************************************************************/
static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    } else {
        return -1;
    }

    return 0;
}
/**************************MPU6050初始化函数*****************************************
 *******************************************************************************/
int MPU6050_DMP_init(void)
{
    int ret;
    struct int_param_s int_param;
    //mpu_init
    ret = mpu_init(&int_param);
    if(ret != 0)
    {
        return ERROR_MPU_INIT;
    }
    //设置传感器
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_SET_SENSOR;
    }
    //设置fifo
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if(ret != 0)
    {
        return ERROR_CONFIG_FIFO;
    }
    //设置采样率
    ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_RATE;
    }
    //加载DMP固件
    ret = dmp_load_motion_driver_firmware();
    if(ret != 0)
    {
        return ERROR_LOAD_MOTION_DRIVER;
    }
    //设置陀螺仪方向
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if(ret != 0)
    {
        return ERROR_SET_ORIENTATION;
    }
    //设置DMP功能
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if(ret != 0)
    {
        return ERROR_ENABLE_FEATURE;
    }
    //设置输出速率
    ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if(ret != 0)
    {
        return ERROR_SET_FIFO_RATE;
    }
    //自检
    ret = run_self_test();
    if(ret != 0)
    {
        return ERROR_SELF_TEST;
    }
    //使能DMP
    ret = mpu_set_dmp_state(1);
    if(ret != 0)
    {
        return ERROR_DMP_STATE;
    }

    return 0;
}
/**************************MPU6050欧拉角解算*****************************************
 *******************************************************************************/
int MPU6050_DMP_Get_Date(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    if(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    if(sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
        *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    }

    return 0;
}

/**************************MPU6050六轴数据读取*****************************************
可看出来：

gyro[]：三轴角速度（单位：LSB）

accel[]：三轴加速度（单位：LSB）

quat[]：四元数（DMP解算结果）

而dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);会更新一次数据
如果dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)==1;
则读取失败（显然写在函数里面了）
 *******************************************************************************/
int MPU6050_DMP_Get6Axis(short *accel, short *gyro)		//获取六轴数据
{
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    return 0;
}

int MPU6050_DMP_Get6Axis_f(float *accel_g, float *gyro_dps)		//得到六轴数据的浮点数（用这个函数）传进去俩数组分别代表加速度和角速度
{
    short accel[3];
    short gyro[3];

    if (MPU6050_DMP_Get6Axis(accel, gyro) != 0)
        return -1;

    // 转为物理量
    accel_g[0] = accel[0] / 16384.0f;
    accel_g[1] = accel[1] / 16384.0f;
    accel_g[2] = accel[2] / 16384.0f;

    gyro_dps[0] = gyro[0] / 16.4f;
    gyro_dps[1] = gyro[1] / 16.4f;
    gyro_dps[2] = gyro[2] / 16.4f;

    return 0;
}
/**************************量程问题？*****************************************

加载 DMP 固件后，DMP 会自动配置 MPU6050 的寄存器（包括陀螺仪和加速度量程）。
默认配置如下（根据官方 Invensense 代码文档）：

传感器	默认DMP设置	对应分辨率
加速度计	±2g	16384 LSB/g
陀螺仪	±2000°/s	16.4 LSB/(°/s)

也就是说：

即使你没有手动设置，DMP 会在固件加载时帮你配置成标准值（2g 和 2000°/s）。
 *******************************************************************************/
/**************************温漂？*****************************************

加载 DMP 固件后，DMP 会启用了 DMP 的 陀螺仪零偏自校准 功能。
这个功能会在设备静止时自动判断偏移量并修正。
它利用内部温度读数做一定的自学习校正。
这相当于是一种“动态温度漂移自补偿”。

（这些都是GPT写的 反正我真的是不太懂 不过这样一说倒是了解了一些基本原理XD）
 *******************************************************************************/