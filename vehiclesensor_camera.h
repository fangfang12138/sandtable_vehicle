#ifndef VEHICLESENSOR_CAMERA_H
#define VEHICLESENSOR_CAMERA_H

#include <QObject>
#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QSerialPort>
#include <QSerialPortInfo>

#include <cmath>

#include "librealsense2/rs.hpp"
#include "opencv4/opencv2/opencv.hpp"

using namespace cv;

/*class: Camera
 * 深度相机
 * 1 包括realsense D435 的基本使用
 *
 */
class Camera : public QObject
{
    Q_OBJECT
public:
    explicit Camera(QObject *parent = nullptr);
    void initCamera(QMainWindow* _vehicle);     //初始化相机对象

    void realsenseTest();                       //D435基本用法
    Mat getImage();                             //获取原始图像
    void trans(Mat oriImage);                   //车道线识别函数
    void calibrate();                           //车道线识别预处理：校正

private:
    QMainWindow* vehicleConsole;                //车辆主控指针
    Mat cali_matrix;                            //透视矫正矩阵
    Mat image_read;                             //实时原始图像

private slots:
    void on_toStartSensor();                    //接收车辆主线程信号
    void on_toStopSensor();                     //接收车辆主线程信号
};

#endif // VEHICLESENSOR_CAMERA_H
