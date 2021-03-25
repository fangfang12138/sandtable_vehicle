#ifndef VEHICLESENSOR_LIDAR_H
#define VEHICLESENSOR_LIDAR_H

#include <QObject>
#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QSerialPort>
#include <QSerialPortInfo>

#include <cmath>

#include "rplidar.h"

/*class: Lidar
 * 激光雷达
 * 1 包括Lidar连接创建和断开、扫描的起止等
 *
 */
class Lidar : public QObject
{
    Q_OBJECT
public:
    explicit Lidar(QObject *parent = nullptr);
    void initLidar(QMainWindow* _vehicle);      //初始化雷达对象
    void connect2Lidar();                       //创建雷达连接
    void disconnect2Lidar();                    //断开雷达连接
    void startScan();                           //开始扫描输出
    void stopScan();                            //停止扫描输出

private:
    QMainWindow* vehicleConsole;                //车辆主控指针
    rp::standalone::rplidar::RPlidarDriver *u;  //雷达驱动
    QTimer* timerLidar;                              //定时检测障碍物

signals:
    void obstacleInfo(bool isStop, double obstacleDir, double obstacleDis_mm, QString info);

private slots:
    void on_toStartSensor();                    //接收车辆主线程信号，启动串口，开始接收激光数据
    void on_toStopSensor();                     //接收车辆主线程信号，关闭串口，停止接收激光数据

    void obstacleDetection();                   //定时检测障碍物
};


#endif // VEHICLESENSOR_LIDAR_H
