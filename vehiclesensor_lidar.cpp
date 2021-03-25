#include "vehiclesensor_lidar.h"

//-------------------------------------------------------------------------------激光雷达
Lidar::Lidar(QObject *parent) : QObject(parent)
{

}

/* Function: Lidar::initLidar
 * 初始化雷达对象
 */
void Lidar::initLidar(QMainWindow *_vehicle)
{
    vehicleConsole = _vehicle;

    connect(vehicleConsole,SIGNAL(toStartSensor()),this,SLOT(on_toStartSensor()),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(toStopSensor()),this,SLOT(on_toStopSensor()),Qt::BlockingQueuedConnection);
    connect(this,SIGNAL(obstacleInfo(bool,double,double,QString)),
            vehicleConsole,SLOT(on_obstacleInfo(bool,double,double,QString)),Qt::QueuedConnection);
}

/* Function: Lidar::on_toStartSensor
 * 接收车辆主线程信号，启动串口，开始接收激光数据
 */
void Lidar::on_toStartSensor()
{
    qInfo()<<"connect & start Lidar...";
//定时器：定时检测障碍物
    timerLidar = new QTimer(this);
    connect(timerLidar,SIGNAL(timeout()),this,SLOT(obstacleDetection()));

    u = nullptr;
    connect2Lidar();
    startScan();
}

/* Function: Lidar::on_toStopSensor
 * 接收车辆主线程信号，关闭串口，停止接收激光数据
 */
void Lidar::on_toStopSensor()
{
    qInfo()<<"stop & disconnect Lidar...";

    timerLidar->stop();
    stopScan();
    disconnect2Lidar();
}

/* Function: Lidar::obstacleDetection
 * 定时检测障碍物
 */
void Lidar::obstacleDetection()
{
    if(!u)
        return;

    rplidar_response_measurement_node_hq_t buffer[1080];

    unsigned int bufmax=1080;
    u->grabScanDataHq(buffer,bufmax);//读取数据到buffer

    for(unsigned int i=0;i<bufmax;i++)//展示
    {
        double angle = buffer[i].angle_z_q14 * 90.f / (1<<14);
        double dis_mm = buffer[i].dist_mm_q2 / (1<<2);

        if((dis_mm < 300) && (dis_mm > 100) && ((angle > 315) || (angle < 45)))
            emit obstacleInfo(true,angle,dis_mm,"info");
    }
}

/* Function: Lidar::connect2Lidar
 * 创建雷达连接
 */
void Lidar::connect2Lidar()
{
    if(!u)
    {
        qInfo()<<"create driver...";
        u=rp::standalone::rplidar::RPlidarDriver::CreateDriver();
    }

    unsigned int baudrate = 115200;
    const char* path ="/dev/ttyUSB1";

    if((u->connect(path,baudrate))==0)
    {
        qInfo()<<"connected...";
        u->startMotor();
    }
    else
    {
        qInfo()<<"time out...";
        u = nullptr;
    }
}

/* Function: Lidar::disconnect2Lidar
 * 断开雷达连接
 */
void Lidar::disconnect2Lidar()
{
    if(u)//如果已经创建了一个驱动，就允许断开连接这个操作，如果没有驱动创建，就不进行任何操作
    {
        u->stopMotor();
        if(u->isConnected())
            u->disconnect();
        rp::standalone::rplidar::RPlidarDriver::DisposeDriver(u);
    }
}

/* Function: Lidar::startScan
 * 开始扫描输出
 */
void Lidar::startScan()
{
    if(u)//如果已经创建了一个驱动，就允许开始扫描这个操作，如果没有驱动创建，就不进行任何操作
    {
        if(u->isConnected())
        {
            u->startScan(false,true);//开始扫描
            timerLidar->start(50);
        }
        else
        {
            timerLidar->stop();
            qInfo()<<"do not connect to the lidar...";
        }
    }
}

/* Function: Lidar::stopScan
 * 停止扫描输出
 */
void Lidar::stopScan()
{
    if(u)//如果已经创建了一个驱动，就允许停止扫描这个操作，如果没有驱动创建，就不进行任何操作
    {
        if(u->isConnected())
            u->stop();
    }
}

