#include "vehiclesensor_pose.h"

//-------------------------------------------------------------------------------车辆位姿
Pose::Pose(QObject *parent) : QObject(parent)
{
    initBaseVariable();
}

/* Function: Pose::initSerial
 * 初始化位姿对象
 */
void Pose::initPose(QMainWindow* _vehicle)
{
    vehicleConsole = _vehicle;

    connect(vehicleConsole,SIGNAL(toStartSensor()),this,SLOT(on_toStartSensor()),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(toStopSensor()),this,SLOT(on_toStopSensor()),Qt::BlockingQueuedConnection);
    connect(this,SIGNAL(curPose(double,double,double)),vehicleConsole,SLOT(on_curPose(double,double,double)),Qt::QueuedConnection);
}

/* Function: Pose::initBaseVariable
 * 初始化基本变量
 */
void Pose::initBaseVariable()
{
    debugPose = false;
    debugNFC = true;

//初始化
    //标定IMU初值
    isCalibration = true;
    averageCount = 0;

    wxAverage_rad_s = wzAverage_rad_s = 0;
    ayAverage_g = 0;
    rollAverage_rad=pitchAverage_rad=0;

    //处理IMU数据
    completeIMUFrame.clear();
    lastTimeStamp = 0;
    wzLast_rad_s = wxLast_rad_s = 0;
    ayCaliLast_g = 0;
    curSpeed_m_s=0;                                 //积分当前车辆纵向速度

    //车辆位姿输出
    poseX_m=poseY_m=poseYaw_rad=poseRoll_rad=0;    //当前车辆位置、偏航角、俯仰角
    poseYaw_rad = 90.0/180.0*3.14159;

    //校准值输入
    curSpeedRaw_r_s=0;                             //编码器校准：速度
    curDirection=true;                             //编码器校准：当前车辆运动方向（前进/后退）

    poseXnfc_m=poseYnfc_m=0;                        //位置NFC校准
    IDnfc = 0;
    poseYawCorrect_rad = 0;                         //偏航相机校准
}

/* Function: Pose::on_toStartSensor
 * 接收车辆主线程信号，启动串口，开始接收IMU和NFC数据
 */
void Pose::on_toStartSensor()
{
    qInfo()<<"connect & start IMU/NFC...";

//建立串口的连接
    serialIMU = new QSerialPort(this);
    serialNFC = new QSerialPort(this);

    connect(serialIMU, SIGNAL(readyRead()), this, SLOT(on_readyread_IMU()));
    connect(serialNFC, SIGNAL(readyRead()), this, SLOT(on_readyread_NFC()));

    if(!enableSerial(serialIMU,"/dev/ttyUSB0",115200))
        return;
    if(!enableSerial(serialNFC,"/dev/ttyAMA2",9600))
        return;

//配置IMU为流模式、ASCII模式并保存
    unsigned char flowMode[11]={0x3a,0x01,0x00,0x07,0x00,0x00,0x00,0x08,0x00,0x0d,0x0a};
    imuSetting(flowMode,11);
    unsigned char asciiMode[15]={0x3a,0x01,0x00,0x84,0x00,0x04,0x00,0x01,0x00,0x00,0x00,0x8a,0x00,0x0d,0x0a};
    imuSetting(asciiMode,15);
    unsigned char save[11]={0x3a,0x01,0x00,0x04,0x00,0x00,0x00,0x05,0x00,0x0d,0x0a};
    imuSetting(save,11);

//定时器
    timerPose = new QTimer(this);
    connect(timerPose,SIGNAL(timeout()),this,SLOT(sendCurPose2VehicleConsole()));
    timerPose->start(50);

    QTimer::singleShot(3000,this,SLOT(on_calibration()));//单次触发标定函数
}

/* Function: Pose::on_toStopSensor
 * 接收车辆主线程信号，关闭串口，停止接收IMU和NFC数据
 */
void Pose::on_toStopSensor()
{
    qInfo()<<"stop & disconnect IMU/NFC...";

    disableSerial(serialIMU);
    disableSerial(serialNFC);

    timerPose->stop();
    delete timerPose;
}

/* Function: Pose::on_readyread_IMU
 * 接收IMU数据
 */
void Pose::on_readyread_IMU()
{
    //从接收缓冲区中读取数据
    QByteArray buffer = serialIMU->readLine();//readAll();//read(120);

    foreach (char b, buffer)
    {
        if(b!='\n')
            completeIMUFrame.append(b);
        else
        {
            //从完整的一帧中输出某些量
            QStringList strList=completeIMUFrame.split(',');
            completeIMUFrame.clear();

            if(strList.count()!=23)
                continue;

//标定IMU：计算平均零点漂移数据，定时触发，只触发一次
            if(isCalibration)
            {
                //下面的量用于：角度积分
                    QString wx = strList.at(13);
                    double wx_rad_s = wx.toDouble()/100.0/180.0*3.1415926;
                    QString wz = strList.at(15);
                    double wz_rad_s = wz.toDouble()/100.0/180.0*3.1415926;

                    wxAverage_rad_s = (wxAverage_rad_s * averageCount + wx_rad_s)/(averageCount+1);
                    wzAverage_rad_s = (wzAverage_rad_s * averageCount + wz_rad_s)/(averageCount+1);

                //下面的量用于：imu积分得到速度
                    QString ay = strList.at(5);
                    double ay_g = ay.toDouble()/1000;
                    QString roll_ = strList.at(20);
                    double roll_rad = roll_.toDouble()/100.0/180.0*3.1415926;
                    QString pitch_ = strList.at(21);
                    double pitch_rad = pitch_.toDouble()/100.0/180.0*3.1415926;

                    ayAverage_g = (ayAverage_g * averageCount + ay_g)/(averageCount+1);
                    rollAverage_rad = (rollAverage_rad * averageCount + roll_rad)/(averageCount+1);
                    pitchAverage_rad = (pitchAverage_rad * averageCount + pitch_rad)/(averageCount+1);

                //计数值++
                averageCount++;
                continue;
            }

//提取时间
            QString stamp = strList.first();
            long long timeStamp = stamp.right(stamp.size()-1).toLongLong();
            double dt = 0.01;
            if(lastTimeStamp > 0)
                dt = (timeStamp - lastTimeStamp)*0.002;//计算帧间的时间差
            lastTimeStamp = timeStamp;

//车辆静止时，不积分
            if(curSpeedRaw_r_s < 0.1)
            {
                wzLast_rad_s = 0;
                wxLast_rad_s = 0;
                ayCaliLast_g = 0;
                continue;
            }

//位置积分
            //提取imu的z轴角速度，并积分得到偏航角
                QString wz = strList.at(15);
                double wz_rad_s = wz.toDouble()/100.0/180.0*3.1415926 - wzAverage_rad_s;
                double poseYawRaw_rad = poseYaw_rad + 0.5*(wz_rad_s+wzLast_rad_s)*dt;//中值积分
                wzLast_rad_s = wz_rad_s;

            //提取imu的x轴角速度，并积分得到俯仰角
                QString wx = strList.at(13);
                double wx_rad_s = wx.toDouble()/100.0/180.0*3.1415926 - wxAverage_rad_s;
                double poseRollRaw_rad = poseRoll_rad + 0.5*(wx_rad_s+wxLast_rad_s)*dt;//中值积分
                wxLast_rad_s = wx_rad_s;


            //统一角度基准：x轴（地图的x轴，非imu的x轴，y轴同理）正向为起始，向y轴负方向旋转0～360度，在循迹中也有基准统一部分
                poseYawRaw_rad = caliAngle(poseYawRaw_rad);


            //提取加速度，并积分得到速度
                QString ay = strList.at(5);
                double ay_g = ay.toDouble()/1000 - ayAverage_g;

                QString roll_ = strList.at(20);
                double roll_rad = roll_.toDouble()/100.0/180.0*3.1415926 - rollAverage_rad;
                roll_rad = caliAngle(roll_rad);

                QString pitch_ = strList.at(21);
                double pitch_rad = pitch_.toDouble()/100.0/180.0*3.1415926 - pitchAverage_rad;
                pitch_rad = caliAngle(pitch_rad);

                double ayCali_g = ay_g - sin(roll_rad) * cos(pitch_rad);//

                curSpeed_m_s = curSpeed_m_s + 0.5*(ayCali_g + ayCaliLast_g)*9.8*dt;
                ayCaliLast_g = ayCali_g;


            //选择速度
                double speed_cali_m_s = 0;
                //speed_cali_m_s = curSpeed_m_s;                                     //使用imu积分速度
                speed_cali_m_s = curSpeedRaw_r_s*2*3.1415926*0.03 - fabs(wz_rad_s)*0.06;  //编码器速度的基础上，采用车辆运动学计算速度

                speed_cali_m_s = speed_cali_m_s<0 ? 0 : speed_cali_m_s;

            //与x轴正向的夹角定义为偏航角
                int i = -1;//curDirection ? 1 : -1;
                double poseX_raw_m = poseX_m + i*speed_cali_m_s*dt*cos(poseYawRaw_rad);
                double poseY_raw_m = poseY_m - i*speed_cali_m_s*dt*sin(poseYawRaw_rad);

//车辆出界
            if((poseX_raw_m < 0) || (poseX_raw_m > 6.1) || (poseY_raw_m < 0) || (poseY_raw_m > 5.1))
            {
                if(debugPose)
                    qWarning()<<"Calculated position out of range.";			//统一修正debug格式		I	D	W	E

                //return;
            }

//位姿输出
            poseYaw_rad = poseYawRaw_rad;
            poseRoll_rad = poseRollRaw_rad;
            poseX_m = poseX_raw_m;
            poseY_m = poseY_raw_m;

            if(debugPose)
                qDebug()<<timeStamp<<dt<<i
                        <<"\tx(m)_"<<poseX_m<<"\ty(m)_"<<poseY_m<<"\tposeYaw(d)_"<<poseYaw_rad/3.14159*180.0
                        <<"\tv_odo_"<<curSpeedRaw_r_s*2*3.1415926*0.03
                        <<"\tv_kin_"<<(curSpeedRaw_r_s*2*3.1415926*0.03 - fabs(wz_rad_s)*0.06)
                        <<"\tv_imu_"<<curSpeed_m_s
                        <<"\t"<<ayCali_g<<0.5*(ayCali_g + ayCaliLast_g)*9.8*dt;
        }
    }
}

/* Function: Pose::on_readyread_NFC
 * 接收NFC数据
 */
void Pose::on_readyread_NFC()
{
    //从接收缓冲区中读取数据
    QByteArray buffer = serialNFC->readAll();//read(120);//readline();

    //查找map，确定NFC定位数据
    Position pos = staticInfo.getNFCInfoS()->value(buffer.toHex());
    if(0 == pos.id)
    {
        if(debugNFC)
            qWarning()<<"can not get the position in card "<<buffer.toHex();

        return;
    }

    if(debugNFC)
        qDebug()<<"Got the position in card "<<pos.id<<" : "<<buffer.toHex();

    poseXnfc_m = pos.x/1000.0;
    poseYnfc_m = pos.y/1000.0;
    IDnfc = pos.id;

    poseX_m = poseXnfc_m;
    poseY_m = poseYnfc_m;
}

/* Function: Pose::on_curSpeedDir
 * 接收motion线程传来的实时速度及其转向
 */
void Pose::on_curSpeedDir(double _curSpeed_r_s, bool _curDirection)
{
    if(_curSpeed_r_s<0.001)
        curSpeed_m_s=0;

    curSpeedRaw_r_s = _curSpeed_r_s;
    curDirection = _curDirection;
}

/* Function: Pose::enableSerial
 * 使能串口
 */
bool Pose::enableSerial(QSerialPort* _serial,
                            QString _portName, int _baudRate,
                            int _dataBits, int _parity, int _StopBits)
{
    //设置串口名
    _serial->setPortName(_portName);
    //设置波特率
    _serial->setBaudRate(_baudRate);
    //设置数据位数
    switch(_dataBits)
    {
        case 8: _serial->setDataBits(QSerialPort::Data8); break;
        default: break;
    }
    //设置奇偶校验
    switch(_parity)
    {
        case 0: _serial->setParity(QSerialPort::NoParity); break;
        default: break;
    }
    //设置停止位
    switch(_StopBits)
    {
        case 1: _serial->setStopBits(QSerialPort::OneStop); break;
        case 2: _serial->setStopBits(QSerialPort::TwoStop); break;
        default: break;
    }
    //设置流控制
    _serial->setFlowControl(QSerialPort::NoFlowControl);
    //打开串口
    if(!_serial->open(QIODevice::ReadWrite))
    {
        qWarning()<<"open failed: "<<_portName;
        return false;
    }

    return true;
}

/* Function: Pose::disableSerial
 * 关闭串口
 */
void Pose::disableSerial(QSerialPort* _serial)
{
    if(_serial->isOpen())
        _serial->close();
}

/* Function: Pose::imuSetting
 * 配置IMU
 */
void Pose::imuSetting(unsigned char *commond,int argc)
{
    QByteArray byte;
    byte.resize(argc);
    for(int i=0;i<argc;i++)
    {
        byte[i]=commond[i];
    }
    serialIMU->write(byte);
}

/* Function: Pose::caliAngle
 * 统一角度基准，逆时针0～360
 */
double Pose::caliAngle(double raw_angle_rad)
{
    if(raw_angle_rad<0)
        return (raw_angle_rad + 2*3.14159);
    else if(raw_angle_rad>2*3.1415926)
        return (raw_angle_rad - 2*3.1415926);
    else
        return raw_angle_rad;
}

/* Function: Pose::sendCurPose2VehicleConsole
 * 定时给主线程发送当前计算的车辆位置
 */
void Pose::sendCurPose2VehicleConsole()
{
    emit curPose(poseX_m,poseY_m,poseYaw_rad);
}

/* Function: Pose::on_calibration
 * imu标定，单次触发
 */
void Pose::on_calibration()
{
    isCalibration = false;
}

