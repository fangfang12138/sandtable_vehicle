#include "vehiclesensor_camera.h"

//-------------------------------------------------------------------------------深度相机
Camera::Camera(QObject *parent) : QObject(parent)
{

}

/* Function: Camera::initCamera
 * 初始化相机对象
 */
void Camera::initCamera(QMainWindow *_vehicle)
{
    vehicleConsole = _vehicle;

    connect(vehicleConsole,SIGNAL(toStartSensor()),this,SLOT(on_toStartSensor()),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(toStopSensor()),this,SLOT(on_toStopSensor()),Qt::BlockingQueuedConnection);
}

/* Function: Camera::on_toStartSensor
 *
 */
void Camera::on_toStartSensor()
{
    qInfo()<<"connect & start Camera...";
    //realsenseTest();

    //calibrate();
    //trans(getImage());
}

/* Function: Camera::on_toStopSensor
 *
 */
void Camera::on_toStopSensor()
{
    qInfo()<<"stop & disconnect Camera...";
}

/* Function: Camera::realsenseTest
 * D435基本用法
 */
void Camera::realsenseTest()
{
//判断当前设备连接情况
    rs2::context ctx;
    auto list = ctx.query_devices();
    if(list.size() == 0)
        return;
    rs2::device dev = list.front();

//配置
    rs2::config cfg;
    int width = 640;
    int height = 480;
    int fps = 30;
    cfg.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_BGR8,fps);

//创建通信管道            this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    //配置并启动pipe
    p.start(cfg);

//获取数据                Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();
    //提取彩色图像
    rs2::frame color_frame = frames.get_color_frame();
    int w = color_frame.as<rs2::video_frame>().get_width();
    int h = color_frame.as<rs2::video_frame>().get_height();
    //转换为Mat格式
    cv::Mat color(cv::Size(w,h),CV_8UC3,(void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
    //......
    imwrite("image.jpg",color);

    p.stop();
}

/* Function: Camera::getImage
 * 获取原始图像
 */
Mat Camera::getImage()
{
    Mat img;
//判断当前设备连接情况
    rs2::context ctx;
    auto list = ctx.query_devices();
    if(list.size() == 0)
        return img;
    rs2::device dev = list.front();

//配置
    rs2::config cfg;
    int width = 640;
    int height = 480;
    int fps = 30;
    cfg.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_BGR8,fps);

//创建通信管道            this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    //配置并启动pipe
    p.start(cfg);

//获取数据                Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();
    //提取彩色图像
    rs2::frame color_frame = frames.get_color_frame();
    int w = color_frame.as<rs2::video_frame>().get_width();
    int h = color_frame.as<rs2::video_frame>().get_height();
    //转换为Mat格式
    cv::Mat color(cv::Size(w,h),CV_8UC3,(void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
    //......
    imwrite("image_read.jpg",color);

    p.stop();

    return color;
}

/* Function: Camera::trans
 * 车道线识别函数
 */
void Camera::trans(Mat oriImage)
{
    Mat image;
    image = oriImage;//imread("image_read.jpg");
    if(image.empty())
    {
        std::cout <<"E：reading image error"<<std::endl;
    }

//反透视，计算鸟瞰图
    Mat image_perspective;
    warpPerspective(image, image_perspective, cali_matrix, Size(960,960), cv::INTER_LINEAR);//image.size()       Size(1280,960)
    imwrite("image_perspective.jpg",image_perspective);

//计算灰度图
    Mat image_gray;
    cvtColor(image_perspective, image_gray, COLOR_BGR2GRAY);//用于图形在不同色彩空间进行转换 6==CV_BGR2GRAY

//高斯平滑：使用高斯滤波平滑图像，减少图像中噪声，一般情况下使用5×5的高斯滤波器
    Mat image_gau;
    GaussianBlur(image_gray, image_gau, Size(5,5), 0, 0); //input, output, gaussian kernel
    imwrite("image_gau.jpg",image_gau);
    /*
        void GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER_DEFAULT);
            ksize，高斯内核的大小，其中ksize.width和ksize.height可以不同，但必须为正数和奇数
            sigmaX，表示高斯核函数在X方向的的标准偏差
            sigmaY，表示高斯核函数在Y方向的的标准偏差，若sigmaY为零，将设为sigmaX，如果sigmaX和sigmaY都是0，那么就由ksize.width和ksize.height计算出来。

            sigma越大，模糊/平滑效果越明显

            https://blog.csdn.net/godadream/article/details/81568844
    */

//canny边缘检测
    Mat image_canny;
    Canny(image_gau, image_canny, 90, 180, 3); //边缘检测，阈值1,阈值2,sobel算子的孔径参数
    imwrite("image_canny.jpg",image_canny);
    /*
        两个阈值区分强边缘和弱边缘，较大阈值与较小阈值的比值在2:1到3:1之间，较高的阈值会降低噪声信息对图像提取边缘结果的影响，但是同时也会减少结果中的边缘信息。
            低于阈值1的像素点认为不是边缘
            高于阈值2的像素点认为是
            阈值1-2之间的像素点，若与高于2的相邻认为是
            https://blog.csdn.net/duwangthefirst/article/details/79971212
            高斯模糊在边缘纹理较多的区域能减少边缘检测的结果，但是对纹理较少的区域影响较小。
    */

//提取感兴趣区域：梯形区域******************************************************************************************************************************标定	1/2点
    Mat dstImg;
    Mat mask = Mat::zeros(image_canny.size(), CV_8UC1);//CV_8UC1 CVmat对应的类型，1表示通道数，表示灰度图

    Point PointArray[4];//顶点信息
    PointArray[0] = Point(0, mask.rows);
    PointArray[1] = Point(100,250);
    PointArray[2] = Point(640,250);
    PointArray[3] = Point(640, mask.rows);

    fillConvexPoly(mask,PointArray,4,Scalar(255));//画出多边形：原图，顶点，边数，线条颜色
    bitwise_and(mask,image_canny,dstImg);

    imwrite("image_interested.jpg",dstImg);
    /*
        图像掩膜
            用选定的图像、图形或物体，对处理的图像（全部或局部）进行遮挡，来控制图像处理的区域或处理过程。
            数字图像处理中,掩模为二维矩阵数组,有时也用多值图像，图像掩模主要用于：
            ①提取感兴趣区,用预先制作的感兴趣区掩模与待处理图像相乘,得到感兴趣区图像,感兴趣区内图像值保持不变,而区外图像值都为0。
            ②屏蔽作用,用掩模对图像上某些区域作屏蔽,使其不参加处理或不参加处理参数的计算,或仅对屏蔽区作处理或统计。
            ③结构特征提取,用相似性变量或图像匹配方法检测和提取图像中与掩模相似的结构特征。
            ④特殊形状图像的制作。
            https://blog.csdn.net/u011028345/article/details/77278467

        fillConvexPoly：https://blog.csdn.net/mao_hui_fei/article/details/100098161

        void bitwise_and(InputArray src1, InputArray src2,OutputArray dst, InputArray mask=noArray());//dst = src1 & src2，像素按位与运算
    */

//霍夫变换：将像素点连成线，一般用累计概率霍夫变换(Progressive Probabilistic Hough Transform ，PPHT)，霍夫变换将笛卡尔坐标变换到极坐标下****************标定	min_line_len，max_line_gap
    std::vector<Vec4i> lines; //包含4个int类的结构体
    int rho = 1;
    double theta = CV_PI/180;
    int threshold = 30;
    int min_line_len = 100;
    int max_line_gap = 100;
    HoughLinesP(dstImg,lines,rho,theta,threshold,min_line_len,max_line_gap);

    /*
        简述原理：霍夫变换将笛卡尔坐标变换到极坐标下，对应极坐标参数空间中，交于一点的曲线的数量超过了阈值threshold, 那么可以认为这个交点所代表的参数表示的直线，在原图像中为一条直线

        dstImg			输入图像（单通道二进制）
        lines			输出线的两个端点（x1,y1,x2,y2）
        rho				直线搜索的步长（单位为像素）
        theta			直线搜索时的角度步长（单位为弧度）
        threshold		多少个点交在一起才认为是一条直线
        min_linelen		最低线段长度（默认为0）
        max_line_gap	两条直线并列多远的时候认为是两条

        什么是霍夫变换 			https://blog.csdn.net/sudohello/article/details/51335237
        OpenCV函数HoughLinesP 	https://blog.csdn.net/vichael_chan/article/details/100669205
    */

//车道线拟合
    //左上为原点，x轴正向朝右，y轴正向朝下
    Mat image_draw = Mat::zeros(image_canny.size(),CV_8UC3);//RGB3
    std::vector<int> right_x, right_y, left_x, left_y;							//车道线的起止坐标
    double slope_right_sum, slope_right_mean, slope_left_sum, slope_left_mean;	//斜率的累计和平均
    double b_right_sum, b_right_mean, b_left_sum, b_left_mean;					//截距的累计和平均
    std::vector<double> slope_right, slope_left,b_right, b_left;				//各检测直线的斜率和截距

    //计算各检测直线的斜率和截距
    for(size_t i = 0; i < lines.size(); i++)
    {
        Vec4i L;
        double slope,b;
        L = lines[i];
        slope = (L[3]-L[1])*1.0/(L[2]-L[0]);
        b = L[1]-L[0]*slope;

        //根据斜率判断是左车道线还是右车道线
        if (slope >=0.2)
        {
            slope_right.push_back(slope);
            b_right.push_back(b);
        }
        else
        {
            slope_left.push_back(slope);
            b_left.push_back(b);
        }
    }

    //计算各检测直线的平均斜率和截距，accumulate实现vector内的值累加，输出格式与最后一个参数的数据格式一致
    slope_right_sum = accumulate(slope_right.begin(), slope_right.end(),0.0);
    b_right_sum = accumulate(b_right.begin(), b_right.end(),0.0);

    slope_left_sum = accumulate(slope_left.begin(),slope_left.end(),0.0);
    b_left_sum = accumulate(b_left.begin(),b_left.end(),0.0);

    slope_right_mean = slope_right_sum/slope_right.size();
    slope_left_mean = slope_left_sum/slope_left.size();

    b_right_mean = b_right_sum/b_right.size();
    b_left_mean = b_left_sum/b_left.size();

    //画出来，x位置在有效范围内任意给出，以此计算对应y
    double x1r = 550;
    double x2r = 850;

    double x1l = 120;
    double x2l = 425;

    int y1r = slope_right_mean * x1r + b_right_mean;
    int y2r = slope_right_mean * x2r + b_right_mean;
    int y1l = slope_left_mean * x1l + b_left_mean;
    int y2l = slope_left_mean * x2l + b_left_mean;

    line(image_draw, Point(x1r,y1r),Point(x2r,y2r),Scalar(0,0,255),5,LINE_AA);
    line(image_draw, Point(x1l,y1l),Point(x2l,y2l),Scalar(0,0,255),5,LINE_AA);

    imwrite("image_draw.jpg",image_draw);
    /*
        void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

            img		要绘制线段的图像
            pt1		线段的起点
            pt2		线段的终点
            color		线段的颜色，通过一个Scalar对象定义
            thickness	线条的宽度
            lineType	线段的类型，可以取值8，4，CV_AA，分别代表8邻接连接线，4邻接连接线和反锯齿连接线。默认值为8邻接。为了获得更好地效果可以选用CV_AA(采用了高斯滤波)。
            shif		坐标点小数点位数

    */

//在原图像上绘制车道线
    Mat image_mix = Mat::zeros(image_canny.size(),CV_8UC3);
    addWeighted(image_draw,1,image_perspective,1,0.0,image_mix);

    imwrite("image_mix.jpg",image_mix);
    /*
        void cvAddWeighted( const CvArr* src1, double alpha,const CvArr* src2, double beta,double gamma, CvArr* dst );
            src1	第一个原数组.
            alpha	第一个数组元素权重
            src2	第二个原数组
            beta	第二个数组元素权重
            gamma	图1与图2作和后添加的数值。不要太大，不然图片一片白。总和等于255以上就是纯白色了。
            dst		输出图片
    */

//输出
    return;
}

/* Function: Camera::calibrate
 * 车道线识别预处理：校正
 */
void Camera::calibrate()
{
    Mat image_calibration = imread("../image_calibration.jpg");
    Mat image;
    image = image_calibration;
    if(image.empty())
    {
        std::cout <<"E：reading image error"<<std::endl;
    }

//计算灰度图
    Mat image_gray;
    cvtColor(image, image_gray, COLOR_BGR2GRAY);//用于图形在不同色彩空间进行转换 6==CV_BGR2GRAY

//高斯平滑：使用高斯滤波平滑图像，减少图像中噪声，一般情况下使用5×5的高斯滤波器
    Mat image_gau;
    GaussianBlur(image_gray, image_gau, Size(5,5), 0, 0); //input, output, gaussian kernel
    imwrite("cali_image_gau.jpg",image_gau);
    /*
        void GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER_DEFAULT);
            ksize，高斯内核的大小，其中ksize.width和ksize.height可以不同，但必须为正数和奇数
            sigmaX，表示高斯核函数在X方向的的标准偏差
            sigmaY，表示高斯核函数在Y方向的的标准偏差，若sigmaY为零，将设为sigmaX，如果sigmaX和sigmaY都是0，那么就由ksize.width和ksize.height计算出来。

            sigma越大，模糊/平滑效果越明显

            https://blog.csdn.net/godadream/article/details/81568844
    */

//canny边缘检测**************************************************************************************************************************标定两阈值
    Mat image_canny;
    Canny(image_gau, image_canny, 90, 180, 3); //边缘检测，阈值1,阈值2,sobel算子的孔径参数
    imwrite("cali_image_canny.jpg",image_canny);
    /*
        两个阈值区分强边缘和弱边缘，较大阈值与较小阈值的比值在2:1到3:1之间，较高的阈值会降低噪声信息对图像提取边缘结果的影响，但是同时也会减少结果中的边缘信息。
            低于阈值1的像素点认为不是边缘
            高于阈值2的像素点认为是
            阈值1-2之间的像素点，若与高于2的相邻认为是
            https://blog.csdn.net/duwangthefirst/article/details/79971212
            高斯模糊在边缘纹理较多的区域能减少边缘检测的结果，但是对纹理较少的区域影响较小。
    */

//提取感兴趣区域：梯形区域******************************************************************************************************************************标定	1/2点
    Mat dstImg;
    Mat mask = Mat::zeros(image_canny.size(), CV_8UC1);//CV_8UC1 CVmat对应的类型，1表示通道数，表示灰度图

    Point PointArray[4];//顶点信息
    PointArray[0] = Point(0, mask.rows);
    PointArray[1] = Point(100,250);
    PointArray[2] = Point(640,250);
    PointArray[3] = Point(mask.cols, mask.rows);

    fillConvexPoly(mask,PointArray,4,Scalar(255));//画出多边形：原图，顶点，边数，线条颜色
    bitwise_and(mask,image_canny,dstImg);

    imwrite("cali_image_interested.jpg",dstImg);
    /*
        图像掩膜
            用选定的图像、图形或物体，对处理的图像（全部或局部）进行遮挡，来控制图像处理的区域或处理过程。
            数字图像处理中,掩模为二维矩阵数组,有时也用多值图像，图像掩模主要用于：
            ①提取感兴趣区,用预先制作的感兴趣区掩模与待处理图像相乘,得到感兴趣区图像,感兴趣区内图像值保持不变,而区外图像值都为0。
            ②屏蔽作用,用掩模对图像上某些区域作屏蔽,使其不参加处理或不参加处理参数的计算,或仅对屏蔽区作处理或统计。
            ③结构特征提取,用相似性变量或图像匹配方法检测和提取图像中与掩模相似的结构特征。
            ④特殊形状图像的制作。
            https://blog.csdn.net/u011028345/article/details/77278467

        fillConvexPoly：https://blog.csdn.net/mao_hui_fei/article/details/100098161

        void bitwise_and(InputArray src1, InputArray src2,OutputArray dst, InputArray mask=noArray());//dst = src1 & src2，像素按位与运算
    */

//霍夫变换：将像素点连成线，一般用累计概率霍夫变换(Progressive Probabilistic Hough Transform ，PPHT)，霍夫变换将笛卡尔坐标变换到极坐标下****************标定	min_line_len，max_line_gap
    std::vector<Vec4i> lines; //包含4个int类的结构体
    int rho = 1;
    double theta = CV_PI/180;
    int threshold = 30;
    int min_line_len = 100;
    int max_line_gap = 100;
    HoughLinesP(dstImg,lines,rho,theta,threshold,min_line_len,max_line_gap);

    /*
        简述原理：霍夫变换将笛卡尔坐标变换到极坐标下，对应极坐标参数空间中，交于一点的曲线的数量超过了阈值threshold, 那么可以认为这个交点所代表的参数表示的直线，在原图像中为一条直线

        dstImg			输入图像（单通道二进制）
        lines			输出线的两个端点（x1,y1,x2,y2）
        rho				直线搜索的步长（单位为像素）
        theta			直线搜索时的角度步长（单位为弧度）
        threshold		多少个点交在一起才认为是一条直线
        min_linelen		最低线段长度（默认为0）
        max_line_gap	两条直线并列多远的时候认为是两条

        什么是霍夫变换 			https://blog.csdn.net/sudohello/article/details/51335237
        OpenCV函数HoughLinesP 	https://blog.csdn.net/vichael_chan/article/details/100669205
    */

//车道线拟合
    //左上为原点，x轴正向朝右，y轴正向朝下
    Mat image_draw = Mat::zeros(image_canny.size(),CV_8UC3);//RGB3
    std::vector<int> right_x, right_y, left_x, left_y;							//车道线的起止坐标
    double slope_right_sum, slope_right_mean, slope_left_sum, slope_left_mean;	//斜率的累计和平均
    double b_right_sum, b_right_mean, b_left_sum, b_left_mean;					//截距的累计和平均
    std::vector<double> slope_right, slope_left,b_right, b_left;				//各检测直线的斜率和截距

    //计算各检测直线的斜率和截距
    for(size_t i = 0; i < lines.size(); i++)
    {
        Vec4i L;
        double slope,b;
        L = lines[i];
        slope = (L[3]-L[1])*1.0/(L[2]-L[0]);
        b = L[1]-L[0]*slope;

        //根据斜率判断是左车道线还是右车道线
        if (slope >=0.2)
        {
            slope_right.push_back(slope);
            b_right.push_back(b);
        }
        else
        {
            slope_left.push_back(slope);
            b_left.push_back(b);
        }
    }

    //计算各检测直线的平均斜率和截距，accumulate实现vector内的值累加，输出格式与最后一个参数的数据格式一致
    slope_right_sum = accumulate(slope_right.begin(), slope_right.end(),0.0);
    b_right_sum = accumulate(b_right.begin(), b_right.end(),0.0);

    slope_left_sum = accumulate(slope_left.begin(),slope_left.end(),0.0);
    b_left_sum = accumulate(b_left.begin(),b_left.end(),0.0);

    slope_right_mean = slope_right_sum/slope_right.size();
    slope_left_mean = slope_left_sum/slope_left.size();

    b_right_mean = b_right_sum/b_right.size();
    b_left_mean = b_left_sum/b_left.size();

    //计算图像中车道线坐标
    double y1r = 480;
    double y2r = 300;
    double y1l = 480;
    double y2l = 300;

    int x1r = (y1r - b_right_mean)/slope_right_mean;
    int x2r = (y2r - b_right_mean)/slope_right_mean;
    int x1l = (y1l - b_left_mean)/slope_left_mean;
    int x2l = (y2l - b_left_mean)/slope_left_mean;

    //qDebug()<<image.size().height<<image.size().width;
    //qDebug()<<"1r:("<<x1r<<y1r<<")"<<"("<<x2r<<y2r<<")";
    //qDebug()<<"1l:("<<x1l<<y1l<<")"<<"("<<x2l<<y2l<<")";

    line(image_draw, Point(x1r,y1r),Point(x2r,y2r),Scalar(0,0,255),5,LINE_AA);
    line(image_draw, Point(x1l,y1l),Point(x2l,y2l),Scalar(0,0,255),5,LINE_AA);

    imwrite("cali_image_draw.jpg",image_draw);
    /*
        void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

            img		要绘制线段的图像
            pt1		线段的起点
            pt2		线段的终点
            color		线段的颜色，通过一个Scalar对象定义
            thickness	线条的宽度
            lineType	线段的类型，可以取值8，4，CV_AA，分别代表8邻接连接线，4邻接连接线和反锯齿连接线。默认值为8邻接。为了获得更好地效果可以选用CV_AA(采用了高斯滤波)。
            shif		坐标点小数点位数

    */

//在原图像上绘制车道线
    Mat image_mix = Mat::zeros(image_canny.size(),CV_8UC3);
    addWeighted(image_draw,1,image,1,0.0,image_mix);

    imwrite("cali_image_mix.jpg",image_mix);
    /*
        void cvAddWeighted( const CvArr* src1, double alpha,const CvArr* src2, double beta,double gamma, CvArr* dst );
            src1	第一个原数组.
            alpha	第一个数组元素权重
            src2	第二个原数组
            beta	第二个数组元素权重
            gamma	图1与图2作和后添加的数值。不要太大，不然图片一片白。总和等于255以上就是纯白色了。
            dst		输出图片
    */

//计算透视矩阵
    //图像中车道线坐标
    Point2f src_points[] = {
        Point2f(x1r, y1r),
        Point2f(x2r, y2r),
        Point2f(x1l, y1l),
        Point2f(x2l, y2l) };
    //鸟瞰图车道线坐标
    Point2f dst_points[] = {
        Point2f(x1r, 900),
        Point2f(x1r, 50),
        Point2f(x1l, 900),
        Point2f(x1l, 50) };

    Mat M = getPerspectiveTransform(src_points, dst_points);

//透视矩阵反透视效果
    Mat image_perspective;
    warpPerspective(image, image_perspective, M, Size(960,960), cv::INTER_LINEAR);//image.size()       Size(1280,960)
    imwrite("cali_image_perspective.jpg",image_perspective);

//输出
    cali_matrix = M;
}


