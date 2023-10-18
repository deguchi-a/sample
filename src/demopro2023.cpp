#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <map>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <random>
#include <ctime>
//
//制御周期
int HZ=1000;
// オドメトリから得られる現在の位置
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度
// 初期速度
double v0 = 0.0;
// 初期角速度
double w0 = 0.0;

// field size[m]
int field_x=2;
int field_y=2;

std::map<std::string, std::string> params_;
// オドメトリのコールバック
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
}

// 円運動用の中心座標とカウンタ
double center_x=0.0;
double center_y=0.0;
double tangency_x=0.0;
double tangency_y=0.0;
double r=0.0;
int flag[4]={0,0,0,0};

double x = 0.0;
double y = 0.0;
double theta = M_PI / 4;
int count=0;    
int N=0;
int M=0;
int spin=0;


void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void follow_line(double x, double y, double th)
{
    // 現在のロボットのroll, pitch, yawを計算
    geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

    // パラメータファイルに入っている係数
    double k_eta = std::stod(params_["L_K1"]) / 100;
    double k_phai = std::stod(params_["L_K2"]) / 100;
    double k_w = std::stod(params_["L_K3"]) / 100;

    // ロボットの最大速度、最大角速度
    double v_max = std::stod(params_["MAX_VEL"]);
    double w_max = std::stod(params_["MAX_W"]);

    // // //thetaが-M_PIからM_PIに収まるように処理
    // while (th <= -M_PI/2 || M_PI/2 <= th)
    // {
    //     if (th <= -M_PI/2)
    //         th = th +  M_PI;
    //     else
    //         th = th -  M_PI;
    // }
    // std::cout<<"theta:"<<th<<std::endl;
    // ロボットと直線の距離(実際には一定以上の距離でクリップ)
    double eta = 0;
    if (th == M_PI / 2.0)
        eta = -(robot_x - x);
    else if (th == -M_PI / 2.0)
        eta = robot_x - x;
    else if (abs(th) < M_PI / 2.0)
        eta = (-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    else
        eta = -(-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    if (eta > std::stod(params_["L_DIST"]))
        eta = std::stod(params_["L_DIST"]);
    else if (eta < -std::stod(params_["L_DIST"]))
        eta = -std::stod(params_["L_DIST"]);

    // 直線に対するロボットの向き(-M_PIからM_PIに収まるように処理)
    double phai = yaw - th;
    while (phai <= -M_PI || M_PI <= phai)
    {
        if (phai <= -M_PI)
            phai = phai + 2 * M_PI;
        else
            phai = phai - 2 * M_PI;
    }

    // 目標となるロボットの角速度と現在の角速度の差
    double w_diff = w0;

    // 角速度
    double w = w0 + (-k_eta * eta - k_phai * phai - k_w* w_diff);
    if (w > w_max)
        w = w_max;
    else if (w < -w_max)
        w = -w_max;

    // 並進速度
    double v = v0;
    if (v > v_max)
        v = v_max;
    else if (v < -v_max)
        v = -v_max;

    twist.linear.x = v;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = w;

    // 現在の角速度を次の時間の計算に使用
    w0 = twist.angular.z;

    // デバック用のプリント
    // std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
    // std::cout << "v: " << twist.linear.x << "   w: " << twist.angular.z << std::endl;
    // std::cout << "(x,y) = (" << robot_x << "," << robot_y << ")" << std::endl;
    // std::cout << "------------------------------" << std::endl;

    
}

class Movement
{
    public:
        void justRotation(int N){
            // std::cout <<"1 start"<< std::endl;
            double w_max = std::stod(params_["MAX_W"]);
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = w_max/2;
            spin+=1;
            if(spin==N*HZ){flag[0]=1;}
        }
        void gostraight(double x, double y, double t)
        {
            //calculate line's angular
            double theta = atan2(y - robot_y, x - robot_x);
            if(flag[1]==0){
                v0=sqrt(pow(x-robot_x,2)+pow(y-robot_y,2))/t;
                flag[1]=1;
            }
            follow_line(x,y,theta);
        }
        void movearc(double x, double y, double t,int rl){
            geometry_quat_to_rpy(roll, pitch, yaw, robot_r);
            if(flag[2]==0){
                center_x=(x+robot_x)/2;
                center_y=(y+robot_y)/2;
                r=sqrt(pow(center_x-robot_x,2)+pow(center_y-robot_y,2));
                v0=M_PI*r/t;
                // if(atan2(y-robot_y,x-robot_x)<-M_PI/2||M_PI/2<atan2(y-robot_y,x-robot_x)){v0=-v0;}
                flag[2]=1;
            }
            double dist=sqrt(pow(center_x-robot_x,2)+pow(center_y-robot_y,2));
            if(dist-r>=0){
                tangency_x=(center_x*(dist-r)+robot_x*r)/dist;
                tangency_y=(center_y*(dist-r)+robot_y*r)/dist;
            }
            else{
                tangency_x=(center_x*(r-dist)+robot_x*r)/dist;
                tangency_y=(center_y*(r-dist)+robot_y*r)/dist;
            }
            theta=atan2(center_y - robot_y, center_x - robot_x)-M_PI/2;
            if(rl==1){
                if(theta<0){theta=theta+M_PI;}
                else if(theta>0){theta=theta-M_PI;}
            }
            follow_line(tangency_x,tangency_y,theta);
        }
        void weaving(double x, double y, double t){
            // std::cout <<"4 start"<< std::endl;
            // ロボットの最大速度、最大角速度
            double v_max = std::stod(params_["MAX_VEL"]);
            double w_max = std::stod(params_["MAX_W"]);
            if (flag[3]==0){
                center_x=(x+robot_x)/4;
                center_y=(y+robot_y)/4;
                r=sqrt(pow(center_x-robot_x,2)+pow(center_y-robot_y,2));
                v0=2*M_PI*r/t;
                //if(atan2(y-robot_y,x-robot_x)<-M_PI/2||M_PI/2<atan2(y-robot_y,x-robot_x)){v0=-v0;}
                flag[3]=1;
            }
            if (flag[3]==1){
                if(pow((x-2*center_x)/3-robot_x,2)+pow((y-2*center_y)/3-robot_y,2)<0.001){
                    flag[3]==2;
                }
                theta=atan2(center_y - robot_y, center_x - robot_x)-M_PI/2;
                if(theta<0){theta=theta+M_PI;}
                else if(theta>0){theta=theta-M_PI;}
            }
            if (flag[3]==2){
                center_x=(x+robot_x)/2;
                center_y=(y+robot_y)/2;
                r=sqrt(pow(center_x-robot_x,2)+pow(center_y-robot_y,2));
                v0=2*M_PI*r/t;
                flag[3]==3;
            }
            if(flag[3]==3){
                theta=atan2(center_y - robot_y, center_x - robot_x)-M_PI/2;
                if(theta<0){theta=theta+M_PI;}
                else if(theta>0){theta=theta-M_PI;}
            }
            double dist=sqrt(pow(center_x-robot_x,2)+pow(center_y-robot_y,2));
            double tangency_x=(center_x*(dist-r)+robot_x*r)/dist;
            double tangency_y=(center_y*(dist-r)+robot_y*r)/dist;
            // theta=atan2(center_y - robot_y, center_x - robot_x)-M_PI/2;
            follow_line(tangency_x,tangency_y,theta);
        }
};

void get_param(void){
	std::ifstream read_param_file;
	read_param_file.open(params_["param_file"], std::ios::in);

	std::string buffer_read;

	while(std::getline(read_param_file, buffer_read))
	{
		std::string row = "";
		std::string name_param = "";

        for (char &x : buffer_read)
        {
            if (x != ' ') {
                //add element to vector row
            std::string row_alpha{x};
			row = row + row_alpha;
			} else if(row != "" && name_param == ""){
				if ((row == "L_K1") | (row == "L_K2") | (row == "L_K3") | (row == "L_DIST") | (row == "MAX_VEL") | (row == "MAX_W")){
					name_param = row;
					row = "";
				}else{
					break;
				}
			} else if(row != ""){
				params_[name_param] = row;
				row = "";
				name_param = "";
			}

        }
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demopro2023");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odom_callback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 100);

	pnh.param<std::string>("param_file", params_["param_file"], "/home/deguchi-a/researches/programs/platform/yp-robot-params/robot-params/beego.param");

	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;

	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;

	std::cout << "v: " << twist.linear.x << "   w: " << twist.angular.z << std::endl;

	ros::Rate loop_rate(HZ);

	get_param();

	// 速度
	v0 = std::stod(params_["MAX_VEL"])*3/4;
	// 現在の角速度
	w0 = twist.angular.z;
	
    Movement a; 
    ros::Time t_start = ros::Time::now();
    srand(time(NULL));

	while (ros::ok())
	{
		ros::spinOnce();
        if(count==0){
            //set the target position randomlly(x=[-2.0~2.0],y=[-2.0~2.0])
            x=pow(-1,(rand()%2))*double(rand()%(field_x*10))/10;
            y=pow(-1,(rand()%2))*double(rand()%(field_y*10))/10;
            //how long the robot do the movement (5~8second)
            N=rand()%3+5;
            //determine the movement(1~4)
            M=rand()%4+1;
            // M=1;
            std::cout<<"------------------------"<<std::endl;
            std::cout << "targetx: " << x << ", targety: " << y << std::endl;
            std::cout << "Loop time: " << N << "   COMMAND: " << M << std::endl;
            std::cout<<"------------------------"<<std::endl;
            count=count+1;
            for(int i=0;i<4;i++){
                flag[i]=0;
            }
        }
        // else if(count==N){
        else if(pow(x-robot_x,2)+pow(y-robot_y,2)<0.05&&count!=0||flag[0]==1){
            count=0;
            // std::cout<<"stop"<<std::endl;
            // twist.linear.x=0.0;
            // twist.angular.z=0.0;
            // M=5;
        }
        else{
            count=count+1;
            if(count%HZ==0){
                std::cout << "count:" << count << std::endl;
                std::cout << "robot_x: " << robot_x << ", y: " << robot_y <<" theta:"<<theta<< std::endl;
                std::cout << "v: " << twist.linear.x << " w: " << twist.angular.z <<" turn:"<<spin<< std::endl;
            }
        }

        switch (M)
        {
        case 1:
            a.justRotation(N);
            break;
        
        case 2:
            a.gostraight(x,y,N);
            break;
        case 3:
            a.movearc(x,y,N,0);
            break;
        case 4:
            a.movearc(x,y,N,1);
            break;
        }
        //set the time limit for the game
        if((ros::Time::now() - t_start).toSec()>60){
            twist.linear.x=0.0;
            twist.angular.z=0.0;
        }
        twist_pub.publish(twist);
        loop_rate.sleep();
	}

	return 0;
}
