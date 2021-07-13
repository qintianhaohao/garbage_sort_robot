#include "car_location_color.h"
std::string  target_direction;
/*
//rqt工具dynamic_reconfigure实现实时调参
void car_init::reconfigCB(wheeltec_arm_pick::paramsConfig &config, uint32_t level)
{
  ROS_INFO("x_p param : %f",config.x_p);
  ROS_INFO("x_d param : %f",config.x_d);
  ROS_INFO("y_p param : %f",config.y_p);
  ROS_INFO("y_d param : %f",config.y_d);
  ROS_INFO("color_location_x param : %f",config.color_location_x);
  ROS_INFO("color_location_y param : %f",config.color_location_y);
  x_p=config.x_p;
  x_i=config.x_i;
  x_d=config.x_d;
  y_p=config.y_p;
  y_i=config.y_i;
  y_d=config.y_d;
  color_location_x=config.color_location_x;
  color_location_y=config.color_location_y;
}
*/
//接收运动底盘的回调函数
void car_init::car_command_callback(const wheeltec_arm_pick::pick_and_put &msg)
{
  car_state=msg.car_state;
  target_angle=msg.angle;

}

//获取色块位置的回调函数，在函数中做pid计算，将色块定位到可夹取位置
void car_init::color_location_callback(const wheeltec_arm_pick::position &msg)
{
  static int count=0;
  distance_y = msg.angleX;
  distance_x = msg.angleY;
  target_liner_x=PID_control_x();
  target_liner_y=PID_control_y();
  if((abs_float(target_liner_x)<0.05)&&(abs_float(target_liner_y)<0.05)&&location_flag==0) count++;
  else count=0;
  if(count>100)  location_flag=1,count=0;//色块定位完成
  if(location_flag==0)  target_angular_z=0; //色块定位过程中只需要x和y轴的线速度
  if(location_flag==1) target_liner_x=0,target_liner_y=0; //色块定位完成后底盘不再定位，防止抖动
  cmd_vel_publish();//将底盘速度发布出去
}

//获取底盘位置的回调函数，利用odom话题来做底盘夹取到色块后旋转多少角度的计算
void car_init::car_pose_callback(const nav_msgs::Odometry &msg)
{
  static float last_target_angle=0,target_position_z=0;
  
  car_position_x=msg.pose.pose.position.x;//获取底盘的位置信息（编码器积分获得）
  car_position_y=msg.pose.pose.position.y;
  car_position_z=msg.pose.pose.position.z;

  if(last_target_angle!=target_angle) target_position_z=car_position_z+target_angle;//状态发生变化，获取需要转多少角度
  if(car_state==1)//夹取到色块后
  {     //给上0.1的允许误差范围
        if(target_position_z<=(car_position_z-0.1))  target_angular_z= -0.3,move_flag=1; //以0.3rad/s的速度旋转
   else if(target_position_z>=(car_position_z+0.1))  target_angular_z=  0.3,move_flag=1;
   else                     target_angular_z= 0,move_flag=2; //夹取色块后自转完成
   last_target_angle=target_angle;
   cmd_vel_publish();//将底盘速度发布出去
   }

  if(car_state==2)//放置完色块后
  {     //给上0.1的允许误差范围
        if(target_position_z<=(car_position_z-0.1))  target_angular_z= -0.3,move_flag=3; //以0.3rad/s的速度旋转
   else if(target_position_z>=(car_position_z+0.1))  target_angular_z=  0.3,move_flag=3;
   else                     target_angular_z= 0,move_flag=4; //放置色块后自转完成
   last_target_angle=target_angle; 
   cmd_vel_publish();//将底盘速度发布出去
   }
   last_target_angle=target_angle;
  }

//运动底盘速度话题发布函数
void car_init::cmd_vel_publish()
{
  geometry_msgs::Twist msg;
  msg.linear.x=target_liner_x;
  msg.linear.y=target_liner_y;
  msg.angular.z=target_angular_z;
  cmd_vel_pub.publish(msg);
}

//机械臂状态的话题发布函数
void car_init::arm_state_publish()
{
  static uint8_t last_move_flag=0,last_location_flag=0;
  std_msgs::String msg;
  msg.data="last_target_angle";
 
  if((location_flag==1)&&(move_flag<2))       msg.data="pick";//如果底盘对色块定位完成，那么发布机械臂夹取的命令
  else if((move_flag==2))                     msg.data="put"; //底盘自转前，如果是夹取了东西的，那么就执行机械臂放置命令
  else                                        msg.data="none";
  
  arm_state_pub.publish(msg); //将机械臂的命令发布出去

}
//底盘在x轴方向（前后）定位色块的PID计算
float car_init::PID_control_x()
{
    static float last_error=0,output=0;
    pid_x.error=distance_x-(color_location_x);
    output=x_d*last_error+x_p*pid_x.error;
    last_error=pid_x.error;
    return output;
    
}

//底盘在y轴方向（左右）定位色块的PID计算
float car_init::PID_control_y()
{
    static float last_error=0,output=0;
    pid_y.error=distance_y-(color_location_y);
    output=y_d*last_error+y_p*pid_y.error;
    last_error=pid_y.error;
    return output;

}
//取绝对值函数
float car_init::abs_float(float input)
{

if(input<0) return -input;
else  return input;
}

//循环执行的内容函数
void car_init::control()
{
  
  while(ros::ok())
  {
    arm_state_publish();  //发布机械臂的命令
    if(move_flag==4) car_state=0,location_flag=0,move_flag=0;//move_flag等于4表示已经完成了色块夹取，自转，放置，自转回原地的一个完整操作，标志位清零  
    ros::spinOnce();
  }
}

//构造函数
car_init::car_init()
{
    //参数服务器
    ros::NodeHandle param_nh("~"); //私有命名空间的节点句柄，用于参数服务器
    param_nh.param("x_p",x_p,0.0); //x轴方向的pid参数
    param_nh.param("x_d",x_d,0.0);
    param_nh.param("y_p",y_p,0.0); //y轴方向的pid参数
    param_nh.param("y_d",y_d,0.0);
    param_nh.param("color_location_x",color_location_x,0.0); //色块的x轴方位的目标位置
    param_nh.param("color_location_y",color_location_y,0.0); //色块的y轴方位的目标位置
    
    arm_state_pub=n.advertise<std_msgs::String>("arm_state",10);
    cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",10);
    car_command_sub=n.subscribe("car_command",10,&car_init::car_command_callback,this);
    color_location_sub=n.subscribe("object_tracker/current_position",10,&car_init::color_location_callback,this);
    car_pose_sub=n.subscribe("odom",10,&car_init::car_pose_callback,this);

    target_angle=0,car_state=0,location_flag=0,move_flag=0; //标志位清零
/*
    dynamic_reconfigure::Server<wheeltec_arm_pick::paramsConfig> *             dynamic_reconfigure_server;
    dynamic_reconfigure::Server<wheeltec_arm_pick::paramsConfig>::CallbackType dynamic_reconfigure_callback;

    dynamic_reconfigure_callback = boost::bind(&car_init::reconfigCB, this, _1, _2);

    dynamic_reconfigure_server = new dynamic_reconfigure::Server<wheeltec_arm_pick::paramsConfig>(param_nh);
    dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);
*/
    ROS_INFO_STREAM("car_location_color_node_init_successful");

}

//析购函数
car_init::~car_init()
{
    ROS_INFO_STREAM("car_location_color_node_close");
}


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "car_location_color"); //节点初始化
    car_init car_control;  //实例化一个对象
    car_control.control(); //循环执行内容
    return 0;
}




