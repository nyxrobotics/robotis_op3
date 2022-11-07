#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include "robotis_controller/robotis_controller.h"
#include "std_msgs/UInt32MultiArray.h"



using namespace robotis_framework;
using namespace dynamixel;


RobotisController::RobotisController()
  : is_timer_running_(false),
    is_offset_enabled_(true),
    offset_ratio_(1.0),
    stop_timer_(false),
    init_pose_loaded_(false),
    timer_thread_(0),
    controller_mode_(MotionModuleMode),
    DEBUG_PRINT(false),
    robot_(0),
    gazebo_mode_(false),
    gazebo_robot_name_("robotis")
{
  direct_sync_write_.clear();
}

FILE *plot = fopen("plot.csv", "w");

class Collection
{
    public:

        Collection(){
            ros::NodeHandle nh;
            ROS_INFO("current_collection->init");
            // controller_.getInstance();


            // _current_pub = nh.advertise<std::vector<double>>("/collection/currents", 0,this);
            /* Load ROS Parameter */
            // nh.param<std::string>("device_name", g_device_name, SUB_CONTROLLER_DEVICE);
            // nh.param<int>("baud_rate", g_baudrate, BAUD_RATE);
            
            nh.param<std::string>("robot_file_path", robot_file_, "");
            nh.param<std::string>("init_file_path", init_file_, "");
            _current_pub = nh.advertise<std_msgs::UInt32MultiArray>("/collection/dxl_currents",0);


            std::string dev_desc_dir_path = ros::package::getPath("robotis_device") + "/devices";
            ROS_INFO("dir path");
            
            // std::string robot_file_path = "/dev/ttyUSB0";
            // ROS_INFO("robot path file");

            // ROS_INFO(robot_file_);
            // ROS_INFO(dev_desc_dir_path);

            // *robot_ = Robot(robot_file_path, dev_desc_dir_path);
            // controller.initialize(robot_file_, init_file_);
            
            // *robot_ = Robot(robot_file_, dev_desc_dir_path);


            ROS_INFO("init_finished");

            while_func();

        }
    
        // void get_current()
        // {
        //     RobotisController *controller = RobotisController::getInstance();

        //     ROS_INFO("get current");
        //     controller->startTimer();
        //     uint32_t robot_currents;
        //     // robot_currents.resize(12);
        //     uint32_t read_data;
        //     for (auto& it : controller->robot_->dxls_)
        //     {
        //         std::string joint_name = it.first;
        //         Dynamixel *dxl = it.second;
            
        //         // dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
        //         // dynamixel::PortHandler   *port_handler  = controller->robot_->ports_[dxl->port_name_];
        //         uint32_t data32 = 0;
        //         int result=controller->readCtrlItem(joint_name,"present_current",&data32);
        //         switch (result){
        //         case COMM_PORT_BUSY:
        //             ROS_INFO("Commport busy");
        //             break;
        //         case COMM_NOT_AVAILABLE:
        //             ROS_INFO("com not available");
        //             break;
        //         case COMM_SUCCESS:
        //             ROS_INFO("comm success");
        //             break;
        //         default:
        //             ROS_INFO("comm none");
        //             break;
        //         }
        //         // int result = pkt_handler->read4ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
        //         // int result = pkt_handler->read2ByteTxRx(port_handler, dxl->id_, 126, &read_data, error);
        //         // if (result == COMM_SUCCESS)
        //         // {
        //         //     robot_currents=read_data;
        //         //     // ROS_INFO("current reading %f",robot_currents);
        //         // }
                    

        //     }
        //     controller->stopTimer();


        // }

        void dxlCurrentCollector()
        {
            ROS_INFO("Starting robot controller");
            RobotisController *controller = RobotisController::getInstance();
            ROS_INFO("Started robot controller");

            // current_msg.resize(controller->robot_->dxls_.size());
            uint8_t  data8 = 0;
            uint16_t  data16 = 0;
            uint32_t  data32 = 0;
            // uint8_t addy = 100;
            // int dxl_index = 0;
            current_msg.data.clear();
            int result = COMM_NOT_AVAILABLE;
            controller->startTimer();

            for (auto& it : controller->robot_->dxls_)
            {
                // ROS_INFO("Current for loop");
                
                std::string joint_name = it.first;
                Dynamixel *dxl = it.second;
                // ControlTableItem *item = dxl->ctrl_table_["present_position"];
                // ROsult = controller->read4Byte(joint_name,item->address_,&data32);
                // if (result == COMM_SUCCESS){
                //   ROS_INFO("Successful commm"); 
                // }
                result=controller->readCtrlItem(joint_name,"present_current",&data32);
                switch (result){
                case COMM_PORT_BUSY:
                    ROS_INFO("Commport busy");
                    break;
                case COMM_NOT_AVAILABLE:
                    ROS_INFO("com not available");
                    break;
                case COMM_SUCCESS:
                    ROS_INFO("comm success");
                    break;
                default:
                    ROS_INFO("comm none");
                    break;
                }

                // if (result == COMM_SUCCESS){
                //   ROS_INFO("Successful commm"); 
                // }
                // ROS_INFO("%u",data32);
                current_msg.data.push_back(data32);
                // current_msg[dxl_index] = data32; 
                // dxl_index ++;
                // ROS_INFO("joint "+it.first+" has current of %i", data32);
                controller->stopTimer();
            }
            // dxl_index = 0;

            _current_pub.publish(current_msg);
            ROS_INFO("Current published");


        }
    

        void while_func(){
            rate = 5;
            ros::Rate r(rate);
            ROS_INFO("Entering while function");
            while(ros::ok){
                dxlCurrentCollector();
                // ros::spinOnce();
                r.sleep();
            }
        }

    private:
        ros::Publisher g_init_pose_pub;
        ros::Publisher g_demo_command_pub;
        ros::Publisher _current_pub;
        std::string robot_file_ = "";
        std::string init_file_ = "";
        std_msgs::UInt32MultiArray current_msg;
        int rate;
        Robot *robot_;
        uint8_t *error;
        robotis_framework::RobotisController controller;

};

int main(int argc, char **argv)
{
    // *robot_ = Robot(robot_file_path, dev_desc_dir_path);

    ros::init(argc, argv, "Collection");
    Collection node;
    ros::spin();
    return 0;
}
  


  


  
// //   ros::Subscriber dxl_torque_sub = nh.subscribe("/robotis/dxl_torque", 1, dxlTorqueCheckCallback);
// //   g_init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
// //   g_demo_command_pub = nh.advertise<std_msgs::String>("/ball_tracker/command", 0);

// //   nh.param<bool>("gazebo", controller->gazebo_mode_, false);
// //   g_is_simulation = controller->gazebo_mode_;

//   /* real robot */
//   if (g_is_simulation == false)
//   {
//     // open port
//     PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
//     bool set_port_result = port_handler->setBaudRate(BAUD_RATE);
//     if (set_port_result == false)
//       ROS_ERROR("Error Set port");

//     PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//     // power on dxls
//     int torque_on_count = 0;

//     while (torque_on_count < 5)
//     {
//       int _return = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

//       if(_return != 0)
//         ROS_ERROR("Torque on DXLs! [%s]", packet_handler->getRxPacketError(_return));
//       else
//         ROS_INFO("Torque on DXLs!");

//       if (_return == 0)
//         break;
//       else
//         torque_on_count++;
//     }

//     usleep(100 * 1000);

//     // set RGB-LED to GREEN
//     int led_full_unit = 0x1F;
//     int led_range = 5;
//     int led_value = led_full_unit << led_range;
//     int _return = packet_handler->write2ByteTxRx(port_handler, SUB_CONTROLLER_ID, RGB_LED_CTRL_TABLE, led_value);

//     if(_return != 0)
//       ROS_ERROR("Fail to control LED [%s]", packet_handler->getRxPacketError(_return));

//     port_handler->closePort();
//   }
 

//   while (ros::ok())
//   {
//     // usleep(1 * 1000);
//     get_current();

//     ros::spin();
//   }

//   return 0;
// }
