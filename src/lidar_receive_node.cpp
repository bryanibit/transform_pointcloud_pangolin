/*
   * PCL Example using ROS and CPP
   */
  #include "lidar_receive/PangoCloud.h"
  #include <future>
  #include <thread>
  #include <boost/thread/thread.hpp>
  #include <sys/time.h>

  // Include the ROS library
  #include <ros/ros.h>
  #include <sensor_msgs/PointCloud2.h>

  // Include pcl
  #include <pcl_conversions/pcl_conversions.h>
  #include <pcl/visualization/cloud_viewer.h>
  #include <pcl/common/common_headers.h>
  #include <pcl/point_cloud.h>
  #include <pcl/point_types.h>
  #include <pcl_ros/transforms.h>

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rs_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Vector3f trans_vec_A{0,0,0};
  Eigen::Translation<float,3> translation_A(trans_vec_A);
  Eigen::Quaternionf rotation_B = static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())) ;
  Eigen::Quaternionf rotation_C = static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())) ;
  Eigen::Quaternionf rotation_D= static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
  Eigen::Vector3f trans_vec_E{0,0,0};
  Eigen::Translation<float,3> translation_E(trans_vec_E);
  Eigen::Transform<float,3,Eigen::Affine> combined = 
      translation_A * rotation_B * rotation_C * rotation_D * translation_E;

  struct timeval timestart;

  //-----------------------
// struct CustomType
// {
//   CustomType()
//         : x(0), y(0.0f) {}

//   CustomType(int x, float y, std::string z)
//         : x(x), y(y), z(z) {}
//   int x;
//   float y;
//   std::string z;
// };

// std::ostream& operator<< (std::ostream& os, const CustomType& o){
//     os << o.x << " " << o.y << " " << o.z;
//     return os;
// }

// std::istream& operator>> (std::istream& is, CustomType& o){
//     is >> o.x;
//     is >> o.y;
//     is >> o.z;
//     return is;
// }

void getCurrentTime()
{
    gettimeofday(&timestart, NULL);
    // double timeofsec = static_cast<double>(timestart.tv_sec + static_cast<double>(timestart.tv_usec)/1000000);
    auto timeofsec = static_cast<int>(timestart.tv_sec);
    ROS_INFO("time is %d s", timeofsec);
}

void pango_init()
{
    ROS_INFO("Enter pangolin thread\n");
    // Load configuration data
    pangolin::ParseVarsFile("/home/ugv-yu/catkin_ws/src/lidar_receive/app.cfg");

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("Main",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
                pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
                );

    const int UI_WIDTH = 180;

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    // Add named Panel and bind to variables beginning 'ui'
    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    // Safe and efficient binding of named variables.
    // Specialisations mean no conversions take place for exact types
    // and conversions between scalar types are cheap.
    pangolin::Var<bool> a_button("ui.A_Button",false,false);
    pangolin::Var<double> doubleX("ui.angleX",0,0,180.0);
    pangolin::Var<double> doubleY("ui.angleY",0,0,180.0);
    pangolin::Var<double> doubleZ("ui.angleZ",0,0,180.0);
    pangolin::Var<double> translateX("ui.translateX",0,-10.0,10.0);
    pangolin::Var<double> translateY("ui.translateY",0,-10.0,10.0);
    pangolin::Var<double> translateZ("ui.translateZ",0,-10.0,10.0);
    // pangolin::Var<int> an_int("ui.An_Int",2,0,180);
    // pangolin::Var<double> a_double_log("ui.Log_scale var",3,1,1E4, true);
    // pangolin::Var<bool> a_checkbox("ui.A_Checkbox",false,true);
    // pangolin::Var<int> an_int_no_input("ui.An_Int_No_Input",2);
    // pangolin::Var<CustomType> any_type("ui.Some_Type", CustomType(0,1.2f,"Hello") );

    // pangolin::Var<bool> save_window("ui.Save_Window",false,false);
    // pangolin::Var<bool> save_cube("ui.Save_Cube",false,false);

    // pangolin::Var<bool> record_cube("ui.Record_Cube",false,false);

    // std::function objects can be used for Var's too. These work great with C++11 closures.
    pangolin::Var<std::function<void(void)> > reset("ui.Time", getCurrentTime);

    // Demonstration of how we can register a keyboard hook to alter a Var
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'x', pangolin::SetVarFunctor<double>("ui.angleX", 180));
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'y', pangolin::SetVarFunctor<double>("ui.angleY", 180));
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'z', pangolin::SetVarFunctor<double>("ui.angleZ", 180));
    // Demonstration of how we can register a keyboard hook to trigger a method
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 't', getCurrentTime);
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        // Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if( pangolin::Pushed(a_button) )
          {
            std::cout << "Angle or translation changed!" << std::endl;
            translation_A.x() = translateX;
            translation_A.y() = translateY;
            translation_A.z() = translateZ;
            rotation_B = static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(doubleX, Eigen::Vector3f::UnitX())) ;
            rotation_C = static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(doubleY, Eigen::Vector3f::UnitY())) ;
            rotation_D = static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(doubleZ, Eigen::Vector3f::UnitZ()));
            combined = translation_A * rotation_B * rotation_C * rotation_D * translation_E;
          }

        // Overloading of Var<T> operators allows us to treat them like
        // their wrapped types, eg:
        // if( a_checkbox )
        //   an_int = (int)a_double;

        // if( !any_type->z.compare("robot"))
        //   any_type = CustomType(1,2.3f,"Boogie");

        // an_int_no_input = an_int;

        // if( pangolin::Pushed(save_window))
        //   pangolin::SaveWindowOnRender("window");

        // if( pangolin::Pushed(save_cube) )
        //   d_cam.SaveOnRender("cube");

        // if( pangolin::Pushed(record_cube) )
        //   pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");

        // Activate efficiently by object
        d_cam.Activate(s_cam);

        // Render some stuff
        // glColor3f(1.0,1.0,1.0);

        PangoCloud pgcloud(temp_cloud.get());
        pgcloud.drawPoints();

        PangoCloud pgcloudrs(rs_cloud.get());
        pgcloudrs.drawPoints();

        pangolin::FinishFrame();
    }
}
  //------------------------

void pcl_init()
{
  ROS_INFO("Enter pcl_init");
  pcl::visualization::CloudViewer viewer("Simple lidar view");
  
  while(!viewer.wasStopped())
  {
    viewer.showCloud(temp_cloud);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
  }
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    if(temp_cloud->empty())
    {
      ROS_INFO("converted cloud is empty");
    }
    gettimeofday(&timestart, NULL);
    // double timeofsec = static_cast<double>(timestart.tv_sec + static_cast<double>(timestart.tv_usec)/1000000);
    auto timeofsec = static_cast<int>(timestart.tv_sec);
    // ROS_INFO("time is %d", timeofsec);
    // if(timeofsec % 10 == 0)
    // {
    // pcl::transformPointCloud (*temp_cloud, *temp_cloud, combined);
    // }
  }

  void cloud_rs(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*rs_cloud);
    if(rs_cloud->empty())
    {
      ROS_INFO("converted cloud is empty");
    }
    gettimeofday(&timestart, NULL);
    // double timeofsec = static_cast<double>(timestart.tv_sec + static_cast<double>(timestart.tv_usec)/1000000);
    auto timeofsec = static_cast<int>(timestart.tv_sec);
    // ROS_INFO("time is %d", timeofsec);
    // if(timeofsec % 10 == 0)
    // {
    pcl::transformPointCloud (*rs_cloud, *rs_cloud, combined);
    // }
  }

  int main (int argc, char** argv)
  {
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init (argc, argv, "roscpp_pcl_example");
    ros::NodeHandle nh;
    // std::thread pcl_thd(pcl_init);
    std::thread pango_thd(pango_init);
    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());
    
    ros::Subscriber sub = nh.subscribe("/pandar_points", 1, cloud_cb);
    ros::Subscriber sub_rs = nh.subscribe("/cloud_node_left/rslidar_points_left", 1, cloud_rs);
    ros::spin();
    // pcl_thd.join();
    pango_thd.join();
    return 0;
    
  }
