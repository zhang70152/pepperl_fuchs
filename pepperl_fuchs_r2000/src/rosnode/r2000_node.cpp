// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
// * Neither the name of Pepperl+Fuchs nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "r2000_node.h"
#include <sensor_msgs/LaserScan.h>
#include <pepperl_fuchs_r2000/r2000_driver.h>

namespace pepperl_fuchs {

//-----------------------------------------------------------------------------
R2000Node::R2000Node():nh_("~")
{
    driver_ = 0;
    // Reading and checking parameters
    //-------------------------------------------------------------------------
    nh_.param("frame_id", frame_id_, std::string("/scan"));
    nh_.param("scanner_ip",scanner_ip_,std::string(""));
    nh_.param("scan_frequency",scan_frequency_,35);
    nh_.param("samples_per_scan",samples_per_scan_,3600);

    if( scanner_ip_ == "" )
    {
        std::cerr << "IP of laser range finder not set!" << std::endl;
        return;
    }

    if( !connect() )
        return;

    // Declare publisher and create timer
    //-------------------------------------------------------------------------
    scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan",100);
    cmd_subscriber_ = nh_.subscribe("control_command",100,&R2000Node::cmdMsgCallback,this);
    ros::Duration publish_interval(2/(2*std::atof(driver_->getParametersCached().at("scan_frequency").c_str())));
    ROS_INFO_STREAM("publish_interval:"<<publish_interval);
    
    //Here change from 1/2 to 2/2. make more sense and seems publish rate and data is ok.
    get_scan_data_timer_ = nh_.createWallTimer(ros::WallDuration(1.00*1.0/(2.0*std::atof(driver_->getParametersCached().at("scan_frequency").c_str()))), &R2000Node::getScanData, this);

    first_data_ = true;

}

//-----------------------------------------------------------------------------
bool R2000Node::connect()
{
    delete driver_;

    // Connecting to laser range finder
    //-------------------------------------------------------------------------
    driver_ = new R2000Driver();
    std::cout << "Connecting to scanner at " << scanner_ip_ << " ... ";
    if( driver_->connect(scanner_ip_,80) )
        std::cout << "OK" << std::endl;
    else
    {
        std::cout << "FAILED!" << std::endl;
        std::cerr << "Connection to scanner at " << scanner_ip_ << " failed!" << std::endl;
        return false;
    }

    // Setting, reading and displaying parameters
    //-------------------------------------------------------------------------
    driver_->setScanFrequency(scan_frequency_);
    driver_->setSamplesPerScan(samples_per_scan_);
    auto params = driver_->getParameters();
    std::cout << "Current scanner settings:" << std::endl;
    std::cout << "============================================================" << std::endl;
    for( const auto& p : params )
        std::cout << p.first << " : " << p.second << std::endl;
    std::cout << "============================================================" << std::endl;

    // Start capturing scanner data
    //-------------------------------------------------------------------------
    std::cout << "Starting capturing: ";
    if( driver_->startCapturingTCP() )
        std::cout << "OK" << std::endl;
    else
    {
        std::cout << "FAILED!" << std::endl;
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
void R2000Node::getScanData(const ros::WallTimerEvent &e)
{
  // ROS_INFO_STREAM("loop time:"<<ros::Time::now()-ttt_);
    ttt_ = ros::Time::now();
    if( !driver_->isCapturing() )
    {
        std::cout << "ERROR: Laser range finder disconnected. Trying to reconnect..." << std::endl;
        while( !connect() )
        {
            std::cout << "ERROR: Reconnect failed. Trying again in 2 seconds..." << std::endl;
            usleep((2*1000000));
        }
    }
     // ros::Duration abc = ros::Time::now() - ttt_;
       //   ROS_INFO_STREAM("process time:"<<abc.toSec());

    auto scandata = driver_->getFullScan();
    if( scandata.amplitude_data.empty() || scandata.distance_data.empty() || scandata.headers.empty() )
        return;
     ros::Duration abc = ros::Time::now() - ttt_;
    // ROS_INFO_STREAM("process time:"<<abc.toSec());


    
    //Get the timestamp of the first package

    std::uint64_t package_header_time = scandata.headers[0].timestamp_raw;

    //Convert NTP time to sec.
    std::uint32_t fractional_part = (package_header_time & 0x00000000FFFFFFFF);
    std::uint32_t integer_part  = package_header_time >>32;
    double scan_time_in_sec = (double)integer_part + (double)fractional_part/pow(2,32);

    if(first_data_)
    {
        ros_base_time_ = ros::Time::now();
        base_time_ = scan_time_in_sec;
        first_data_ = false;
        return;
    }

    double time_diff = scan_time_in_sec - base_time_;
 
    ros::Duration delta_t = ros::Duration().fromSec(time_diff);
    ros::Time scan_time_ros = ros_base_time_ + delta_t;


    //ROS_INFO_STREAM("Elapse since base time:"<<delta_t);
    //ROS_INFO_STREAM("Ros scan time:"<<scan_time_ros);
    //ROS_INFO_STREAM("Delta time:"<<time_diff - last_time_diff_);
    
    last_time_diff_ = time_diff; 
    sensor_msgs::LaserScan scanmsg;
    scanmsg.header.frame_id = frame_id_;
    //scanmsg.header.stamp = scan_time_ros;
    scanmsg.header.stamp = ros::Time::now();

    scanmsg.angle_min = -M_PI;
    scanmsg.angle_max = +M_PI;
    scanmsg.angle_increment = 2*M_PI/float(scandata.distance_data.size());
    scanmsg.time_increment = 1/35.0f/float(scandata.distance_data.size());

    scanmsg.scan_time = 1/std::atof(driver_->getParametersCached().at("scan_frequency").c_str());
    scanmsg.range_min = std::atof(driver_->getParametersCached().at("radial_range_min").c_str());
    scanmsg.range_max = std::atof(driver_->getParametersCached().at("radial_range_max").c_str());

    scanmsg.ranges.resize(scandata.distance_data.size());
    scanmsg.intensities.resize(scandata.amplitude_data.size());
    for( std::size_t i=0; i<scandata.distance_data.size(); i++ )
    {
        scanmsg.ranges[i] = float(scandata.distance_data[i])/1000.0f;
        scanmsg.intensities[i] = scandata.amplitude_data[i];
    }
    scan_publisher_.publish(scanmsg);

    
   // ros::Duration  scan_msg_interval = scan_time_ros - last_scan_time_;
    ros::Duration  scan_msg_interval = ros::Time::now() - last_scan_time_;
    //ROS_INFO_STREAM("scan delta time in ROS time:"<<scan_msg_interval);
    
    //Reset the new base time to now.
    ros_base_time_ = ros::Time::now(); 
   // last_scan_time_ = scan_time_ros;
   last_scan_time_ = ros::Time::now();
   // ROS_INFO_STREAM("process time:"<<ros_base_time_ - ttt_);
    
    

}

//-----------------------------------------------------------------------------
void R2000Node::cmdMsgCallback(const std_msgs::StringConstPtr &msg)
{
    const std::string& cmd = msg->data;
    static const std::string set_scan_frequency_cmd("set scan_frequency=");
    static const std::string set_samples_per_scan_cmd("set samples_per_scan=");

    // Setting of scan_frequency
    //-------------------------------------------------------------------------
    if( cmd.substr(0,set_scan_frequency_cmd.size()) == set_scan_frequency_cmd )
    {
        std::string value = cmd.substr(set_scan_frequency_cmd.size());
        int frequency = std::atoi(value.c_str());
        if(frequency>=10 && frequency<=50)
        {
            scan_frequency_ = frequency;
            driver_->setScanFrequency(frequency);
        }
    }

    // Setting of samples_per_scan
    //-------------------------------------------------------------------------
    if( cmd.substr(0,set_samples_per_scan_cmd.size()) == set_samples_per_scan_cmd )
    {
        std::string value = cmd.substr(set_samples_per_scan_cmd.size());
        int samples = std::atoi(value.c_str());
        if(samples>=72 && samples<=25200)
        {
            samples_per_scan_ = samples;
            driver_->setSamplesPerScan(samples);
        }
    }
}

//-----------------------------------------------------------------------------
} // NS

int main(int argc, char **argv)
{
    ros::init(argc, argv, "r2000_node", ros::init_options::AnonymousName);
    new pepperl_fuchs::R2000Node();
    ros::spin();
    return 0;
}
