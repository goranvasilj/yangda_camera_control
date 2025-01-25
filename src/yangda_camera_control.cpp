/***
 * Modbus master for communicating over serial port
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <yangda_camera_control/gimbal_feedback.h>
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 115200


serial::Serial ser;
uint8_t data1[500];
double received_data[13];
double roll=0, pitch=0 ,yaw=0, zoom;
int GetValue(uint8_t data1,uint8_t data2)
{
	int value=data2*256+data1;
	if (value>32767)
	{
		value=65536-value;
	}
	return value;
}

bool Parse_gimbal(const char* data,int count)
{
	int suma=0;
	if (data[0]==0x3E && data[1]==0x3D && data[2]==0x36 && data[3]==0x73)
	{
		for (int i = 4; i < count-1; i++)
		{
			suma=suma+(uint8_t) data[i];
		}
		if (suma%256==(uint8_t) data[count-1])
		{
			roll=0.02197*GetValue(data[4],data[5]);
			pitch=0.02197*GetValue(data[22],data[23]);
			yaw=0.02197*GetValue(data[40],data[41]);
//			printf("roll pitch yaw %.2f %.2f %.2f\n",roll,pitch, yaw);
			return true;

		}
		else
		{
			std::cout<<"Wrong checksum"<<std::endl;
		}

	}
	else
	{
		std::cout<<"Wrong header"<<std::endl;
	}

	return false;
}
bool Parse_zoom(const char* data,int count)
{
	int suma=0;
	double zoom_p=0,zoom_q=0, zoom_r=0,zoom_s=0;
	if ((uint8_t)data[0]==0x90 && (uint8_t)data[1]==0x50)
	{
		if (0xFF==(uint8_t) data[count-1])
		{
			zoom_p=(uint8_t)data[2];
			zoom_q=(uint8_t)data[3];
			zoom_r=(uint8_t)data[4];
			zoom_s=(uint8_t)data[5];

//			printf("zoom p q r s %.2f %.2f %.2f %.2f       %.2f\n",zoom_p,zoom_q,zoom_r,zoom_s,zoom_p*16*16*16+zoom_q*16*16+zoom_r*16+zoom_s);
			zoom=zoom_p*16*16*16+zoom_q*16*16+zoom_r*16+zoom_s;
			return true;

		}
		else
		{
			printf("%d %d\n",suma%256,(uint8_t) data[count-1]);
			std::cout<<"Wrong checksum"<<std::endl;
		}

	}
	else
	{
		std::cout<<"Wrong header"<<std::endl;
	}

	return false;
}
void cmd_roll_callback(const std_msgs::Float32::ConstPtr& msg){

	double roll_cmd=(*msg).data;

	//get byte representation
	int roll=roll_cmd/0.02197;
	int suma=0;
	if (roll<0)
	{
		roll=65536-abs(roll);
	}
	int b1=roll/256;
	int b2=roll%256;
	//command for writing roll
    uint8_t write_roll[30]={0xFF, 0x01, 0x0F, 0x10,
    		0x05, 0x00, 0x00,
			0x00, 0x00, b2, b1,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00};
	int count=20;
	//add checksum
	for (int i = 4; i < count-1; i++)
	{
		suma=suma+(uint8_t) write_roll[i];
	}
	write_roll[count-1]=suma%256;

	//send command
    ser.write(write_roll,count);
    std::string data;
    int count1 = ser.readline(data,500,(std::string)"\n");
}

void cmd_pitch_callback(const std_msgs::Float32::ConstPtr& msg){

	double cmd=(*msg).data;

	//get byte representation
	int pitch=cmd/0.02197;
	int suma=0;
	if (pitch<0)
	{
		roll=65536-abs(pitch);
	}
	int b1=pitch/256;
	int b2=pitch%256;
	//command for writing roll
    uint8_t write[30]={0xFF, 0x01, 0x0F, 0x10,
    		0x00, 0x05, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, b2, b1,
			0x00, 0x00, 0x00, 0x00,
			0x00};
	int count=20;
	//add checksum
	for (int i = 4; i < count-1; i++)
	{
		suma=suma+(uint8_t) write[i];
	}
	write[count-1]=suma%256;

	//send command
    ser.write(write,count);
    std::string data;
    int count1 = ser.readline(data,500,(std::string)"\n");
}
void cmd_yaw_callback(const std_msgs::Float32::ConstPtr& msg){

	double cmd=(*msg).data;

	//get byte representation
	int yaw=cmd/0.02197;
	int suma=0;
	if (yaw<0)
	{
		roll=65536-abs(yaw);
	}
	int b1=yaw/256;
	int b2=yaw%256;
	//command for writing roll
    uint8_t write[30]={0xFF, 0x01, 0x0F, 0x10,
    		0x00, 0x00, 0x05,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, b2, b1,
			0x00};
	int count=20;
	//add checksum
	for (int i = 4; i < count-1; i++)
	{
		suma=suma+(uint8_t) write[i];
	}
	write[count-1]=suma%256;

	//send command
    ser.write(write,count);
    std::string data;
    int count1 = ser.readline(data,500,(std::string)"\n");
}


int main (int argc, char** argv){
    ros::init(argc, argv, "yangda_camera_control");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    std::string port, topic,cmd_roll,cmd_pitch,cmd_yaw,cmd_zoom;
    int baudrate;
    nh_ns.param("port", port, (std::string) DEFAULT_SERIAL_PORT); 
    nh_ns.param("baudrate", baudrate, DEFAULT_BAUD_RATE);
    nh_ns.param("topic", topic , (std::string) "/camera_gimbal");

    nh_ns.param("cmd_roll", cmd_roll , (std::string) "/camera_cmd_roll");
    nh_ns.param("cmd_pitch", cmd_pitch , (std::string) "/camera_cmd_pitch");
    nh_ns.param("cmd_yaw", cmd_yaw , (std::string) "/camera_cmd_yaw");
    nh_ns.param("cmd_zoom", cmd_zoom , (std::string) "/camera_cmd_zoom");

    ros::Subscriber sub_roll = nh.subscribe(cmd_roll, 1000, cmd_roll_callback);
    ros::Subscriber sub_pitch = nh.subscribe(cmd_pitch, 1000, cmd_pitch_callback);
    ros::Subscriber sub_yaw = nh.subscribe(cmd_yaw, 1000, cmd_yaw_callback);


   //open port
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(30);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(1);
    bool first = true;
    double start_stamp=0;
    ros::Publisher gimbal_info_publisher = nh.advertise<yangda_camera_control::gimbal_feedback>(topic, 1000);
    ros::Time start_time;
    std::string data_start;
    uint8_t read_feedback[30]={0x3e,0x3d,0x00,0x3d,0x00};
    int read_feedback_len=5;

    uint8_t read_zoom[30]={0x81,0x09,0x04,0x47,0xff};
    int read_zoom_len=5;


    int count = ser.read(data_start,500);

    ros::spinOnce();
    while(ros::ok()){
    	// read gimbal feedback
        ros::spinOnce();
        std::string data2;
        ser.write(read_feedback,read_feedback_len);
        int count = ser.readline(data2,500,(std::string)"\n");
        int succ=true;
        if (count>5)
        {
        	succ=Parse_gimbal(data2.c_str(),count);
        }

        //read zoom feedback
        ser.write(read_zoom,read_zoom_len);
        std::string data3;
        count = ser.readline(data3,500,(std::string)"\n");

        if (count>5 && succ==true)
        {
        	succ=Parse_zoom(data3.c_str(),count);
        }


       	if ((int)received_data[0]==0)
        {
                   	yangda_camera_control::gimbal_feedback pub_data1;
                   	pub_data1.roll=roll;
                   	pub_data1.pitch=pitch;
                   	pub_data1.yaw=yaw;
                   	pub_data1.zoom=zoom;

                   	gimbal_info_publisher.publish(pub_data1);
        }


        if (ser.available()<500)
        {
        	loop_rate.sleep();
        }else
        {
            ros::spinOnce();
        }

    }
}

