#ifndef __LOAD_DATA_H_
#define __LOAD_DATA_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#define total_readings 180


typedef struct
{
        int data_type;                                                  // 0-里程计数据,1-激光雷达数据
        float x_robot,y_robot,theta_robot;              // 机器人位姿
        float x_lidar,y_lidar,theta_lidar;              // 激光雷达位姿
        float* readings;                                                // 激光雷达读数
        float time_stamp;                                               // 时间戳
}log_data;


class LoadLog
{
public:
        LoadLog(const char* log_str)
	{
		readFromData( log_str, this->log );		
	}

        ~LoadLog()
	{

	}

        int readFromData(const char* logfile, std::vector<log_data>& logfile_data)
	{
		std::ifstream file (logfile);
	        std::string logline;
        	log_data logdata_indv;

		if(!file.is_open()){
			return false;
		}

		while(std::getline(file, logline)){
			std::istringstream iss(logline);
                        char c;
                        iss >> c;
                        int j = 0;
                        switch(c) {
				case 'L':
                                	logdata_indv.data_type = 1;             //此行数据包括odom和lidar
                                	iss >> logdata_indv.x_robot >> logdata_indv.y_robot >> logdata_indv.theta_robot;        //读取机器人位姿,单位：cm;cm;rad
                                	iss >> logdata_indv.x_lidar >> logdata_indv.y_lidar >> logdata_indv.theta_lidar;        //读取激光雷达位姿,单位：cm;cm;rad
                                	logdata_indv.readings = (float*) malloc(sizeof(float) * total_readings);
                                	while (j < total_readings){
                                		iss >> logdata_indv.readings[j];
                                        	logdata_indv.readings[j] /= 10.0;    //lidar读数换算成dm,因为地图的分辨率是1dm,统一单位方便后面计算
                                        	j++;
                                        	// cout << logdata_indv.readings[j] << endl;
                                	}
                                	iss >> logdata_indv.time_stamp;    //读取时间戳
                                	break;
				case 'O':       //此行数据只包括odom
                                        logdata_indv.data_type = 0;
                                        iss >> logdata_indv.x_robot >> logdata_indv.y_robot >> logdata_indv.theta_robot;
                                        iss >> logdata_indv.time_stamp;
                                        break;
                                default:
                                        break;
			}
			logfile_data.push_back(logdata_indv);   //存入vector中

		}
	}

        std::vector<log_data> getLog()
	{
		return log;
	}
        void showLogData()	
	{
		log_data indv;
	        for (int i = 0;i < this->log.size();i++){
	                indv = this->log[i];
        	        std::cout << indv.data_type << std::endl;
        	}
	}

private:
        std::vector<log_data> log;
};


#endif
