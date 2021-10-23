#ifndef __LOAD_DATA_H_
#define __LOAD_DATA_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#define total_readings 360


typedef struct
{
        int data_type;                                                  // 0-里程计数据,1-激光雷达数据
        float x_robot,y_robot,theta_robot;              // 机器人位姿
        //float x_lidar,y_lidar,theta_lidar;              // 激光雷达位姿
        float* readings;                                                // 激光雷达读数
        //float time_stamp;                                               // 时间戳
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

		if(!file.is_open()){
			return false;
		}

		while(std::getline(file, logline)){
			log_data logdata_indv;
	
			std::istringstream iss(logline);
                        std::string tag;
                        iss >> tag;
                        int j = 0;
			
			if( tag.compare( "laser" ) == 0 ){
                                	logdata_indv.data_type = 1;             //此行数据包括lidar
				
                                	logdata_indv.readings = (float*) malloc(sizeof(float) * total_readings);
                                	while (j < total_readings){
						std::string num;
						iss >> num;
						if( num.compare("inf") ){
							logdata_indv.readings[j] = std::stof( num ) * 10.0;
						}
						else {
							logdata_indv.readings[j] = 65536.0;
						}
                                        	j++;
                                        	// cout << logdata_indv.readings[j] << endl;
                                	}
			}

			if( tag.compare( "odom" ) == 0 ){
				//此行数据只包括odom
                                logdata_indv.data_type = 0;
                                iss >> logdata_indv.x_robot >> logdata_indv.y_robot >> logdata_indv.theta_robot;
			}

			logfile_data.push_back(logdata_indv);   //存入vector中

		}
	}

        const std::vector<log_data> getLog() const
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
