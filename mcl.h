#ifndef __MCL_H_
#define __MCL_H_

#include <iostream>
#include "mcl_map.h"
#include "load_data.h"
#include <vector>

#define	pi            3.1415926536
#define	ODOM_DATA             0
#define	LIDAR_DATA            1
#define	LASER_BEAM_NUM        360


typedef struct
{
        float x = 0;
        float y = 0;
        float theta = 0;
}robot_state;


typedef struct
{
        float x = 0;
        float y = 0;
        float theta = 0;
        float weight = 0;
}particle_state;

typedef struct
{
        float x_front = 0;
        float y_front  = 0;
        float theta_front = 0;
        float x_rear = 0;
        float y_rear = 0;
        float theta_rear = 0;
}motion_model;


typedef struct
{
        float* readings;
}lidar_measure;

class MCL
{
public:
	MCL()
	{
		map_ = new Map();	
	}

	~MCL()
	{
		delete map_;
	}

	bool loadMap( const std::string &fileName )
	{
		return map_->loadMap( fileName );
	}

	void initParticles()
	{
		int count = 1;
		while( count <= numParticles_  ){
			particle_state particle_temp;
			
			particle_temp.x = rand() / (float)RAND_MAX * (map_->getValidAreaMaxX() - map_->getValidAreaMinX()) + map_->getValidAreaMinX(); //初始化粒子X坐标
                	particle_temp.y = rand() / (float)RAND_MAX * (map_->getValidAreaMaxY() - map_->getValidAreaMinY()) + map_->getValidAreaMinY();  //初始化粒子Y坐标

			if (map_->getProb( (int) particle_temp.x, (int) particle_temp.y ) >= 0.0)  //若随机出的粒子不在地图有效范围内,则重新生成粒子
				continue;
	
			count ++;

			particle_temp.theta = rand() / (float)RAND_MAX * 2 * pi;  //初>始化粒子角度theta 
			//将粒子角度转换到 -pi ～ pi 之间
                	if(particle_temp.theta > pi)
                        	particle_temp.theta -= 2 * pi;

                	if(particle_temp.theta < -pi)
                        	particle_temp.theta += 2 * pi;
		
			particle_temp.weight = 1.0 / numParticles_;   // 初始化粒子权重为1/NUM
			particles_.push_back(particle_temp);   //存入粒子集
			
		}
	
		std::cout<<"initialized the particles ..."<<std::endl;
	}

	//概率机器人P103---基于里程计运动模型中采样算法 
	particle_state sampleMotionModelOdometry(particle_state &particle)
	{
        	float deltarot1 = atan2(motion_.y_rear - motion_.y_front,motion_.x_rear - motion_.x_front) - motion_.theta_rear;
        	float deltatrans1 = sqrt(pow((motion_.x_rear - motion_.x_front),2) + pow((motion_.y_rear - motion_.y_front),2));
        	float deltarot2 = motion_.theta_rear - motion_.theta_front - deltarot1;

       	 	float deltarot1_hat = deltarot1 - sampleStandardNormalDistribution(alpha1_*deltarot1 + alpha2_*deltatrans1);
        	float deltatrans1_hat  = deltatrans1 -sampleStandardNormalDistribution(alpha3_*deltatrans1 + alpha4_*(deltarot1 + deltarot2));
        	float deltarot2_hat  = deltarot2 - sampleStandardNormalDistribution(alpha1_*deltarot2 + alpha2_*deltatrans1);

        	particle_state particle_temp;
        	//地图是以dm为单位,初始化的粒子位置是基于地图生成的,所以也是dm单位,而里程计数据单位是m,需在这里进行单位转换
        	particle_temp.x = particle.x + (deltatrans1_hat * cos(particle.theta + deltarot1_hat)) * 10;
        	particle_temp.y = particle.y + (deltatrans1_hat * sin(particle.theta + deltarot1_hat)) * 10;
        	particle_temp.theta = particle.theta + deltarot1_hat + deltarot2_hat;
        	particle_temp.weight = particle.weight;

        	return particle_temp;
	}

	//从标准正态分布中采样
	float sampleStandardNormalDistribution(float var)
	{
        	float sum = 0;
        	for (int i = 0;i < 12; i++)
                	//LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)))
                	sum += (rand() - RAND_MAX / 2) / (float)RAND_MAX * 2;
        	return (var / 6.0) * sum;
	}

	//根据得分模型计算每个粒子的权重
	float measurementScoreModel(particle_state particle)
	{
        	robot_state lidar_pose;
	        float laser_end_x,laser_end_y,score = 0, zkt = 0;

	    	//计算激光雷达在map坐标系下的位姿
        	lidar_pose.x = particle.x + (lidar_offset_ * cos(particle.theta)) * resolution_;        //(单位：dm)
        	lidar_pose.y = particle.y + (lidar_offset_ * sin(particle.theta)) * resolution_;        //(单位：dm)
        	lidar_pose.theta = particle.theta;

        	//若雷达位姿在地图有效区域外,则终止此次计算,该粒子权重为0
        	//if(map_->getProb( (int)lidar_pose.x, (int)lidar_pose.y ) <= map_threshold_)  
               	// 	return 0.0;
		
		float step_theta = -3.12413907051f;
		for (int i = 0; i < LASER_BEAM_NUM; i++){
                	zkt = measurement_.readings[i];         //第i个激光束的测距，单位：dm,从log文件读取时就已经转换成dm了

                	//若超出设置的lidar最大有效值，则此光束无效
                	if (zkt > (lidar_range_max_ * resolution_))
                        	continue;

	                //计算第i个激光束在世界坐标系下的角度
			step_theta += 0.0174532924f;
		
                	laser_end_x = lidar_pose.x + zkt * cos(step_theta);     //计算>此激光束末端在map坐标系下的X坐标
	                laser_end_y = lidar_pose.y + zkt * sin(step_theta);     //计算>此激光束末端在map坐标系下的Y坐标

        	        //若激光束末端在地图未知区域或无效区域，则跳过此次得分计算
                	//if(laser_end_x >= map_->getValidAreaMaxX() || laser_end_y >= map_->getValidAreaMaxY() || laser_end_x < map_->getValidAreaMinX() || laser_end_y < map_->getValidAreaMinY() || map_->getProb( (int)laser_end_x, (int)laser_end_y ) < 0)
                   	//	continue;

                	// if(map_->prob[(int)laser_end_x][(int)laser_end_y] >= 0 && map_->prob[(int)laser_end_x][(int)laser_end_y] < 0.15)
                	//      score++;

                	score += map_->getProb( (int)laser_end_x, (int)laser_end_y )  > 0 ? 1 : 0; //累加，计算此帧lidar数据的得分
        		//score += 10 * map_->getProb( (int)laser_end_x, (int)laser_end_y ); //累加，计算此帧lidar数据的得分

		}

        	return score;   //返回当前帧lidar数据的得分，用此来表示粒子权重
	}
		

	void displayMap()
	{
		int sizeX = map_->getSizeX();
		int sizeY = map_->getSizeY();

		cv::Mat image = cv::Mat( sizeX, sizeY, CV_8UC3, cv::Scalar::all(125) );

		for( int x = 0; x < sizeX; x ++ ){
	                for( int y = 0; y < sizeY; y ++ ){
                		if( this->map_->getProb(x, y) > 0.0 ){
                                        cv::Vec3b p;
                                        p[0] = 0;
                                        p[1] = 0;
                                        p[2] = 0;

                                        image.at<cv::Vec3b>(x, y) = p;
                                }
                                else if( this->map_->getProb(x, y) < 0.0 ){
                                        cv::Vec3b p;
                                        p[0] = 255;
                                        p[1] = 255;
                                        p[2] = 255;

                                        image.at<cv::Vec3b>(x, y) = p;
                                }
			}
       	 	}
	
		for( int i = 0; i < numParticles_; i ++ ){
                	cv::Vec3b p;
                	p[0] = 0;
                	p[1] = 0;
                	p[2] = 255;

                	image.at<cv::Vec3b>((int)particles_[i].x, (int)particles_[i].y) = p;
                	//cv::circle(image, cv::Point2d(particles_[i].y, particles_[i].x), 2, cv::Scalar(0, 0, 255), -1);
        	}

        	// robot pose
        	cv::circle(image, cv::Point2d(robot_pose_.y, robot_pose_.x), 1, cv::Scalar(0, 255, 0), -1);

        	cv::imshow( "particle", image );
	}

	void mclTest()
	{
		particle_state particle_state_temp;
	        std::vector<particle_state> particles_temp;
        	particles_temp.resize(numParticles_);

		for (int i = 1;i < log_data_.size(); i++) {
	                //取数据集中相邻两帧数据，作为里程计运动模型的相邻两次状态
        	        motion_.x_front = log_data_[i-1].x_robot;
                	motion_.y_front = log_data_[i-1].y_robot;
	                motion_.theta_front = log_data_[i-1].theta_robot;
        	
		        motion_.x_rear = log_data_[i].x_robot;
	                motion_.y_rear =log_data_[i].y_robot;
        	        motion_.theta_rear = log_data_[i].theta_robot;
			
			for(int j = 0; j < numParticles_; j++){
                        	particle_state_temp = sampleMotionModelOdometry(particles_[j]);   //从里程计运动模型中采样
                        	// particles_temp.push_back(particle_state_temp);
                        	particles_temp[j] = particle_state_temp;
                	}

			particles_ = particles_temp;
	
			if(log_data_[i].data_type == ODOM_DATA ){         //此帧数据只包括odometry
                        	displayMap();    //显示实时更新的粒子集状态
				continue;
                	}
			else if(log_data_[i].data_type == LIDAR_DATA) { //此帧数据包
				measurement_.readings = log_data_[i].readings;
	
				double total_weight = 0;
                        	for(int j = 0; j < numParticles_; j++) {
                                	float weight = measurementScoreModel(particles_[j]);      //根据似然法计算每个粒子的权重
                                	// cout << " each weight is : " << weight << endl;
                               	 	particles_[j].weight = weight;
                                	total_weight += particles_[j].weight;     //粒子权重总和
                        	}
				
				for(int j = 0; j < numParticles_; j++) {
                                	particles_[j].weight /= total_weight;  //所>有粒子权重归一化
                        	}
		
				float avg_weight = total_weight / numParticles_;
 	                        // cout << " The average weight is : " << avg_weight << endl;
				lowVarianceSampler();           //低方差重采样算法
			}
			else {
				std::cout << " ERROR：The Log Data Is Error!!!" << std::endl;
          
			}
			cacuRobotPose();

			displayMap();
			cv::waitKey(0);
		}
	}

	void setLogData( std::vector<log_data> &logData )
	{
		log_data_ = logData;
	}

private:
	//概率机器人P78---低方差重采样算法
	void lowVarianceSampler()
	{
        	std::vector<particle_state> particles_temp = particles_;
        	float r = (rand() / (float)RAND_MAX) * (1.0 / (float)numParticles_); //初始化一个0~1之间的随机数
        	float c = particles_[0].weight;
        	int i = 0;

        	for (int m = 0;m < numParticles_; m++){
                	float u = r + (float) m/ numParticles_;                 //移动随机数

                	while (u > c && i < numParticles_ - 1){
                        	i++;                                                                            //移动到下一个粒子
                        	c += particles_temp[i].weight;
                	}
                	particles_[m] = particles_temp[i];                              //复制被选择的粒子
                	particles_[m].weight = 1.0 / numParticles_;             //重采样后粒子权重重置
                	// cout << " each weight is : " << particles_[m].weight << endl;
        	}
	}
	
	//计算机器人位姿，并发布状态
	void cacuRobotPose()
	{
        	float total_x = 0.0;
	        float total_y = 0.0;
        	float total_theta = 0.0;

	        for(int i = 0; i < numParticles_; i++) {
        	        total_x += particles_[i].x;
                	total_y += particles_[i].y;
	                total_theta += particles_[i].theta;
        	}
	        robot_pose_.x = total_x / numParticles_;
        	robot_pose_.y = total_y / numParticles_;
	        robot_pose_.theta = total_theta / numParticles_;

        	std::cout << "Robot pose:  X: " << robot_pose_.x << ", Y: " << robot_pose_.y << ", Theta: " << robot_pose_.theta << std::endl;

	        // display the pose of the robot
	}


private:
	Map *map_;

	std::vector<log_data> log_data_;
	
	std::vector<particle_state> particles_;
	robot_state robot_pose_;
	motion_model motion_;
	lidar_measure measurement_;	

	float alpha1_ = 0.025, alpha2_ = 0.025, alpha3_ = 0.4, alpha4_ = 0.4;
        float threshold_, map_threshold_ = 0.95, obstacle_threshold_ = 0.2, lidar_offset_ = 0;
        int numParticles_ = 1000, ray_tracing_step_ = 1, resolution_ = 10, lidar_range_max_ = 10;
};

#endif
