#ifndef __MCL_MAP_H_
#define __MCL_MAP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/opencv.hpp>

class Map
{
public:
	Map()
	{

	}
	
	~Map()
	{

	}

	Map( 	const float resolution_, 
		const int sizeX_,
		const int sizeY_, 
		const int validAreaMinX_,
		const int validAreaMaxX_,
		const int validAreaMinY_,
		const int validAreaMaxY_,
		const float offsetX_,
		const float offsetY_ ) : 
		resolution( resolution_ ),
		sizeX( sizeX_ ),
		sizeY( sizeY_ ),
		validAreaMinX( validAreaMinX_ ),
		validAreaMinY( validAreaMinY_ ),
		validAreaMaxX( validAreaMaxX_ ),
		validAreaMaxY( validAreaMaxY_ ),
		offsetX( offsetX_ ),
		offsetY( offsetY_ )
	{

	} 

	void setResolution( const float resolution_  )
	{
		resolution = resolution_;
	}

	void setSizeX( const int sizeX_ )
	{
		sizeX = sizeX_;
	}
	
	void setSizeY( const int sizeY_ )
	{
		sizeY = sizeY_;
	}	
	
	void setValidAreaMinX( const int validAreaMinX_ )
	{
		validAreaMinX = validAreaMinX_;
	}

	void setValidAreaMinY( const int validAreaMinY_ )
	{
		validAreaMinY = validAreaMinY_;
	}

	void setValidAreaMaxX( const int validAreaMaxX_ )
	{
		validAreaMaxX = validAreaMaxX_;
	}
	
	void setValidAreaMaxY( const int validAreaMaxY_ )
	{
		validAreaMaxY = validAreaMaxY_;
	} 

	void setOffset( const float offsetX_, const float offsetY_ )
	{
		offsetX = offsetX_;
		offsetY = offsetY_;
	}

	const float getResolution() const 
	{
		return resolution;
	}

	const int getSizeX() const 
	{
		return sizeX;
	}

	const int getSizeY() const
	{
		return sizeY;
	}

	const int getValidAreaMinX() const 
	{
		return validAreaMinX;
	}

	const int getValidAreaMinY() const 
	{
		return validAreaMinY;
	}

	const int getValidAreaMaxX() const
	{
		return validAreaMaxX;
	}

	const int getValidAreaMaxY() const
	{
		return validAreaMaxY;
	}

	const float getOffsetX() const 
	{
		return offsetX;
	}

	const float getOffsetY() const
	{
		return offsetY;
	}

	bool allocateMap()
	{
		prob = ( float ** )calloc( sizeX, sizeof( float * ) );
		if( prob == nullptr ){
			std::cerr<<"allocate memory for the map failed ..."<<std::endl;
			return false;
		}
		for( int i = 0; i < sizeY; i ++ ){
			prob[i] = ( float * )calloc( sizeY, sizeof( float ) );
			
			if( prob[i] == nullptr ){
				std::cerr<<"allocate memory for the map failed ..."<<std::endl;
				return false;
			}
		}

		std::cerr<<"allocate the memory for the map now ..."<<std::endl;

		return true;
	}

	bool freeMap()
	{
		for( int i = 0; i < sizeY; i ++ ){
			if( prob[i] != nullptr ){
				free( prob[i] );
			}
		}

		if( prob != nullptr ){
			free( prob );
		}
	
		std::cerr<<"release the memory for the map now ..."<<std::endl;
		
		return true;
	}

	bool loadMap( const std::string &fileName )
	{
		FILE *fp;
		
		char line[256];

		// 1. open the map file
		if( ( fp = fopen( fileName.c_str(), "rt" ) ) == NULL ){
			std::cerr<<"ERROR: Cloud not open file "<<fileName<<std::endl;
			return false;
		}
		std::cerr<<"# Reading Map: "<<fileName<<std::endl;
		
		// 2. read the data 
		while( ( fgets( line, 256, fp ) != NULL ) && (strncmp("global_map[0]", line , 13) != 0) ){
			if(strncmp(line, "robot_specifications->resolution", 32) == 0){
            			//从字符串line[]中读取%d格式输入,并存入map->resolution中
            			if(sscanf(&line[32], "%g", &(resolution)) != 0){
                			printf("# Map resolution: %g cm\n", resolution);
				}
        		}
	
			if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0){
            			if(sscanf(&line[35], "%g", &(offsetX)) != 0){
                			printf("# Map offsetX: %g cm\n", offsetX);
            			}        
			}

			if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0){
           	 		if (sscanf(&line[35], "%g", &(offsetY)) != 0){
                			printf("# Map offsetY: %g cm\n", offsetY);
            			}
        		}

		}

		if( sscanf( line, "global_map[0]: %d %d", &sizeY, &sizeX ) != 2 ) { //如果成功则返回成功匹配和赋值的个数
        		fprintf(stderr, "ERROR: corrupted file %s\n", fileName.c_str());
        		fclose(fp);
        		return false;
    		}

    		printf("# Map size: %d %d\n", sizeX, sizeY);

	
		//
		allocateMap();		

		validAreaMinX = sizeX;
		validAreaMaxX = 0;
		validAreaMinY = sizeY;
		validAreaMaxY = 0;
	
		int count = 0;
		float temp = 0;
		for( int x = 0; x < sizeX; x ++ ){
			for( int y = 0; y < sizeY;y ++, count ++ ){
				fscanf(fp,"%e", &temp);         //每次读取地图一个像素点的概率值
				if( temp < 0.0 ){
					prob[x][y] = -1;
				}		
				else {
					if( x < validAreaMinX ){
						validAreaMinX = x;
					}
					else if( x > validAreaMaxX ){
						validAreaMaxX = x;
					}

					if( y < validAreaMinY ){
						validAreaMinY = y;
					}	
					else if( y > validAreaMaxY ){
						validAreaMaxY = y;
					}

					prob[x][y] = temp;
				}
			}
		}

		fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",count / (float)(sizeX * sizeY) * 100);
		fclose( fp );
		
		return true;
	}

	void displayMap()
	{
		cv::Mat image = cv::Mat( sizeX, sizeY, CV_8UC3, cv::Scalar::all(125) );
        	for( int x = 0; x < sizeX; x ++ ){
                	for( int y = 0; y < sizeY; y ++ ){
                        	//image.at<uchar>( x, y ) = (uint8_t)(this->map->prob[x][y] * 255);
                        	uint8_t v = (uint8_t)(this->prob[x][y] * 255);
                        	cv::Vec3b p;
                       	 	p[0] = v;
                        	p[1] = v;
                        	p[2] = v;

                        	image.at<cv::Vec3b>(x, y) = p;
                	}
        	}

        	cv::imshow( "map", image );
	}

	const float getProb( const int x, const int y )
	{
		return prob[x][y];
	}

private:
	float resolution = 0.1;
	int sizeX = 1000;
	int sizeY = 1000;
	int validAreaMinX = 0;
	int validAreaMinY = 0;
	int validAreaMaxX = 0;
	int validAreaMaxY = 0;
	
	float offsetX = 0.0f;
	float offsetY = 0.0f;
	
	float **prob;
};

#endif











