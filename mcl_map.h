#ifndef __MCL_MAP_H_
#define __MCL_MAP_H_


#include <iostream>
#include <vector>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "KDTree.hpp"

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
		std::ifstream input_file;
		input_file.open( fileName, std::ifstream::in );

        	if( !input_file.is_open() ){
                	std::cout<<"Failed to open the simulation file ..."<<std::endl;
	                return false;
        	}

	        std::cout<<"............Open the Simulation File ............."<<std::endl;
		std::string line;
		for( int i = 0; i < 6; i ++ ){
			std::getline( input_file, line );
			std::istringstream iss( line );
			
			std::string tag;
			
			iss >> tag;
			
			std::string num;	
			if( tag.compare( "sizeX" ) == 0 ){
				iss >> num;	
				std::cout<<"size x = "<<num<<std::endl;
				sizeX = std::stof( num );
			}			
			if( tag.compare( "sizeY" ) == 0 ){
                                iss >> num;
                                std::cout<<"size y = "<<num<<std::endl;
                                sizeY = std::stof( num );
                        }
			if( tag.compare( "centerX" ) == 0 ){
                                iss >> num;
                                std::cout<<"centerX = "<<num<<std::endl;
                                //sizeX = std::stof( num );
                        }
			if( tag.compare( "centerY" ) == 0 ){
                                iss >> num;
                                std::cout<<"centerY = "<<num<<std::endl;
                                //sizeX = std::stof( num );
                        }
			if( tag.compare( "cellLength" ) == 0 ){
                                iss >> num;
                                std::cout<<"cellLength = "<<num<<std::endl;
                                resolution = std::stof( num );
                        }

		}
		//
		allocateMap();		

		validAreaMinX = sizeX;
		validAreaMaxX = 0;
		validAreaMinY = sizeY;
		validAreaMaxY = 0;
	
		int count = 0;
		float temp = 0;

		std::getline( input_file, line );
		std::istringstream iss( line );

		for( int x = 0; x < sizeX; x ++ ){
			for( int y = 0; y < sizeY;y ++, count ++ ){
				std::string num;
				iss >> num;
				float temp = std::stof( num );
				
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

		fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",count / (float)(sizeX * sizeY) * 100);
		
		input_file.close();

		return true;
	}

	void displayMap()
	{
		cv::Mat image = cv::Mat( sizeX, sizeY, CV_8UC3, cv::Scalar::all(125) );
        	for( int x = 0; x < sizeX; x ++ ){
                	for( int y = 0; y < sizeY; y ++ ){
                        	//image.at<uchar>( x, y ) = (uint8_t)(this->map->prob[x][y] * 255);
                        	//uint8_t v = (uint8_t)(this->prob[x][y] * 255);
                        	if( this->prob[x][y] > 0.0 ){
					cv::Vec3b p;
                       	 		p[0] = 0;
                        		p[1] = 0;
                        		p[2] = 0;
				
                        		image.at<cv::Vec3b>(x, y) = p;
				}
				else if( this->prob[x][y] < 0.0 ){
					cv::Vec3b p;
                                        p[0] = 255;
                                        p[1] = 255;
                                        p[2] = 255;

                                        image.at<cv::Vec3b>(x, y) = p;
				}
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











