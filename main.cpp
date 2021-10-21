#include "mcl_map.h"
#include "mcl.h"
#include "load_data.h"

int main() 
{
	std::cout<<"---------------------- MC Localization -----------------------"<<std::endl;
	
	LoadLog log( "./data/log/robotdata5.log" );
        std::vector<log_data> logData = log.getLog();

	MCL mcl;
	mcl.loadMap( "./data/map/wean.dat" );
	mcl.initParticles();

	mcl.displayMap();

	cv::waitKey(0);

	mcl.setLogData( logData );

	mcl.mclTest();

	return 0;
	
}
