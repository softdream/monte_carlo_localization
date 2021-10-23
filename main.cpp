#include "mcl_map.h"
#include "mcl.h"
#include "load_data.h"

int main() 
{
	std::cout<<"---------------------- MC Localization -----------------------"<<std::endl;
	
	LoadLog log( "mcl_data.txt" );
        std::vector<log_data> logData = log.getLog();

	MCL mcl;
        mcl.loadMap( "test3.map" );
        mcl.initParticles();

        mcl.displayMap();

        cv::waitKey(0);

        mcl.setLogData( logData );

        mcl.mclTest();

	return 0;
	
}
