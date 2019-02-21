void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int temp;
	int sumBinElements;
	float largestAverage;
	float outputAngle;
	int numValidPoints;
	int numElementsInBin = 5;
	
	int numDataPoints = scan->scan_time / scan->time_increment;
	int numAngles = 13;
	int angles[numAngles] = {
		-30, -25, -20, -15, -10,
		-5, 0, 5, 10, 15, 20, 
		25, 30
	};
	float dataBins[numAngles-1][numElementsInBin]; 
	
	// clear all elements in bins
	for(int i=0; i<numAngles-1; i++){ // set all bin elements to 0
		for(int j=0; j<5; j++){
			dataBins[i][j] = 0;
		}
	}
	
	// go thru each data point and put into bins
	for(int i=150; i<211; i++){
		float degree = -210+i;
		float distance = scan->ranges[i];
		
		if(!isinf(distance)){                                       //ignore any data points where distance value is inf
			for(int bin=0; bin<numAngles-1; bin++){                 //check element against each bin
				if(degree>angles[bin] && degree<=angles[bin+1]){	// if element fits in current bin
					temp=0;
					while(dataBins[bin][temp]){                     // place element in first non-zero element of current bin
						temp++;
					}
					dataBins[bin][temp] = distance;
				}
			}
		}
	} 
	
	// find largest bin average
	largestAverage = 0;
	for(int bin=0; bin<numAngles-1; bin++){
		numValidPoints = 0;
		sumBinElements = 0;
		for(int i=0; i<numElementsInBin; i++){
			sumBinElements = sumBinElements + dataBins[bin][i]; // running sum of bin elements
			if(dataBins[bin][i] != 0){    // keep track of non-zero elements in bin
				numValidPoints++;
			}
		}
		if( (((float)sumBinElements/(float)numValidPoints)>largestAverage) && (numValidPoints>1) ){
			outputAngle = angles[bin] + 2.5;
		}
	}
	
	
}
