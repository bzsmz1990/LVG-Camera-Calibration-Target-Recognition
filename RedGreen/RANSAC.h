#include <vector>
/**
* This class defines the interface for parameter estimators.
* Classes which inherit from it can be used by the RanSac class to perfrom robust
* parameter estimation.
* The interface includes three methods:
*                           1.estimate() - Estimation of the parameters using the minimal
*                                          amount of data (exact estimate).
*                           2.leastSquaresEstimate() - Estimation of the parameters using
*                                                      more than the minimal amount of data, so that the estimate
*                                                      minimizes a least squres error criteria.
*                           3.agree() - Does the given data agree with the model parameters.
*
* Author: Ziv Yaniv (zivy@cs.huji.ac.il)
*/

template<class T, class S>
class ParameterEsitmator {
public:	
	/**
	* Exact estimation of parameters.
	* @param data The data used for the estimate.
	* @param parameters This vector is cleared and then filled with the computed parameters.
	*/
	virtual void estimate( std::vector<T *> &data, std::vector<S> &parameters ) = 0;


	/**
	* Least squares estimation of parameters.
	* @param data The data used for the estimate.
	* @param parameters This vector is cleared and then filled with the computed parameters.
	*/
	virtual void leastSquaresEstimate( std::vector<T *> &data, std::vector<S> &parameters ) = 0;

	/**
	* This method tests if the given data agrees with the given model parameters.
	*/
	virtual bool agree(std::vector<S> &parameters, T &data) = 0;


	virtual double medianResidual( std::vector<T *> &data, std::vector<S> &parameters ) = 0;
};


/**
* This class implements the Random Sample Consensus (Ransac) framework,
* a framework for robust parameter estimation.
* Given data containing outliers we estimate the model parameters using sub-sets of
* the original data:
* 1. Choose the minimal subset from the data for computing the exact model parameters.
* 2. See how much of the input data agrees with the computed parameters.
* 3. Goto step 1. This can be done up to (m choose N) times, where m is the number of
*    data objects required for an exact estimate and N is the total number of data objects.
* 4. Take the largest subset of objects which agreed on the parameters and compute a
*    least squares fit using them.
* 
* This is based on:
* Martin A. Fischler, Robert C. Bolles, 
* ``Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography'', 
* Communications of the ACM, Vol. 24(6), 1981.
*
* Richard I. Hartely, Andrew Zisserman, "Multiple View Geometry in Computer Vision", Cambridge University Press, 2000.
*
* The class template parameters are T - objects used for the parameter estimation (e.g. Point2D).
*                                   S - type of parameter (e.g. double).                          
*
* Author: Ziv Yaniv (zivy@cs.huji.ac.il)
*/
template<class T, class S>
class Ransac {

public:
	/**
	* Estimate the model parameters using the Ransac framework.
	* @param parameters A vector which will contain the estimated parameters.
	*                   If there is an error in the input then this vector will be empty.
	*                   Errors are: 1. Less data objects than required for an exact fit.
	* @param paramEstimator An object which can estimate the desired parameters using either an exact fit or a 
	*                       least squares fit.
	* @param data The input from which the parameters will be estimated.
	* @param numForEstimate The number of data objects required for an exact fit.
	* @param desiredProbabilityForNoOutliers The probability that at least one of the selected subsets doesn't contain an
	*                                        outlier.
	* @param maximalOutlierPercentage The maximal expected percentage of outliers.
	* @return Returns the percentage of data used in the least squares estimate.
	*/
	static double compute(std::vector<S> &parameters, 
		ParameterEsitmator<T,S> *paramEstimator , 
		std::vector<T> &data, 
		int numForEstimate, 
		double desiredProbabilityForNoOutliers,
		double maximalOutlierPercentage, short *bestVotes );


	/**
	* Estimate the model parameters using the maximal consensus set by going over ALL possible
	* subsets (brute force approach).
	* Given: n -  data.size()
	*        k - numForEstimate
	* We go over all n choose k subsets       n!
	*                                     ------------
	*                                      (n-k)! * k!
	* @param parameters A vector which will contain the estimated parameters.
	*                   If there is an error in the input then this vector will be empty.
	*                   Errors are: 1. Less data objects than required for an exact fit.
	* @param paramEstimator An object which can estimate the desired parameters using either an exact fit or a 
	*                       least squares fit.
	* @param data The input from which the parameters will be estimated.
	* @param numForEstimate The number of data objects required for an exact fit.
	* @return Returns the percentage of data used in the least squares estimate.
	*
	* NOTE: This method should be used only when n choose k is small (i.e. k or (n-k) are approximatly equal to n)
	*
	*/
	static double compute(std::vector<S> &parameters, 
		ParameterEsitmator<T,S> *paramEstimator , 
		std::vector<T> &data, 
		int numForEstimate);

private:

	static void computeAllChoices(ParameterEsitmator<T,S> *paramEstimator, std::vector<T> &data, int numForEstimate,
		short *bestVotes, short *curVotes, int &numVotesForBest, int startIndex, int n, int k, int arrIndex, int *arr);

	static void estimate(ParameterEsitmator<T,S> *paramEstimator, std::vector<T> &data, int numForEstimate,
		short *bestVotes, short *curVotes, int &numVotesForBest, int *arr);

	class SubSetIndexComparator {
	private:
		int m_length;
	public:
		SubSetIndexComparator(int arrayLength) : m_length(arrayLength){}
		bool operator()(const int *arr1, const int *arr2) const {
			for(int i=0; i<m_length; i++)
			{
				if(arr1[i] < arr2[i])
				{
					return true;
				}
				else if (arr1[i] == arr2[i])
				{
					if (i+1 == m_length)
						return false;

					continue ;
				}
				else
				{
					return false;
				}
			}		
		}
	};

};


/*******************************Ransac Implementation*************************/

template<class T, class S>
double Ransac<T,S>::compute(std::vector<S> &parameters, 
	ParameterEsitmator<T,S> *paramEstimator , 
	std::vector<T> &data, 
	int numForEstimate, 		                     
	double desiredProbabilityForNoOutliers,
	double maximalOutlierPercentage, short *bestVotes)
{
	int numDataObjects = data.size();
	//there are less data objects than the minimum required for an exact fit, or
	//all the data is outliers?
	if(numDataObjects < numForEstimate || maximalOutlierPercentage>=1.0) 
		return 0;

	std::vector<T *> exactEstimateData;
	std::vector<T *> leastSquaresEstimateData;
	std::vector<S> exactEstimateParameters;
	int i, j, k, l, numVotesForBest, numVotesForCur, maxIndex, numTries;
	short *curVotes = new short[numDataObjects];  //one if data[i] agrees with the current model, otherwise zero
	short *notChosen = new short[numDataObjects]; //not zero if data[i] is NOT chosen for computing the exact fit, otherwise zero
	SubSetIndexComparator subSetIndexComparator(numForEstimate);
	std::set<int *, SubSetIndexComparator > chosenSubSets(subSetIndexComparator);
	int *curSubSetIndexes;
	double dBestMedRes, dCurMedRes;
	double outlierPercentage = maximalOutlierPercentage;
	double numerator = log(1.0-desiredProbabilityForNoOutliers);
	double denominator = log(1- pow(1-maximalOutlierPercentage, numForEstimate));

	parameters.clear();

	numVotesForBest = -1; //initalize with -1 so that the first computation will be set to best
	srand((unsigned)time(NULL)); //seed random number generator
	numTries = (int)(numerator/denominator + 0.5);

	//===========================================================================
	__int64 nTemp1 = 1;
	__int64 nTemp2 = 1;

	for ( int i = 1; i <= numForEstimate; ++i )
		nTemp1 *= i;

	for ( int i = numDataObjects; i > numDataObjects - numForEstimate; --i )
	{
		nTemp2 *= i;

		if ( nTemp2 >= nTemp1 * numTries )	break;
	}

	__int64 nMaxTriesNum = nTemp2 / nTemp1;
	//===========================================================================


	for(i=0; i<numTries; i++) {

		//randomly select data for exact model fit ('numForEstimate' objects).
		memset(notChosen,'1',numDataObjects*sizeof(short));
		curSubSetIndexes = new int[numForEstimate];

		exactEstimateData.clear();

		maxIndex = numDataObjects-1; 
		for(l=0; l<numForEstimate; l++) {
			//selectedIndex is in [0,maxIndex]
			int selectedIndex = (int)(((float)rand()/(float)RAND_MAX)*maxIndex + 0.5);
			for(j=-1,k=0; k<numDataObjects && j<selectedIndex; k++) {
				if(notChosen[k])
					j++;
			}
			k--;
			exactEstimateData.push_back(&(data[k]));
			notChosen[k] = 0;
			maxIndex--;
		}
		//get the indexes of the chosen objects so we can check that this sub-set hasn't been
		//chosen already
		for(l=0, j=0; j<numDataObjects; j++) {
			if(!notChosen[j]) {
				curSubSetIndexes[l] = j+1;
				l++;
			}
		}

		//check that the sub set just chosen is unique
		std::pair< std::set<int *, SubSetIndexComparator >::iterator, bool > res = chosenSubSets.insert(curSubSetIndexes);

		if(res.second == true) { //first time we chose this sub set

			//use the selected data for an exact model parameter fit
			paramEstimator->estimate(exactEstimateData,exactEstimateParameters);
			//see how many agree on this estimate
			numVotesForCur = 0;
			memset(curVotes,'\0',numDataObjects*sizeof(short));
			for(j=0; j<numDataObjects; j++) {
				if(paramEstimator->agree(exactEstimateParameters, data[j])) {
					curVotes[j] = 1;
					numVotesForCur++;
				}
			}
			if(numVotesForCur >= numVotesForBest) {
				if ( numVotesForCur == numVotesForBest )
				{
					leastSquaresEstimateData.clear();

					for(j=0; j<numDataObjects; j++) {
						if(curVotes[j])
							leastSquaresEstimateData.push_back(&(data[j]));
					}

					paramEstimator->leastSquaresEstimate(leastSquaresEstimateData,parameters);
					dCurMedRes = paramEstimator->medianResidual(leastSquaresEstimateData,parameters);

					if ( dCurMedRes < dBestMedRes )
					{
						numVotesForBest = numVotesForCur;
						memcpy(bestVotes,curVotes, numDataObjects*sizeof(short));
					}
				} 
				else
				{
					leastSquaresEstimateData.clear();

					for(j=0; j<numDataObjects; j++) {
						if(bestVotes[j])
							leastSquaresEstimateData.push_back(&(data[j]));
					}

					paramEstimator->leastSquaresEstimate(leastSquaresEstimateData,parameters);
					dBestMedRes = paramEstimator->medianResidual(leastSquaresEstimateData,parameters);

					numVotesForBest = numVotesForCur;
					memcpy(bestVotes,curVotes, numDataObjects*sizeof(short));
				}			

				if (numVotesForBest == numDataObjects )	break;
			}

			//update the estimate of outliers and the number of iterations we need
			outlierPercentage = 1 - (double)numVotesForCur/(double)numDataObjects;
			if(outlierPercentage < maximalOutlierPercentage) {
				maximalOutlierPercentage = outlierPercentage;
				denominator = log(1- pow(1-maximalOutlierPercentage, numForEstimate));
				numTries = (int)(numerator/denominator + 0.5);
			}
		}
		else {  //this sub set already appeared, don't count this iteration
			delete [] curSubSetIndexes;
			i--;

			if ( chosenSubSets.size() >= nMaxTriesNum )		break;
		}
	}

	//release the memory
	std::set<int *, SubSetIndexComparator >::iterator it = chosenSubSets.begin();
	std::set<int *, SubSetIndexComparator >::iterator chosenSubSetsEnd = chosenSubSets.end();
	while(it!=chosenSubSetsEnd) {
		delete [] (*it);
		it++;
	}
	chosenSubSets.clear();

	//compute the least squares estimate using the largest sub set
	leastSquaresEstimateData.clear();

	for(j=0; j<numDataObjects; j++) {
		if(bestVotes[j])
			leastSquaresEstimateData.push_back(&(data[j]));
	}

	paramEstimator->leastSquaresEstimate(leastSquaresEstimateData,parameters);
	dBestMedRes = paramEstimator->medianResidual(leastSquaresEstimateData,parameters);

	delete [] curVotes;
	delete [] notChosen;

	return dBestMedRes;
}
/*****************************************************************************/
template<class T, class S>
double Ransac<T,S>::compute(std::vector<S> &parameters, 
	ParameterEsitmator<T,S> *paramEstimator , 
	std::vector<T> &data, 
	int numForEstimate)
{
	std::vector<T *> leastSquaresEstimateData;
	int numDataObjects = data.size();
	int numVotesForBest = -1;
	int *arr = new int[numForEstimate];
	short *curVotes = new short[numDataObjects];  //one if data[i] agrees with the current model, otherwise zero
	short *bestVotes = new short[numDataObjects];  //one if data[i] agrees with the best model, otherwise zero


	//there are less data objects than the minimum required for an exact fit
	if(numDataObjects < numForEstimate) 
		return 0;

	computeAllChoices(paramEstimator,data,numForEstimate,
		bestVotes, curVotes, numVotesForBest, 0, data.size(), numForEstimate, 0, arr);

	//compute the least squares estimate using the largest sub set
	for(int j=0; j<numDataObjects; j++) {
		if(bestVotes[j])
			leastSquaresEstimateData.push_back(&(data[j]));
	}
	paramEstimator->leastSquaresEstimate(leastSquaresEstimateData,parameters);

	delete [] arr;
	delete [] bestVotes;
	delete [] curVotes;	

	return paramEstimator->medianResidual(leastSquaresEstimateData,parameters);
}
/*****************************************************************************/
template<class T, class S>
void Ransac<T,S>::computeAllChoices(ParameterEsitmator<T,S> *paramEstimator, std::vector<T> &data, int numForEstimate,
	short *bestVotes, short *curVotes, int &numVotesForBest, int startIndex, int n, int k, int arrIndex, int *arr)
{
	//we have a new choice of indexes
	if(k==0) {
		estimate(paramEstimator, data, numForEstimate, bestVotes, curVotes, numVotesForBest, arr);
		return;
	}

	//continue to recursivly generate the choice of indexes
	int endIndex = n-k;
	for(int i=startIndex; i<=endIndex; i++) {
		arr[arrIndex] = i;
		computeAllChoices(paramEstimator, data, numForEstimate, bestVotes, curVotes, numVotesForBest,
			i+1, n, k-1, arrIndex+1, arr);
	}

}
/*****************************************************************************/
template<class T, class S>
void Ransac<T,S>::estimate(ParameterEsitmator<T,S> *paramEstimator, std::vector<T> &data, int numForEstimate,
	short *bestVotes, short *curVotes, int &numVotesForBest, int *arr)
{
	std::vector<T *> exactEstimateData;
	std::vector<S> exactEstimateParameters;
	int numDataObjects;
	int numVotesForCur;//initalize with -1 so that the first computation will be set to best
	int j;

	numDataObjects = data.size();
	memset(curVotes,'\0',numDataObjects*sizeof(short));
	numVotesForCur=0;

	for(j=0; j<numForEstimate; j++)
		exactEstimateData.push_back(&(data[arr[j]]));
	paramEstimator->estimate(exactEstimateData,exactEstimateParameters);

	for(j=0; j<numDataObjects; j++) {
		if(paramEstimator->agree(exactEstimateParameters, data[j])) {
			curVotes[j] = 1;
			numVotesForCur++;
		}
	}
	if(numVotesForCur > numVotesForBest) {
		numVotesForBest = numVotesForCur;
		memcpy(bestVotes,curVotes, numDataObjects*sizeof(short));
	}
}


