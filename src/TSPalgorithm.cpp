
#include "TSPalgorithm.h"
#include "list_customers.h"
#include <iostream>
#include <functional>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <random>
#include <chrono>
#include <vector>  
#include<limits>
#include <cstdlib>
#include <climits>

using namespace std;

TSPalgorithm::TSPalgorithm(void)
{
}


TSPalgorithm::~TSPalgorithm(void)
{
}


//Set up TSP problem to run
void TSPalgorithm::Initialize( //const int& iter,
							   CoordMatrix* mat )
{
	// Set the matrix pointer for Tour object
	T.SetMatrix( mat );
    T_drone.SetMatrix(mat);
    T_vehicle.SetMatrix(mat);
    T_opt_vehicle.SetMatrix(mat);
    newTour.SetMatrix(mat);
    T_opt_drone.resize(m);
    for (int i = 0; i < m; i++)
    {
        T_opt_drone.at(i).SetMatrix(mat);
     
    }

	
	//iterations = iter;
}

// void TSPalgorithm::SetupMatrix(CoordMatrix *mat )
// {
// 	for (int i = 0; i < NumberOfParticles; i++)
// 	{
// 		for (int j = 0; j <NumberOfSalesman; j++) 
// 		{
// 			A.AllSolution.at(i).Solution.at(j).SetMatrix(mat);
// 		}
// 	}

// }

default_random_engine dre (chrono::steady_clock::now().time_since_epoch().count());     // provide seed
int random (int lim)
{
    uniform_int_distribution<int> uid {1,lim};   // help dre to generate nos from 0 to lim (lim included);
    return uid(dre);    // pass dre as an argument to uid to generate the random no
}

void TSPalgorithm::TwoOptSwap(ListOfCustomers &tour, const int& i, const int& k){
 int size = tour.GetTourSize();
    // 1. take route[0] to route[i-1] and add them in order to new_route
	for ( int c = 0; c <= i - 1; ++c )
	{
		newTour.SetCity( c, tour.GetCity( c ) );
	}
    
	// 2. take route[i] to route[k] and add them in reverse order to new_route
	int dec = 0;
	for ( int c = i; c <= k; ++c )
	{
		newTour.SetCity( c, tour.GetCity( k - dec ) );
		dec++;
	}

    // 3. take route[k+1] to end and add them in order to new_route
	for ( int c = k + 1; c < size; ++c )
	{
		newTour.SetCity( c, tour.GetCity( c ) );
	}
}

/*******************
 ** Name: TwoOpt optimization algorithm.
 ** Description: Optimizes a solution to the TSP problem.
 **    Swaps edges in the solution to make improvements until no further improvements can be made.
 ** Recieves: A solution to the TSP problem.
 ** Returns: Returns the total distance of the new solution. Also changes the input tour to match the new solution.
 ******************/
void TSPalgorithm::TwoOpt(ListOfCustomers &tour){

    //Add first city to the end to complete the cycle.
    //tour.cities.push_back(tour.cities[0]);
	tour.cities.push_back(0);
    int size = tour.GetTourSize() - 1;
	


    //This do-while loops swaps edges in the solution until no improvement can be made via the swapping mechanism.
    newTour = tour;

	// repeat until no improvement is made 
	int improve = 0;

	while ( improve < 100 )
	{
		double best_distance = tour.Tour_VehicleCost();
       
		for ( int i = 0; i < size - 1; i++ ) 
		{
			for ( int k = i + 1; k < size; k++) 
			{
				TwoOptSwap( tour,i, k );

				double new_distance = newTour.Tour_VehicleCost();

				if ( new_distance < best_distance ) 
				{	

					// Improvement found so reset
					improve = 0;
					tour = newTour;
					best_distance = new_distance;
					// for (int i = 0; i < tour.GetTourSize(); i++) 
					// {
					// 	std::cout << tour.cities.at(i) << " " ; 
					// }
					// std::cout << std::endl;

				}
			}
		}

		improve ++;
	}
    //Remove last city to obtain proper solution format.
   tour.cities.pop_back();

   
}


void TSPalgorithm::CreateSolution()
{
	   
    // readTSP file
    matrix.Initialize("/home/quanghuy205/truck_uav_sal/tsp_data/att48.tsp");
    
   std::cout << "Problem name: "<<  matrix.GetFileTitle() << std::endl;
   // Initialize algorithm
   Initialize(&matrix);

}

void TSPalgorithm::test()
{   


    matrix.Initialize("/home/quanghuy205/2_opt/tsp_data/att48.tsp");
 	// std::string filename = matrix.GetFileTitle();

    matrix.SetVehicleCostMatrix();

    n = matrix.coords.size() - 1;
  
    Initialize(&matrix);
    T.CreateNearestNeighbourTour();
    std::cout << T.Tour_VehicleCost() << std::endl;
    T.printTour();
    TwoOpt(T);
    // TwoOpt(T);
    // TwoOpt(T);
    // TwoOpt(T);
    T.printTour();
    std::cout << T.Tour_VehicleCost() << std::endl;
}



	
		
