#pragma once

#include <queue>

class Simple_Moving_Average {
  public:
    //Empty constructor
    Simple_Moving_Average();

    //Set the period for this object
    void set_period(int period);
  
    //Add new data in the list and update the sum so that we get the new mean
    void add_data(double num);

    void clear();
    //Calculate the mean
    double get_mean();

  private:
    //Queue that stores the data used to get the average
    std::queue<double> dataset;
    
    int period;
    double sum;
};