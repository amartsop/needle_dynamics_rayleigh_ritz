#include <iostream>
#include <armadillo>
#include <vector>
#include "include/gnuplot-iostream.h"

int main(int argc, char *argv[])
{
    arma::fvec x = arma::linspace<arma::fvec>(0, 10, 100);
    arma::fvec y = arma::linspace<arma::fvec>(0, 20, 100);

    Gnuplot gp;

    arma::fmat xy = arma::join_horiz(x, y);
    std::cout << xy << std::endl;

	gp << "plot" << gp.file1d(xy) << "with lines" << std::endl; 
}