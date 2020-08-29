#include <iostream>
#include <math.h>
#include <tuple>
#include <Eigen/Core> // Eigen Library
#include <Eigen/LU> // Eigen Library

using namespace std;
using namespace Eigen;

float measurements[3] = { 1, 2, 3 };

tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf u, MatrixXf F, MatrixXf H, MatrixXf R, MatrixXf I)
{
    for (int n = 0; n < sizeof(measurements) / sizeof(measurements[0]); n++) {
        //****** TODO: Kalman-filter function********//

        /*
            Given: 
            x: prior state 
            P: prior uncertainty 
            u: external motion
            F: state translation function
            H: state measurement function 
            R: measurement uncertainity
            I: identity matrix
            measurements at len(measurements) time steps

            Want: 
            y, S: measurement update (mean~state, cov)
            K: kalman gain 
            x, P: posterior (mean~state, cov)

            Note: var_dash correspond to estimated variables 

        */

        // 1. Measurement Update
        // Initialize and Compute Z, y, S, K, x, and P
        MatrixXf Z(1, 1);
        MatrixXf y(1, 1);
        MatrixXf S(1, 1);
        MatrixXf K(2, 1);

        Z << measurements[n];

        y << Z - (H * x);
        S << H * P * H.transpose() + R;

        // Kalman gain 
        K << P * H.transpose() * S.inverse();

        // Posterior from measurement update
        x << x + K * y;
        P << (I - K * H) * P;

        // 2. State Prediction
		MatrixXf x_dash(2, 1);
        MatrixXf P_dash(2, 2);

        // Compute x and P
        x_dash << F * x + u;

        // Q = [0] assuming gaussian noise 
        P_dash << (F * P * F.transpose()); 

        // Prior for next step = updated Posterior
        x << x_dash; P << P_dash;
        
    }

    return make_tuple(x, P);
}

int main()
{

    MatrixXf x(2, 1);// Initial state (location and velocity) 
    x << 0,
    	 0; 
    MatrixXf P(2, 2);//Initial Uncertainty
    P << 100, 0, 
    	 0, 100; 
    MatrixXf u(2, 1);// External Motion
    u << 0,
    	 0; 
    MatrixXf F(2, 2);//Next State Function
    F << 1, 1,
    	 0, 1; 
    MatrixXf H(1, 2);//Measurement Function
    H << 1,
    	 0; 
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1, 0,
    	 0, 1; 

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
    cout << "x= " << x << endl;
    cout << "P= " << P << endl;

    return 0;
}