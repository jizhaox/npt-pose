#include "construct_coeff_objective.h"

bool normalize_data(double* P, int n_obs)
{
    double x, y, z;
    for (int i = 0; i < n_obs; i++)
    {
        x = P[i*3];
        y = P[i*3+1];
        z = P[i*3+2];

        if (true) // normalization method 1: unit vector
        {
            double s = sqrt(x*x + y*y + z*z);
            P[i*3]   = x/s;
            P[i*3+1] = y/s;
            P[i*3+2] = z/s;
        }
        else // normalization method 2: the 3rd coordinate is 1
        {
            P[i*3]   = x/z;
            P[i*3+1] = y/z;
            P[i*3+2] = 1.0;
        }
    }
    return true;
}

bool construct_coeff_objective(double* C, double*P1, double*P2, int n_obs, bool is_normalize, double* weight)
{
    int sz = 9;
    for (int i = 0; i < sz*sz; i++)
    {
        C[i] = 0;
    }
    if (n_obs < 8)
    {
        return false;
    }

    // P1: points in view 1
    // P2: points in view 2
    normalize_data(P1, n_obs);
    normalize_data(P2, n_obs);

    double x1, y1, z1, x2, y2, z2;
    int idx = 0;
    double g[9];
    for (int k = 0; k < n_obs; k++)
    {
        x1 = P1[idx];
        x2 = P2[idx];
        idx++;

        y1 = P1[idx];
        y2 = P2[idx];
        idx++;
        
        z1 = P1[idx];
        z2 = P2[idx];
        idx++;

        g[0] = x1*x2;
        g[1] = x1*y2;
        g[2] = x1*z2;
        g[3] = y1*x2;
        g[4] = y1*y2;
        g[5] = y1*z2;
        g[6] = z1*x2;
        g[7] = z1*y2;
        g[8] = z1*z2;
        for (int i = 0; i < sz; i++)
        {
            for (int j = i; j < sz; j++)
            {
                double t = (g[i] * g[j]) * weight[k];
                C[i*sz+j] += t;
                if (i!=j)
                {
                    C[j*sz+i] += t;
                }
            }
        }
    }
    if (is_normalize)
    {
        double s = 0;
        double s2 = 0;
        double num = sz*sz;
        for (int i = 0; i < num; i++)
        {
            double tmp = C[i];
            s += tmp;
            s2 += tmp*tmp;
        }
        double EX = s / num;
        double EX2 = s2 / num;
        double c_std = sqrt(EX2 - EX*EX);
        for (int i = 0; i < num; i++)
        {
            C[i] = C[i] / c_std;
        }
    }

    return true;
}

void construct_coeff_objective(double* C, double*P1, double*P2, int n_obs, bool is_normalize)
{
    double* weight = new double[n_obs];
    for (int i = 0; i < n_obs; i++)
    {
        weight[i] = 1.0;
    }
    construct_coeff_objective(C, P1, P2, n_obs, is_normalize, weight);

    delete[] weight;
    return;
}
