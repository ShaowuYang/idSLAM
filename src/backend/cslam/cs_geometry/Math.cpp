#include "Math.h"

#include <cstdlib>

using namespace cs_geom;

void cs_geom::randPerm(int n,std::vector<int>& p)
{
    if ((int) p.size() != n)
        p.resize(n);

    for (int i = 0; i < n; i++) {
        int j = rand() % (i + 1);
        p[i] = p[j];
        p[j] = i;
    }
}

void cs_geom::randSampleNoReplacement(int nmax, int nsamples, int* p)
{
   for (int i = 0; i < nsamples; i++) {
        bool unique = false;
        while (!unique)
        {
            p[i] = rand() % nmax;
            unique = true;
            for (int j = 0; j < i; j++)
                if (p[j] == p[i])
                    unique = false;
        }
    }
}


void cs_geom::randSampleNoReplacement(int nmax, int nsamples, std::vector<int>& p)
{
    if ((int) p.size() != nsamples)
        p.resize(nsamples);

    for (int i = 0; i < nsamples; i++) {
        bool unique = false;
        while (!unique)
        {
            p[i] = rand() % nmax;
            unique = true;
            for (int j = 0; j < i; j++)
                if (p[j] == p[i])
                    unique = false;
        }
    }
}
