#ifndef _MATH_H_
#define _MATH_H_

#include <vector>

namespace cs_geom {

/*!
* \brief Compute a random permutation of the numbers 1..n
* \param n The number of elements.
* \param p Output: The vector to be filled with the permutation.
*/
void randPerm(int n,std::vector<int>& p);

/*!
* \brief Randomly sample nsamples ints from [1..nmax] without replacement and
*        store them in the provided array.
* \param nmax     The maximum number to sample.
* \param nsamples How many numbers to sample.
* \param p        Output: The array to be filled with random samples.
*/
void randSampleNoReplacement(int nmax, int nsamples, int* p);

/*!
* \brief Randomly sample nsamples ints from [1..nmax] without replacement and
*        store them in the provided array.
* \param nmax     The maximum number to sample.
* \param nsamples How many numbers to sample.
* \param p        Output: The vector to be filled with random samples.
*/
void randSampleNoReplacement(int nmax, int nsamples, std::vector<int>& p);

}

#endif /* _MATH_H_ */
