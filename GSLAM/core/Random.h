#ifndef RANDOM_H
#define RANDOM_H

#include <cstdlib>

namespace GSLAM {

class Random
{
public:
    Random();

    /**
     * Returns a random int in the range [min..max]
     * @param min
     * @param max
     * @return random int in [min..max]
     */
    static int RandomInt(int min, int max);

    /**
     * Returns a random number in the range [0..1]
     * @return random T number in [0..1]
     */
    template <class T>
    static T RandomValue(){
        return (T)rand()/(T)RAND_MAX;
    }

    /**
     * Returns a random number in the range [min..max]
     * @param min
     * @param max
     * @return random T number in [min..max]
     */
    template <class T>
    static T RandomValue(T min, T max){
        return Random::RandomValue<T>() * (max - min) + min;
    }

    /**
     * Returns a random number from a gaussian distribution
     * @param mean
     * @param sigma standard deviation
     */
    template <class T>
    static T RandomGaussianValue(T mean=0., T sigma=1.)
    {
    // Box-Muller transformation
    T x1, x2, w, y1;

    do {
      x1 = (T)2. * RandomValue<T>() - (T)1.;
      x2 = (T)2. * RandomValue<T>() - (T)1.;
      w = x1 * x1 + x2 * x2;
    } while ( w >= (T)1. || w == (T)0. );

    w = sqrt( ((T)-2.0 * log( w ) ) / w );
    y1 = x1 * w;

    return( mean + y1 * sigma );
    }

};

inline Random::Random()
{

}

inline int Random::RandomInt(int min, int max){
    int d = max - min + 1;
    return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}

}//end of namespace
#endif // RANDOM_H
