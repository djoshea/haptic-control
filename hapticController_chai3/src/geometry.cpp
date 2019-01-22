#include <math.h>
#include "math/CVector3d.h"
#include "geometry.h"

//-----------------------------------------------------------------------
/*!
    Decompose me into components parallel and perpendicular to the input
    vector.

    \param    a_input         Reference vector.
    \param    a_parallel      Parallel component
    \param    a_perpendicular Perpendicular component
*/
//-----------------------------------------------------------------------
void decomposeVector3d(const cVector3d& a_this, const cVector3d& a_input, cVector3d& a_parallel, cVector3d& a_perpendicular)
{
  double scale = (a_this.dot(a_input) / (a_input.dot(a_input)));
  a_parallel = a_input;
  a_parallel.mul(scale);
  a_this.subr(a_parallel,a_perpendicular);
}

void printVector(const cVector3d& a_this, const unsigned int a_precision)
{
    std::string s = a_this.str(a_precision);
    printf("%s\n",s.c_str());
}
