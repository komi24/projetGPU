#include "vector.cuh"

Vector& Zeros()
{
  static Vector u(0.,0.,0.);
  return u;
}

__device__ __host__ Vector operator*( Real s, Vector &u) {
  return u*s;
}
std::ostream &operator<< (std::ostream &stream, const Vector & u){
  stream<<u.x<<" "<<u.y<<" "<<u.z<<std::endl;
  return stream;
}
