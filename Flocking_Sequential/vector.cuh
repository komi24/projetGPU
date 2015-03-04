#ifndef VECTOR
#define VECTOR

#include <iostream>
#include <cmath>
#include "types.hxx"
#include <limits>
#include <cuda.h>

#define EPSILON 0.000001

class Vector
{
  public:
    Real x, y ,z;

    // Default constructor
   __device__ __host__  Vector(){
    this->x =0;
    this->y =0;
    this->z =0;
   }

    // Constructor from three real numbers
   __device__ __host__  Vector(Real x0, Real y0, Real z0){
      this->x = x0; this->y = y0; this->z = z0;
    }

    // Operators
   __device__ __host__   Vector operator+( const Real& r) const {
      return Vector( x + r, y + r, z + r);
    }

   __device__ __host__  Vector operator+( const Vector& rhs ) const {
      return Vector( x + rhs.x, y + rhs.y, z + rhs.z );
    }

   __device__ __host__  Vector& operator+=( const Vector& rhs ) {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
      return *this;
    }

   __device__ __host__  Vector operator-( const Vector& rhs ) const {
      return Vector( x - rhs.x, y - rhs.y, z - rhs.z );
    }

   __device__ __host__  Vector& operator-=( const Vector& rhs ) {
      x -= rhs.x;
      y -= rhs.y;
      z -= rhs.z;
      return *this;
    }

   __device__ __host__  Vector operator*( Real s ) const {
      return Vector( x * s, y * s, z * s );
    }

   __device__ __host__  Vector& operator*=( Real s ) {
      x *= s;
      y *= s;
      z *= s;
      return *this;
    }

   __device__ __host__  Vector operator/( Real s ) const {
      Real inv = (s>EPSILON) ? 1.0 / s : 2;
      return Vector( x * inv, y * inv, z * inv );
    }

   __device__ __host__  Vector& operator/=( Real s ) {
      Real inv = (s>EPSILON) ? 1.0 / s : 2;
      x *= inv;
      y *= inv;
      z *= inv;
      return *this;
    }

   __device__ __host__  Vector operator-() const {
      return Vector( -x, -y, -z );
    }

   __device__ __host__  bool operator==( const Vector& rhs ) const {
      Real absx = (x - rhs.x > 0) ? x - rhs.x : rhs.x - x;
      Real absy = (y - rhs.y > 0) ? y - rhs.y : rhs.y - y;
      Real absz = (z - rhs.z > 0) ? z - rhs.z : rhs.z - z;
      return absx<EPSILON && absy<EPSILON && absz<EPSILON;
    }

   __device__ __host__  bool operator!=( const Vector& rhs ) const {
      return !operator==( rhs );
    }
  __device__ __host__ bool operator>( const Vector& rhs ) const {
      return (x > rhs.x) && (y > rhs.y) && (z >rhs.z);
    }

    __device__ __host__ bool operator>=( const Vector& rhs ) const {
      return (x >= rhs.x) && (y >= rhs.y) && (z >=rhs.z);
    }

    __device__ __host__ Real norm() {
      return sqrt(x * x + y * y + z * z);
    }

    __device__ __host__ Vector normalized(){
      float inorm = (this->norm() > EPSILON) ? 1./this->norm() : 2;
    
      return Vector(x*inorm,y*inorm,z*inorm);
    }

    __device__ __host__ void normalize(){
      float inorm = (this->norm() > EPSILON) ? 1./this->norm() : 2;
      x*=inorm;y*=inorm;z*=inorm;
    }

};

Vector& Zeros();
__device__ __host__ Vector operator*( Real s, Vector &u);
std::ostream &operator<< (std::ostream &stream, const Vector & u);

#endif
