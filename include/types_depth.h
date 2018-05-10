// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Modified by Raúl Mur Artal (2014)
// - Added EdgeInverseSim3ProjectXYZ 
// - Modified VertexSim3Expmap to represent relative transformation between two cameras. Includes calibration of both cameras.

#ifndef G2O_DEPTH_TYPES
#define G2O_DEPTH_TYPES

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace g2o {

  using namespace Eigen;

  /**
 * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 7d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class VertexDepth : public BaseVertex<7, Sim3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexDepth();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
      _estimate = Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
      Eigen::Map<Vector7d> update(const_cast<double*>(update_));

      if (_fix_scale)
        update[6] = 0;

      Sim3 s(update);
      setEstimate(s*estimate());
    }

    Vector2d _principle_point;
    Vector2d _focal_length;
    double _width, _height;

    Vector2d cam_map(const Vector3d & v) const
    {
      Vector2d res;
      res[0] = v[0]*_focal_length[0]/v[2] + _principle_point[0];
      res[1] = v[1]*_focal_length[1]/v[2] + _principle_point[1];
      return res;
    }


    const float* ImageD;
    const float* ImageGx;
    const float* ImageGy;
    const float* ImageInfo;

    bool _fix_scale;


  protected:
  };



 /**
 * \brief 7D edge between pointxyz and depth (YJKim)
 */
class EdgeXYZDepth : public  BaseBinaryEdge<1, double, VertexSBAPointXYZ, VertexDepth>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeXYZDepth();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexDepth* v1 = static_cast<const VertexDepth*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Vector2d Ipos( v1->cam_map(v1->estimate().map(v2->estimate())) );
      int idx = (int)(((int)Ipos[1])*v1->_width+((int)Ipos[0]));

      if (Ipos[0]>=v1->_width || Ipos[0]<0 || Ipos[1]>=v1->_height || Ipos[1]<0 )
      {
          _error<< 0.0f;
          _information<< 0.0f;
          _measurement = 0.0f;
      }
      else if(!std::isfinite(v1->ImageD[idx]))
      {
          _error<< 0.0f;
          _information<< 0.0f;
          _measurement = 0.0f;
      }
     else if(!std::isfinite(v1->ImageGx[idx]) || !std::isfinite(v1->ImageGy[idx]))
     {
          _error<< 0.0f;
          _information<< 0.0f;
          _measurement = 0.0f;

     }
      else
      {
          Matrix<double, 1, 1> e1(v1->ImageD[idx]);
          Matrix<double, 1, 1> obsz(v1->estimate().map(v2->estimate())[2]);
          _information<< v1->ImageInfo[idx];
          _error = obsz-e1;
          _measurement = 1.0f; 
//         if(_error[0]>5.0 || _error[0]<-5.0)
//         {
//           _error<< 0.0f;
//           _measurement = 0.0f;
//         } 
      }
    }


    virtual void linearizeOplus();

};



} // end namespace

#endif

