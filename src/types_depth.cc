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

#include "types_depth.h"

namespace g2o {

  VertexDepth::VertexDepth() : BaseVertex<7, Sim3>()
  {
    _marginalized=false;
    _fix_scale = false;
  }


  bool VertexDepth::read(std::istream& is)
  {
    Vector7d cam2world;
    for (int i=0; i<6; i++){
      is >> cam2world[i];
    }
    is >> cam2world[6];
//    if (! is) {
//      // if the scale is not specified we set it to 1;
//      std::cerr << "!s";
//      cam2world[6]=0.;
//    }

    for (int i=0; i<2; i++)
    {
      is >> _focal_length[i];
    }
    for (int i=0; i<2; i++)
    {
      is >> _principle_point[i];
    }

    setEstimate(Sim3(cam2world).inverse());
    return true;
  }

  bool VertexDepth::write(std::ostream& os) const
  {
    Sim3 cam2world(estimate().inverse());
    Vector7d lv=cam2world.log();
    for (int i=0; i<7; i++){
      os << lv[i] << " ";
    }
    for (int i=0; i<2; i++)
    {
      os << _focal_length[i] << " ";
    }
    for (int i=0; i<2; i++)
    {
      os << _principle_point[i] << " ";
    }
    return os.good();
  }

  bool EdgeXYZDepth::read(std::istream& is)
  {
    return true;
  }

  bool EdgeXYZDepth::write(std::ostream& os) const
  {
    return true;
  }

 

  void EdgeXYZDepth::linearizeOplus()
  {
    VertexDepth* vj = static_cast<VertexDepth*>(_vertices[1]);
    Sim3 T = vj->estimate();
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d xyz = vi->estimate();
    Vector3d xyz_trans = T.map(xyz);

    Vector2d Ipos(vj->cam_map(xyz_trans));
    int idx = (int)(((int)Ipos[1])*vj->_width+((int)Ipos[0]));


    if (_measurement<0.1f)
    {
//      _error<< 0.0f;
      _jacobianOplusXi << 0,0,0;
      _jacobianOplusXj<<0,0,0,0,0,0,0;
//      _information<<0.0f;
    }
    else
    {

//        std::cout<<_error[0]<<std::endl;
        Matrix<double,1,2> D_u;
        Matrix<double,2,4> K_p;
        Matrix<double,4,6> p_note;

        D_u(0,0) = vj->ImageGx[idx];
        D_u(0,1) = vj->ImageGy[idx];


        K_p(0,0) = vj->_focal_length[0]/xyz_trans[2];
        K_p(0,1) = 0;
        K_p(0,2) = -vj->_focal_length[0]*xyz_trans[0]/(xyz_trans[2]*xyz_trans[2]);//0;//vj->_principle_point1[0]/xyz_trans[2];//0;////
        K_p(0,3) = 0;//vj->_principle_point1[0];//
        K_p(1,0) = 0;
        K_p(1,1) = vj->_focal_length[1]/xyz_trans[2];
        K_p(1,2) = -vj->_focal_length[1]*xyz_trans[1]/(xyz_trans[2]*xyz_trans[2]);//0;//vj->_principle_point1[1]/xyz_trans[2];//0;////
        K_p(1,3) = 0;//vj->_principle_point1[1];//


        Matrix<double,4,6> Tp_note;
        Tp_note.block<3,3>(0,0) = -1*skew(xyz_trans);//Matrix3d::Zero();//
        Tp_note.block<3,3>(0,3) = 1*Matrix3d::Identity();
        Tp_note(3,0) = 0;
        Tp_note(3,1) = 0;
        Tp_note(3,2) = 0;
        Tp_note(3,3) = 0;
        Tp_note(3,4) = 0;
        Tp_note(3,5) = 0;


        Matrix<double,1,4> dm;
        dm<<0,0,1.0f,0;
        _jacobianOplusXi << 0,0,0;
        _jacobianOplusXj.block<1,6>(0,0) = (dm-D_u*K_p)*Tp_note;//-D_u*K_p*Tp_note;//
         
    }



  }





} // end namespace
