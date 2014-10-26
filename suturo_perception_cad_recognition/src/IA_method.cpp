#include <suturo_perception_cad_recognition/IA_method.h>

using namespace suturo_perception;

Eigen::Matrix< float, 4, 4 > IAMethod::rotateAroundCrossProductOfNormals(
    Eigen::Vector3f base_normal,
    Eigen::Vector3f normal_to_rotate,
    bool store_transformation)
{
  normal_to_rotate *= -1; // The model is standing upside down, rotate the normal by 180 DEG
  float costheta = normal_to_rotate.dot(base_normal) / (normal_to_rotate.norm() * base_normal.norm() );

  Eigen::Vector3f axis;
  Eigen::Vector3f firstAxis = normal_to_rotate.cross(base_normal);
  firstAxis.normalize();
  axis=firstAxis;
  float c = costheta;
  std::cout << "rotate COSTHETA: " << acos(c) << " RAD, " << ((acos(c) * 180) / M_PI) << " DEG" << std::endl;
  float s = sqrt(1-c*c);
  float CO = 1-c;

  float x = axis(0);
  float y = axis(1);
  float z = axis(2);

  Eigen::Matrix< float, 4, 4 > rotationBox;
  rotationBox(0,0) = x*x*CO+c;
  rotationBox(1,0) = y*x*CO+z*s;
  rotationBox(2,0) = z*x*CO-y*s;

  rotationBox(0,1) = x*y*CO-z*s;
  rotationBox(1,1) = y*y*CO+c;
  rotationBox(2,1) = z*y*CO+x*s;

  rotationBox(0,2) = x*z*CO+y*s;
  rotationBox(1,2) = y*z*CO-x*s;
  rotationBox(2,2) = z*z*CO+c;
  // Translation vector
  rotationBox(0,3) = 0;
  rotationBox(1,3) = 0;
  rotationBox(2,3) = 0;

  // The rest of the 4x4 matrix
  rotationBox(3,0) = 0;
  rotationBox(3,1) = 0;
  rotationBox(3,2) = 0;
  rotationBox(3,3) = 1;

  if(store_transformation)
    rotations_.push_back(rotationBox);

  return rotationBox;
}
