#ifndef SEMANTICMAP_3D_H_
#define SEMANTICMAP_3D_H_

#include <vector>
#include <boost/thread.hpp>
#include <geometry_msgs/Point.h>

class DetectedObject
{
public:
  DetectedObject();
  DetectedObject(const DetectedObject& obj);
protected:
  std::vector<geometry_msgs::Point> points;
};
  
class Semanticmap3D
{

public:
  Semanticmap3D(unsigned int cells_size_x, unsigned int cells_size_y, unsigned int cells_size_z, double resolution,
                double origin_x, double origin_y, double origin_z);

  Semanticmap3D(const Semanticmap3D& map);

  inline unsigned int getIndex(unsigned int mx, unsigned int my, unsigned int mz) const
  {
    return mz * size_y_ * size_x_ + my * size_x_ + mx;
  }

  typedef boost::recursive_mutex mutex_t;
  mutex_t* access_;
protected:
  unsigned int size_x_;
  unsigned int size_y_;
  unsigned int size_z_;  
  double resolution_;
  double origin_x_;
  double origin_y_;
  double origin_z_;
  std::vector<DetectedObject *>* semanticmap_;

  virtual void initMaps(unsigned int size_x, unsigned int size_y, unsigned int size_z);
  virtual void resetMaps();
  virtual void setObject(unsigned int mx, unsigned int my, unsigned int mz, DetectedObject* obj);
  
};
#endif
