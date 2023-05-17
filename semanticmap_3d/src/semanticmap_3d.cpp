#include <semanticmap_3d/semanticmap_3d.h>

Semanticmap3D::Semanticmap3D(unsigned int cells_size_x, unsigned int cells_size_y, unsigned int cells_size_z, double resolution,
                             double origin_x, double origin_y, double origin_z) :
  size_x_(cells_size_x), size_y_(cells_size_y), size_z_(cells_size_z), resolution_(resolution),
  origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z), semanticmap_(NULL)
{
  access_ = new mutex_t();

  // create the costmap
  initMaps(size_x_, size_y_, size_z_);
  // resetMaps();
}

void Semanticmap3D::initMaps(unsigned int size_x, unsigned int size_y, unsigned int size_z)
{
  boost::unique_lock<mutex_t> lock(*access_);
  delete[] semanticmap_;
  semanticmap_ = new std::vector<DetectedObject *>[size_x * size_y * size_z];
}

void Semanticmap3D::setObject(unsigned int mx, unsigned int my, unsigned int mz, DetectedObject* obj)
{
  semanticmap_[getIndex(mx, my, mz)].push_back(obj);
}
