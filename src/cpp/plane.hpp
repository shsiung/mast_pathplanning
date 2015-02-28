#ifndef PLANE
#define PLANE

/***********************
* This class defines the plane
***********************/

class Plane
{

public:
  //! Constructor.
  Plane();
  //! Destructor.
  ~Plane();

  double x, y, z,theta;
  double vx,vy,vth;

};

#endif // PLANE