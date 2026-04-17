#pragma once
#include "../simulation/simThread.hpp"
//#include <Eigen/Dense>
#include "raylib.h"
#include "raymath.h"


class RaylibGraphics{
 public:
   RaylibGraphics(SimThread* simThread);

   void cast();
   void castingLoop();
   void updateCamera();
   void initRaylib();
   void closeRaylib();

 private:
   SimThread* simThread;
   bool followDrone = true;
   Camera3D camera;
   int screenWidth = 1200; 
   int screenHeight = 600;
};