#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  collisionLookup = new Constants::config [Constants::headings * Constants::positions];
  collisionLookupTrailer = new Constants::config [Constants::headings * Constants::positions];
  Lookup::collisionLookup(collisionLookup, Constants::length, Constants::width, Constants::bbSize);
  Lookup::collisionLookup(collisionLookupTrailer, Constants::trailer_length, Constants::trailer_width, Constants::trailer_bbSize);
}

CollisionDetection::~CollisionDetection() {
  delete [] collisionLookup ;
  delete [] collisionLookupTrailer ;
}

void CollisionDetection::collisionReconfigure(){
  if(collisionLookup)
    delete [] collisionLookup;
  if(collisionLookupTrailer)
    delete [] collisionLookupTrailer;
  collisionLookup = new Constants::config [Constants::headings * Constants::positions];
  collisionLookupTrailer = new Constants::config [Constants::headings * Constants::positions];
  Lookup::collisionLookup(collisionLookup, Constants::length, Constants::width, Constants::bbSize);
  Lookup::collisionLookup(collisionLookupTrailer, Constants::trailer_length, Constants::trailer_width, Constants::trailer_bbSize);
}


bool CollisionDetection::configurationTest(float x, float y, float t, bool isTrailer) const {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;
  Constants::config * const lookupTable = (isTrailer ? collisionLookupTrailer : collisionLookup);
  for (int i = 0; i < lookupTable[idx].length; ++i) {
    cX = (X + lookupTable[idx].pos[i].x);
    cY = (Y + lookupTable[idx].pos[i].y);
    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX] >= Constants::lethal_th || grid->data[cY * grid->info.width + cX]== Constants::unknown_th) {
        return false;
      }
    }
  }

  return true;
}