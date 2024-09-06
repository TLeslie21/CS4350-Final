#include "WO.h"
#include "GLViewFinalProject.h"
#include "WorldList.h"

class Coin : public WO {
public:
    Coin() : isCoinFlag(true) {} // Set flag to true for coins
    bool isCoin() const { return isCoinFlag; }
private:
    bool isCoinFlag;
};
