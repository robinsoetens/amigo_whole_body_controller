#ifndef WBC_CONSTRAINT_H_
#define WBC_CONSTRAINT_H_

#include "Chain.h"
#include "Component.h"

// Eigen
#include <Eigen/Core>

class Constraint {

public:

    Constraint();

    virtual ~Constraint();

    virtual bool initialize(const std::vector<Chain*> chains, const std::vector<Component*> components) = 0;

    virtual bool isActive() = 0;

    virtual void apply() = 0;

};

#endif
