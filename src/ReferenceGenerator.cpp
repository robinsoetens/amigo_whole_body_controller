/**
 * @file ref_generator.cpp
 *
 * @author Sjoerd van den Dries, Kris van Tienhoven
 * @date December, 2012
 * @version 1.0
 *
 * @brief Computes a reference velocity, acceleration or position for
 * a control to try to reach. This is based on the current and desired
 * velocity, acceleration and position.
 */

/************************************************************************
 *	Copyright (C) 2012 Eindhoven University of Technology (TU/e).		*
 *	All rights reserved.												*
 ************************************************************************
 *	Redistribution and use in source and binary forms, with or without	*
 *	modification, are permitted provided that the following conditions	*
 *	are met:															*
 *																		*
 *		1.	Redistributions of source code must retain the above		*
 * 			copyright notice, this list of conditions and the following *
 * 			disclaimer.													*
 *																		*
 *		2. 	Redistributions in binary form must reproduce the above		*
 *			copyright notice, this list of conditions and the following *
 *			disclaimer in the documentation and/or other materials 		*
 *			provided with the distribution.								*
 *																		*
 *	THIS SOFTWARE IS PROVIDED BY TU/e "AS IS" AND ANY EXPRESS OR 		*
 *	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 		*
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE	*
 *	ARE DISCLAIMED. IN NO EVENT SHALL TU/e OR CONTRIBUTORS BE LIABLE 	*
 *	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 		*
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 	*
 *	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 	*
 *	OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 		*
 *	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 			*
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 	*
 *	USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH	*
 *	DAMAGE.																*
 *																		*
 *	The views and conclusions contained in the software and 			*
 *	documentation are those of the authors and should not be 			*
 *	interpreted as representing official policies, either expressed or 	*
 *	implied, of TU/e.													*
 ************************************************************************/

#include "ReferenceGenerator.h"

#include <math.h> /* fabs */

using namespace controller;

const double RefGenerator::EPSILON = 1e-5;

RefGenerator::RefGenerator() :
    pos_(0), vel_(0), acc_(0), max_vel_(0), max_acc_(0) {
}

RefGenerator::RefGenerator(double current_pos, double max_vel, double max_acc) :
    pos_(current_pos), vel_(0), acc_(0), max_vel_(max_vel), max_acc_(max_acc) {
}

RefGenerator::~RefGenerator() {
}

int RefGenerator::signum(double a)  {
    if (a < 0) {
        return -1;
    } else {
        return 1;
    }
}


void RefGenerator::setCurrentPosition(double x) {
    pos_ = x;
}

void RefGenerator::setCurrentVelocity(double vel) {
    vel_ = vel;
}

double RefGenerator::getPositionReference() {
    return pos_;
}

double RefGenerator::getVelocityReference() {
    return vel_;
}

double RefGenerator::getAccelerationReference() {
    return acc_;
}

void RefGenerator::generate(double x_desired, double dt, bool force_stop) {
    double dx = x_desired - pos_;
    double dx_mag = fabs(dx);
    double vel_mag = fabs(vel_);

    if (dx_mag < EPSILON && vel_mag < EPSILON) {
        // already at correct location with zero velocity, so nothing happens
        return;
    }

    double dir = signum(vel_);
    double dir_desired = signum(dx);

    if (force_stop || dir != dir_desired) {
        // force stopping or going in the wrong direction, so decelerate
        acc_ = -max_acc_;
    } else {

        double dt_dec = vel_mag / max_acc_; //deceleration segment time
        double dx_dec = 0.5 * max_acc_ * dt_dec * dt_dec; //deceleration distance

        if (dx_mag <= dx_dec) {
            // decelerate to prevent overshoot
            acc_ = -max_acc_;
        } else {
            acc_ = max_acc_;
        }
    }

    vel_ = vel_ + acc_ * dir * dt;
    if (vel_ < -max_vel_) {
        vel_ = -max_vel_;
        acc_ = 0;
    } else if (vel_ > max_vel_) {
        vel_ = max_vel_;
        acc_ = 0;
    }

    pos_ += vel_ * dt;
}
