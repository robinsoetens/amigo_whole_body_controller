/**
 * @file ref_generator.h
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
*************************************************************************
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

namespace controller {

class RefGenerator {

private:

	double pos_; //the current position of the joint to generate a reference for
	double vel_; //the current velocity of the joint to generate a reference for
	double acc_; //the current acceleration of the joint to generate a reference for

	double max_vel_; //the maximum allowed velocity of the joint to generate a reference for
	double max_acc_; //the maximum allowed acceleration of the joint to generate a reference for

    const static double EPSILON; //error margin

	/**
	 * \return the sign of a i.e. -1 or 1
	 */
	int signum(double a);

public:

	RefGenerator();

	/**
	 * \brief Constructor for the RefGenerator
	 *
	 * pos_ is set to current_x
	 * vel_ to 0
	 * acc_ to 0
	 * max_vel_ to max_vel
	 * max_acc_ to mac_acc
	 */
	RefGenerator(double current_x, double max_vel, double max_acc);

	/**
	 * \brief Deconstructor for the RefGenerator.
	 */
	~RefGenerator();

	/**
	 * Sets pos_ to x
	 */
	void setCurrentPosition(double x);

	/**
	 * Sets vel_ to vel
	 */
	void setCurrentVelocity(double vel);

	/**
	 * \return the value of pos_
	 */
	double getPositionReference();

	/**
	 * \return the value of vel_
	 */
	double getVelocityReference();

	/**
	 * \return the value of acc_
	 */
	double getAccelerationReference();

	/**
	 * \brief Generates the actual reference
	 *
	 * Creates new values for pos_, vel_ and acc_ such that progress is made to the desired position,
	 * but no (velocity or acceleration) limit is exceeded.
	 * \param x_desired the desired position
	 * \param dt the time passed since the last generate
	 * \param force_stop whether to stop immediately or not
	 */
	void generate(double x_desired, double dt, bool force_stop);

};


}

