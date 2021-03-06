//
// Copyright (c) 2014 CNRS
// Authors: Renaud Viry
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/core/manipulation-solver.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/orientation.hh>
#include <hpp/constraints/relative-position.hh>
#include <hpp/constraints/relative-orientation.hh>
#include <hpp/constraints/relative-transformation.hh>
#include <hpp/constraints/relative-com.hh>

namespace hpp {
  namespace core {
	ManipulationSolver::ManipulationSolver ()
	{}

	void ManipulationSolver::addPositionConstraint (const std::string& name, const hpp::constraints::PositionPtr_t& constraint)
	{
		NumericalConstraintMap_ [name] = constraint;
		positionConstraints_ [name] = constraint;
	}

	void ManipulationSolver::addOrientationConstraint (const std::string& name, const hpp::constraints::OrientationPtr_t& constraint)
	{
		NumericalConstraintMap_ [name] = constraint;
		orientationConstraints_ [name] = constraint;
	}

	void ManipulationSolver::addRelativePositionConstraint (const std::string& name, const hpp::constraints::RelativePositionPtr_t& constraint)
	{
		NumericalConstraintMap_ [name] = constraint;
		relativePositionConstraints_ [name] = constraint;
	}

	void ManipulationSolver::addRelativeOrientationConstraint (const std::string& name, const hpp::constraints::RelativeOrientationPtr_t& constraint)
	{
		NumericalConstraintMap_ [name] = constraint;
		relativeOrientationConstraints_ [name] = constraint;
	}

	void ManipulationSolver::addRelativeTransformationConstraint (const std::string& name, const hpp::constraints::RelativeTransformationPtr_t& constraint)
	{
		NumericalConstraintMap_ [name] = constraint;
		relativeTransformationConstraints_ [name] = constraint;
	}

	void ManipulationSolver::addRelativeComConstraint (const std::string& name, const hpp::constraints::RelativeComPtr_t& constraint)
	{
		NumericalConstraintMap_ [name] = constraint;
		relativeComConstraints_ [name] = constraint;
	}

	hpp::constraints::PositionPtr_t ManipulationSolver::getPositionConstraint (const std::string& constraintName)
	{
		hpp::constraints::PositionPtr_t result;

		std::map <std::string, hpp::constraints::PositionPtr_t>::iterator it = positionConstraints_.find(constraintName);
		if (it != positionConstraints_.end())
			result = it->second;
		// else
		// 	std::cout << "ERROR : Position constraints does not exist in this solver" << std::endl;
		return result;
	}

	hpp::constraints::OrientationPtr_t ManipulationSolver::getOrientationConstraint (const std::string& constraintName)
	{
		hpp::constraints::OrientationPtr_t result;

		std::map <std::string, hpp::constraints::OrientationPtr_t>::iterator it = orientationConstraints_.find(constraintName);
		if (it != orientationConstraints_.end())
			result = it->second;
		// else
		// 	std::cout << "ERROR : Orientation constraints does not exist in this solver" << std::endl;
		return result;
	}

	hpp::constraints::RelativePositionPtr_t ManipulationSolver::getRelativePositionConstraint (const std::string& constraintName)
	{
		hpp::constraints::RelativePositionPtr_t result;

		std::map <std::string, hpp::constraints::RelativePositionPtr_t>::iterator it = relativePositionConstraints_.find(constraintName);
		if (it != relativePositionConstraints_.end())
			result = it->second;
		// else
		// 	std::cout << "ERROR : Relative Position constraints does not exist in this solver" << std::endl;
		return result;
	}

	hpp::constraints::RelativeOrientationPtr_t ManipulationSolver::getRelativeOrientationConstraint (const std::string& constraintName)
	{
		hpp::constraints::RelativeOrientationPtr_t result;

		std::map <std::string, hpp::constraints::RelativeOrientationPtr_t>::iterator it = relativeOrientationConstraints_.find(constraintName);
		if (it != relativeOrientationConstraints_.end())
			result = it->second;
		// else
		// 	std::cout << "ERROR : Relative Orientation constraints does not exist in this solver" << std::endl;
		return result;
	}

	hpp::constraints::RelativeTransformationPtr_t ManipulationSolver::getRelativeTransformationConstraint (const std::string& constraintName)
	{
		hpp::constraints::RelativeTransformationPtr_t result;

		std::map <std::string, hpp::constraints::RelativeTransformationPtr_t>::iterator it = relativeTransformationConstraints_.find(constraintName);
		if (it != relativeTransformationConstraints_.end())
			result = it->second;
		// else
		// 	std::cout << "ERROR : Relative Transformation constraints does not exist in this solver" << std::endl;
		return result;
	}

	hpp::constraints::RelativeComPtr_t ManipulationSolver::getRelativeComConstraint (const std::string& constraintName)
	{
		hpp::constraints::RelativeComPtr_t result;

		std::map <std::string, hpp::constraints::RelativeComPtr_t>::iterator it = relativeComConstraints_.find(constraintName);
		if (it != relativeComConstraints_.end())
			result = it->second;
		// else
		// 	std::cout << "ERROR : Relative Com constraints does not exist in this solver" << std::endl;
		return result;
	}
  }
}