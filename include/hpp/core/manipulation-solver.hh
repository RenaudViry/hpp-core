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

#ifndef HPP_CORE_MANIPULATION_SOLVER_HH
# define HPP_CORE_MANIPULATION_SOLVER_HH

# include <hpp/core/problem-solver.hh>
# include <hpp/constraints/fwd.hh>

namespace hpp {
  namespace core {
    /// Set and solve a path planning problem
    ///
    /// This class is a container that does the interface between
    /// hpp-core library and component to be running in a middleware
    /// like CORBA or ROS.
    class HPP_CORE_DLLAPI ManipulationSolver : public ProblemSolver {
    public:
    	ManipulationSolver();
		hpp::constraints::PositionPtr_t 				getPositionConstraint 				(const std::string& constraintName);
		hpp::constraints::OrientationPtr_t 				getOrientationConstraint 			(const std::string& constraintName);
		hpp::constraints::RelativePositionPtr_t 		getRelativePositionConstraint 		(const std::string& constraintName);
		hpp::constraints::RelativeOrientationPtr_t 		getRelativeOrientationConstraint 	(const std::string& constraintName);
		hpp::constraints::RelativeTransformationPtr_t 	getRelativeTransformationConstraint	(const std::string& constraintName);
		hpp::constraints::RelativeComPtr_t 				getRelativeComConstraint			(const std::string& constraintName);

		void addPositionConstraint (const std::string& name, const hpp::constraints::PositionPtr_t& constraint);
		void addOrientationConstraint (const std::string& name, const hpp::constraints::OrientationPtr_t& constraint);
		void addRelativePositionConstraint (const std::string& name, const hpp::constraints::RelativePositionPtr_t& constraint);
		void addRelativeOrientationConstraint (const std::string& name, const hpp::constraints::RelativeOrientationPtr_t& constraint);
		void addRelativeTransformationConstraint (const std::string& name, const hpp::constraints::RelativeTransformationPtr_t& constraint);
		void addRelativeComConstraint (const std::string& name, const hpp::constraints::RelativeComPtr_t& constraint);

    private:
		std::vector <hpp::constraints::PositionPtr_t>				positionConstraints_;
		std::vector <hpp::constraints::OrientationPtr_t> 			orientationConstraints_;
		std::vector <hpp::constraints::RelativePositionPtr_t>		relativePositionConstraints_;
		std::vector <hpp::constraints::RelativeOrientationPtr_t>	relativeOrientationConstraints_;
		std::vector <hpp::constraints::RelativeTransformationPtr_t>	relativeTransformationConstraints_;
		std::vector <hpp::constraints::RelativeComPtr_t>			relativeComConstraints_;
	};
  }
}

#endif // HPP_CORE_MANIPULATION_SOLVER_HH