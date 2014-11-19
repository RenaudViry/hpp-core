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

#include <map>

namespace hpp {
  namespace core {
	/// Handle constraints for a manipulation planning problem
	///
	/// This class is derived from ProblemSolver class and
	/// is a container that does the interface between
	/// hpp-core library and component to be running in a middleware
	/// like CORBA or ROS.
	class HPP_CORE_DLLAPI ManipulationSolver : public ProblemSolver {
	public:
		/// Constructor
		ManipulationSolver();

		/// Accessor to Position Constraints
		hpp::constraints::PositionPtr_t 				getPositionConstraint 				(const std::string& constraintName);
		/// Accessor to Orientation Constraints
		hpp::constraints::OrientationPtr_t 				getOrientationConstraint 			(const std::string& constraintName);
		/// Accessor to Relative Position Constraints
		hpp::constraints::RelativePositionPtr_t 		getRelativePositionConstraint 		(const std::string& constraintName);
		/// Accessor to Relative Orienation Constraints
		hpp::constraints::RelativeOrientationPtr_t 		getRelativeOrientationConstraint 	(const std::string& constraintName);
		/// Accessor to Relative Transformation Constraints
		hpp::constraints::RelativeTransformationPtr_t 	getRelativeTransformationConstraint	(const std::string& constraintName);
		/// Accessor to Relative Center Of Mass Constraints
		hpp::constraints::RelativeComPtr_t 				getRelativeComConstraint			(const std::string& constraintName);

		/// Create a new Position Constraint
		void addPositionConstraint (const std::string& name, const hpp::constraints::PositionPtr_t& constraint);
		/// Create a new Orientation Constraint
		void addOrientationConstraint (const std::string& name, const hpp::constraints::OrientationPtr_t& constraint);
		/// Create a new Relative Position Constraint
		void addRelativePositionConstraint (const std::string& name, const hpp::constraints::RelativePositionPtr_t& constraint);
		/// Create a new Relative Orientation Constraint
		void addRelativeOrientationConstraint (const std::string& name, const hpp::constraints::RelativeOrientationPtr_t& constraint);
		/// Create a new Relative Transformation Constraint
		void addRelativeTransformationConstraint (const std::string& name, const hpp::constraints::RelativeTransformationPtr_t& constraint);
		/// Create a new Relative Center Of Mass Constraint
		void addRelativeComConstraint (const std::string& name, const hpp::constraints::RelativeComPtr_t& constraint);

	private:
		/// Vector of Position Constraints
		std::map <std::string, hpp::constraints::PositionPtr_t>					positionConstraints_;
		/// Vector of Orientation Constraints
		std::map <std::string, hpp::constraints::OrientationPtr_t> 				orientationConstraints_;
		/// Vector of Relative Position Constraints
		std::map <std::string, hpp::constraints::RelativePositionPtr_t>			relativePositionConstraints_;
		/// Vector of Relative Orientation Constraints
		std::map <std::string, hpp::constraints::RelativeOrientationPtr_t>		relativeOrientationConstraints_;
		/// Vector of Relative Transformation Constraints
		std::map <std::string, hpp::constraints::RelativeTransformationPtr_t>	relativeTransformationConstraints_;
		/// Vector of Relative Center Of Mass Constraints
		std::map <std::string, hpp::constraints::RelativeComPtr_t>				relativeComConstraints_;
	};
  }
}

#endif // HPP_CORE_MANIPULATION_SOLVER_HH