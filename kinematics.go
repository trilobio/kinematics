/*
Package kinematics calculates forward and inverse kinematics for robotic arm
systems.

Forward kinematics takes joint angles and returns the end effector Pose. Inverse
kinematics takes a Pose and returns joint angles that move the end effector to
that Pose.

 ForwardKinematics (joint angles	-> Pose)
 InverseKinematics (Pose	-> joint angles)

Each function also requires a Denavit-Hartenberg Parameter set for the arm of
interest. This package provides defaults for the following arm systems:

 AR2/AR3
*/
package kinematics

import (
	"errors"
	"math"
	"math/rand"

	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/optimize"
)

// DhParameters stand for "Denavit-Hartenberg Parameters". These parameters
// define a robotic arm for input into forward or reverse kinematics.
type DhParameters struct {
	ThetaOffsets []float64
	AlphaValues  []float64
	AValues      []float64
	DValues      []float64
}

type Quaternion struct {
	W float64
	X float64
	Y float64
	Z float64
}

// Position represents a position in 3D cartesian space.
type Position struct {
	X float64
	Y float64
	Z float64
}

// Pose represents a position and rotation, where Position is the translational
// component, and Rot is the quaternion representing the rotation.
type Pose struct {
	Position Position
	Rotation Quaternion
}

// ApproxEqual checks if this Quaternion is approximately equal to another
// Quaternion. This checks the positive and negative Quaternion (which are
// equivalent).
func (quatA Quaternion) ApproxEqual(quatB Quaternion, tol float64) bool {
	isEqual := false
	if (math.Abs(quatA.X-quatB.X) < tol) &&
		(math.Abs(quatA.Y-quatB.Y) < tol) &&
		(math.Abs(quatA.Z-quatB.Z) < tol) &&
		(math.Abs(quatA.W-quatB.W) < tol) {
		isEqual = true
	} else if (math.Abs(quatA.X+quatB.X) < tol) &&
		(math.Abs(quatA.Y+quatB.Y) < tol) &&
		(math.Abs(quatA.Z+quatB.Z) < tol) &&
		(math.Abs(quatA.W+quatB.W) < tol) {
		isEqual = true
	}
	return isEqual
}

// ApproxEqual checks if two Positions are approximately equal.
func (posA Position) ApproxEqual(posB Position, tol float64) bool {
	isEqual := false
	if math.Abs(posA.X-posB.X) < tol && math.Abs(posA.Y-posB.Y) < tol &&
		math.Abs(posA.Z-posB.Z) < tol {
		isEqual = true
	}
	return isEqual
}

// ApproxEqual checks if two Poses are approximately equal.
func (poseA Pose) ApproxEqual(poseB Pose, tol float64) bool {
	return poseA.Pos.ApproxEqual(poseB.Pos, tol) &&
		poseA.Rot.ApproxEqual(poseB.Rot, tol)
}

// RandTheta creates a random value from -pi to pi
func RandTheta() float64 {
	return 2 * math.Pi * (rand.Float64() - 0.5)
}

// ForwardKinematics calculates the end effector Pose coordinates given
// joint angles and robotic arm parameters.
func ForwardKinematics(thetas []float64, dhParameters DhParameters) Pose {
	// First, setup variables. We use 4 variables - theta, alpha, a and d to
	// calculate a matrix which is then multiplied to an accumulator matrix.
	var theta float64
	var alpha float64
	var a float64
	var d float64
	// Setup accumulator matrix - an identity matrix.
	accumulatortMat := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	// Iterate through each joint and built a new
	// matrix, multiplying it against the accumulator.
	for jointIdx := 0; jointIdx < len(thetas); jointIdx++ {
		theta = thetas[jointIdx]
		theta = theta + dhParameters.ThetaOffsets[jointIdx]
		alpha = dhParameters.AlphaValues[jointIdx]
		a = dhParameters.AValues[jointIdx]
		d = dhParameters.DValues[jointIdx]
		tMat := mat.NewDense(4, 4, []float64{
			// First row
			math.Cos(theta),
			-math.Sin(theta) * math.Cos(alpha),
			math.Sin(theta) * math.Sin(alpha),
			a * math.Cos(theta),
			// Second row
			math.Sin(theta),
			math.Cos(theta) * math.Cos(alpha),
			-math.Cos(theta) * math.Sin(alpha),
			a * math.Sin(theta),
			// Third row
			0,
			math.Sin(alpha),
			math.Cos(alpha),
			d,
			// Forth row
			0,
			0,
			0,
			1,
		})
		// Multiply tMat against accumulatortMat
		x := mat.NewDense(4, 4, nil)
		x.Mul(accumulatortMat, tMat)
		accumulatortMat = x
	}

	// Now that we have the final accumulatorMatrix, lets figure out the
	// output Pose.
	var output Pose
	output.Position.X = accumulatortMat.At(0, 3)
	output.Position.Y = accumulatortMat.At(1, 3)
	output.Position.Z = accumulatortMat.At(2, 3)
	output.Rotation = matrixToQuaterion(accumulatortMat)
	return output
}

// MaxInverseKinematicIteration is the max number of times InverseKinematics
// should try new seeds before failing. 50 is used here because it is
// approximately the number of iterations that will take 1 second to compute.
var MaxInverseKinematicIteration int = 50

// InverseKinematics calculates joint angles to achieve a desired end effector
// Pose, robotic arm DH parameters and the intial joint angles.
func InverseKinematics(desiredEndEffector Pose, dhParameters DhParameters,
	thetasInit []float64) ([]float64, error) {
	// Initialize an objective function for the optimization problem
	objectiveFunction := func(s []float64) float64 {
		currentEndEffector := ForwardKinematics(s, dhParameters)

		// Get XYZ offsets
		xOffset := desiredEndEffector.Position.X - currentEndEffector.Position.X
		yOffset := desiredEndEffector.Position.Y - currentEndEffector.Position.Y
		zOffset := desiredEndEffector.Position.Z - currentEndEffector.Position.Z

		// Get rotational offsets. Essentially, do this in Golang (from python):
		// np.arccos(np.clip(2*(np.dot(target_quat, source_quat)**2) - 1, -1, 1))
		dotOffset := (desiredEndEffector.Rotation.W * currentEndEffector.Rotation.W) +
			(desiredEndEffector.Rotation.X * currentEndEffector.Rotation.X) +
			(desiredEndEffector.Rotation.Y * currentEndEffector.Rotation.Y) +
			(desiredEndEffector.Rotation.Z * currentEndEffector.Rotation.Z)
		dotOffset = (2*(dotOffset*dotOffset) - 1)
		if dotOffset > 1 {
			dotOffset = 1
		}
		rotationalOffset := math.Acos(dotOffset)

		// Get the error vector
		errorVector := ((xOffset * xOffset) +
			(yOffset * yOffset) +
			(zOffset * zOffset) +
			(rotationalOffset * rotationalOffset)) * 0.25

		return errorVector
	}
	// Setup problem and method for solving
	problem := optimize.Problem{Func: objectiveFunction}

	// Solve
	result, err := optimize.Minimize(problem, thetasInit, nil, nil)
	if err != nil {
		return []float64{}, err
	}
	f := result.Location.F

	// If the results aren't up to spec, queue up another theta seed and test
	// again. We arbitrarily choose 1e-6 because that is small enough that the
	// errors do not matter.
	for i := 0; f > 1e-6; i++ {
		// Get a random seed between -pi and pi in radians.
		randomSeed := make([]float64, len(thetasInit))
		for j := range randomSeed {
			randomSeed[j] = RandTheta()
		}
		// Solve
		result, err := optimize.Minimize(problem, randomSeed, nil, nil)
		if err != nil {
			return []float64{}, err
		}
		f = result.Location.F
		if i == MaxInverseKinematicIteration {
			return []float64{}, errors.New("desired position out of range of" +
				" the robotic arm")
		}
	}
	return result.Location.X, nil
}

// matrixToQuaterion converts a rotation matrix to a quaterion. This code has
// been tested in all cases vs the python implementation with scipy rotation
// and works properly.
func matrixToQuaterion(accumulatortMat *mat.Dense) Quaternion {
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	var qw float64
	var qx float64
	var qy float64
	var qz float64
	var tr float64
	var s float64
	tr = accumulatortMat.At(0, 0) + accumulatortMat.At(1, 1) +
		accumulatortMat.At(2, 2)
	switch {
	case tr > 0:
		s = math.Sqrt(tr+1.0) * 2
		qw = 0.25 * s
		qx = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		qy = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		qz = (accumulatortMat.At(1, 0) - accumulatortMat.At(0, 1)) / s
	case accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1) &&
		accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2):
		s = math.Sqrt(1.0+accumulatortMat.At(0, 0)-accumulatortMat.At(1, 1)-
			accumulatortMat.At(2, 2)) * 2
		qw = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		qx = 0.25 * s
		qy = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		qz = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
	case accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2):
		s = math.Sqrt(1.0+accumulatortMat.At(1, 1)-accumulatortMat.At(0, 0)-
			accumulatortMat.At(2, 2)) * 2
		qw = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		qx = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		qy = 0.25 * s
		qz = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
	default:
		s = math.Sqrt(1.0+accumulatortMat.At(2, 2)-accumulatortMat.At(0, 0)-
			accumulatortMat.At(1, 1)) * 2
		qw = (accumulatortMat.At(0, 1) - accumulatortMat.At(1, 0))
		qx = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
		qy = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
		qz = 0.25 * s
	}
	return Quaternion{W: qw, X: qx, Y: qy, Z: qz}
}
