package kinematics

import (
	"gonum.org/v1/gonum/mat"
	"testing"
)

func TestApproxEquals(t *testing.T) {
	p0 := Position{1, 2, 3}
	p1 := Position{1 + 1.1e-6, 2 - 3e-7, 3 + 2.45e-6}
	tol := 1e-5
	if !p0.approxEqual(p1, tol) {
		t.Errorf("approxEqual failed at tol %f with p0: %+v, p1: %+v", tol, p0, p1)
	}
	tol = 1e-6
	if p0.approxEqual(p1, tol) {
		t.Errorf("approxEqual failed at tol %f with p0: %+v, p1: %+v", tol, p0, p1)
	}

	q0 := Quaternion{1, 2, 3, 4}
	q1 := Quaternion{-1, -2, -3, -4}
	if !q0.approxEqual(q1, tol) {
		t.Errorf("approxEqual failed for negative quaternions q0: %+v, q1: %+v", q0, q1)
	}

	tol = 1e-5
	q1 = Quaternion{-1 + 2.345e-6, -2 - 4.3e-7, -3 + 2e-6, -4}
	if !q0.approxEqual(q1, tol) {
		t.Errorf("approxEqual failed for close negative quaternions q0: %+v, q1: %+v", q0, q1)
	}
	q1 = Quaternion{1 + 2.345e-6, 2 - 4.3e-7, 3 + 2e-6, 4}
	if !q0.approxEqual(q1, tol) {
		t.Errorf("approxEqual failed for close quaternions q0: %+v, q1: %+v", q0, q1)
	}

	tol = 1e-7
	q1 = Quaternion{1 + 2.345e-6, 2 - 4.3e-7, 3 + 2e-6, 4}
	if q0.approxEqual(q1, tol) {
		t.Errorf("approxEqual failed for close quaternions q0: %+v, q1: %+v", q0, q1)
	}
}

func TestForwardKinematics(t *testing.T) {
	testThetas := []float64{10, 1, 1, 0, 0, 0}
	f := ForwardKinematics(testThetas, SixDOFDhParameters)
	switch {
	case f.Position.X != -101.74590611879692:
		t.Errorf("Forward kinematics failed on f.Position.X = %f", f.Position.X)
	case f.Position.Y != -65.96805988175777:
		t.Errorf("Forward kinematics failed on f.Position.Y = %f", f.Position.Y)
	case f.Position.Z != -322.27756822304093:
		t.Errorf("Forward kinematics failed on f.Position.Z = %f", f.Position.Z)
	case f.Rotation.X != 0.06040824945687102:
		t.Errorf("Forward kinematics failed on f.Rotation.X = %f", f.Rotation.X)
	case f.Rotation.Y != -0.20421099379003957:
		t.Errorf("Forward kinematics failed on f.Rotation.Y = %f", f.Rotation.Y)
	case f.Rotation.Z != 0.2771553334491873:
		t.Errorf("Forward kinematics failed on f.Rotation.Z = %f", f.Rotation.Z)
	case f.Rotation.W != 0.9369277637862541:
		t.Errorf("Forward kinematics failed on f.Rotation.W = %f", f.Rotation.W)
	}
}

func TestInverseKinematics(t *testing.T) {
	thetasInit := []float64{0, 0, 0, 0, 0, 0}
	desiredEndEffector := Pose{
		Position{
			-91.72345062922584,
			386.93155027870745,
			382.30917872225154},
		Quaternion{
			W: 0.41903052745255764,
			X: 0.4007833787652043,
			Y: -0.021233218878182854,
			Z: 0.9086418268616911}}
	_, err := InverseKinematics(desiredEndEffector, SixDOFDhParameters, thetasInit)
	if err != nil {
		t.Errorf("Inverse Kinematics failed with error: %s", err)
	}

	// This case should fail because the X required is too large
	desiredEndEffector = Pose{
		Position{-91000000.72345062922584,
			386.93155027870745,
			382.30917872225154},
		Quaternion{
			W: 0.41903052745255764,
			X: 0.4007833787652043,
			Y: -0.021233218878182854,
			Z: 0.9086418268616911}}
	_, err = InverseKinematics(desiredEndEffector, SixDOFDhParameters, thetasInit)
	if err == nil {
		t.Errorf("Inverse Kinematics should have failed with large X")
	}
}

func TestInverseKinematicsSevenDOF(t *testing.T) {
	thetasInit := []float64{0, 0, 0, 0, 0, 0, 0}

	thetasTarg := []float64{0, 0, 0, 0, 0, 0, 0}
	for i := range thetasTarg {
		thetasTarg[i] = RandTheta() * 0.1
	}
	pTarg := ForwardKinematics(thetasTarg, SevenDOFDhParameters)

	thetasSolve, err := InverseKinematics(pTarg, SevenDOFDhParameters, thetasInit)
	if err != nil {
		t.Errorf("Inverse Kinematics failed with error: %s", err)
	}
	pSolve := ForwardKinematics(thetasSolve, SevenDOFDhParameters)

	if !pSolve.approxEqual(pTarg, 1e-3) {
		t.Errorf("Inverse Kinematics didn't arrive at the correct"+
			" solution: %+v", pSolve)
	}
}

func TestmatrixToQuaterion(t *testing.T) {
	var q Quaternion

	// Test tr > 0
	accumulatortMat1 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	q = matrixToQuaterion(accumulatortMat1)
	switch {
	case q.W != 1:
		t.Errorf("Failed mat1 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat1 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat1 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat1 with q.Z = %f", q.Z)
	}

	// Test (accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1)) &&
	// (accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2))
	accumulatortMat2 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 0,
	})
	q = matrixToQuaterion(accumulatortMat2)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat2 with qw = %f", q.W)
	case q.X != 1:
		t.Errorf("Failed mat2 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat2 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat2 with q.Z = %f", q.Z)
	}

	// Test accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2)
	accumulatortMat3 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, -2, 0,
		0, 0, 0, 1,
	})
	q = matrixToQuaterion(accumulatortMat3)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat3 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat3 with q.X = %f", q.X)
	case q.Y != 1:
		t.Errorf("Failed mat3 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat3 with q.Z = %f", q.Z)
	}

	// Test default
	accumulatortMat4 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, -2, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1,
	})
	q = matrixToQuaterion(accumulatortMat4)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat4 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat4 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat4 with q.Y = %f", q.Y)
	case q.Z != 1:
		t.Errorf("Failed mat4 with q.Z = %f", q.Z)
	}
}

func BenchmarkInverseKinematics(b *testing.B) {
	thetasInit := []float64{0, 0, 0, 0, 0, 0}

	for i := 0; i < b.N; i++ {
		randomSeed := []float64{RandTheta(), RandTheta(), RandTheta(),
			RandTheta(), RandTheta(), RandTheta()}
		desiredEndEffector := ForwardKinematics(randomSeed, SixDOFDhParameters)
		_, err := InverseKinematics(desiredEndEffector, SixDOFDhParameters, thetasInit)
		if err != nil {
			b.Errorf("Failed inverse kinematics benchmark with: %s\nSeed:"+
				" %f-%f-%f-%f-%f-%f", err,
				randomSeed[0], randomSeed[1], randomSeed[2],
				randomSeed[3], randomSeed[4], randomSeed[5])
		}
	}
}
