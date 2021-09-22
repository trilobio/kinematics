package kinematics_test

import (
	"fmt"
	"github.com/trilobio/kinematics"
)

func ExampleForwardKinematics() {
	angles := []float64{10, 1, 1, 0, 0, 0}
	coordinates := kinematics.ForwardKinematics(angles, kinematics.SixDOFDhParameters)

	fmt.Println(coordinates)
	// Output {{-101.74590611879692 -65.96805988175777 -322.27756822304093} {0.06040824945687102 -0.20421099379003957 0.2771553334491873 0.9369277637862541}}
}

func ExampleInverseKinematics() {
	// Establish the original joint angles
	thetasInit := []float64{0, 0, 0, 0, 0, 0}

	// Establish coordinates to go to
	coordinates := kinematics.Pose{Position: kinematics.Position{X: -100, Y: 250, Z: 250}, Rotation: kinematics.Quaternion{W: 0.41903052745255764, X: 0.4007833787652043, Y: -0.021233218878182854, Z: 0.9086418268616911}}

	// Run kinematics procedure
	angles, _ := kinematics.InverseKinematics(coordinates, kinematics.SixDOFDhParameters, thetasInit)

	// Math works slightly differently on arm and x86 machines when calculating
	// inverse kinematics. We check 5 decimals deep, since it appears numbers can
	// have slight variations between arm and x86 at 6 decimals.
	fmt.Printf("%5f, %5f, %5f, %5f, %5f, %5f\n", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
	// Output: 1.846274, 0.341672, -2.313720, -1.776501, 2.221810, 1.231879
}
