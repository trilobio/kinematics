package kinematics

import (
	"math"
)

// Denavit-Hartenberg Parameters of AR3 provided by AR2 Version 2.0 software
// executable files from https://www.anninrobotics.com/downloads
// Those parameters are the same between the AR2 and AR3.
// We use these as an example for a 6 Degrees-Of-Freedom arm.
var SixDOFDhParameters DhParameters = DhParameters{
	ThetaOffsets: []float64{0, 0, -math.Pi / 2, 0, 0, math.Pi},
	AlphaValues:  []float64{-(math.Pi / 2), 0, math.Pi / 2, -(math.Pi / 2), math.Pi / 2, 0},
	AValues:      []float64{64.2, 305, 0, 0, 0, 0},
	DValues:      []float64{169.77, 0, 0, -222.63, 0, -36.25},
}

// Denavit-Hartenberg Parameters of a ficticious 7 Degress-Of-Freedom arm
var SevenDOFDhParameters DhParameters = DhParameters{
	ThetaOffsets: []float64{0, 0, 0, 0, 0, 0, 0},
	AlphaValues:  []float64{-math.Pi / 2, math.Pi / 2, -math.Pi / 2, math.Pi / 2, -math.Pi / 2, 0, 0},
	AValues:      []float64{0, 200, 250, 300, 200, 200, 100},
	DValues:      []float64{500, 0, 0, 0, 0, 0, 0},
}
