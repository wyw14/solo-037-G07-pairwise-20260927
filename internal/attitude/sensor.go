package attitude

import (
	"errors"
	"strings"
	"time"

	"github.com/wyw14/solo-037/internal/model"
)

// Axis names reported by attitude sensors.
const (
	AxisRoll  = "roll"
	AxisPitch = "pitch"
)

// Sensor describes an attitude sensor registered for a measured run. The
// expected interval drives gap detection and ThresholdDeg marks abnormal
// angles in degrees; a zero threshold disables abnormal-interval detection.
type Sensor struct {
	SensorID         string        `json:"sensor_id"`
	Run              model.RunID   `json:"run"`
	Label            string        `json:"label"`
	MountLocation    string        `json:"mount_location"`
	ExpectedInterval time.Duration `json:"expected_interval"`
	ThresholdDeg     float64       `json:"threshold_deg"`
	RegisteredAt     time.Time     `json:"registered_at"`
}

func NewSensor(run model.RunID, sensorID, label, mountLocation string, expectedInterval time.Duration, thresholdDeg float64, at time.Time) (Sensor, error) {
	if strings.TrimSpace(sensorID) == "" || strings.TrimSpace(label) == "" {
		return Sensor{}, errors.New("sensor id and label are required")
	}
	if expectedInterval <= 0 {
		return Sensor{}, errors.New("expected interval must be positive")
	}
	if thresholdDeg < 0 {
		return Sensor{}, errors.New("threshold must not be negative")
	}
	return Sensor{
		SensorID:         sensorID,
		Run:              run,
		Label:            label,
		MountLocation:    mountLocation,
		ExpectedInterval: expectedInterval,
		ThresholdDeg:     thresholdDeg,
		RegisteredAt:     at.UTC(),
	}, nil
}
