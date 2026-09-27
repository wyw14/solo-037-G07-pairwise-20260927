package service

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/wyw14/solo-037/internal/attitude"
	"github.com/wyw14/solo-037/internal/model"
)

// AttitudeSensorInput is the operator-supplied registration for one attitude
// sensor. ExpectedIntervalMS is the expected sampling interval in
// milliseconds and drives gap detection; ThresholdDeg marks abnormal angles
// in degrees (0 disables abnormal-interval detection).
type AttitudeSensorInput struct {
	SensorID           string  `json:"sensor_id"`
	Label              string  `json:"label"`
	MountLocation      string  `json:"mount_location"`
	ExpectedIntervalMS int64   `json:"expected_interval_ms"`
	ThresholdDeg       float64 `json:"threshold_deg"`
}

// AttitudeArchive persists the attitude sensors registered per measured run.
// Angle samples themselves live in the MeasurementArchive as roll/pitch
// evidence.
type AttitudeArchive struct {
	mu      sync.Mutex
	path    string
	sensors map[model.RunID][]attitude.Sensor
}

func OpenAttitudeArchive(directory string) (*AttitudeArchive, error) {
	a := &AttitudeArchive{sensors: make(map[model.RunID][]attitude.Sensor)}
	if directory == "" {
		return a, nil
	}
	if err := os.MkdirAll(directory, 0o750); err != nil {
		return nil, err
	}
	a.path = filepath.Join(directory, "attitude-sensors.json")
	data, err := os.ReadFile(a.path)
	if errors.Is(err, os.ErrNotExist) {
		return a, nil
	}
	if err != nil {
		return nil, err
	}
	if err := json.Unmarshal(data, &a.sensors); err != nil {
		return nil, fmt.Errorf("read attitude sensor archive: %w", err)
	}
	if a.sensors == nil {
		return nil, errors.New("attitude sensor archive must contain a sensor map")
	}
	return a, nil
}

func (a *AttitudeArchive) commit(next map[model.RunID][]attitude.Sensor) error {
	if a.path != "" {
		data, err := json.MarshalIndent(next, "", "  ")
		if err != nil {
			return err
		}
		file, err := os.CreateTemp(filepath.Dir(a.path), "attitude-sensors-*.tmp")
		if err != nil {
			return err
		}
		name := file.Name()
		defer os.Remove(name)
		if _, err = file.Write(data); err == nil {
			err = file.Sync()
		}
		closeErr := file.Close()
		if err != nil {
			return err
		}
		if closeErr != nil {
			return closeErr
		}
		if err := os.Rename(name, a.path); err != nil {
			return err
		}
	}
	a.sensors = next
	return nil
}

func (a *AttitudeArchive) Register(id model.RunID, input AttitudeSensorInput) (attitude.Sensor, error) {
	sensor, err := attitude.NewSensor(id, input.SensorID, input.Label, input.MountLocation, time.Duration(input.ExpectedIntervalMS)*time.Millisecond, input.ThresholdDeg, time.Now().UTC())
	if err != nil {
		return attitude.Sensor{}, err
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	for _, existing := range a.sensors[id] {
		if existing.SensorID == sensor.SensorID {
			return attitude.Sensor{}, fmt.Errorf("sensor %q already registered for run", sensor.SensorID)
		}
	}
	next := make(map[model.RunID][]attitude.Sensor, len(a.sensors))
	for key, value := range a.sensors {
		next[key] = append([]attitude.Sensor(nil), value...)
	}
	next[id] = append(next[id], sensor)
	if err := a.commit(next); err != nil {
		return attitude.Sensor{}, err
	}
	return sensor, nil
}

func (a *AttitudeArchive) List(id model.RunID) []attitude.Sensor {
	a.mu.Lock()
	defer a.mu.Unlock()
	return append([]attitude.Sensor(nil), a.sensors[id]...)
}

func (r *Runtime) RegisterAttitudeSensor(id model.RunID, input AttitudeSensorInput) (attitude.Sensor, error) {
	if _, exists := r.Measurements.Get(id); !exists {
		return attitude.Sensor{}, errors.New("run not found")
	}
	return r.Attitude.Register(id, input)
}

func (r *Runtime) AttitudeSensors(id model.RunID) ([]attitude.Sensor, bool) {
	if _, exists := r.Measurements.Get(id); !exists {
		return nil, false
	}
	return r.Attitude.List(id), true
}

func (r *Runtime) AnalyzeAttitude(id model.RunID) (attitude.RunAnalysis, error) {
	run, exists := r.Measurements.Get(id)
	if !exists {
		return attitude.RunAnalysis{}, errors.New("run not found")
	}
	samples := make([]model.Evidence, 0)
	positions := make([]model.Evidence, 0)
	for _, evidence := range run.Evidence {
		switch evidence.Kind {
		case model.EvidenceRoll, model.EvidencePitch:
			samples = append(samples, evidence)
		case model.EvidencePosition:
			positions = append(positions, evidence)
		}
	}
	return attitude.Analyze(r.Attitude.List(id), samples, positions, time.Now().UTC()), nil
}
