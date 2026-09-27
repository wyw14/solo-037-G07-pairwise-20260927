package service

import (
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/wyw14/solo-037/internal/attitude"
	"github.com/wyw14/solo-037/internal/model"
)

type AttitudeSampleInput struct {
	SourceID         string    `json:"source_id"`
	Roll             float64   `json:"roll"`
	Pitch            float64   `json:"pitch"`
	CarriagePosition float64   `json:"carriage_position"`
	RecordedAt       time.Time `json:"recorded_at"`
}

type AttitudeAnalysis struct {
	Operation     model.Operation           `json:"operation"`
	Parameters    attitude.Parameters       `json:"parameters"`
	Sensors       []attitude.SensorAnalysis `json:"sensors"`
	SilentSensors []string                  `json:"silent_sensors"`
	Evaluated     time.Time                 `json:"evaluated_at"`
}

type attitudeArchiveState struct {
	Sensors map[string]model.AttitudeSensor        `json:"sensors"`
	Samples map[model.RunID][]model.AttitudeSample `json:"samples"`
}

// AttitudeArchive keeps registered attitude sensors and operator-supplied
// angle samples. Like the measurement archive, mutations are committed to
// disk before they are published in memory.
type AttitudeArchive struct {
	mu      sync.Mutex
	path    string
	sensors map[string]model.AttitudeSensor
	samples map[model.RunID][]model.AttitudeSample
}

func OpenAttitudeArchive(directory string) (*AttitudeArchive, error) {
	a := &AttitudeArchive{
		sensors: make(map[string]model.AttitudeSensor),
		samples: make(map[model.RunID][]model.AttitudeSample),
	}
	if directory == "" {
		return a, nil
	}
	if err := os.MkdirAll(directory, 0o750); err != nil {
		return nil, err
	}
	a.path = filepath.Join(directory, "attitude-archive.json")
	data, err := os.ReadFile(a.path)
	if errors.Is(err, os.ErrNotExist) {
		return a, nil
	}
	if err != nil {
		return nil, err
	}
	var state attitudeArchiveState
	if err := json.Unmarshal(data, &state); err != nil {
		return nil, fmt.Errorf("read attitude archive: %w", err)
	}
	if state.Sensors != nil {
		a.sensors = state.Sensors
	}
	if state.Samples != nil {
		a.samples = state.Samples
	}
	return a, nil
}

func (a *AttitudeArchive) commit(next attitudeArchiveState) error {
	if a.path != "" {
		data, err := json.MarshalIndent(next, "", "  ")
		if err != nil {
			return err
		}
		file, err := os.CreateTemp(filepath.Dir(a.path), "attitude-*.tmp")
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
	a.sensors = next.Sensors
	a.samples = next.Samples
	return nil
}

func (a *AttitudeArchive) snapshot() attitudeArchiveState {
	sensors := make(map[string]model.AttitudeSensor, len(a.sensors))
	for code, sensor := range a.sensors {
		sensors[code] = sensor
	}
	samples := make(map[model.RunID][]model.AttitudeSample, len(a.samples))
	for run, list := range a.samples {
		samples[run] = append([]model.AttitudeSample(nil), list...)
	}
	return attitudeArchiveState{Sensors: sensors, Samples: samples}
}

func (a *AttitudeArchive) Register(code, label string, nominalInterval time.Duration) (model.AttitudeSensor, error) {
	code = strings.TrimSpace(code)
	label = strings.TrimSpace(label)
	if code == "" {
		return model.AttitudeSensor{}, errors.New("sensor code is required")
	}
	if nominalInterval <= 0 {
		return model.AttitudeSensor{}, errors.New("nominal interval must be positive")
	}
	if label == "" {
		label = code
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	if _, exists := a.sensors[code]; exists {
		return model.AttitudeSensor{}, fmt.Errorf("attitude sensor %q already registered", code)
	}
	sensor := model.NewAttitudeSensor(code, label, nominalInterval, time.Now().UTC())
	next := a.snapshot()
	next.Sensors[code] = sensor
	if err := a.commit(next); err != nil {
		return model.AttitudeSensor{}, err
	}
	return sensor, nil
}

func (a *AttitudeArchive) Sensors() []model.AttitudeSensor {
	a.mu.Lock()
	defer a.mu.Unlock()
	result := make([]model.AttitudeSensor, 0, len(a.sensors))
	for _, sensor := range a.sensors {
		result = append(result, sensor)
	}
	sort.Slice(result, func(i, j int) bool { return result[i].Code < result[j].Code })
	return result
}

func (a *AttitudeArchive) Record(id model.RunID, input AttitudeSampleInput) (model.AttitudeSample, error) {
	if _, exists := a.sensors[input.SourceID]; !exists {
		return model.AttitudeSample{}, fmt.Errorf("attitude sensor %q is not registered", input.SourceID)
	}
	if math.IsNaN(input.Roll) || math.IsInf(input.Roll, 0) || math.Abs(input.Roll) > 180 {
		return model.AttitudeSample{}, errors.New("roll must be finite and within ±180 degrees")
	}
	if math.IsNaN(input.Pitch) || math.IsInf(input.Pitch, 0) || math.Abs(input.Pitch) > 90 {
		return model.AttitudeSample{}, errors.New("pitch must be finite and within ±90 degrees")
	}
	if math.IsNaN(input.CarriagePosition) || math.IsInf(input.CarriagePosition, 0) {
		return model.AttitudeSample{}, errors.New("carriage position must be finite")
	}
	if input.RecordedAt.IsZero() || input.RecordedAt.After(time.Now().Add(time.Minute)) {
		return model.AttitudeSample{}, errors.New("valid observation time is required")
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	sample := model.NewAttitudeSample(id, input.SourceID, input.Roll, input.Pitch, input.CarriagePosition, input.RecordedAt).WithDurability(a.path != "")
	next := a.snapshot()
	next.Samples[id] = append(next.Samples[id], sample)
	if err := a.commit(next); err != nil {
		return model.AttitudeSample{}, err
	}
	return sample, nil
}

func (a *AttitudeArchive) Samples(id model.RunID) []model.AttitudeSample {
	a.mu.Lock()
	defer a.mu.Unlock()
	result := append([]model.AttitudeSample{}, a.samples[id]...)
	sort.SliceStable(result, func(i, j int) bool { return result[i].RecordedAt.Before(result[j].RecordedAt) })
	return result
}

func (a *AttitudeArchive) Analyze(operation model.Operation, parameters attitude.Parameters) (AttitudeAnalysis, error) {
	if math.IsNaN(parameters.RollLimit) || math.IsInf(parameters.RollLimit, 0) || parameters.RollLimit <= 0 {
		return AttitudeAnalysis{}, errors.New("roll limit must be positive")
	}
	if math.IsNaN(parameters.PitchLimit) || math.IsInf(parameters.PitchLimit, 0) || parameters.PitchLimit <= 0 {
		return AttitudeAnalysis{}, errors.New("pitch limit must be positive")
	}
	if parameters.MinDuration < 0 {
		return AttitudeAnalysis{}, errors.New("minimum duration must not be negative")
	}
	if math.IsNaN(parameters.GapMultiplier) || math.IsInf(parameters.GapMultiplier, 0) || parameters.GapMultiplier < 1 {
		return AttitudeAnalysis{}, errors.New("gap multiplier must be at least 1")
	}
	a.mu.Lock()
	samples := append([]model.AttitudeSample{}, a.samples[operation.ID]...)
	sensors := make([]model.AttitudeSensor, 0, len(a.sensors))
	for _, sensor := range a.sensors {
		sensors = append(sensors, sensor)
	}
	a.mu.Unlock()
	result := attitude.Analyze(samples, sensors, parameters)
	return AttitudeAnalysis{
		Operation:     operation,
		Parameters:    result.Parameters,
		Sensors:       result.Sensors,
		SilentSensors: result.SilentSensors,
		Evaluated:     time.Now().UTC(),
	}, nil
}

func (r *Runtime) RegisterAttitudeSensor(code, label string, nominalInterval time.Duration) (model.AttitudeSensor, error) {
	return r.Attitude.Register(code, label, nominalInterval)
}

func (r *Runtime) AttitudeSensors() []model.AttitudeSensor { return r.Attitude.Sensors() }

func (r *Runtime) RecordAttitudeSample(id model.RunID, input AttitudeSampleInput) (model.AttitudeSample, error) {
	if _, ok := r.Measurements.Get(id); !ok {
		return model.AttitudeSample{}, errors.New("run not found")
	}
	return r.Attitude.Record(id, input)
}

func (r *Runtime) AttitudeSamples(id model.RunID) ([]model.AttitudeSample, error) {
	if _, ok := r.Measurements.Get(id); !ok {
		return nil, errors.New("run not found")
	}
	return r.Attitude.Samples(id), nil
}

func (r *Runtime) AnalyzeAttitude(id model.RunID, parameters attitude.Parameters) (AttitudeAnalysis, error) {
	run, ok := r.Measurements.Get(id)
	if !ok {
		return AttitudeAnalysis{}, errors.New("run not found")
	}
	return r.Attitude.Analyze(run.Operation, parameters)
}
