package attitude

import (
	"math"
	"sort"
	"time"

	"github.com/wyw14/solo-037/internal/model"
)

// PositionFix is the carriage position nearest in time to an attitude event.
type PositionFix struct {
	Position   float64   `json:"position"`
	RecordedAt time.Time `json:"recorded_at"`
}

// Extremum is the largest-magnitude angle observed on one axis.
type Extremum struct {
	Axis             string       `json:"axis"`
	Value            float64      `json:"value"`
	RecordedAt       time.Time    `json:"recorded_at"`
	CarriagePosition *PositionFix `json:"carriage_position,omitempty"`
}

// AbnormalInterval is a maximal run of above-threshold samples. An interval
// with at least two samples is sustained; a single sample is a spike.
type AbnormalInterval struct {
	Axis             string        `json:"axis"`
	Start            time.Time     `json:"start"`
	End              time.Time     `json:"end"`
	Duration         time.Duration `json:"duration"`
	Peak             float64       `json:"peak"`
	Samples          int           `json:"samples"`
	Sustained        bool          `json:"sustained"`
	CarriagePosition *PositionFix  `json:"carriage_position,omitempty"`
}

// Gap marks a sensor outage: consecutive samples farther apart than the
// expected sampling interval.
type Gap struct {
	Axis     string        `json:"axis"`
	Start    time.Time     `json:"start"`
	End      time.Time     `json:"end"`
	Duration time.Duration `json:"duration"`
}

// AxisSeries is the time-series analysis of one axis of one sensor.
type AxisSeries struct {
	Axis              string             `json:"axis"`
	Samples           int                `json:"samples"`
	Extremum          *Extremum          `json:"extremum,omitempty"`
	AbnormalIntervals []AbnormalInterval `json:"abnormal_intervals"`
	Gaps              []Gap              `json:"gaps"`
}

// SensorAnalysis is the analysis of one registered sensor. NoData marks a
// sensor that was registered but never reported a sample.
type SensorAnalysis struct {
	Sensor Sensor       `json:"sensor"`
	NoData bool         `json:"no_data"`
	Axes   []AxisSeries `json:"axes"`
}

// RunAnalysis is the attitude analysis of one measured run.
type RunAnalysis struct {
	Run       model.RunID      `json:"run"`
	Sensors   []SensorAnalysis `json:"sensors"`
	Evaluated time.Time        `json:"evaluated_at"`
}

// Analyze computes the attitude time-series analysis for a run. samples are
// the run's roll/pitch evidence and positions the run's carriage-position
// evidence; both may be empty or unordered.
func Analyze(sensors []Sensor, samples []model.Evidence, positions []model.Evidence, evaluated time.Time) RunAnalysis {
	bySensor := make(map[string][]model.Evidence)
	for _, sample := range samples {
		bySensor[sample.SourceID] = append(bySensor[sample.SourceID], sample)
	}
	orderedPositions := sortByTime(positions)
	result := RunAnalysis{Evaluated: evaluated.UTC(), Sensors: []SensorAnalysis{}}
	for _, sensor := range sensors {
		result.Sensors = append(result.Sensors, analyzeSensor(sensor, bySensor[sensor.SensorID], orderedPositions))
	}
	return result
}

func analyzeSensor(sensor Sensor, samples []model.Evidence, positions []model.Evidence) SensorAnalysis {
	result := SensorAnalysis{Sensor: sensor, NoData: len(samples) == 0, Axes: []AxisSeries{}}
	for _, axis := range []struct {
		kind model.EvidenceKind
		name string
	}{
		{model.EvidenceRoll, AxisRoll},
		{model.EvidencePitch, AxisPitch},
	} {
		series := AxisSeries{Axis: axis.name, AbnormalIntervals: []AbnormalInterval{}, Gaps: []Gap{}}
		ordered := selectKind(samples, axis.kind)
		series.Samples = len(ordered)
		if len(ordered) > 0 {
			extremum := findExtremum(axis.name, ordered, positions)
			series.Extremum = &extremum
			series.AbnormalIntervals = abnormalIntervals(sensor, axis.name, ordered, positions)
			series.Gaps = findGaps(sensor, axis.name, ordered)
		}
		result.Axes = append(result.Axes, series)
	}
	return result
}

func selectKind(samples []model.Evidence, kind model.EvidenceKind) []model.Evidence {
	ordered := make([]model.Evidence, 0, len(samples))
	for _, sample := range samples {
		if sample.Kind == kind {
			ordered = append(ordered, sample)
		}
	}
	return sortByTime(ordered)
}

func sortByTime(samples []model.Evidence) []model.Evidence {
	ordered := append([]model.Evidence(nil), samples...)
	sort.SliceStable(ordered, func(i, j int) bool {
		return ordered[i].RecordedAt.Before(ordered[j].RecordedAt)
	})
	return ordered
}

func findExtremum(axis string, ordered []model.Evidence, positions []model.Evidence) Extremum {
	best := ordered[0]
	for _, sample := range ordered[1:] {
		if math.Abs(sample.Value) > math.Abs(best.Value) {
			best = sample
		}
	}
	return Extremum{
		Axis:             axis,
		Value:            best.Value,
		RecordedAt:       best.RecordedAt,
		CarriagePosition: nearestPosition(positions, best.RecordedAt),
	}
}

func abnormalIntervals(sensor Sensor, axis string, ordered []model.Evidence, positions []model.Evidence) []AbnormalInterval {
	result := []AbnormalInterval{}
	if sensor.ThresholdDeg <= 0 {
		return result
	}
	var current []model.Evidence
	flush := func() {
		if len(current) == 0 {
			return
		}
		peak := current[0]
		for _, sample := range current[1:] {
			if math.Abs(sample.Value) > math.Abs(peak.Value) {
				peak = sample
			}
		}
		start := current[0].RecordedAt
		end := current[len(current)-1].RecordedAt
		result = append(result, AbnormalInterval{
			Axis:             axis,
			Start:            start,
			End:              end,
			Duration:         end.Sub(start),
			Peak:             peak.Value,
			Samples:          len(current),
			Sustained:        len(current) >= 2,
			CarriagePosition: nearestPosition(positions, peak.RecordedAt),
		})
		current = nil
	}
	for _, sample := range ordered {
		if math.Abs(sample.Value) <= sensor.ThresholdDeg {
			flush()
			continue
		}
		if len(current) > 0 && sample.RecordedAt.Sub(current[len(current)-1].RecordedAt) > sensor.ExpectedInterval {
			// an outage inside an abnormal stretch breaks continuity
			flush()
		}
		current = append(current, sample)
	}
	flush()
	return result
}

func findGaps(sensor Sensor, axis string, ordered []model.Evidence) []Gap {
	result := []Gap{}
	for index := 1; index < len(ordered); index++ {
		previous := ordered[index-1].RecordedAt
		current := ordered[index].RecordedAt
		if current.Sub(previous) > sensor.ExpectedInterval {
			result = append(result, Gap{Axis: axis, Start: previous, End: current, Duration: current.Sub(previous)})
		}
	}
	return result
}

// nearestPosition returns the carriage position closest in time to at, or
// nil when the run has no position evidence. positions must be time-ordered.
func nearestPosition(positions []model.Evidence, at time.Time) *PositionFix {
	if len(positions) == 0 {
		return nil
	}
	index := sort.Search(len(positions), func(i int) bool {
		return !positions[i].RecordedAt.Before(at)
	})
	best := -1
	for _, candidate := range []int{index - 1, index} {
		if candidate < 0 || candidate >= len(positions) {
			continue
		}
		if best == -1 || absDuration(positions[candidate].RecordedAt.Sub(at)) < absDuration(positions[best].RecordedAt.Sub(at)) {
			best = candidate
		}
	}
	fix := PositionFix{Position: positions[best].Value, RecordedAt: positions[best].RecordedAt}
	return &fix
}

func absDuration(value time.Duration) time.Duration {
	if value < 0 {
		return -value
	}
	return value
}
