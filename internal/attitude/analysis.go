package attitude

import (
    "math"
    "sort"
    "time"

    "github.com/wyw14/solo-037/internal/model"
)

// Parameters controls anomaly and dropout evaluation. Limits are degrees of
// absolute angle; a violation run must span at least MinDuration to be
// reported, and a sample gap beyond GapMultiplier times the sensor's nominal
// interval is reported as a dropout and breaks any open violation run.
type Parameters struct {
    RollLimit float64 `json:"roll_limit"`
    PitchLimit float64 `json:"pitch_limit"`
    MinDuration time.Duration `json:"min_duration"`
    GapMultiplier float64 `json:"gap_multiplier"`
}

func DefaultParameters() Parameters {
    return Parameters{RollLimit: 10, PitchLimit: 5, MinDuration: 2 * time.Second, GapMultiplier: 2}
}

// Extreme is the largest absolute angle of one axis, located in time and
// along the tank.
type Extreme struct {
    Value float64 `json:"value"`
    At time.Time `json:"at"`
    CarriagePosition float64 `json:"carriage_position"`
}

type AnomalyInterval struct {
    Axis string `json:"axis"`
    Start time.Time `json:"start"`
    End time.Time `json:"end"`
    Duration time.Duration `json:"duration"`
    StartPosition float64 `json:"start_position"`
    EndPosition float64 `json:"end_position"`
    Peak float64 `json:"peak"`
    PeakAt time.Time `json:"peak_at"`
    PeakPosition float64 `json:"peak_position"`
    Samples int `json:"samples"`
}

// Gap is a dropout: two consecutive samples farther apart than the sensor's
// tolerated interval.
type Gap struct {
    Start time.Time `json:"start"`
    End time.Time `json:"end"`
    Duration time.Duration `json:"duration"`
    FromPosition float64 `json:"from_position"`
    ToPosition float64 `json:"to_position"`
    MissingEstimate int `json:"missing_estimate"`
}

type SensorAnalysis struct {
    Code string `json:"code"`
    Label string `json:"label"`
    Samples int `json:"samples"`
    First time.Time `json:"first"`
    Last time.Time `json:"last"`
    RollExtreme Extreme `json:"roll_extreme"`
    PitchExtreme Extreme `json:"pitch_extreme"`
    Anomalies []AnomalyInterval `json:"anomalies"`
    Gaps []Gap `json:"gaps"`
}

type Analysis struct {
    Parameters Parameters `json:"parameters"`
    Sensors []SensorAnalysis `json:"sensors"`
    SilentSensors []string `json:"silent_sensors"`
}

// Analyze evaluates every registered sensor against the run's samples.
// Registered sensors without any sample are reported as silent.
func Analyze(samples []model.AttitudeSample, sensors []model.AttitudeSensor, parameters Parameters) Analysis {
    bySource := make(map[string][]model.AttitudeSample)
    for _, sample := range samples {
        bySource[sample.SourceID] = append(bySource[sample.SourceID], sample)
    }
    known := make(map[string]model.AttitudeSensor, len(sensors))
    for _, sensor := range sensors {
        known[sensor.Code] = sensor
    }
    result := Analysis{Parameters: parameters, Sensors: []SensorAnalysis{}, SilentSensors: []string{}}
    for _, sensor := range sensors {
        if len(bySource[sensor.Code]) == 0 {
            result.SilentSensors = append(result.SilentSensors, sensor.Code)
        }
    }
    sort.Strings(result.SilentSensors)
    codes := make([]string, 0, len(bySource))
    for code := range bySource {
        codes = append(codes, code)
    }
    sort.Strings(codes)
    for _, code := range codes {
        result.Sensors = append(result.Sensors, analyzeSensor(known[code], bySource[code], parameters))
    }
    return result
}

func analyzeSensor(sensor model.AttitudeSensor, samples []model.AttitudeSample, parameters Parameters) SensorAnalysis {
    ordered := append([]model.AttitudeSample{}, samples...)
    sort.SliceStable(ordered, func(i int, j int) bool {
        return ordered[i].RecordedAt.Before(ordered[j].RecordedAt)
    })
    code := sensor.Code
    if code == "" {
        code = ordered[0].SourceID
    }
    result := SensorAnalysis{
        Code: code,
        Label: sensor.Label,
        Samples: len(ordered),
        First: ordered[0].RecordedAt,
        Last: ordered[len(ordered)-1].RecordedAt,
        Anomalies: []AnomalyInterval{},
        Gaps: []Gap{},
    }
    rollExtreme, pitchExtreme := ordered[0], ordered[0]
    for _, sample := range ordered {
        if math.Abs(sample.Roll) > math.Abs(rollExtreme.Roll) {
            rollExtreme = sample
        }
        if math.Abs(sample.Pitch) > math.Abs(pitchExtreme.Pitch) {
            pitchExtreme = sample
        }
    }
    result.RollExtreme = Extreme{Value: rollExtreme.Roll, At: rollExtreme.RecordedAt, CarriagePosition: rollExtreme.CarriagePosition}
    result.PitchExtreme = Extreme{Value: pitchExtreme.Pitch, At: pitchExtreme.RecordedAt, CarriagePosition: pitchExtreme.CarriagePosition}
    tolerance := time.Duration(parameters.GapMultiplier * float64(sensor.NominalInterval))
    if sensor.NominalInterval > 0 && parameters.GapMultiplier > 0 {
        for index := 1; index < len(ordered); index++ {
            previous, current := ordered[index-1], ordered[index]
            gap := current.RecordedAt.Sub(previous.RecordedAt)
            if gap > tolerance {
                missing := int(gap/sensor.NominalInterval) - 1
                if missing < 0 {
                    missing = 0
                }
                result.Gaps = append(result.Gaps, Gap{
                    Start: previous.RecordedAt,
                    End: current.RecordedAt,
                    Duration: gap,
                    FromPosition: previous.CarriagePosition,
                    ToPosition: current.CarriagePosition,
                    MissingEstimate: missing,
                })
            }
        }
    }
    result.Anomalies = append(result.Anomalies, axisAnomalies(ordered, "roll", parameters.RollLimit, parameters.MinDuration, tolerance)...)
    result.Anomalies = append(result.Anomalies, axisAnomalies(ordered, "pitch", parameters.PitchLimit, parameters.MinDuration, tolerance)...)
    sort.SliceStable(result.Anomalies, func(i int, j int) bool {
        return result.Anomalies[i].Start.Before(result.Anomalies[j].Start)
    })
    return result
}

// axisAnomalies finds maximal runs of consecutive samples beyond the limit.
// A dropout (gap larger than tolerance) closes the run: an anomaly interval
// never spans missing data.
func axisAnomalies(ordered []model.AttitudeSample, axis string, limit float64, minimum time.Duration, tolerance time.Duration) []AnomalyInterval {
    value := func(sample model.AttitudeSample) float64 {
        if axis == "roll" {
            return sample.Roll
        }
        return sample.Pitch
    }
    result := []AnomalyInterval{}
    start := -1
    peak := 0
    close := func(end int) {
        if start < 0 {
            return
        }
        first, last := ordered[start], ordered[end]
        if last.RecordedAt.Sub(first.RecordedAt) >= minimum {
            peakSample := ordered[peak]
            result = append(result, AnomalyInterval{
                Axis: axis,
                Start: first.RecordedAt,
                End: last.RecordedAt,
                Duration: last.RecordedAt.Sub(first.RecordedAt),
                StartPosition: first.CarriagePosition,
                EndPosition: last.CarriagePosition,
                Peak: value(peakSample),
                PeakAt: peakSample.RecordedAt,
                PeakPosition: peakSample.CarriagePosition,
                Samples: end - start + 1,
            })
        }
        start = -1
    }
    for index, sample := range ordered {
        if start >= 0 && tolerance > 0 && sample.RecordedAt.Sub(ordered[index-1].RecordedAt) > tolerance {
            close(index - 1)
        }
        if math.Abs(value(sample)) > limit {
            if start < 0 {
                start = index
                peak = index
            }
            if math.Abs(value(sample)) > math.Abs(value(ordered[peak])) {
                peak = index
            }
        } else {
            close(index - 1)
        }
    }
    close(len(ordered) - 1)
    return result
}
