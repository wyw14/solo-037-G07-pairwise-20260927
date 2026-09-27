package model

import "time"

// AttitudeSensor is a registered roll/pitch sensor. Personnel reference it by
// Code when submitting angle samples, and its nominal interval is the
// baseline for dropout detection during analysis.
type AttitudeSensor struct {
    Code string `json:"code"`
    Label string `json:"label"`
    NominalInterval time.Duration `json:"nominal_interval"`
    RegisteredAt time.Time `json:"registered_at"`
}

func NewAttitudeSensor(code string, label string, nominalInterval time.Duration, at time.Time) AttitudeSensor {
    return AttitudeSensor{
        Code: code,
        Label: label,
        NominalInterval: nominalInterval,
        RegisteredAt: at.UTC(),
    }
}

// AttitudeSample is one operator-supplied roll/pitch observation, stamped
// with the carriage position so extremes and anomalies can be located along
// the tank.
type AttitudeSample struct {
    ID EventID `json:"id"`
    Run RunID `json:"run"`
    SourceID string `json:"source_id"`
    Roll float64 `json:"roll"`
    Pitch float64 `json:"pitch"`
    CarriagePosition float64 `json:"carriage_position"`
    RecordedAt time.Time `json:"recorded_at"`
    Durable bool `json:"durable"`
}

func NewAttitudeSample(run RunID, source string, roll float64, pitch float64, position float64, at time.Time) AttitudeSample {
    return AttitudeSample{
        ID: NewEventID(),
        Run: run,
        SourceID: source,
        Roll: roll,
        Pitch: pitch,
        CarriagePosition: position,
        RecordedAt: at.UTC(),
    }
}

func (s AttitudeSample) WithDurability(durable bool) AttitudeSample {
    s.Durable = durable
    return s
}
