package service

import (
	"testing"
	"time"

	"github.com/wyw14/solo-037/internal/model"
)

func TestAttitudeSensorsPersistAndAnalyzeAcrossReload(t *testing.T) {
	directory := t.TempDir()
	runtime, err := OpenRuntime(directory)
	if err != nil {
		t.Fatal(err)
	}
	run, err := runtime.CreateMeasuredRun("tow C", "hull Y", "section-a", "carriage-1")
	if err != nil {
		t.Fatal(err)
	}
	id := run.Operation.ID
	if _, err := runtime.RegisterAttitudeSensor("missing", AttitudeSensorInput{SensorID: "imu-1", Label: "bow IMU", ExpectedIntervalMS: 100, ThresholdDeg: 5}); err == nil {
		t.Fatal("registration for unknown run accepted")
	}
	if _, err := runtime.RegisterAttitudeSensor(id, AttitudeSensorInput{SensorID: "imu-1", Label: "bow IMU", MountLocation: "bow", ExpectedIntervalMS: 100, ThresholdDeg: 5}); err != nil {
		t.Fatal(err)
	}
	if _, err := runtime.RegisterAttitudeSensor(id, AttitudeSensorInput{SensorID: "imu-1", Label: "duplicate", ExpectedIntervalMS: 100}); err == nil {
		t.Fatal("duplicate sensor id accepted")
	}
	if _, err := runtime.RegisterAttitudeSensor(id, AttitudeSensorInput{SensorID: "imu-idle", Label: "spare IMU", ExpectedIntervalMS: 100, ThresholdDeg: 5}); err != nil {
		t.Fatal(err)
	}

	base := time.Now().Add(-time.Minute).UTC()
	at := func(ms int) time.Time { return base.Add(time.Duration(ms) * time.Millisecond) }
	for _, input := range []MeasurementInput{
		{Kind: model.EvidenceRoll, SourceID: "imu-1", Value: 1.5, RecordedAt: at(0)},
		{Kind: model.EvidenceRoll, SourceID: "imu-1", Value: 6.5, RecordedAt: at(100)},
		{Kind: model.EvidenceRoll, SourceID: "imu-1", Value: 7.0, RecordedAt: at(200)},
		{Kind: model.EvidenceRoll, SourceID: "imu-1", Value: 0.5, RecordedAt: at(900)},
		{Kind: model.EvidencePitch, SourceID: "imu-1", Value: 0.4, RecordedAt: at(0)},
		{Kind: model.EvidencePosition, SourceID: "carriage", Value: 8.0, RecordedAt: at(190)},
	} {
		if _, err := runtime.RecordMeasurement(id, input); err != nil {
			t.Fatal(err)
		}
	}

	reloaded, err := OpenRuntime(directory)
	if err != nil {
		t.Fatal(err)
	}
	sensors, ok := reloaded.AttitudeSensors(id)
	if !ok || len(sensors) != 2 || sensors[0].SensorID != "imu-1" || sensors[0].ExpectedInterval != 100*time.Millisecond {
		t.Fatalf("registered sensors not persisted: %+v", sensors)
	}
	analysis, err := reloaded.AnalyzeAttitude(id)
	if err != nil {
		t.Fatal(err)
	}
	if len(analysis.Sensors) != 2 {
		t.Fatalf("expected 2 sensor analyses: %+v", analysis.Sensors)
	}
	roll := analysis.Sensors[0].Axes[0]
	if roll.Extremum == nil || roll.Extremum.Value != 7.0 {
		t.Fatalf("roll extremum mismatch after reload: %+v", roll.Extremum)
	}
	if roll.Extremum.CarriagePosition == nil || roll.Extremum.CarriagePosition.Position != 8.0 {
		t.Fatalf("extremum not linked to carriage position: %+v", roll.Extremum.CarriagePosition)
	}
	if len(roll.AbnormalIntervals) != 1 || !roll.AbnormalIntervals[0].Sustained || roll.AbnormalIntervals[0].Peak != 7.0 {
		t.Fatalf("sustained abnormal interval missing: %+v", roll.AbnormalIntervals)
	}
	if len(roll.Gaps) != 1 || roll.Gaps[0].Duration != 700*time.Millisecond {
		t.Fatalf("sensor outage not marked: %+v", roll.Gaps)
	}
	if !analysis.Sensors[1].NoData {
		t.Fatal("sensor without samples must be marked no_data")
	}
	if _, err := reloaded.AnalyzeAttitude("missing"); err == nil {
		t.Fatal("analysis for unknown run accepted")
	}
}
