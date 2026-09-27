package service

import (
	"math"
	"testing"
	"time"

	"github.com/wyw14/solo-037/internal/attitude"
	"github.com/wyw14/solo-037/internal/model"
)

func TestAttitudeSensorsPersistAndSamplesValidate(t *testing.T) {
	directory := t.TempDir()
	a, err := OpenAttitudeArchive(directory)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := a.Register("imu-1", "mast IMU", 100*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if _, err := a.Register("imu-1", "duplicate", 100*time.Millisecond); err == nil {
		t.Fatal("duplicate sensor code accepted")
	}
	if _, err := a.Register("imu-2", "", 0); err == nil {
		t.Fatal("zero nominal interval accepted")
	}
	run := model.NewOperation("tow A", "hull X", "section-a", "carriage-1", time.Now().UTC())
	at := time.Now().Add(-time.Minute).UTC()
	if _, err := a.Record(run.ID, AttitudeSampleInput{SourceID: "ghost", Roll: 1, Pitch: 1, CarriagePosition: 0, RecordedAt: at}); err == nil {
		t.Fatal("unregistered source accepted")
	}
	if _, err := a.Record(run.ID, AttitudeSampleInput{SourceID: "imu-1", Roll: 200, Pitch: 1, CarriagePosition: 0, RecordedAt: at}); err == nil {
		t.Fatal("roll beyond ±180 accepted")
	}
	if _, err := a.Record(run.ID, AttitudeSampleInput{SourceID: "imu-1", Roll: 1, Pitch: math.NaN(), CarriagePosition: 0, RecordedAt: at}); err == nil {
		t.Fatal("NaN pitch accepted")
	}
	sample, err := a.Record(run.ID, AttitudeSampleInput{SourceID: "imu-1", Roll: 3, Pitch: -1, CarriagePosition: 1.5, RecordedAt: at})
	if err != nil {
		t.Fatal(err)
	}
	if !sample.Durable {
		t.Fatal("persisted sample not marked durable")
	}
	loaded, err := OpenAttitudeArchive(directory)
	if err != nil {
		t.Fatal(err)
	}
	if len(loaded.Sensors()) != 1 || loaded.Sensors()[0].Code != "imu-1" {
		t.Fatalf("sensors not reloaded: %+v", loaded.Sensors())
	}
	samples := loaded.Samples(run.ID)
	if len(samples) != 1 || samples[0].Roll != 3 || samples[0].CarriagePosition != 1.5 {
		t.Fatalf("samples not reloaded: %+v", samples)
	}
}

func TestAttitudeAnalysisExtremesAnomaliesGapsAndSilentSensors(t *testing.T) {
	a, err := OpenAttitudeArchive("")
	if err != nil {
		t.Fatal(err)
	}
	if _, err := a.Register("imu-1", "mast IMU", 100*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	if _, err := a.Register("imu-idle", "spare", 100*time.Millisecond); err != nil {
		t.Fatal(err)
	}
	run := model.NewOperation("tow B", "hull Y", "section-a", "carriage-1", time.Now().UTC())
	base := time.Now().Add(-time.Minute).UTC()
	// One dropout between 400ms and 900ms; roll exceeds the limit across
	// 200-400ms and once (too briefly) after the gap.
	type point struct {
		offsetMs int
		roll     float64
		pitch    float64
	}
	points := []point{
		{0, 2, 1}, {100, 3, 2}, {200, 12, 1}, {300, 15, 2}, {400, -13, 1},
		{900, 20, 1}, {1000, 2, 8},
	}
	for _, p := range points {
		at := base.Add(time.Duration(p.offsetMs) * time.Millisecond)
		position := float64(p.offsetMs) / 100
		if _, err := a.Record(run.ID, AttitudeSampleInput{SourceID: "imu-1", Roll: p.roll, Pitch: p.pitch, CarriagePosition: position, RecordedAt: at}); err != nil {
			t.Fatal(err)
		}
	}
	parameters := attitude.Parameters{RollLimit: 10, PitchLimit: 5, MinDuration: 100 * time.Millisecond, GapMultiplier: 2}
	analysis, err := a.Analyze(run, parameters)
	if err != nil {
		t.Fatal(err)
	}
	if len(analysis.Sensors) != 1 {
		t.Fatalf("expected one analyzed sensor: %+v", analysis.Sensors)
	}
	sensor := analysis.Sensors[0]
	if sensor.Samples != 7 {
		t.Fatalf("sample count: %+v", sensor)
	}
	if sensor.RollExtreme.Value != 20 || sensor.RollExtreme.CarriagePosition != 9 {
		t.Fatalf("roll extreme not located: %+v", sensor.RollExtreme)
	}
	if sensor.PitchExtreme.Value != 8 || sensor.PitchExtreme.CarriagePosition != 10 {
		t.Fatalf("pitch extreme not located: %+v", sensor.PitchExtreme)
	}
	if len(sensor.Anomalies) != 1 {
		t.Fatalf("expected one sustained anomaly: %+v", sensor.Anomalies)
	}
	anomaly := sensor.Anomalies[0]
	if anomaly.Axis != "roll" || anomaly.Samples != 3 || anomaly.Peak != 15 {
		t.Fatalf("anomaly content: %+v", anomaly)
	}
	if anomaly.Start != base.Add(200*time.Millisecond) || anomaly.End != base.Add(400*time.Millisecond) {
		t.Fatalf("anomaly interval: %+v", anomaly)
	}
	if anomaly.StartPosition != 2 || anomaly.EndPosition != 4 || anomaly.PeakPosition != 3 {
		t.Fatalf("anomaly positions: %+v", anomaly)
	}
	if len(sensor.Gaps) != 1 {
		t.Fatalf("expected one dropout: %+v", sensor.Gaps)
	}
	gap := sensor.Gaps[0]
	if gap.Start != base.Add(400*time.Millisecond) || gap.End != base.Add(900*time.Millisecond) || gap.Duration != 500*time.Millisecond {
		t.Fatalf("gap interval: %+v", gap)
	}
	if gap.MissingEstimate != 4 || gap.FromPosition != 4 || gap.ToPosition != 9 {
		t.Fatalf("gap detail: %+v", gap)
	}
	if len(analysis.SilentSensors) != 1 || analysis.SilentSensors[0] != "imu-idle" {
		t.Fatalf("silent sensors: %+v", analysis.SilentSensors)
	}
}

func TestAttitudeAnalysisRejectsInvalidParameters(t *testing.T) {
	a, err := OpenAttitudeArchive("")
	if err != nil {
		t.Fatal(err)
	}
	run := model.NewOperation("tow C", "hull Z", "section-a", "carriage-1", time.Now().UTC())
	valid := attitude.DefaultParameters()
	invalid := []attitude.Parameters{
		{RollLimit: 0, PitchLimit: 5, MinDuration: 0, GapMultiplier: 2},
		{RollLimit: 10, PitchLimit: -1, MinDuration: 0, GapMultiplier: 2},
		{RollLimit: 10, PitchLimit: 5, MinDuration: -time.Second, GapMultiplier: 2},
		{RollLimit: 10, PitchLimit: 5, MinDuration: 0, GapMultiplier: 0.5},
	}
	for _, parameters := range invalid {
		if _, err := a.Analyze(run, parameters); err == nil {
			t.Fatalf("invalid parameters accepted: %+v", parameters)
		}
	}
	if _, err := a.Analyze(run, valid); err != nil {
		t.Fatal(err)
	}
}
