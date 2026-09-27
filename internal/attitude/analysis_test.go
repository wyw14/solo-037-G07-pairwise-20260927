package attitude

import (
	"testing"
	"time"

	"github.com/wyw14/solo-037/internal/model"
)

func sample(run model.RunID, source string, kind model.EvidenceKind, value float64, at time.Time) model.Evidence {
	return model.NewEvidence(run, model.NewRevisionID(), source, kind, value, at)
}

func TestAnalyzeMarksExtremaAbnormalIntervalsAndGaps(t *testing.T) {
	run := model.NewRunID()
	base := time.Date(2026, 9, 27, 10, 0, 0, 0, time.UTC)
	at := func(ms int) time.Time { return base.Add(time.Duration(ms) * time.Millisecond) }

	sensor, err := NewSensor(run, "imu-1", "bow IMU", "bow", 100*time.Millisecond, 5, base)
	if err != nil {
		t.Fatal(err)
	}
	idle, err := NewSensor(run, "imu-idle", "spare IMU", "stern", 100*time.Millisecond, 5, base)
	if err != nil {
		t.Fatal(err)
	}
	samples := []model.Evidence{
		sample(run, "imu-1", model.EvidenceRoll, 2.0, at(0)),
		sample(run, "imu-1", model.EvidenceRoll, 6.0, at(100)),
		sample(run, "imu-1", model.EvidenceRoll, -7.5, at(200)),
		sample(run, "imu-1", model.EvidenceRoll, 1.0, at(1000)),
		sample(run, "imu-1", model.EvidencePitch, 12.0, at(0)),
		sample(run, "imu-1", model.EvidencePitch, 1.0, at(100)),
	}
	positions := []model.Evidence{
		sample(run, "carriage", model.EvidencePosition, 10.0, at(50)),
		sample(run, "carriage", model.EvidencePosition, 12.5, at(220)),
	}

	result := Analyze([]Sensor{sensor, idle}, samples, positions, base.Add(time.Hour))
	if len(result.Sensors) != 2 {
		t.Fatalf("expected 2 sensor analyses: %+v", result.Sensors)
	}

	primary := result.Sensors[0]
	if primary.NoData {
		t.Fatal("imu-1 wrongly marked as having no data")
	}
	roll := primary.Axes[0]
	if roll.Axis != AxisRoll || roll.Samples != 4 {
		t.Fatalf("unexpected roll series: %+v", roll)
	}
	if roll.Extremum == nil || roll.Extremum.Value != -7.5 || !roll.Extremum.RecordedAt.Equal(at(200)) {
		t.Fatalf("roll extremum mismatch: %+v", roll.Extremum)
	}
	if roll.Extremum.CarriagePosition == nil || roll.Extremum.CarriagePosition.Position != 12.5 {
		t.Fatalf("extremum not linked to nearest carriage position: %+v", roll.Extremum.CarriagePosition)
	}
	if len(roll.AbnormalIntervals) != 1 {
		t.Fatalf("expected one sustained roll interval: %+v", roll.AbnormalIntervals)
	}
	interval := roll.AbnormalIntervals[0]
	if !interval.Start.Equal(at(100)) || !interval.End.Equal(at(200)) || interval.Samples != 2 || !interval.Sustained || interval.Peak != -7.5 {
		t.Fatalf("sustained interval mismatch: %+v", interval)
	}
	if interval.CarriagePosition == nil || interval.CarriagePosition.Position != 12.5 {
		t.Fatalf("interval peak not linked to carriage position: %+v", interval.CarriagePosition)
	}
	if len(roll.Gaps) != 1 || !roll.Gaps[0].Start.Equal(at(200)) || !roll.Gaps[0].End.Equal(at(1000)) || roll.Gaps[0].Duration != 800*time.Millisecond {
		t.Fatalf("roll outage not marked: %+v", roll.Gaps)
	}

	pitch := primary.Axes[1]
	if pitch.Axis != AxisPitch || pitch.Samples != 2 {
		t.Fatalf("unexpected pitch series: %+v", pitch)
	}
	if len(pitch.AbnormalIntervals) != 1 || pitch.AbnormalIntervals[0].Sustained || pitch.AbnormalIntervals[0].Samples != 1 {
		t.Fatalf("single exceedance must be a spike, not sustained: %+v", pitch.AbnormalIntervals)
	}
	if len(pitch.Gaps) != 0 {
		t.Fatalf("pitch has no outage: %+v", pitch.Gaps)
	}

	if !result.Sensors[1].NoData {
		t.Fatal("registered sensor without samples must be marked no_data")
	}
	if result.Sensors[1].Axes[0].Extremum != nil {
		t.Fatal("sensor without samples must not report extrema")
	}
}

func TestAnalyzeWithoutThresholdOrPositions(t *testing.T) {
	run := model.NewRunID()
	base := time.Date(2026, 9, 27, 10, 0, 0, 0, time.UTC)
	sensor, err := NewSensor(run, "imu-2", "free IMU", "", 50*time.Millisecond, 0, base)
	if err != nil {
		t.Fatal(err)
	}
	samples := []model.Evidence{
		sample(run, "imu-2", model.EvidenceRoll, 30.0, base),
		sample(run, "imu-2", model.EvidenceRoll, -31.0, base.Add(10*time.Millisecond)),
	}
	result := Analyze([]Sensor{sensor}, samples, nil, base)
	roll := result.Sensors[0].Axes[0]
	if len(roll.AbnormalIntervals) != 0 {
		t.Fatalf("zero threshold must disable abnormal detection: %+v", roll.AbnormalIntervals)
	}
	if roll.Extremum == nil || roll.Extremum.Value != -31.0 {
		t.Fatalf("extremum mismatch: %+v", roll.Extremum)
	}
	if roll.Extremum.CarriagePosition != nil {
		t.Fatalf("no position evidence means no carriage position: %+v", roll.Extremum.CarriagePosition)
	}
}

func TestNewSensorValidation(t *testing.T) {
	run := model.NewRunID()
	if _, err := NewSensor(run, "", "label", "", time.Second, 1, time.Now()); err == nil {
		t.Fatal("empty sensor id accepted")
	}
	if _, err := NewSensor(run, "imu", "", "", time.Second, 1, time.Now()); err == nil {
		t.Fatal("empty label accepted")
	}
	if _, err := NewSensor(run, "imu", "label", "", 0, 1, time.Now()); err == nil {
		t.Fatal("zero interval accepted")
	}
	if _, err := NewSensor(run, "imu", "label", "", time.Second, -1, time.Now()); err == nil {
		t.Fatal("negative threshold accepted")
	}
}
