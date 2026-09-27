package api

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/wyw14/solo-037/internal/service"
)

func TestAttitudeHTTPFlow(t *testing.T) {
	runtime, err := service.OpenRuntime(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	server := NewRouter(runtime).Handler()

	created := httptest.NewRecorder()
	server.ServeHTTP(created, httptest.NewRequest(http.MethodPost, "/api/measured-runs", strings.NewReader(`{"name":"tow B","model_name":"hull B","section":"section-a","carriage":"carriage-1"}`)))
	if created.Code != http.StatusOK {
		t.Fatalf("create run: %d %s", created.Code, created.Body)
	}
	var runResponse struct {
		Data service.MeasuredRun `json:"data"`
	}
	if err := json.Unmarshal(created.Body.Bytes(), &runResponse); err != nil {
		t.Fatal(err)
	}
	id := string(runResponse.Data.Operation.ID)

	registered := httptest.NewRecorder()
	server.ServeHTTP(registered, httptest.NewRequest(http.MethodPost, "/api/measured-runs/"+id+"/attitude-sensors", strings.NewReader(`{"sensor_id":"imu-1","label":"bow IMU","mount_location":"bow","expected_interval_ms":100,"threshold_deg":5}`)))
	if registered.Code != http.StatusOK {
		t.Fatalf("register sensor: %d %s", registered.Code, registered.Body)
	}
	invalid := httptest.NewRecorder()
	server.ServeHTTP(invalid, httptest.NewRequest(http.MethodPost, "/api/measured-runs/"+id+"/attitude-sensors", strings.NewReader(`{"sensor_id":"imu-2","label":"no interval"}`)))
	if invalid.Code != http.StatusBadRequest {
		t.Fatalf("registration without interval must fail: %d %s", invalid.Code, invalid.Body)
	}

	listed := httptest.NewRecorder()
	server.ServeHTTP(listed, httptest.NewRequest(http.MethodGet, "/api/measured-runs/"+id+"/attitude-sensors", nil))
	if listed.Code != http.StatusOK || !strings.Contains(listed.Body.String(), `"sensor_id":"imu-1"`) {
		t.Fatalf("list sensors: %d %s", listed.Code, listed.Body)
	}

	base := time.Now().Add(-time.Minute).UTC()
	stamp := func(ms int) string {
		return base.Add(time.Duration(ms) * time.Millisecond).Format(time.RFC3339Nano)
	}
	for _, body := range []string{
		`{"kind":"roll","source_id":"imu-1","value":1.0,"recorded_at":"` + stamp(0) + `"}`,
		`{"kind":"roll","source_id":"imu-1","value":6.5,"recorded_at":"` + stamp(100) + `"}`,
		`{"kind":"roll","source_id":"imu-1","value":7.5,"recorded_at":"` + stamp(200) + `"}`,
		`{"kind":"roll","source_id":"imu-1","value":0.5,"recorded_at":"` + stamp(900) + `"}`,
		`{"kind":"pitch","source_id":"imu-1","value":0.3,"recorded_at":"` + stamp(0) + `"}`,
		`{"kind":"position","source_id":"carriage","value":11.5,"recorded_at":"` + stamp(210) + `"}`,
	} {
		recorded := httptest.NewRecorder()
		server.ServeHTTP(recorded, httptest.NewRequest(http.MethodPost, "/api/measured-runs/"+id+"/evidence", strings.NewReader(body)))
		if recorded.Code != http.StatusOK {
			t.Fatalf("record evidence %s: %d %s", body, recorded.Code, recorded.Body)
		}
	}

	analysis := httptest.NewRecorder()
	server.ServeHTTP(analysis, httptest.NewRequest(http.MethodGet, "/api/measured-runs/"+id+"/attitude-analysis", nil))
	if analysis.Code != http.StatusOK {
		t.Fatalf("analysis: %d %s", analysis.Code, analysis.Body)
	}
	body := analysis.Body.String()
	for _, fragment := range []string{`"value":7.5`, `"position":11.5`, `"sustained":true`, `"gaps":[{"axis":"roll"`, `"no_data":false`} {
		if !strings.Contains(body, fragment) {
			t.Fatalf("analysis missing %s: %s", fragment, body)
		}
	}

	missing := httptest.NewRecorder()
	server.ServeHTTP(missing, httptest.NewRequest(http.MethodGet, "/api/measured-runs/unknown/attitude-analysis", nil))
	if missing.Code != http.StatusNotFound {
		t.Fatalf("unknown run analysis: %d %s", missing.Code, missing.Body)
	}
}
