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

func TestAttitudeHTTPRegistersSensorsAndAnalyzesSamples(t *testing.T) {
	runtime, err := service.OpenRuntime(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	server := NewRouter(runtime).Handler()
	created := httptest.NewRecorder()
	server.ServeHTTP(created, httptest.NewRequest(http.MethodPost, "/api/measured-runs", strings.NewReader(`{"name":"tow A","model_name":"hull A","section":"section-a","carriage":"carriage-1"}`)))
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
	server.ServeHTTP(registered, httptest.NewRequest(http.MethodPost, "/api/attitude-sensors", strings.NewReader(`{"code":"imu-1","label":"mast IMU","nominal_interval_ms":100}`)))
	if registered.Code != http.StatusOK {
		t.Fatalf("register: %d %s", registered.Code, registered.Body)
	}
	duplicate := httptest.NewRecorder()
	server.ServeHTTP(duplicate, httptest.NewRequest(http.MethodPost, "/api/attitude-sensors", strings.NewReader(`{"code":"imu-1","label":"again","nominal_interval_ms":100}`)))
	if duplicate.Code != http.StatusBadRequest {
		t.Fatalf("duplicate registration: %d %s", duplicate.Code, duplicate.Body)
	}
	stamp := time.Now().Add(-time.Second).UTC().Format(time.RFC3339Nano)
	rejected := httptest.NewRecorder()
	server.ServeHTTP(rejected, httptest.NewRequest(http.MethodPost, "/api/measured-runs/"+id+"/attitude", strings.NewReader(`{"source_id":"ghost","roll":1,"pitch":1,"carriage_position":0,"recorded_at":"`+stamp+`"}`)))
	if rejected.Code != http.StatusBadRequest {
		t.Fatalf("unregistered source: %d %s", rejected.Code, rejected.Body)
	}
	for _, body := range []string{
		`{"source_id":"imu-1","roll":12,"pitch":1,"carriage_position":2.5,"recorded_at":"` + stamp + `"}`,
		`{"source_id":"imu-1","roll":3,"pitch":2,"carriage_position":2.6,"recorded_at":"` + stamp + `"}`,
	} {
		recorded := httptest.NewRecorder()
		server.ServeHTTP(recorded, httptest.NewRequest(http.MethodPost, "/api/measured-runs/"+id+"/attitude", strings.NewReader(body)))
		if recorded.Code != http.StatusOK {
			t.Fatalf("record: %d %s", recorded.Code, recorded.Body)
		}
	}
	samples := httptest.NewRecorder()
	server.ServeHTTP(samples, httptest.NewRequest(http.MethodGet, "/api/measured-runs/"+id+"/attitude", nil))
	if samples.Code != http.StatusOK || !strings.Contains(samples.Body.String(), `"imu-1"`) {
		t.Fatalf("samples: %d %s", samples.Code, samples.Body)
	}
	analysis := httptest.NewRecorder()
	server.ServeHTTP(analysis, httptest.NewRequest(http.MethodGet, "/api/measured-runs/"+id+"/attitude-analysis", nil))
	if analysis.Code != http.StatusOK || !strings.Contains(analysis.Body.String(), `"roll_extreme"`) || !strings.Contains(analysis.Body.String(), `"value":12`) {
		t.Fatalf("analysis: %d %s", analysis.Code, analysis.Body)
	}
	tuned := httptest.NewRecorder()
	server.ServeHTTP(tuned, httptest.NewRequest(http.MethodGet, "/api/measured-runs/"+id+"/attitude-analysis?roll_limit=20&pitch_limit=1&min_duration=0s&gap_multiplier=3", nil))
	if tuned.Code != http.StatusOK || !strings.Contains(tuned.Body.String(), `"roll_limit":20`) {
		t.Fatalf("tuned analysis: %d %s", tuned.Code, tuned.Body)
	}
	invalid := httptest.NewRecorder()
	server.ServeHTTP(invalid, httptest.NewRequest(http.MethodGet, "/api/measured-runs/"+id+"/attitude-analysis?min_duration=soon", nil))
	if invalid.Code != http.StatusBadRequest {
		t.Fatalf("invalid parameter: %d %s", invalid.Code, invalid.Body)
	}
	missing := httptest.NewRecorder()
	server.ServeHTTP(missing, httptest.NewRequest(http.MethodGet, "/api/measured-runs/no-such-run/attitude-analysis", nil))
	if missing.Code != http.StatusNotFound {
		t.Fatalf("missing run: %d %s", missing.Code, missing.Body)
	}
}
