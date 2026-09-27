package api

import (
	"fmt"
	"net/http"
	"strconv"
	"time"

	"github.com/go-chi/chi/v5"
	"github.com/wyw14/solo-037/internal/attitude"
	"github.com/wyw14/solo-037/internal/model"
	"github.com/wyw14/solo-037/internal/service"
)

type AttitudeService interface {
	RegisterAttitudeSensor(code, label string, nominalInterval time.Duration) (model.AttitudeSensor, error)
	AttitudeSensors() []model.AttitudeSensor
	RecordAttitudeSample(model.RunID, service.AttitudeSampleInput) (model.AttitudeSample, error)
	AttitudeSamples(model.RunID) ([]model.AttitudeSample, error)
	AnalyzeAttitude(model.RunID, attitude.Parameters) (service.AttitudeAnalysis, error)
}

type registerAttitudeSensorRequest struct {
	Code              string  `json:"code"`
	Label             string  `json:"label"`
	NominalIntervalMs float64 `json:"nominal_interval_ms"`
}

func (r *Router) registerAttitudeSensor(w http.ResponseWriter, request *http.Request) {
	var input registerAttitudeSensorRequest
	if err := decodeMeasurementBody(request, &input); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	interval := time.Duration(input.NominalIntervalMs * float64(time.Millisecond))
	value, err := r.dependencies.RegisterAttitudeSensor(input.Code, input.Label, interval)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	writeJSON(w, value)
}

func (r *Router) attitudeSensors(w http.ResponseWriter, _ *http.Request) {
	writeJSON(w, r.dependencies.AttitudeSensors())
}

func (r *Router) recordAttitudeSample(w http.ResponseWriter, request *http.Request) {
	var input service.AttitudeSampleInput
	if err := decodeMeasurementBody(request, &input); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	value, err := r.dependencies.RecordAttitudeSample(model.RunID(chi.URLParam(request, "id")), input)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	writeJSON(w, value)
}

func (r *Router) attitudeSamples(w http.ResponseWriter, request *http.Request) {
	value, err := r.dependencies.AttitudeSamples(model.RunID(chi.URLParam(request, "id")))
	if err != nil {
		http.Error(w, err.Error(), http.StatusNotFound)
		return
	}
	writeJSON(w, value)
}

func attitudeParameters(request *http.Request) (attitude.Parameters, error) {
	parameters := attitude.DefaultParameters()
	query := request.URL.Query()
	if raw := query.Get("roll_limit"); raw != "" {
		value, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return parameters, fmt.Errorf("invalid roll_limit: %w", err)
		}
		parameters.RollLimit = value
	}
	if raw := query.Get("pitch_limit"); raw != "" {
		value, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return parameters, fmt.Errorf("invalid pitch_limit: %w", err)
		}
		parameters.PitchLimit = value
	}
	if raw := query.Get("min_duration"); raw != "" {
		value, err := time.ParseDuration(raw)
		if err != nil {
			return parameters, fmt.Errorf("invalid min_duration: %w", err)
		}
		parameters.MinDuration = value
	}
	if raw := query.Get("gap_multiplier"); raw != "" {
		value, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return parameters, fmt.Errorf("invalid gap_multiplier: %w", err)
		}
		parameters.GapMultiplier = value
	}
	return parameters, nil
}

func (r *Router) attitudeAnalysis(w http.ResponseWriter, request *http.Request) {
	parameters, err := attitudeParameters(request)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	value, err := r.dependencies.AnalyzeAttitude(model.RunID(chi.URLParam(request, "id")), parameters)
	if err != nil {
		http.Error(w, err.Error(), http.StatusNotFound)
		return
	}
	writeJSON(w, value)
}
