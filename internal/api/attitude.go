package api

import (
	"net/http"

	"github.com/go-chi/chi/v5"
	"github.com/wyw14/solo-037/internal/attitude"
	"github.com/wyw14/solo-037/internal/model"
	"github.com/wyw14/solo-037/internal/service"
)

type AttitudeService interface {
	RegisterAttitudeSensor(model.RunID, service.AttitudeSensorInput) (attitude.Sensor, error)
	AttitudeSensors(model.RunID) ([]attitude.Sensor, bool)
	AnalyzeAttitude(model.RunID) (attitude.RunAnalysis, error)
}

func (r *Router) registerAttitudeSensor(w http.ResponseWriter, request *http.Request) {
	var input service.AttitudeSensorInput
	if err := decodeMeasurementBody(request, &input); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	value, err := r.dependencies.RegisterAttitudeSensor(model.RunID(chi.URLParam(request, "id")), input)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	writeJSON(w, value)
}

func (r *Router) attitudeSensors(w http.ResponseWriter, request *http.Request) {
	value, ok := r.dependencies.AttitudeSensors(model.RunID(chi.URLParam(request, "id")))
	if !ok {
		http.Error(w, "run not found", http.StatusNotFound)
		return
	}
	writeJSON(w, value)
}

func (r *Router) attitudeAnalysis(w http.ResponseWriter, request *http.Request) {
	value, err := r.dependencies.AnalyzeAttitude(model.RunID(chi.URLParam(request, "id")))
	if err != nil {
		http.Error(w, err.Error(), http.StatusNotFound)
		return
	}
	writeJSON(w, value)
}
