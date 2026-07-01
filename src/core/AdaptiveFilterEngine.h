#pragma once

#include <QObject>
#include <QHash>
#include <QString>
#include <QVector>

#include <climits>                 // INT_MIN (SliceState sentinels)

#include "core/OccupiedRegion.h"   // OccupiedRegion + measureOccupiedRegion()

namespace AetherSDR {

class SliceModel;

// AdaptiveFilterEngine — GUI-thread coordinator that, for each SSB slice with
// the adaptive filter enabled, measures the occupied bandwidth of the tuned
// signal every FFT frame and fits the RX passband to it — within operator
// bounds, stably (median -> peak-hold -> dwell) and smoothly (glide), falling
// back to the operator's selected filter when no confident fit is possible.
//
// It is NOT signal DSP and adds no thread: it is driven per frame by the
// existing queued spectrumReady, and drives the radio only through
// SliceModel::applyAdaptiveFilter() (the filter stays radio-authoritative).
class AdaptiveFilterEngine : public QObject {
    Q_OBJECT

public:
    explicit AdaptiveFilterEngine(QObject* parent = nullptr);

    // Drive one FFT frame for the slice that owns the pan it came from. The
    // caller resolves the pan span + noise floor (e.g. from SpectrumWidget) and
    // the slice. No-op unless the slice is SSB with the adaptive filter enabled.
    void processFrame(SliceModel* slice, double centerMhz, double bandwidthMhz,
                      const QVector<float>& binsDbm, float noiseFloorDbm);

    // Forget per-slice smoothing/glide state (e.g. on mode change or disable).
    void resetSlice(int sliceId);

private:
    struct SliceState {
        QVector<int> rawLow, rawHigh;   // recent raw measurements (median input)
        QVector<int> medLow, medHigh;   // recent medians (peak-hold input)
        int  candLow{0},  candHigh{0};  // last candidate (for dwell tracking)
        int  dwell{0};                  // consecutive frames candidate held
        int  refractory{0};             // settle countdown after a committed change
        int  sinceWrite{0};             // frames since last filt send (throttle)
        int  confScore{0};              // confidence integrator (Schmitt trigger)
        bool active{false};             // debounced AUTO state (no per-frame flicker)
        int  curLow{INT_MIN}, curHigh{INT_MIN};   // currently applied (signed)
        int  tgtLow{INT_MIN}, tgtHigh{INT_MIN};   // glide target (signed)
        int  baseLow{0}, baseHigh{0};   // operator's manual baseline (signed)
        bool haveBaseline{false};
        double lastFreqMhz{0.0};        // detect a tune -> re-fit fresh
        QString lastMode;               // detect USB<->LSB -> re-baseline (sign flip)
        quint64 lastUserEpoch{0};       // detect a manual filter edit -> disable
        int  framesSinceTune{1000};     // post-tune settle gate (starts "settled")
        QVector<float> avgEnv;          // temporal average (video averaging)
    };

    // Commit a glide target and step the live passband toward it.
    void glideToward(SliceModel* slice, SliceState& st, bool active);

    QHash<int, SliceState> m_state;
};

} // namespace AetherSDR
