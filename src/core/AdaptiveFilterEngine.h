#pragma once

#include <QObject>
#include <QHash>
#include <QString>
#include <QVector>

namespace AetherSDR {

class SliceModel;

// Result of a single-signal occupied-bandwidth measurement, in *audio*
// magnitudes (Hz from the suppressed carrier, always low < high). The caller
// maps these to signed filter offsets per mode (USB: lo=+low, hi=+high;
// LSB: lo=-high, hi=-low).
struct OccupiedRegion {
    bool   valid{false};   // a confident measurement was obtained
    int    lowHz{0};       // low-cut: nearest-carrier edge of the energy
    int    highHz{0};      // high-cut: far edge of the energy
    float  peakDbm{-1000.0f};
};

// measureOccupiedRegion — a NEW single-signal occupied-bandwidth edge-finder
// (RFC #3878). This is deliberately NOT VoiceSignalDetector::detectVoiceSignals(),
// which is a band-scan marker detector that splits wide regions into ~2.7 kHz
// chunks and would fragment a wide ESSB signal. Here we anchor on the slice
// carrier, scan only a local window on the signal's energy side, find the peak,
// and take the contiguous run above a peak-relative threshold (so a strength-
// independent width, and an adjacent station separated by a sub-threshold valley
// is naturally excluded — clarity over bandwidth). Only the noise-floor gating
// idea is shared with VoiceSignalDetector.
//
//  binsDbm        full-pan FFT magnitudes (dBm)
//  centerMhz/bandwidthMhz   the pan span
//  carrierMhz     the slice's suppressed-carrier frequency
//  mode           "USB" or "LSB" (selects the energy side)
//  noiseFloorDbm  rolling floor from SpectrumWidget (sentinel <= -500 => unknown)
//  avgEnv         in/out per-slice temporal average (video averaging): a
//                 per-offset EMA of the envelope that reduces frame-to-frame
//                 noise before the edge threshold — stabilises edges on
//                 weak/medium signals. Persistent per-slice; reinit on geometry
//                 change. (NOT a peak-hold — a per-bin peak-hold accumulated and
//                 inflated the width over time; QSB is ridden by the bounded
//                 edge peak-hold instead.)
OccupiedRegion measureOccupiedRegion(const QVector<float>& binsDbm,
                                     double centerMhz, double bandwidthMhz,
                                     double carrierMhz, const QString& mode,
                                     float noiseFloorDbm,
                                     QVector<float>& avgEnv);

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
