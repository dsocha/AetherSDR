#include "AdaptiveFilterEngine.h"
#include "models/SliceModel.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>

namespace AetherSDR {

namespace {
    // ── Measurement constants (RFC #3878) ───────────────────────────────────
    // Starting values, biased toward stability. Expect on-air tuning before
    // release (the maintainer signs off on the DSP feel per the RFC).
    constexpr double kScanHz        = 6500.0; // scan this far from the carrier
    // Spectral-envelope smoothing: a moving average over kEnvHz suppresses
    // narrow spikes (hets/carriers) and speech fine structure, leaving the
    // voice "hump". Edge finding runs on the envelope, NOT the raw bins, so a
    // sharp spike on top of a broad ESSB signal can't fool the threshold.
    constexpr double kEnvHz         = 300.0;
    constexpr float  kEnvGateDb     = 5.0f;   // occupied edge: env >= floor + this
    constexpr float  kSignalGateDb  = 3.0f;   // env below floor+this = no signal
    constexpr double kSilenceHz     = 600.0;  // ...for this span => a true band edge
                                              // (wide enough to bridge deep
                                              // internal QSB notches; a separate
                                              // STRONGER lobe is still cut by the
                                              // pre-gap-level rebound test)
    constexpr float  kMinPeakDb     = 7.0f;   // env peak < floor+this => not confident.
                                              // Low enough to engage on weak-but-
                                              // clear signals (the waterfall shows
                                              // them; we should fit them). The
                                              // time-averaged envelope keeps noise
                                              // excursions ~1-3 dB, so floor+7 does
                                              // not false-engage on pure noise.
    // SSB-voice shape gate: voice energy starts near the carrier. If the
    // occupied band starts above this (e.g. a 1600-4000 data/het signal), it
    // isn't the SSB voice we're tuned to -> reject -> caller keeps the manual
    // filter. Generous (well above any real voice low-cut, well below 1600).
    constexpr int    kMaxVoiceLowCutHz = 600;
    // Fixed guardrails for a valid voice passband (applied within operator
    // bounds): low-cut never above kMaxLowCutHz (keep warmth), high-cut never
    // below kMinHighCutHz (keep intelligibility).
    constexpr int    kMaxLowCutHz   = 400;
    constexpr int    kMinHighCutHz  = 1800;
    // Splatter rejection by strength (RFC follow-up). (a) Anchor the wanted
    // signal's peak within this distance of the carrier so a much-STRONGER
    // adjacent station farther out can't hijack the threshold. (b) Past the
    // peak, if the envelope dips by >= kReboundDb and then rises >= kReboundDb
    // back up, that rebound is a DISTINCT lobe (a separate/splatter signal) —
    // cut the high-cut at the valley. A much-weaker splatter is already below
    // the peak-relative occupied threshold, so it never extends the edge.
    constexpr double kVoicePeakHz   = 2800.0;
    constexpr float  kReboundDb     = 8.0f;
    constexpr int    kMarginHz      = 150;    // intelligibility margin
    // Temporal averaging (video averaging): per-offset envelope EMA coefficient.
    // Slow (~0.06, time constant ~0.5 s) — averages noise down (like the
    // waterfall's persistence) so weak signals read reliably above the gate, and
    // keeps the measured edges from chasing speech/sibilance dynamics (float).
    constexpr float  kEnvAvgAlpha = 0.06f;

    // ── Pipeline constants (in frames; panadapter ~30 fps) ──────────────────
    // Biased toward LOCKING: once fitted, the passband should sit still and move
    // only on a real width change, not chase speech dynamics. A wider deadband
    // and longer dwell are the levers that stop the float.
    constexpr int    kMedianFrames  = 15;     // ~0.5 s single-frame outlier reject
    constexpr int    kHoldFrames    = 75;     // ~2.5 s edge peak-hold (rides QSB;
                                              // bounded, so no long-term drift)
    constexpr int    kDwellFrames   = 20;     // ~0.65 s confirm before WIDENING
    constexpr int    kDwellNarrowFrames = 36; // ~1.2 s confirm before NARROWING
    constexpr int    kRefractoryFrames  = 24;  // ~0.8 s settle after a change
    // After a tune, ignore filter changes for this long (band-stack restore can
    // land a frame or two after the retune) so they don't read as a manual edit.
    constexpr int    kPostTuneSettleFrames = 15;  // ~0.5 s
    // AUTO/active confidence integrator (Schmitt trigger). It both debounces the
    // AUTO badge AND decides when to hold the fit vs fall back to the default.
    // The decay is deliberately SLOW (and the ceiling high) so it rides natural
    // speech pauses — a between-words gap briefly drops envPeak below the gate
    // (reg invalid), and we must NOT fall back to the default for that. From a
    // full ceiling it now takes ~1.7 s of *continuous* invalid to release, so a
    // genuine signal loss still falls back, but ordinary pauses don't. (This is
    // the bounded replacement for the pause-riding the old spectral memory did.)
    constexpr int    kConfMax       = 60;
    constexpr int    kConfUp        = 4;
    constexpr int    kConfDown      = 1;
    constexpr int    kConfHigh      = 28;     // rise above -> AUTO latches ON (~0.25 s)
    constexpr int    kConfLow       = 10;     // fall below -> AUTO releases OFF (~1.7 s)
    constexpr int    kDeadbandHz    = 220;    // ignore sub-deadband wiggle (locks
                                              // the edge; only a real width change
                                              // > this commits a move)
    constexpr int    kSnapHz        = 50;     // 50 Hz grid
    constexpr int    kMinBwHz       = 50;     // never narrower than this
    // Glide + send throttle (RFC #3878 cond. 2 — no filt command storm). Emit
    // at most one filt per kSendIntervalFrames (~130 ms => <= ~8/s), each move
    // a proportion of the remaining distance so the passband converges smoothly
    // in a few sends rather than one write per frame.
    constexpr int    kSendIntervalFrames = 4;
    constexpr int    kGlideFracPct       = 50;  // % of remaining distance per send
    constexpr int    kGlideMinStepHz     = 60;  // ...but at least this much

    // Peak-hold percentile: hold the *sustained* wide extent without letting a
    // single brief over-wide reading dominate. 80 -> a value exceeded only ~20%
    // of the hold window, so a transient that occupies < ~20% of the window is
    // ignored (fixes the "high-cut ~200 Hz too wide for 1-3 s then snaps back"
    // — a sibilant/edge overshoot getting held by a plain max).
    constexpr int    kHoldPctHigh   = 80;     // for the high-cut (wider = bigger)
    constexpr int    kHoldPctLow    = 20;     // for the low-cut  (wider = smaller)

    int snap50(int hz) { return ((hz + kSnapHz / 2) / kSnapHz) * kSnapHz; }

    int medianOf(QVector<int> v)
    {
        if (v.isEmpty()) return 0;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    }

    // Value at the given percentile (0-100) of v.
    int percentileOf(QVector<int> v, int pct)
    {
        if (v.isEmpty()) return 0;
        std::sort(v.begin(), v.end());
        const int idx = std::clamp(pct * (static_cast<int>(v.size()) - 1) / 100,
                                   0, static_cast<int>(v.size()) - 1);
        return v[idx];
    }
}

OccupiedRegion measureOccupiedRegion(const QVector<float>& binsDbm,
                                     double centerMhz, double bandwidthMhz,
                                     double carrierMhz, const QString& mode,
                                     float noiseFloorDbm,
                                     QVector<float>& avgEnv)
{
    OccupiedRegion r;
    const int N = binsDbm.size();
    if (N < 32 || bandwidthMhz <= 0.0) return r;

    const double hzPerBin = bandwidthMhz * 1.0e6 / N;
    if (hzPerBin <= 0.0) return r;
    const double startMhz = centerMhz - bandwidthMhz / 2.0;
    const bool   isUsb    = (mode != QStringLiteral("LSB"));  // USB-family default

    const int carrierBin = static_cast<int>(
        std::lround((carrierMhz - startMhz) / bandwidthMhz * N));
    const int scanBins = std::max(8, static_cast<int>(kScanHz / hzPerBin));

    // Energy side: USB above the carrier (higher bins), LSB below (lower bins).
    int lo = isUsb ? carrierBin : carrierBin - scanBins;
    int hi = isUsb ? carrierBin + scanBins : carrierBin;
    lo = std::clamp(lo, 0, N - 1);
    hi = std::clamp(hi, 0, N - 1);
    if (hi - lo < 8) return r;

    // Noise floor: prefer the caller's rolling value; else local 10th percentile.
    float floorDbm = noiseFloorDbm;
    if (floorDbm <= -500.0f) {
        QVector<float> w(binsDbm.begin() + lo, binsDbm.begin() + hi + 1);
        std::sort(w.begin(), w.end());
        floorDbm = w[w.size() / 10];
    }

    // Spectral envelope: a moving average (in dB) over ~kEnvHz. dB-domain
    // averaging compresses spikes, so a narrow het/carrier sitting on top of a
    // broad ESSB signal barely shifts the envelope — fixing the under-fit where
    // a sharp spike's peak-relative threshold excluded the wider voice energy.
    // Prefix sum makes the per-bin average O(1).
    const int W = hi - lo + 1;
    QVector<double> pref(W + 1, 0.0);
    for (int i = 0; i < W; ++i) pref[i + 1] = pref[i] + binsDbm[lo + i];
    const int envHalf = std::max(1, static_cast<int>(kEnvHz / hzPerBin / 2.0));
    const auto env = [&](int bin) -> float {
        const int a = std::clamp(bin - envHalf, lo, hi) - lo;
        const int b = std::clamp(bin + envHalf, lo, hi) - lo;
        return static_cast<float>((pref[b + 1] - pref[a]) / (b - a + 1));
    };
    // Materialise the instantaneous envelope over the energy-side offsets.
    const int span = scanBins + 1;
    QVector<float> envInst(span);
    for (int o = 0; o < span; ++o)
        envInst[o] = env(isUsb ? carrierBin + o : carrierBin - o);

    // ── Temporal averaging (video averaging) ─────────────────────────────
    // Per-offset EMA of the envelope reduces frame-to-frame noise BEFORE the
    // edge threshold, so weak/medium-signal edges (which sit near the noise
    // floor) stop jittering and single noisy frames can't extend an edge.
    if (avgEnv.size() != span) {
        avgEnv = envInst;
    } else {
        for (int o = 0; o < span; ++o)
            avgEnv[o] += kEnvAvgAlpha * (envInst[o] - avgEnv[o]);
    }

    // The edge-finder runs on the temporally-averaged envelope (a stable mean).
    // We deliberately do NOT peak-hold this per-bin: a per-bin peak-hold
    // accumulates loud speech moments, transient splatter and noise excursions
    // over time, which slowly inflated the measured width (the "works for
    // ~10-20 s then drifts" bug). QSB is ridden instead at the bounded result
    // level by the edge peak-hold (kHoldFrames), which the tune-reset clears.
    const auto envAt = [&](int o) -> float { return avgEnv[o]; };

    // Envelope peak — the representative voice level (not a spike). Search only
    // the near-carrier window (~kVoicePeakHz): SSB voice energy is concentrated
    // close to the carrier, so this anchors the reference on the WANTED signal
    // and prevents a much-stronger adjacent station farther out from hijacking
    // the threshold (which would otherwise exclude the wanted signal).
    const int peakSearchBins = std::min(scanBins,
                                        static_cast<int>(kVoicePeakHz / hzPerBin));
    int   peakO   = 0;
    float envPeak = envAt(0);
    for (int o = 1; o <= peakSearchBins; ++o) {
        const float v = envAt(o);
        if (v > envPeak) { envPeak = v; peakO = o; }
    }
    if (envPeak < floorDbm + kMinPeakDb) return r;  // weak / ambiguous

    // Occupied threshold: FLOOR-relative (a fixed margin above the stable noise
    // floor). It must NOT be peak-relative — the TX filter edge is fixed, but a
    // peak-relative threshold rises/falls with the speaker's loudness, so the
    // measured edge would creep narrower on loud syllables and wider on quiet
    // ones (the "floats too wide and too narrow during continuous speech" bug).
    // Floor-relative pins the crossing to the actual TX cliff regardless of
    // level. Splatter is handled by the rebound cut + silence-stop + maxHigh
    // bound, not by clamping this threshold.
    const float occThr    = floorDbm + kEnvGateDb;
    const float floorGate = floorDbm + kSignalGateDb;
    const int   silBins   = std::max(1, static_cast<int>(kSilenceHz / hzPerBin));

    // Scan the contiguous occupied region from the CARRIER outward. The inner
    // edge (low-cut) is the first occupied bin; up to and past the peak all gaps
    // are bridged (near-carrier audio + the main hump are one station).
    //
    // INTERNAL GAP handling (frequency-selective fades inside the voice): when
    // energy resumes after a gap at a level CONSISTENT with the wanted signal's
    // decay (<= the pre-gap level + kReboundDb), it's the same signal recovering
    // — bridge it and keep going, even across a deep, wide notch. We only stop
    // at:
    //   (a) TRUE silence — below floorGate for silBins of contiguous spectrum
    //       (a real band edge: nothing resumes), or
    //   (b) a SEPARATE STRONGER lobe — energy resuming ABOVE the pre-gap level
    //       by kReboundDb (an adjacent / splatter station rising up).
    // Comparing the resumption to the PRE-GAP level (not to the valley floor) is
    // the fix: a deep internal fade recovers to <= its pre-gap level, so it is
    // no longer mistaken for a separate lobe and cut.
    int nearO = -1, farO = -1;
    float preGapLevel = envPeak;   // envelope level entering the current gap
    bool  inGap = false;
    for (int o = 0, silence = 0; o <= scanBins; ++o) {
        const float v = envAt(o);
        if (v >= occThr) {
            if (nearO < 0) nearO = o;
            // Resumption after a gap that rises above the pre-gap level is a
            // separate, stronger station — stop before it (only checked right
            // after a gap, so in-band formant rises don't false-trigger).
            if (inGap && o > peakO && v > preGapLevel + kReboundDb) break;
            farO = o;
            preGapLevel = v;     // track the (decaying) wanted-signal level
            inGap = false;
            silence = 0;
        } else if (nearO >= 0 && o > peakO) {
            inGap = true;
            if (v >= floorGate) silence = 0;
            else if (++silence > silBins) break;
        }
    }
    if (nearO < 0 || farO < 0) return r;  // nothing occupied above threshold

    // Offset indices -> audio cut magnitudes (Hz), plus intelligibility margin.
    int audioLow  = static_cast<int>(std::floor(nearO * hzPerBin)) - kMarginHz;
    int audioHigh = static_cast<int>(std::ceil (farO  * hzPerBin)) + kMarginHz;
    audioLow = std::max(0, audioLow);
    if (audioHigh - audioLow < kMinBwHz) return r;
    // SSB-voice shape gate: the energy must start near the carrier. A band that
    // starts well above it (e.g. a 1600-4000 data/het signal) isn't the SSB
    // voice we're tuned to -> reject (r.valid stays false) so the engine keeps
    // the operator's manual filter.
    if (audioLow > kMaxVoiceLowCutHz) return r;

    r.valid   = true;
    r.lowHz   = audioLow;
    r.highHz  = audioHigh;
    r.peakDbm = envPeak;
    return r;
}

AdaptiveFilterEngine::AdaptiveFilterEngine(QObject* parent) : QObject(parent) {}

void AdaptiveFilterEngine::resetSlice(int sliceId) { m_state.remove(sliceId); }

void AdaptiveFilterEngine::processFrame(SliceModel* slice, double centerMhz,
                                        double bandwidthMhz,
                                        const QVector<float>& binsDbm,
                                        float noiseFloorDbm)
{
    if (!slice) return;
    const QString mode = slice->mode();
    const bool isUsb = (mode == QStringLiteral("USB"));
    const bool ssb   = isUsb || (mode == QStringLiteral("LSB"));
    if (!ssb || !slice->adaptiveFilterEnabled()) {
        if (m_state.contains(slice->sliceId())) resetSlice(slice->sliceId());
        return;
    }

    SliceState& st = m_state[slice->sliceId()];

    // Clear the per-fit smoothing/measurement state for a fresh fit, WITHOUT
    // touching the baseline (the operator's manual filter). Shared by the tune
    // and mode-change resets so both always clear the same fields.
    const auto clearFit = [](SliceState& s) {
        s.rawLow.clear(); s.rawHigh.clear();
        s.medLow.clear(); s.medHigh.clear();
        s.candLow = 0; s.candHigh = 0;
        s.dwell = 0; s.refractory = 0; s.sinceWrite = 0;
        s.confScore = 0; s.active = false;
        s.avgEnv.clear();
    };

    // ── Tune detection: re-fit fresh on a frequency jump ────────────────────
    // Rotating-QSO case — when the operator tunes to a different station, drop
    // the previous station's smoothing so the filter re-fits promptly instead of
    // dragging the old width. NOTE: the baseline is deliberately KEPT — it is the
    // operator's manually-selected filter (the weak-signal fallback) and must
    // persist across tunes (re-capturing it from the current adaptive-driven
    // filter polluted the fallback after a frequency change).
    const double freqMhz = slice->frequency();
    if (st.lastFreqMhz != 0.0 && std::abs(freqMhz - st.lastFreqMhz) > 0.0003) {
        clearFit(st);
        st.framesSinceTune = 0;                 // open the post-tune settle window
    }
    st.lastFreqMhz = freqMhz;
    if (st.framesSinceTune < 1000000) ++st.framesSinceTune;

    // ── Mode change (USB <-> LSB): full re-baseline ─────────────────────────
    // The signed-offset convention flips between sidebands (USB filterLow =
    // +low-cut; LSB filterLow = -high-cut), and the radio applies the new mode's
    // default filter. Without this, the cached signed cur/tgt/baseline keep the
    // previous sideband's wrong-signed values, so the filter stays on the old
    // side and only crawls across as it glides. Re-capture the baseline from the
    // new mode's filter (haveBaseline=false below) and drop the smoothing.
    if (!st.lastMode.isEmpty() && st.lastMode != mode) {
        clearFit(st);
        st.haveBaseline = false;     // re-capture baseline from the new mode
        st.framesSinceTune = 0;
    }
    st.lastMode = mode;

    // ── Baseline tracking (the operator's selected filter) ──────────────────
    // The baseline is what we fall back to when there's no signal to fit. It must
    // track genuine operator edits but never be corrupted by our own writes or
    // the radio's async echoes of them. We detect a real operator edit via
    // SliceModel's userFilterEpoch(), which is bumped ONLY by setFilterWidth()
    // (preset click / passband drag) — never by applyAdaptiveFilter() or status
    // echoes. So a manual change is unambiguous regardless of engine state: we
    // re-baseline, re-sync cur/tgt to it, and re-fit from there (fixes "manual
    // filter lost" and "auto-adjust stops after a manual change").
    const int fLow = slice->filterLow();
    const int fHigh = slice->filterHigh();
    const quint64 userEpoch = slice->userFilterEpoch();
    const bool epochChanged = st.haveBaseline && userEpoch != st.lastUserEpoch;

    if (epochChanged && st.framesSinceTune >= kPostTuneSettleFrames) {
        // A genuine manual filter edit on a stable station — user intent wins
        // (RFC #3878 cond. 3). Turn the feature OFF; the operator re-enables it
        // explicitly. setAdaptiveFilterEnabled(false) clears AUTO + markers via
        // the model signals; resetSlice() drops our per-slice state.
        slice->setAdaptiveFilterEnabled(false);
        resetSlice(slice->sliceId());
        return;
    }

    if (!st.haveBaseline || epochChanged) {
        // First frame, post-tune re-capture, or a filter change within the
        // post-tune settle window (band-stack restore) — adopt as baseline and
        // re-fit cleanly; do NOT disable.
        st.baseLow = fLow; st.baseHigh = fHigh;
        st.curLow = fLow;  st.curHigh = fHigh;
        st.tgtLow = fLow;  st.tgtHigh = fHigh;
        st.haveBaseline = true;
        if (epochChanged) clearFit(st);   // operator/band-stack filter change -> re-fit cleanly
    }
    st.lastUserEpoch = userEpoch;

    const int minLow  = slice->adaptiveMinLowCut();
    const int maxHigh = slice->adaptiveMaxHighCut();

    // ── Measure (new single-signal edge-finder; NOT detectVoiceSignals) ──────
    const OccupiedRegion reg = measureOccupiedRegion(
        binsDbm, centerMhz, bandwidthMhz, slice->frequency(), mode, noiseFloorDbm,
        st.avgEnv);

    // Debounce the AUTO state with a Schmitt-trigger confidence integrator so a
    // weak/marginal signal cannot flicker the badge between AUTO and the value.
    st.confScore = reg.valid ? std::min(st.confScore + kConfUp, kConfMax)
                             : std::max(st.confScore - kConfDown, 0);
    if (!st.active && st.confScore >= kConfHigh)      st.active = true;
    else if (st.active && st.confScore <= kConfLow)   st.active = false;

    if (!reg.valid) {
        // A momentary measurement gap (e.g. a speech pause). If we are
        // confidently active, HOLD the current fit — do NOT lurch back to
        // baseline or wipe the smoothing, or the filter would oscillate on
        // every syllable. Only once confidence has truly decayed (active
        // false) do we fall back to the operator's selected filter.
        if (!st.active) {
            st.tgtLow = st.baseLow; st.tgtHigh = st.baseHigh;
            st.rawLow.clear(); st.rawHigh.clear();
            st.medLow.clear(); st.medHigh.clear();
            st.dwell = 0;
        }
        glideToward(slice, st, st.active);
        return;
    }

    // Clamp to operator bounds AND fixed SSB-voice guardrails, enforce MIN_BW,
    // snap to 50 Hz. low-cut in [minLow, 400] (keep warmth); high-cut in
    // [1800, maxHigh] (keep intelligibility). The combo ranges guarantee
    // minLow <= 400 < 1800 <= maxHigh, so the clamps never invert.
    int audioLow  = std::clamp(reg.lowHz,  minLow, kMaxLowCutHz);
    int audioHigh = std::clamp(reg.highHz, kMinHighCutHz, maxHigh);
    if (audioHigh - audioLow < kMinBwHz) audioHigh = audioLow + kMinBwHz;
    audioLow  = snap50(audioLow);
    audioHigh = snap50(audioHigh);

    // ── Smooth: median (outlier reject) -> peak-hold (expand fast/contract slow)
    st.rawLow.append(audioLow);   st.rawHigh.append(audioHigh);
    if (st.rawLow.size()  > kMedianFrames) st.rawLow.removeFirst();
    if (st.rawHigh.size() > kMedianFrames) st.rawHigh.removeFirst();
    st.medLow.append(medianOf(st.rawLow));
    st.medHigh.append(medianOf(st.rawHigh));
    if (st.medLow.size()  > kHoldFrames) st.medLow.removeFirst();
    if (st.medHigh.size() > kHoldFrames) st.medHigh.removeFirst();
    // Widest over the hold window: low-cut min (lower = wider), high-cut max.
    // Percentile (not min/max) so a single brief over-wide reading can't be
    // held for the whole window: low-cut takes the 20th pct (favours wider =
    // lower), high-cut the 80th pct (favours wider = higher), each ignoring the
    // outer ~20% of transient excursions.
    int holdLow  = percentileOf(st.medLow,  kHoldPctLow);
    int holdHigh = percentileOf(st.medHigh, kHoldPctHigh);
    holdLow  = std::max(holdLow,  minLow);
    holdHigh = std::min(holdHigh, maxHigh);

    // ── Inertia: candidate must hold (deadband + dwell) before committing ────
    const bool sameCandidate = std::abs(holdLow - st.candLow) <= kDeadbandHz &&
                               std::abs(holdHigh - st.candHigh) <= kDeadbandHz;
    if (sameCandidate) {
        ++st.dwell;
    } else {
        st.candLow = holdLow; st.candHigh = holdHigh; st.dwell = 0;
    }

    // Audio magnitudes -> signed filter offsets for this mode.
    int wantLo, wantHi;
    if (isUsb) { wantLo = holdLow;   wantHi = holdHigh; }
    else       { wantLo = -holdHigh; wantHi = -holdLow; }

    const bool differs = std::abs(wantLo - st.tgtLow) > kDeadbandHz ||
                         std::abs(wantHi - st.tgtHigh) > kDeadbandHz;
    // Narrowing cuts into the signal — the dangerous direction. Demand a much
    // longer confirmation for it than for widening, so a transient dip that
    // slipped past the gap-bridge + peak-hold still cannot pinch the passband.
    const int wantWidth = wantHi - wantLo;
    const int curWidth  = (st.tgtHigh == INT_MIN) ? 0 : st.tgtHigh - st.tgtLow;
    const bool narrowing = wantWidth < curWidth - kDeadbandHz;
    const int needDwell = narrowing ? kDwellNarrowFrames : kDwellFrames;
    // Settle after each change before allowing the next, so the filter feels
    // calm rather than continuously nudging.
    if (st.refractory > 0) --st.refractory;
    if (differs && st.dwell >= needDwell && st.refractory == 0) {
        st.tgtLow = wantLo; st.tgtHigh = wantHi;
        st.dwell = 0;
        st.refractory = kRefractoryFrames;
    }

    glideToward(slice, st, st.active);
}

void AdaptiveFilterEngine::glideToward(SliceModel* slice, SliceState& st, bool active)
{
    // AUTO-badge state is cheap and not a radio command — update every frame.
    slice->setAdaptiveActive(active);

    // Already at target: nothing to send (steady state stays silent).
    if (st.curLow == st.tgtLow && st.curHigh == st.tgtHigh) { st.sinceWrite = 0; return; }

    // Throttle: emit at most one filt per kSendIntervalFrames (anti-storm).
    if (++st.sinceWrite < kSendIntervalFrames) return;
    st.sinceWrite = 0;

    // Proportional step toward the target — smooth, converges in a few sends.
    const auto step = [](int cur, int tgt) {
        if (cur == INT_MIN) return tgt;
        const int d = tgt - cur;
        if (std::abs(d) <= kGlideMinStepHz) return tgt;
        const int s = std::max(kGlideMinStepHz, std::abs(d) * kGlideFracPct / 100);
        return cur + (d > 0 ? s : -s);
    };
    const int nextLow  = step(st.curLow,  st.tgtLow);
    const int nextHigh = step(st.curHigh, st.tgtHigh);

    if (nextLow != st.curLow || nextHigh != st.curHigh) {
        st.curLow = nextLow; st.curHigh = nextHigh;
        slice->applyAdaptiveFilter(nextLow, nextHigh);
    }
}

} // namespace AetherSDR
