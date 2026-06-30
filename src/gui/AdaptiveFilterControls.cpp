#include "AdaptiveFilterControls.h"

#include "ComboStyle.h"
#include "core/AppSettings.h"
#include "models/SliceModel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>

namespace AetherSDR {

AdaptiveFilterControls::AdaptiveFilterControls(bool withHeader, bool compact,
                                               QWidget* parent)
    : QWidget(parent)
{
    // Compact preset matches the dense applet rows (smaller text, shorter combos);
    // the roomier preset suits the VFO flag.
    const int fpx       = compact ? 11 : 12;   // label/checkbox font px
    const int comboPx   = compact ? 11 : 12;
    const int comboMaxH = compact ? 22 : 0;    // 0 = no cap

    auto* av = new QVBoxLayout(this);
    av->setContentsMargins(0, compact ? 0 : 2, 0, 0);
    av->setSpacing(compact ? 2 : 3);

    if (withHeader) {
        // Separator — same style as the S-meter/SmartMTR config menu.
        auto* sep = new QFrame;
        sep->setFrameShape(QFrame::HLine);
        sep->setFrameShadow(QFrame::Plain);
        sep->setFixedHeight(1);
        sep->setStyleSheet("QFrame { border: none; background: #304050; max-height: 1px; }");
        sep->setAttribute(Qt::WA_TransparentForMouseEvents);
        av->addWidget(sep);
    }

    // Checkbox — "Adaptive RX filter".
    m_chk = new QCheckBox(tr("Adaptive RX filter"));
    m_chk->setCursor(Qt::PointingHandCursor);
    m_chk->setStyleSheet(QStringLiteral(
        "QCheckBox { background: transparent; color: #c8d8e8; font-size: %1px; spacing: 5px; }"
        "QCheckBox::indicator { width: 13px; height: 13px; border-radius: 2px; "
        "border: 1px solid #304050; background: #1a2a3a; }"
        "QCheckBox::indicator:checked { background: #0070c0; border: 1px solid #0090e0; }"
        "QCheckBox:disabled { color: #5a6a78; }"
        "QCheckBox::indicator:disabled { border: 1px solid #243240; background: #141f2a; }")
        .arg(fpx));
    m_chk->setAccessibleName(tr("Adaptive RX filter"));
    m_chk->setAccessibleDescription(
        tr("Automatically fit the SSB RX passband to the received signal width"));
    av->addWidget(m_chk);

    const auto makeOptLabel = [fpx](const QString& text) {
        auto* lbl = new QLabel(text);
        lbl->setStyleSheet(QStringLiteral(
            "QLabel { background: transparent; border: none; "
            "color: #c8d8e8; font-size: %1px; } "
            "QLabel:disabled { color: #5e6e7c; }").arg(fpx));
        return lbl;
    };

    // A labelled row holding a stretched combo (no fixed width -> flexes to host).
    const auto makeRow = [&](const QString& label, const QString& accName)
        -> std::pair<QWidget*, QComboBox*> {
        auto* row = new QWidget;
        row->setAttribute(Qt::WA_TranslucentBackground);
        auto* lay = new QHBoxLayout(row);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(4);
        lay->addWidget(makeOptLabel(label));
        auto* cmb = new QComboBox;
        AetherSDR::applyComboStyle(cmb);
        QFont cf = cmb->font(); cf.setPixelSize(comboPx); cmb->setFont(cf);
        if (comboMaxH > 0) cmb->setMaximumHeight(comboMaxH);
        cmb->setAccessibleName(accName);
        lay->addWidget(cmb, 1);
        av->addWidget(row);
        return {row, cmb};
    };

    // Bound combos (Hz value stored as item data).
    std::tie(m_loRow, m_minLow) = makeRow(tr("Min low-cut"), tr("Adaptive minimum low cut (Hz)"));
    for (int v : {0, 50, 100, 200}) m_minLow->addItem(QString::number(v), v);
    std::tie(m_hiRow, m_maxHigh) = makeRow(tr("Max high-cut"), tr("Adaptive maximum high cut (Hz)"));
    for (int v : {3000, 3500, 4000, 6000}) m_maxHigh->addItem(QString::number(v), v);

    // Preset combos — the combo INDEX is the level (0/1/2) stored on the slice.
    std::tie(m_snrRow, m_minSnr) = makeRow(tr("Min SNR"), tr("Adaptive minimum SNR"));
    for (const QString& o : {tr("Sensitive"), tr("Normal"), tr("Strong")}) m_minSnr->addItem(o);
    std::tie(m_responseRow, m_response) = makeRow(tr("Response"), tr("Adaptive response speed"));
    for (const QString& o : {tr("Fast"), tr("Normal"), tr("Slow")}) m_response->addItem(o);
    std::tie(m_splatterRow, m_splatter) = makeRow(tr("Splatter"), tr("Adaptive splatter rejection"));
    for (const QString& o : {tr("Tight"), tr("Normal"), tr("Wide")}) m_splatter->addItem(o);

    // ── User-action handlers (act on the currently bound slice) ─────────────
    // Cross-instance updates set controls under QSignalBlocker, so these never
    // refire from a programmatic sync — only the touched widget persists.
    connect(m_chk, &QCheckBox::toggled, this, [this](bool on) {
        if (m_slice) { m_slice->setAdaptiveFilterEnabled(on); savePrefs(m_slice); }
        updateVisibility();
    });
    connect(m_minLow, &QComboBox::currentIndexChanged, this, [this] {
        if (m_slice) { m_slice->setAdaptiveMinLowCut(m_minLow->currentData().toInt()); savePrefs(m_slice); }
    });
    connect(m_maxHigh, &QComboBox::currentIndexChanged, this, [this] {
        if (m_slice) { m_slice->setAdaptiveMaxHighCut(m_maxHigh->currentData().toInt()); savePrefs(m_slice); }
    });
    connect(m_minSnr, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_slice) { m_slice->setAdaptiveMinSnr(i); savePrefs(m_slice); }
    });
    connect(m_response, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_slice) { m_slice->setAdaptiveResponse(i); savePrefs(m_slice); }
    });
    connect(m_splatter, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_slice) { m_slice->setAdaptiveSplatter(i); savePrefs(m_slice); }
    });

    updateVisibility();
}

void AdaptiveFilterControls::setSlice(SliceModel* slice)
{
    if (m_slice) disconnect(m_slice, nullptr, this, nullptr);
    m_slice = slice;
    if (!slice) { updateVisibility(); return; }

    // Reflect current slice state (blocked so we don't echo back as edits).
    { QSignalBlocker b(m_chk);      m_chk->setChecked(slice->adaptiveFilterEnabled()); }
    { QSignalBlocker b(m_minLow);   m_minLow->setCurrentIndex(std::max(0, m_minLow->findData(slice->adaptiveMinLowCut()))); }
    { QSignalBlocker b(m_maxHigh);  m_maxHigh->setCurrentIndex(std::max(0, m_maxHigh->findData(slice->adaptiveMaxHighCut()))); }
    { QSignalBlocker b(m_minSnr);   m_minSnr->setCurrentIndex(std::clamp(slice->adaptiveMinSnr(), 0, 2)); }
    { QSignalBlocker b(m_response); m_response->setCurrentIndex(std::clamp(slice->adaptiveResponse(), 0, 2)); }
    { QSignalBlocker b(m_splatter); m_splatter->setCurrentIndex(std::clamp(slice->adaptiveSplatter(), 0, 2)); }

    // Live sync: slice value changes (from any host, or the engine) -> controls.
    connect(slice, &SliceModel::adaptiveFilterEnabledChanged, this, [this](bool on) {
        QSignalBlocker b(m_chk); m_chk->setChecked(on); updateVisibility();
    });
    connect(slice, &SliceModel::adaptiveMinLowCutChanged, this, [this](int hz) {
        QSignalBlocker b(m_minLow); m_minLow->setCurrentIndex(std::max(0, m_minLow->findData(hz)));
    });
    connect(slice, &SliceModel::adaptiveMaxHighCutChanged, this, [this](int hz) {
        QSignalBlocker b(m_maxHigh); m_maxHigh->setCurrentIndex(std::max(0, m_maxHigh->findData(hz)));
    });
    connect(slice, &SliceModel::adaptiveMinSnrChanged, this, [this](int i) {
        QSignalBlocker b(m_minSnr); m_minSnr->setCurrentIndex(std::clamp(i, 0, 2));
    });
    connect(slice, &SliceModel::adaptiveResponseChanged, this, [this](int i) {
        QSignalBlocker b(m_response); m_response->setCurrentIndex(std::clamp(i, 0, 2));
    });
    connect(slice, &SliceModel::adaptiveSplatterChanged, this, [this](int i) {
        QSignalBlocker b(m_splatter); m_splatter->setCurrentIndex(std::clamp(i, 0, 2));
    });

    updateVisibility();
}

void AdaptiveFilterControls::updateVisibility()
{
    // Bounds/presets are only meaningful when the feature is on. Driven from the
    // slice (source of truth) so it stays correct even when enabled is toggled
    // programmatically (e.g. the engine disabling adaptive on a manual edit).
    const bool on = m_slice && m_slice->adaptiveFilterEnabled();
    for (QWidget* r : {m_loRow, m_hiRow, m_snrRow, m_responseRow, m_splatterRow})
        if (r) r->setVisible(on);

    // Invalidate our own cached sizeHint so the host reads the new (shorter)
    // height; the host relayouts its container/flag on sizeChanged().
    if (auto* l = layout()) l->invalidate();
    updateGeometry();
    emit sizeChanged();
}

void AdaptiveFilterControls::loadPrefs(SliceModel* slice)
{
    if (!slice) return;
    const QJsonObject root = QJsonDocument::fromJson(
        AppSettings::instance().value("AdaptiveFilter", QString{})
            .toString().toUtf8()).object();
    const QJsonObject o = root.value(QString::number(slice->sliceId())).toObject();
    slice->setAdaptiveMinLowCut(o.value("minLowCut").toInt(0));
    slice->setAdaptiveMaxHighCut(o.value("maxHighCut").toInt(4000));
    slice->setAdaptiveMinSnr(o.value("minSnr").toInt(1));      // default Normal
    slice->setAdaptiveResponse(o.value("response").toInt(1));  // default Normal
    slice->setAdaptiveSplatter(o.value("splatter").toInt(1));  // default Normal
    slice->setAdaptiveFilterEnabled(o.value("enabled").toBool(false));
}

void AdaptiveFilterControls::savePrefs(SliceModel* slice)
{
    if (!slice) return;
    auto& s = AppSettings::instance();
    QJsonObject root = QJsonDocument::fromJson(
        s.value("AdaptiveFilter", QString{}).toString().toUtf8()).object();
    QJsonObject o;
    o["enabled"]    = slice->adaptiveFilterEnabled();
    o["minLowCut"]  = slice->adaptiveMinLowCut();
    o["maxHighCut"] = slice->adaptiveMaxHighCut();
    o["minSnr"]     = slice->adaptiveMinSnr();
    o["response"]   = slice->adaptiveResponse();
    o["splatter"]   = slice->adaptiveSplatter();
    root[QString::number(slice->sliceId())] = o;
    s.setValue("AdaptiveFilter",
               QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Compact)));
    s.save();
}

} // namespace AetherSDR
