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

AdaptiveFilterControls::AdaptiveFilterControls(int sections, bool withHeader,
                                               bool compact, bool twoColumn,
                                               QWidget* parent)
    : QWidget(parent)
{
    // Compact preset matches the dense applet rows (smaller text, shorter combos);
    // the roomier preset suits the VFO flag.
    const int fpx         = compact ? 11 : 12;   // label/checkbox font px
    const int comboPx     = compact ? 11 : 12;
    const int comboFixedH = compact ? 20 : 0;    // 0 = natural height; 20 matches
                                                 // the applet's mode/SQL controls

    auto* av = new QVBoxLayout(this);
    // Small bottom margin so the group isn't flush against the flag's edge
    // (matches the S-meter/SmartMTR menu). Harmless in the applet, where a
    // stretch sits below this widget.
    av->setContentsMargins(0, compact ? 0 : 2, 0, 6);
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

    // Single column by default; in twoColumn mode the checkbox + filter bounds go
    // left and the behaviour presets go right (applet-style), at the same 40/60
    // proportions as the surrounding controls.
    QVBoxLayout* leftTarget  = av;
    QVBoxLayout* rightTarget = av;
    if (twoColumn) {
        auto* cols = new QHBoxLayout;
        cols->setContentsMargins(0, 0, 0, 0);
        cols->setSpacing(8);
        cols->setAlignment(Qt::AlignTop);
        leftTarget  = new QVBoxLayout; leftTarget->setContentsMargins(0, 0, 0, 0);
        leftTarget->setSpacing(av->spacing());
        rightTarget = new QVBoxLayout; rightTarget->setContentsMargins(0, 0, 0, 0);
        rightTarget->setSpacing(av->spacing());
        cols->addLayout(leftTarget, 2);
        cols->addLayout(rightTarget, 3);
        av->addLayout(cols);
    }

    // Checkbox — "Adaptive RX filter".
    if (sections & SecCheckbox) {
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
        leftTarget->addWidget(m_chk);
    }

    const auto makeOptLabel = [fpx](const QString& text) {
        auto* lbl = new QLabel(text);
        lbl->setStyleSheet(QStringLiteral(
            "QLabel { background: transparent; border: none; "
            "color: #c8d8e8; font-size: %1px; } "
            "QLabel:disabled { color: #5e6e7c; }").arg(fpx));
        return lbl;
    };

    // A labelled row: a short label + a stretched combo. The label carries the
    // full name as a tooltip; the combo is allowed to shrink (it does not impose
    // its longest item's width) so the row never forces the host column wider and
    // breaks the applet's 2-column split. accName is the full, descriptive name.
    const auto makeRow = [&](QVBoxLayout* target, const QString& label, const QString& accName)
        -> std::pair<QWidget*, QComboBox*> {
        auto* row = new QWidget;
        row->setAttribute(Qt::WA_TranslucentBackground);
        auto* lay = new QHBoxLayout(row);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(4);
        auto* lbl = makeOptLabel(label);
        lbl->setToolTip(accName);
        lay->addWidget(lbl);
        auto* cmb = new QComboBox;
        AetherSDR::applyComboStyle(cmb);
        QFont cf = cmb->font(); cf.setPixelSize(comboPx); cmb->setFont(cf);
        if (comboFixedH > 0) cmb->setFixedHeight(comboFixedH);
        if (compact) {
            // Don't let the widest item ("Sensitive") set a wide minimum width.
            cmb->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
            cmb->setMinimumContentsLength(4);
        }
        cmb->setAccessibleName(accName);
        cmb->setToolTip(accName);
        lay->addWidget(cmb, 1);
        target->addWidget(row);
        return {row, cmb};
    };

    // Bound combos (Hz value stored as item data) -> left column. Short labels
    // keep the rows narrow; the full name is the tooltip/accessible name.
    if (sections & SecBounds) {
        std::tie(m_loRow, m_minLow) = makeRow(leftTarget, tr("Lo cut"), tr("Adaptive minimum low-cut (Hz)"));
        for (int v : {0, 50, 100, 200}) m_minLow->addItem(QString::number(v), v);
        std::tie(m_hiRow, m_maxHigh) = makeRow(leftTarget, tr("Hi cut"), tr("Adaptive maximum high-cut (Hz)"));
        for (int v : {3000, 3500, 4000, 6000}) m_maxHigh->addItem(QString::number(v), v);
    }

    // Preset combos (combo INDEX is the level 0/1/2) -> right column.
    if (sections & SecPresets) {
        std::tie(m_snrRow, m_minSnr) = makeRow(rightTarget, tr("SNR"), tr("Adaptive minimum SNR"));
        for (const QString& o : {tr("Sensitive"), tr("Normal"), tr("Strong")}) m_minSnr->addItem(o);
        std::tie(m_responseRow, m_response) = makeRow(rightTarget, tr("Speed"), tr("Adaptive response speed"));
        for (const QString& o : {tr("Fast"), tr("Normal"), tr("Slow")}) m_response->addItem(o);
        std::tie(m_splatterRow, m_splatter) = makeRow(rightTarget, tr("Splat"), tr("Adaptive splatter rejection"));
        for (const QString& o : {tr("Tight"), tr("Normal"), tr("Wide")}) m_splatter->addItem(o);
    }

    // ── User-action handlers (act on the currently bound slice) ─────────────
    // Cross-instance updates set controls under QSignalBlocker, so these never
    // refire from a programmatic sync — only the touched widget persists. Only the
    // controls this instance built exist; the rest stay null.
    if (m_chk) connect(m_chk, &QCheckBox::toggled, this, [this](bool on) {
        if (m_slice) { m_slice->setAdaptiveFilterEnabled(on); savePrefs(m_slice); }
        updateVisibility();
    });
    if (m_minLow) connect(m_minLow, &QComboBox::currentIndexChanged, this, [this] {
        if (m_slice) { m_slice->setAdaptiveMinLowCut(m_minLow->currentData().toInt()); savePrefs(m_slice); }
    });
    if (m_maxHigh) connect(m_maxHigh, &QComboBox::currentIndexChanged, this, [this] {
        if (m_slice) { m_slice->setAdaptiveMaxHighCut(m_maxHigh->currentData().toInt()); savePrefs(m_slice); }
    });
    if (m_minSnr) connect(m_minSnr, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_slice) { m_slice->setAdaptiveMinSnr(i); savePrefs(m_slice); }
    });
    if (m_response) connect(m_response, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_slice) { m_slice->setAdaptiveResponse(i); savePrefs(m_slice); }
    });
    if (m_splatter) connect(m_splatter, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_slice) { m_slice->setAdaptiveSplatter(i); savePrefs(m_slice); }
    });

    updateVisibility();
}

void AdaptiveFilterControls::setSlice(SliceModel* slice)
{
    if (m_slice) disconnect(m_slice, nullptr, this, nullptr);
    m_slice = slice;
    if (!slice) { updateVisibility(); return; }

    // Reflect current slice state (blocked so we don't echo back as edits). Only
    // the controls this instance built are touched.
    if (m_chk)      { QSignalBlocker b(m_chk);      m_chk->setChecked(slice->adaptiveFilterEnabled()); }
    if (m_minLow)   { QSignalBlocker b(m_minLow);   m_minLow->setCurrentIndex(std::max(0, m_minLow->findData(slice->adaptiveMinLowCut()))); }
    if (m_maxHigh)  { QSignalBlocker b(m_maxHigh);  m_maxHigh->setCurrentIndex(std::max(0, m_maxHigh->findData(slice->adaptiveMaxHighCut()))); }
    if (m_minSnr)   { QSignalBlocker b(m_minSnr);   m_minSnr->setCurrentIndex(std::clamp(slice->adaptiveMinSnr(), 0, 2)); }
    if (m_response) { QSignalBlocker b(m_response); m_response->setCurrentIndex(std::clamp(slice->adaptiveResponse(), 0, 2)); }
    if (m_splatter) { QSignalBlocker b(m_splatter); m_splatter->setCurrentIndex(std::clamp(slice->adaptiveSplatter(), 0, 2)); }

    // Live sync: slice value changes (from any host, or the engine) -> controls.
    // Every instance tracks enabled (for visibility); the rest only if built.
    connect(slice, &SliceModel::adaptiveFilterEnabledChanged, this, [this](bool on) {
        if (m_chk) { QSignalBlocker b(m_chk); m_chk->setChecked(on); }
        updateVisibility();
    });
    if (m_minLow) connect(slice, &SliceModel::adaptiveMinLowCutChanged, this, [this](int hz) {
        QSignalBlocker b(m_minLow); m_minLow->setCurrentIndex(std::max(0, m_minLow->findData(hz)));
    });
    if (m_maxHigh) connect(slice, &SliceModel::adaptiveMaxHighCutChanged, this, [this](int hz) {
        QSignalBlocker b(m_maxHigh); m_maxHigh->setCurrentIndex(std::max(0, m_maxHigh->findData(hz)));
    });
    if (m_minSnr) connect(slice, &SliceModel::adaptiveMinSnrChanged, this, [this](int i) {
        QSignalBlocker b(m_minSnr); m_minSnr->setCurrentIndex(std::clamp(i, 0, 2));
    });
    if (m_response) connect(slice, &SliceModel::adaptiveResponseChanged, this, [this](int i) {
        QSignalBlocker b(m_response); m_response->setCurrentIndex(std::clamp(i, 0, 2));
    });
    if (m_splatter) connect(slice, &SliceModel::adaptiveSplatterChanged, this, [this](int i) {
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
