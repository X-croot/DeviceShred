//----------------------------------------------------------------------------------

#include <QtWidgets>
#include <QtCore>
#include <QtGui>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <linux/fs.h>
#include <sys/ioctl.h>
#include <atomic>

// ---------- helpers ----------
static bool isRoot() { return geteuid() == 0; }

static QByteArray readFile(const QString &path) {
    QFile f(path);
    if (f.open(QIODevice::ReadOnly)) return f.readAll();
    return {};
}

static QString firstLine(const QString &path) {
    const auto b = readFile(path);
    if (b.isEmpty()) return {};
    return QString::fromLatin1(b).split('\n').value(0).trimmed();
}

static quint64 parseULL(const QString &s, quint64 d = 0) {
    bool ok = false; auto v = s.toULongLong(&ok); return ok ? v : d;
}

static QString humanSize(quint64 bytes) {
    const char *suf[] = {"B","KB","MB","GB","TB","PB"};
    double v = bytes; int i = 0;
    while (v >= 1024.0 && i < 5) { v /= 1024.0; ++i; }
    return QString::number(v, 'f', i ? 2 : 0) + " " + suf[i];
}

static QString runCmd(const QString &cmd) {
    QProcess p; p.start("bash", {"-lc", cmd}); p.waitForFinished();
    return QString::fromUtf8(p.readAllStandardOutput()).trimmed();
}

static QString baseDeviceFor(const QString &devPath) {
    QString d = QFileInfo(devPath).canonicalFilePath();
    if (d.isEmpty()) d = devPath;
    QRegularExpression rxNvme("^(/dev/nvme\\d+n\\d+)(p\\d+)?$");
    QRegularExpression rxSd("^(/dev/sd[a-z]+)(\\d+)?$");
    QRegularExpression rxMmc("^(/dev/mmcblk\\d+)(p\\d+)?$");
    auto m = rxNvme.match(d); if (m.hasMatch()) return m.captured(1);
    m = rxSd.match(d); if (m.hasMatch()) return m.captured(1);
    m = rxMmc.match(d); if (m.hasMatch()) return m.captured(1);
    return d; // mapper/others -> keep as is
}

static QString rootDevice() {
    QFile f("/proc/mounts"); if (!f.open(QIODevice::ReadOnly)) return {};
    QTextStream ts(&f);
    while (!ts.atEnd()) {
        const auto line = ts.readLine();
        const auto parts = line.split(' ');
        if (parts.size() < 2) continue;
        if (parts[1] == "/") return baseDeviceFor(parts[0]);
    }
    return {};
}

struct DeviceInfo {
    QString name, dev; quint64 size = 0; QString vendor, model; bool removable = false;
};

static bool unmountAllOfBase(const QString &devBase, QString *err = nullptr) {
    bool ok = true;
    QFile f("/proc/mounts"); if (!f.open(QIODevice::ReadOnly)) return true;
    QTextStream ts(&f);
    while (!ts.atEnd()) {
        const auto line = ts.readLine();
        const auto parts = line.split(' ');
        if (parts.size() < 2) continue;
        const QString src = parts[0], mp = parts[1];
        if (baseDeviceFor(src) == devBase) {
            QProcess p; p.start("umount", {mp}); p.waitForFinished();
            if (p.exitCode() != 0) {
                ok = false;
                if (err) *err += QString("umount %1 -> %2\n").arg(mp, QString::fromUtf8(p.readAllStandardError()));
            }
        }
    }
    return ok;
}

static QList<DeviceInfo> listRemovableFiltered() {
    QList<DeviceInfo> out; const QString rootDevBase = baseDeviceFor(rootDevice());
    QDir d("/sys/block");
    for (const auto &e : d.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        if (e.startsWith("loop") || e.startsWith("ram")) continue;
        const QString devNode = "/dev/" + e; if (!QFile::exists(devNode)) continue;
        const bool isRem = firstLine("/sys/block/" + e + "/removable") == "1" || e.startsWith("mmcblk");
        if (!isRem) continue;
        DeviceInfo di; di.name = e; di.dev = devNode; di.removable = true;
        const quint64 sectors = parseULL(firstLine("/sys/block/" + e + "/size"));
        di.size = sectors * 512ull;
        di.vendor = firstLine("/sys/block/" + e + "/device/vendor");
        di.model  = firstLine("/sys/block/" + e + "/device/model");
        if (baseDeviceFor(devNode) == rootDevBase) continue; // never list system disk
        out << di;
    }
    std::sort(out.begin(), out.end(), [](const DeviceInfo &a, const DeviceInfo &b){ return a.name < b.name; });
    return out;
}

// ---------- wipe worker ----------
class WipeWorker : public QObject {
    Q_OBJECT
public:
    enum Pattern { Zeros, Random };
    QString dev; quint64 size = 0; Pattern pattern = Zeros; quint64 blockSize = 4*1024*1024;
    bool useDiscard = false;

signals:
    void progress(quint64 written, double mbps, quint64 etaSec);
    void log(const QString &line);
    void finished(bool ok, const QString &err);

public slots:
    void run() {
        int fd = ::open(dev.toLocal8Bit().constData(), O_RDWR | O_SYNC);
        if (fd < 0) { emit finished(false, QString("open %1 failed: %2").arg(dev, strerror(errno))); return; }

        auto tryDiscard = [&](const char *when){
            if (!useDiscard) return;
            unsigned long long range[2] = {0ULL, size};
            int rc = ioctl(fd, BLKDISCARD, &range);
            emit log(QString("BLKDISCARD %1: %2").arg(when).arg(rc == 0 ? "ok" : "not supported"));
        };

        tryDiscard("pre");

        QByteArray zeros; zeros.resize(int(blockSize)); zeros.fill('\0');
        QByteArray buf;   buf.resize(int(blockSize));
        QFile urnd("/dev/urandom");

        quint64 written = 0;
        QElapsedTimer timer; timer.start();
        qint64 lastEmitMs = 0;

        while (written < size) {
            if (cancelled.load()) { ::close(fd); emit finished(false, "Cancelled"); return; }
            while (paused.load()) {
                QThread::msleep(100);
                if (cancelled.load()) { ::close(fd); emit finished(false, "Cancelled"); return; }
            }

            const quint64 todo = qMin<quint64>(blockSize, size - written);
            const char *ptr = nullptr;

            if (pattern == Zeros) {
                ptr = zeros.constData();
            } else {
                if (!urnd.isOpen()) urnd.open(QIODevice::ReadOnly);
                if (urnd.read(buf.data(), qint64(todo)) != qint64(todo)) {
                    ::close(fd); emit finished(false, "/dev/urandom read failed"); return;
                }
                ptr = buf.constData();
            }

            const ssize_t w = ::write(fd, ptr, size_t(todo));
            if (w < 0) {
                const QString e = QString("write failed at %1: %2").arg(written).arg(strerror(errno));
                ::close(fd); emit finished(false, e); return;
            }
            written += quint64(w);

            const qint64 ms = timer.elapsed();
            if (ms - lastEmitMs >= 200) {
                const double mbps = (written / 1e6) / qMax(1.0, ms / 1000.0);
                const double remainSec = (size > 0 && mbps > 0) ? double(size - written) / (mbps * 1e6) : 0.0;
                emit progress(written, mbps, quint64(remainSec));
                lastEmitMs = ms;
            }
        }

        ::fsync(fd);
        tryDiscard("post");
        ::close(fd);
        emit progress(size, 0, 0);
        emit finished(true, {});
    }

    void pause()  { paused.store(true);  }
    void resume() { paused.store(false); }
    void cancel() { cancelled.store(true); }

private:
    std::atomic<bool> paused{false};
    std::atomic<bool> cancelled{false};
};

// ---------- animation helpers ----------
static void fadeIn(QWidget *w, int msec = 350, qreal from = 0.0, qreal to = 1.0) {
    auto *fx = new QGraphicsOpacityEffect(w);
    w->setGraphicsEffect(fx);
    auto *anim = new QPropertyAnimation(fx, "opacity", w);
    anim->setDuration(msec);
    anim->setStartValue(from);
    anim->setEndValue(to);
    anim->setEasingCurve(QEasingCurve::InOutCubic);
    QObject::connect(anim, &QPropertyAnimation::finished, [fx]{ fx->setEnabled(false); fx->deleteLater(); });
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}

static void slideIn(QWidget *w, const QPoint &delta = QPoint(0, 18), int msec = 420) {
    auto start = w->pos() + delta;
    auto end   = w->pos();
    w->move(start);
    auto *anim = new QPropertyAnimation(w, "pos", w);
    anim->setDuration(msec);
    anim->setStartValue(start);
    anim->setEndValue(end);
    anim->setEasingCurve(QEasingCurve::OutCubic);
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}

static void pulse(QWidget *w, int msec = 1200) {
    auto *fx = new QGraphicsOpacityEffect(w);
    w->setGraphicsEffect(fx);
    auto *anim = new QPropertyAnimation(fx, "opacity", w);
    anim->setDuration(msec);
    anim->setStartValue(0.65);
    anim->setKeyValueAt(0.5, 1.0);
    anim->setEndValue(0.65);
    anim->setEasingCurve(QEasingCurve::InOutSine);
    anim->setLoopCount(-1);
    anim->start(QAbstractAnimation::DeleteWhenStopped);
}

static void subtleGlow(QWidget *w) {
    auto *shadow = new QGraphicsDropShadowEffect(w);
    shadow->setBlurRadius(28);
    shadow->setColor(QColor(10,132,255,140));
    shadow->setOffset(0, 0);
    w->setGraphicsEffect(shadow);
}

static void applyDarkPalette() {
    QPalette p;
    const QColor win("#0e1116");
    const QColor base("#0b1017");
    const QColor alt ("#10141b");
    const QColor text("#e8eaed");
    const QColor btn ("#141a23");
    const QColor tip ("#0f141c");
    const QColor high("#0a84ff");
    const QColor link("#4cc9ff");

    p.setColor(QPalette::Window, win);
    p.setColor(QPalette::WindowText, text);
    p.setColor(QPalette::Base, base);
    p.setColor(QPalette::AlternateBase, alt);
    p.setColor(QPalette::ToolTipBase, tip);
    p.setColor(QPalette::ToolTipText, text);
    p.setColor(QPalette::Text, text);
    p.setColor(QPalette::Button, btn);
    p.setColor(QPalette::ButtonText, text);
    p.setColor(QPalette::BrightText, QColor("#ff3b30"));
    p.setColor(QPalette::Highlight, high);
    p.setColor(QPalette::HighlightedText, Qt::white);
    p.setColor(QPalette::Link, link);
    qApp->setPalette(p);
}

static void applyQss() {
    const QString qss = R"(
    * { font-family: "Inter","Segoe UI","Ubuntu",sans-serif; font-size: 14px; }
    QMainWindow, QDialog, QMessageBox, QInputDialog, QFileDialog, QWidget {
        background: #0e1116; color: #e8eaed; }
    QMenuBar, QMenu {
        background:#0e1116; color:#e8eaed; border:1px solid #263042; }
    QMenu::item { padding:7px 12px; }
    QMenu::item:selected { background:#202b3d; }
    QToolTip { background:#0f141c; color:#e8eaed; border:1px solid #263042; padding:6px; border-radius:8px; }

    /* Inputs */
    QLineEdit, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox, QDateTimeEdit {
        background:#0b1017; color:#e8eaed; border:1px solid #263042; border-radius:12px; padding:8px; selection-background-color:#0a84ff; }
    QLineEdit:disabled, QTextEdit:disabled, QPlainTextEdit:disabled { color:#8a93a2; }
    QComboBox {
        background:#121722; color:#e8eaed; border:1px solid #263042; border-radius:12px; padding:8px 34px 8px 10px; }
    QComboBox::drop-down { width:28px; border:0; }
    QComboBox QAbstractItemView {
        background:#0b1017; color:#e8eaed; border:1px solid #263042; selection-background-color:#202b3d; selection-color:#ffffff; }

    /* Buttons */
    QPushButton {
        background:#141a23; color:#e8eaed; border:1px solid #263042; border-radius:14px; padding:11px 18px; }
    QPushButton:hover { border-color:#3a4a63; }
    QPushButton:pressed { padding-top:12px; padding-bottom:10px; }
    QPushButton:disabled { color:#7c8699; border-color:#1c2534; background:#0f141c; }
    QPushButton#danger {
        background:qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #ff3b30, stop:1 #ff5e57);
        color:white; font-weight:800; border:0; }
        QPushButton#primary { background:#0a84ff; color:white; font-weight:700; border:0; }

        /* Check/Radio */
        QCheckBox, QRadioButton { color:#e8eaed; }
        QCheckBox::indicator, QRadioButton::indicator { width:18px; height:18px; }
        QCheckBox::indicator:unchecked { border:1px solid #3a4a63; background:#0b1017; border-radius:4px; }
        QCheckBox::indicator:checked { border:1px solid #0a84ff; background:#0a84ff; image:none; }
        QRadioButton::indicator:unchecked { border:2px solid #3a4a63; background:#0b1017; border-radius:9px; }
        QRadioButton::indicator:checked { border:2px solid #0a84ff; background:#0a84ff; }

        /* Table / headers */
        QAbstractScrollArea, QTableView, QTreeView, QListView { background:#0b1017; alternate-background-color:#10141b; }
        QTableWidget {
            background:#0b1017; color:#e8eaed; gridline-color:#263042; border:1px solid #263042; border-radius:14px; }
            QHeaderView::section {
                background:#141a23; color:#aeb7c2; padding:10px; border:0; border-right:1px solid #202938; }
                QTableCornerButton::section { background:#141a23; border:0; }

                /* Progress bar */
                QProgressBar {
                    background:#0f141c; border:1px solid #263042; border-radius:12px; text-align:center; height:30px; color:#dfe3e8; font-weight:600; }
                    QProgressBar::chunk {
                        background:qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #0a84ff, stop:1 #4cc9ff); border-radius:12px; }

                        /* Scrollbars */
                        QScrollBar:vertical, QScrollBar:horizontal {
                            background:#0f141c; border:1px solid #263042; border-radius:10px; margin:4px; }
                            QScrollBar::handle:vertical, QScrollBar::handle:horizontal {
                                background:#1f2a3b; min-height:28px; min-width:28px; border-radius:9px; }
                                QScrollBar::handle:hover { background:#2a3a54; }
                                QScrollBar::add-line, QScrollBar::sub-line { background:transparent; border:0; height:0; width:0; }
                                QScrollBar::add-page, QScrollBar::sub-page { background:transparent; }

                                /* Sliders */
                                QSlider::groove:horizontal { height:6px; background:#182130; border-radius:3px; }
                                QSlider::handle:horizontal { width:18px; height:18px; margin:-6px 0; background:#0a84ff; border-radius:9px; }
                                QSlider::groove:vertical { width:6px; background:#182130; border-radius:3px; }
                                QSlider::handle:vertical { width:18px; height:18px; margin:0 -6px; background:#0a84ff; border-radius:9px; }

                                /* Tabs */
                                QTabWidget::pane { border:1px solid #263042; border-radius:12px; }
                                QTabBar::tab { background:#141a23; color:#e8eaed; padding:8px 14px; margin:2px; border-radius:10px; }
                                QTabBar::tab:selected { background:#1a2230; }

                                /* GroupBox */
                                QGroupBox {
                                    color:#e8eaed; border:1px solid #263042; border-radius:14px; margin-top:14px; }
                                    QGroupBox::title { subcontrol-origin: margin; left:14px; padding:0 8px; }

                                    QLabel[hint="true"] { color:#aeb7c2; }
                                    )";
    qApp->setStyleSheet(qss);
}

// ---------- main window ----------
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow() {
        if (!isRoot()) {
            QMessageBox::critical(nullptr, "Root required",
                                  "Please run this application with sudo.\n\n  sudo ./DeviceShred");
            ::exit(1);
        }
        applyDarkPalette();
        applyQss();

        setWindowTitle("DeviceShred — Secure Removable Wiper");
        resize(1100, 720);
        setupUi();
        connectUi();
        refreshDevices();

        // entry animations
        QTimer::singleShot(50,  [=]{ fadeIn(banner); slideIn(banner); });
        QTimer::singleShot(120, [=]{ fadeIn(tbl); });
        QTimer::singleShot(180, [=]{ fadeIn(opt); slideIn(opt, QPoint(0,24)); });
        QTimer::singleShot(240, [=]{ fadeIn(bar); });
        QTimer::singleShot(260, [=]{ pulse(btnStart); subtleGlow(btnStart); });
    }

private:
    // widgets
    QLabel *banner = nullptr;
    QTableWidget *tbl = nullptr; QLabel *sel = nullptr; QProgressBar *bar = nullptr; QLabel *stats = nullptr;
    QTextEdit *logv = nullptr; QPushButton *btnRefresh = nullptr; QPushButton *btnStart = nullptr;
    QPushButton *btnPause = nullptr; QPushButton *btnResume = nullptr; QPushButton *btnCancel = nullptr;
    QComboBox *cmbPattern = nullptr; QComboBox *cmbBlock = nullptr; QCheckBox *chkDiscard = nullptr;
    QGroupBox *opt = nullptr;

    // state
    QList<DeviceInfo> devices; QThread thr; WipeWorker *worker = nullptr;

    QWidget* chip(const QString &text, const QColor &c) {
        auto *lbl = new QLabel(text);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setStyleSheet(QString("QLabel{background:%1; color:white; padding:6px 10px; border-radius:999px; font-weight:600;}").arg(c.name()));
        return lbl;
    }

    void setupUi() {
        auto *c = new QWidget; setCentralWidget(c);
        auto *v = new QVBoxLayout(c); v->setSpacing(12); v->setContentsMargins(16,16,16,16);

        auto *top = new QHBoxLayout; v->addLayout(top);
        banner = new QLabel(R"(<div style="font-size:22px; font-weight:800; letter-spacing:0.3px;">
            DeviceShred <span style="opacity:.65;font-weight:600;">— secure removable wiper</span>
        </div>)");
        top->addWidget(banner); top->addStretch();

        btnRefresh = new QPushButton("Refresh"); btnRefresh->setObjectName("primary"); top->addWidget(btnRefresh);
        btnStart   = new QPushButton("START WIPE"); btnStart->setObjectName("danger"); top->addWidget(btnStart);

        auto *chips = new QHBoxLayout; v->addLayout(chips);
        chips->addWidget(chip("sudo-only", QColor("#0a84ff")));
        chips->addWidget(chip("removable only", QColor("#7c4dff")));
        chips->addWidget(chip("system disk hidden", QColor("#ff9f0a")));
        chips->addStretch();

        tbl = new QTableWidget(0, 5);
        tbl->setHorizontalHeaderLabels({"Device","Size","Vendor","Model","Removable"});
        tbl->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        tbl->setSelectionBehavior(QAbstractItemView::SelectRows);
        tbl->setSelectionMode(QAbstractItemView::SingleSelection);
        tbl->setShowGrid(true);
        v->addWidget(new QLabel("Removable devices (system disk hidden):"));
        v->addWidget(tbl, 1);

        opt = new QGroupBox("Options");
        auto *g = new QGridLayout(opt);
        cmbPattern = new QComboBox; cmbPattern->addItems({"Zeros (fill)","Random bytes (/dev/urandom)"});
        cmbBlock = new QComboBox; cmbBlock->addItems({"256 KiB","512 KiB","1 MiB","4 MiB","8 MiB","16 MiB"}); cmbBlock->setCurrentIndex(3);
        chkDiscard = new QCheckBox("Attempt TRIM/BLKDISCARD before/after");
        g->addWidget(new QLabel("Pattern"),0,0); g->addWidget(cmbPattern,0,1);
        g->addWidget(new QLabel("Block size"),0,2); g->addWidget(cmbBlock,0,3);
        g->addWidget(chkDiscard,1,1,1,3);
        v->addWidget(opt);

        sel = new QLabel("Select a device."); v->addWidget(sel);

        bar = new QProgressBar; bar->setRange(0, 1000); v->addWidget(bar);
        stats = new QLabel; v->addWidget(stats);

        auto *btns = new QHBoxLayout; v->addLayout(btns);
        btnPause = new QPushButton("Pause");
        btnResume = new QPushButton("Resume");
        btnCancel = new QPushButton("Cancel");
        btnPause->setEnabled(false); btnResume->setEnabled(false); btnCancel->setEnabled(false);
        btns->addStretch(); btns->addWidget(btnPause); btns->addWidget(btnResume); btns->addWidget(btnCancel);

        v->addWidget(new QLabel("Log:"));
        logv = new QTextEdit; logv->setReadOnly(true); v->addWidget(logv, 1);

        // tooltips
        cmbPattern->setToolTip("Choose write pattern: zeros (fast) or cryptographically random bytes (slow).");
        cmbBlock->setToolTip("Larger blocks can be faster on USB 3.x media; try 4–16 MiB.");
        chkDiscard->setToolTip("Sends BLKDISCARD/TRIM (if supported) before and after the wipe.");
        btnStart->setToolTip("Destructive! Confirms before starting.");
    }

    void connectUi() {
        connect(btnRefresh, &QPushButton::clicked, this, &MainWindow::refreshDevices);
        connect(tbl, &QTableWidget::itemSelectionChanged, this, &MainWindow::updateSelection);
        connect(btnStart, &QPushButton::clicked, this, &MainWindow::startWipe);
        connect(btnPause, &QPushButton::clicked, this, [=]{ if (worker) worker->pause(); });
        connect(btnResume, &QPushButton::clicked, this, [=]{ if (worker) worker->resume(); });
        connect(btnCancel, &QPushButton::clicked, this, [=]{ if (worker) worker->cancel(); });
    }

    void bumpStats() {
        auto *fx = new QGraphicsOpacityEffect(stats);
        stats->setGraphicsEffect(fx);
        auto *anim = new QPropertyAnimation(fx, "opacity", stats);
        anim->setDuration(240);
        anim->setStartValue(0.55);
        anim->setEndValue(1.0);
        anim->setEasingCurve(QEasingCurve::OutCubic);
        connect(anim, &QPropertyAnimation::finished, [fx]{ fx->deleteLater(); });
        anim->start(QAbstractAnimation::DeleteWhenStopped);
    }

    void appendLog(const QString &m) { logv->append(QTime::currentTime().toString("HH:mm:ss ") + m); }

    int selectedRow() const {
        auto sm = tbl->selectionModel();
        if (!sm || !sm->hasSelection()) return -1;
        return sm->selectedRows().first().row();
    }

    void refreshDevices() {
        devices = listRemovableFiltered();
        tbl->setRowCount(0);
        for (int i = 0; i < devices.size(); ++i) {
            const auto &d = devices[i];
            tbl->insertRow(i);
            auto add = [&](int col, const QString &t){
                auto *it = new QTableWidgetItem(t);
                it->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
                tbl->setItem(i, col, it);
            };
            add(0, d.dev);
            add(1, humanSize(d.size));
            add(2, d.vendor);
            add(3, d.model);
            add(4, d.removable ? "yes" : "?");
        }
        updateSelection();
        fadeIn(tbl, 250);
    }

    void updateSelection() {
        const int r = selectedRow();
        if (r < 0) { sel->setText("Select a device."); return; }
        const auto &d = devices[r];
        sel->setText(QString("<b>Selected:</b> %1  |  <b>Size:</b> %2  |  <b>Vendor:</b> %3  |  <b>Model:</b> %4")
        .arg(d.dev, humanSize(d.size), d.vendor, d.model));
        bumpStats();
    }

    static QString formatETA(quint64 s) {
        quint64 h = s/3600, m = (s%3600)/60, se = s%60;
        if (h) return QString("%1h %2m %3s").arg(h).arg(m).arg(se);
        if (m) return QString("%1m %2s").arg(m).arg(se);
        return QString("%1s").arg(se);
    }

    void startWipe() {
        const int r = selectedRow();
        if (r < 0) { QMessageBox::warning(this, "No device", "Please select a device."); return; }
        const auto d = devices[r];

        if (QMessageBox::warning(this, "Confirm destructive action",
            QString("This will COMPLETELY ERASE:\n%1\n\nAll data will be lost. Proceed?")
            .arg(d.dev),
                                 QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
            return;
                                 }

                                 QString emsg;
                                 if (!unmountAllOfBase(baseDeviceFor(d.dev), &emsg) && !emsg.isEmpty()) {
                                     appendLog("Unmount issues (continuing):\n" + emsg.trimmed());
                                 }

                                 worker = new WipeWorker;
                                 worker->dev = d.dev;
                                 worker->size = d.size;
                                 worker->useDiscard = chkDiscard->isChecked();
                                 worker->pattern = (cmbPattern->currentIndex() == 0) ? WipeWorker::Zeros : WipeWorker::Random;
                                 switch (cmbBlock->currentIndex()) {
                                     case 0: worker->blockSize = 256ull*1024; break;
                                     case 1: worker->blockSize = 512ull*1024; break;
                                     case 2: worker->blockSize = 1024ull*1024; break;
                                     case 3: worker->blockSize = 4ull*1024*1024; break;
                                     case 4: worker->blockSize = 8ull*1024*1024; break;
                                     default: worker->blockSize = 16ull*1024*1024; break;
                                 }

                                 worker->moveToThread(&thr);
                                 connect(&thr, &QThread::started, worker, &WipeWorker::run);
                                 connect(worker, &WipeWorker::progress, this, [=](quint64 w, double mbps, quint64 eta){
                                     int permil = int((w * 1000.0) / double(d.size));
                                     if (permil < 0) permil = 0; if (permil > 1000) permil = 1000;
                                     bar->setValue(permil);
                                     stats->setText(QString("%1 / %2 — %3 MB/s — ETA %4")
                                     .arg(humanSize(w), humanSize(d.size),
                                          QString::number(mbps,'f',1), formatETA(eta)));
                                     bumpStats();
                                 });
                                 connect(worker, &WipeWorker::log, this, [=](const QString &s){ appendLog(s); });
                                 connect(worker, &WipeWorker::finished, this, [=](bool ok, const QString &e){
                                     thr.quit(); thr.wait();
                                     worker->deleteLater(); worker = nullptr;
                                     btnPause->setEnabled(false); btnResume->setEnabled(false); btnCancel->setEnabled(false);
                                     btnStart->setEnabled(true); btnRefresh->setEnabled(true);
                                     if (ok) { appendLog("Completed successfully."); QMessageBox::information(this, "Done", "Wipe completed successfully."); }
                                     else   { appendLog("FAILED: " + e); QMessageBox::critical(this, "Failed", e); }
                                 });

                                 btnPause->setEnabled(true); btnResume->setEnabled(true); btnCancel->setEnabled(true);
                                 btnStart->setEnabled(false); btnRefresh->setEnabled(false);
                                 appendLog(QString("Starting wipe: %1 (%2)").arg(d.dev, humanSize(d.size)));
                                 thr.start();

                                 fadeIn(bar, 250, 0.4, 1.0);
    }
};

// ---------- app ----------
int main(int argc, char **argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("DeviceShred");
    QApplication::setOrganizationName("DeviceShred");

    if (!isRoot()) {
        QMessageBox::critical(nullptr, "Root required",
                              "Please run this application with sudo.\n\n  sudo ./DeviceShred");
        return 1;
    }

    applyDarkPalette();
    applyQss();

    MainWindow w; w.show();
    return app.exec();
}

#include "main.moc"
