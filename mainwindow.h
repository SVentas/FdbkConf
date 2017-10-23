#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include <QTimer>

#include "telemetry.h"
#include "serialthread.h"

/* PID gain structure. */
typedef struct tagPID {
  float P;        /* Proportional gain. */
  float I;        /* Integral gain.     */
  float D;        /* Differential gain. */
} __attribute__((packed)) PID, *PPID;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void serialPortConnect();
    void serialPortError(const QString &s);
    void serialPortTimeout(const QString &s);
    void streamingGO();
    void streamingUpdateChannelID(bool checked);
    void fbkEnableUpdate(bool fEnable);
    void fbkErrInvUpdate(bool fEnable);
    void fbkKpUpdate(double k);
    void fbkKiUpdate(double k);
    void fbkKdUpdate(double k);
    void fbkSetpointUpdate(int setpoint);
    void fbkActPosUpdate(int pos);
    void boardReboot();
    void processTelemetryMessage(const TelemetryMessage &msg);
    void processStreamData(QVector<double> y);
    void processTimeout();

private:
    void boardReadSettings();
    void fillSerialPortInfo();
    void sendTelemetryMessage(const TelemetryMessage &msg);

private:
    Ui::MainWindow *ui;
    QComboBox *m_serialPortList;
    SerialThread m_serialThread;
    QTimer m_serialTimer;
    bool m_serialConnected;
    TelemetryMessage m_msg;
    PID m_pid;
    bool m_breakLoopKp;
    bool m_breakLoopKi;
    bool m_breakLoopKd;
    bool m_breakLoopSetpoint;
    bool m_breakLoopActPos;
};

#endif // MAINWINDOW_H
