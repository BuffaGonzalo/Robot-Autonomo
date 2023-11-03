#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include <QtNetwork/QUdpSocket>
#include <QLabel>
#include <QInputDialog>
#include <QTimer>
#include <QSerialPortInfo>



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void dataReceived();

    void decodeData(uint8_t *datosRx, uint8_t source);

    void sendDataSerial();

    void timeOut();

    void sendDataUDP();

    void OnUdpRxData();

    void on_pushButton_connectSerial_clicked();

    void on_pushButton_sendSerial_clicked();

    void on_pushButton_clean_clicked();

    void on_pushButton_connectUdp_clicked();

    void on_pushButton_sendUdp_clicked();

    /**
     * @brief sendSerial - Comando utilizado para enviar datos por el puerto serial en segundo plano
     * @param buf - Comando enviado por serial
     * @param length - longitud del comando en bytes
     */

    void sendSerial(uint8_t *buf, uint8_t length);

    /**
     * @brief sendUdp - Comando utilizado para enviar datos por wifi median udp en segundo plano
     * @param buf - Comando enviado por serial
     * @param length - longitud del comando en bytes
     */
    void sendUdp(uint8_t *buf, uint8_t length);

    void getData();

    bool eventFilter(QObject *watched, QEvent *event);



private:
    Ui::MainWindow *ui;
    QSerialPort *QSerialPort1;

    QLabel *estadoSerial;
    QTimer  *timer1;
    QTimer  *timer2;

    QUdpSocket *QUdpSocket1;
    QHostAddress RemoteAddress;
    quint16 RemotePort;
    QHostAddress clientAddress;
    int puertoremoto;

    typedef enum{
        START,
        HEADER_1,
        HEADER_2,
        HEADER_3,
        NBYTES,
        TOKEN,
        PAYLOAD
    }_eProtocolo;

    _eProtocolo estadoProtocolo,estadoProtocoloUdp;

    typedef enum{
        UDP=0,
        SERIE=1,
        ACK=0x0D,
        GETALIVE=0xF0,
        GETFIRMWARE=0xF1,
        UNKNOWCMD=0xFF,
        SETLEDS=0x10,
        GETSWITCHES=0x12,
        GETANALOGSENSORS=0xA0,
        SETMOTORTEST=0xA1,
        SETSERVOANGLE=0xA2,
        SERVOMOVESTOP=0x0A,
        GETDISTANCE=0xA3,
        GETSPEED=0xA4,
        SETSERVOLIMITS=0xA5,
        SETBLACKCOLORDETECTED=0xA6,
        SETWHITECOLORDETECTED=0xA7,
        OTHERS
    }_eCmd;


    typedef struct{
        uint8_t timeOut;
        uint8_t cheksum;
        uint8_t payLoad[256];
        uint8_t nBytes;
        uint8_t index;
    }_sDatos ;

    _sDatos rxData, rxDataUdp;

    typedef union {
        double  d32;
        float f32;
        int i32;
        unsigned int ui32;
        unsigned short ui16[2];
        short i16[2];
        uint8_t ui8[4];
        char chr[4];
        unsigned char uchr[4];
        int8_t  i8[4];
    }_udat;

    _udat myWord;

    int contadorAlive=0;

};
#endif // MAINWINDOW_H
