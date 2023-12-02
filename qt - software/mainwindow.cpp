#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    timer1 = new QTimer(this);
    timer2 = new QTimer(this); //Utilizado para la transmision de datos
    timer3 = new QTimer(this); //Utilizado para el radar
    timer4 = new QTimer(this); //Utilizado para pedir la distancia
    timer5 = new QTimer(this); //Utilizado para mover el servo
    timer6 = new QTimer(this);
    timer7 = new QTimer(this); //Utilizado para pintar el radar al inicio

    //dibujo
    QPaintBox1 = new QPaintBox(0,0,ui->widget); //el padre es el widget
    QPaintBox2 = new QPaintBox(0,0,ui->carWidget);

    //comunicacion
    QSerialPort1=new QSerialPort(this);
    QUdpSocket1 = new QUdpSocket(this);

    //debug de comandos
    myDebug = new Debug(this);


    estadoSerial = new QLabel(this);
    estadoSerial->setText("DISCONNECTED");
    ui->statusBar->addWidget(estadoSerial);

    ui->comboBox_PORT->installEventFilter(this);

    //connects del puerto serial
    connect(ui->pushButton_sendSerial,&QPushButton::clicked, this, &MainWindow::sendDataSerial);
    connect(QSerialPort1,&QSerialPort::readyRead,this,&MainWindow::dataReceived);

    //connects de los timers con las funciones
    connect(timer1,&QTimer::timeout,this,&MainWindow::timeOut);
    connect(timer2,&QTimer::timeout,this,&MainWindow::getData);
    connect(timer3,&QTimer::timeout,this,&MainWindow::radar);
    connect(timer4,&QTimer::timeout,this,&MainWindow::onTimer4);
    connect(timer5,&QTimer::timeout,this,&MainWindow::onTimer5);
    connect(timer6, &QTimer::timeout,this,&MainWindow::carStatus);
    connect(timer7, &QTimer::timeout,this,&MainWindow::onTimer7);

    //connects de udp
    connect(QUdpSocket1,&QUdpSocket::readyRead,this,&MainWindow::OnUdpRxData);
    connect(ui->pushButton_sendUdp,&QPushButton::clicked,this,&MainWindow::sendDataUDP);

    //connect(ui->actionScanPorts, &QAction::triggered, settingPorts,&SettingsDialog::show);
    connect(ui->actionPROTOCOL_DATA, &QAction::triggered, myDebug, &Debug::show);

    //añadimos los comandos
    ui->comboBox_CMD->addItem("ALIVE", 0xF0);
    ui->comboBox_CMD->addItem("FIRMWARE", 0xF1);
    ui->comboBox_CMD->addItem("SENSORES", 0xA0);
    ui->comboBox_CMD->addItem("MOTORES", 0xA1);
    ui->comboBox_CMD->addItem("SERVO", 0xA2);
    ui->comboBox_CMD->addItem("DISTANCIA", 0xA3);
    ui->comboBox_CMD->addItem("VELOCIDAD", 0xA4);
    ui->comboBox_CMD->addItem("SWITCHS", 0x12);
    ui->comboBox_CMD->addItem("LEDS", 0x10);
    ui->comboBox_CMD->addItem("CONFIGSERVO", 0xA5);
    ui->comboBox_CMD->addItem("CONFIGBLACK", 0xA6);
    ui->comboBox_CMD->addItem("CONFIGWHITE", 0xA7);

    //inicializamos
    estadoProtocolo=START;
    rxData.timeOut=0;

    //desabilitamos los botones con el fin de que no puedan ser presionados si no esta conectado
    ui->pushButton_sendUdp->setEnabled(false);
    ui->pushButton_sendSerial->setEnabled(false);

    timer1->start(100);
    timer2->start(500); //timer encargado del envio de datos cada 500ms
    timer3->start(100); //iniciamos para pintar al inicio
    timer6->start(100); //pintamos el estado del auto cada 100ms

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::dataReceived(){
    unsigned char *incomingBuffer;
    int count;

    count = QSerialPort1->bytesAvailable();

    if(count<=0)
        return;

    incomingBuffer = new unsigned char[count];

    QSerialPort1->read((char *)incomingBuffer,count);

    QString str="";

    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg((char)incomingBuffer[i]);
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }

    myDebug->showMessage("MBED-->SERIAL-->PC (" + str + ")");

    //Cada vez que se recibe un dato reinicio el timeOut
    rxData.timeOut=6;

    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
        case START:
            if (incomingBuffer[i]=='U'){
                estadoProtocolo=HEADER_1;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                estadoProtocolo=HEADER_2;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                estadoProtocolo=HEADER_3;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocolo=NBYTES;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case NBYTES:
            rxData.nBytes=incomingBuffer[i];
            estadoProtocolo=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                estadoProtocolo=PAYLOAD;
                rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                rxData.payLoad[0]=rxData.nBytes;
                rxData.index=1;
            }
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case PAYLOAD:
            if (rxData.nBytes>1){
                rxData.payLoad[rxData.index++]=incomingBuffer[i];
                rxData.cheksum^=incomingBuffer[i];
            }
            rxData.nBytes--;
            if(rxData.nBytes==0){
                estadoProtocolo=START;
                if(rxData.cheksum==incomingBuffer[i]){
                    decodeData(&rxData.payLoad[0], SERIE);
                }else{
                    myDebug->showMessage("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    myDebug->showMessage("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **" );

                }
            }
            break;
        default:
            estadoProtocolo=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void MainWindow::decodeData(uint8_t *datosRx, uint8_t source){
    int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
    QString str, strOut;
    _udat w;
    for(int i = 1; i<length; i++){
        if(isalnum(datosRx[i]))
            str = str + QString("%1").arg(char(datosRx[i]));
        else
            str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
    }
    myDebug->showMessage("*(MBED-S->PC)->decodeData (" + str + ")");

    switch (datosRx[1]) {
    case GETANALOGSENSORS://     ANALOGSENSORS=0xA0,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "LEFT IR: " + str;
        myDebug->showMessage(strOut);
        ui->label_left_ir_data->setText(str);
        w.ui8[0] = datosRx[4];
        w.ui8[1] = datosRx[5];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "CENTER IR: " + str;
        myDebug->showMessage(strOut);
        ui->label_center_ir_data->setText(str);
        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        str =QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "RIGHT IR: " + str;
        myDebug->showMessage(strOut);
        ui->label_right_ir_data->setText(str);
        break;
    case SETMOTORTEST://     MOTORTEST=0xA1,
        if(datosRx[2]==0x0D)
            str= "Test Motores ACK";
        myDebug->showMessage(str);
        break;
    case SETSERVOANGLE://     SERVOANGLE=0xA2,
        if(datosRx[2]==0x0D)
            str= "Servo moviendose. Esperando posición Final!!!";
                else{
                if(datosRx[2]==0x0A)
                    str= "Servo en posición Final!!!";
            }
        myDebug->showMessage(str);
        break;
    case GETDISTANCE://     GETDISTANCE=0xA3,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        str = QString().number(w.ui32/58);

        //distancia a utilizar en el radar
        distance = w.ui32/58;
        if(distance>50 || distance<4){
                distance = 0;
        }

        //mostramos datos
        ui->label_distance_data->setText(str+ "cm");
        myDebug->showMessage("DISTANCIA: "+QString().number(w.ui32/58)+ "cm");
        break;
    case GETSPEED://     GETSPEED=0xA4,
        str = "VM1: ";
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        strOut = QString("%1").arg(w.i32, 4, 10, QChar('0'));
        ui->label_left_encoder_data->setText(strOut);
        str = str + QString("%1").arg(w.i32, 4, 10, QChar('0')) + " - VM2: ";
        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        w.ui8[2] = datosRx[8];
        w.ui8[3] = datosRx[9];
        strOut = QString("%1").arg(w.i32, 4, 10, QChar('0'));
        ui->label_right_encoder_data->setText(strOut);
        str = str + QString("%1").arg(w.i32, 4, 10, QChar('0'));
        myDebug->showMessage(str);
        break;
    case GETSWITCHES: //GETSWITCHES=0xA5
        str = "SW3: ";
        if(datosRx[2] & 0x08)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - SW2: ";
        if(datosRx[2] & 0x04)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - SW1: ";
        if(datosRx[2] & 0x02)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - SW0: ";
        if(datosRx[2] & 0x01)
            str = str + "HIGH";
        else
            str = str + "LOW";
        myDebug->showMessage(str);
        break;

    case GETALIVE://     GETALIVE=0xF0,
        if(datosRx[2]==ACK){
            contadorAlive++;
            if(source)
                str="ALIVE BLUEPILL VIA *SERIE* RECIBIDO!!!";
            else{
                contadorAlive++;
                str="ALIVE BLUEPILL VIA *UDP* RECIBIDO N°: " + QString().number(contadorAlive,10);
            }
        }else{
            str= "ALIVE BLUEPILL VIA *SERIE*  NO ACK!!!";
        }
        myDebug->showMessage(str);
        break;
    case GETFIRMWARE://     GETFIRMWARE=0xF1
        str = "FIRMWARE:";
        for(uint8_t a=0;a<(datosRx[0]-1);a++){
            str += (QChar)datosRx[2+a];
        }
        myDebug->showMessage(str);

        break;
    case SETLEDS:
        str = "LD3: ";
        if(datosRx[2] & 0x08)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - LD2: ";
        if(datosRx[2] & 0x04)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - LD1: ";
        if(datosRx[2] & 0x02)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - LD0: ";
        if(datosRx[2] & 0x01)
            str = str + "HIGH";
        else
            str = str + "LOW";
        myDebug->showMessage(str);
        break;
    case SETBLACKCOLOR:
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];

        strOut = QString("%1").arg(w.i32 & 0xFFFF, 4, 10, QChar('0')); //con el 0xFFFF utilizamos solamente los 16 bits mas significativos

        break;
    case SETWHITECOLOR:
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];

        strOut = QString("%1").arg(w.i32 & 0xFFFF, 4, 10, QChar('0')); //con el 0xFFFF utilizamos solamente los 16 bits mas significativos
        strOut = QString("%1").arg(w.i16[0], 4, 10, QChar('0'));
        break;
    case PATHLENGHT:
        //distancia primera linea
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        strOut = QString("%1").arg(w.i16[0], 4, 10, QChar('0'));
        ui->label_pathFstData->setText(strOut);
        //distancia segunda linea
        w.ui8[0] = datosRx[4];
        w.ui8[1] = datosRx[5];
        strOut = QString("%1").arg(w.i16[0], 4, 10, QChar('0'));
        ui->label_pathSndData->setText(strOut);
        //distanccia tercera linea
        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        strOut = QString("%1").arg(w.i16[0], 4, 10, QChar('0'));
        ui->label_pathTrdData->setText(strOut);
        //distancia cuarta linea
        w.ui8[0] = datosRx[8];
        w.ui8[1] = datosRx[9];
        strOut = QString("%1").arg(w.i16[0], 4, 10, QChar('0'));
        ui->label_pathFthData->setText(strOut);
        break;
    case SETSERVOLIMITS:
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];

        strOut = QString("%1").arg(w.i16[0], 4, 10, QChar('0')); //con el 0xFFFF utilizamos solamente los 16 bits mas significativos

        strOut = QString("%1").arg(w.i16[1], 4, 10, QChar('0')); //con el 0xFFFF utilizamos solamente los 16 bits mas significativos

        break;
    default:
        str = str + "Comando DESCONOCIDO!!!!";
        myDebug->showMessage(str);
    }
}

void MainWindow::sendDataSerial(){
    uint8_t cmdId;
    _udat   w;
    bool ok;

    unsigned char dato[256];
    unsigned char indice=0, chk=0;

    QString str="";

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case SETMOTORTEST://MOTORTEST=0xA1,
        dato[indice++] =SETMOTORTEST;
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor1:", 0, -100, 100, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor2:", 0, -100, 100, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        dato[NBYTES]= 0x0A;
        break;
    case SETSERVOANGLE://SERVOANGLE=0xA2,
        dato[indice++] =SETSERVOANGLE;
        w.i32 = QInputDialog::getInt(this, "SERVO", "Angulo:", 0, -90, 90, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.i8[0];
        dato[NBYTES]= 0x03;
        break;
    case GETALIVE:
    case GETDISTANCE://GETDISTANCE=0xA3,
    case GETSPEED://GETSPEED=0xA4,
    case GETSWITCHES://GETSWITCHES=0xA5
    case GETFIRMWARE:// GETFIRMWARE=0xF1
    case GETANALOGSENSORS://ANALOGSENSORS=0xA0,
    case SETLEDS:
    case SETBLACKCOLOR:
    case SETWHITECOLOR:
    case PATHLENGHT:
        dato[indice++]=cmdId;
        dato[NBYTES]=0x02;
    break;
    case SETSERVOLIMITS:
        dato[indice++]=cmdId;

        w.i32 = QInputDialog::getInt(this, "Configuracion Servo", "MinMs:", 0, 700, 1500, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        w.ui32 = QInputDialog::getInt(this, "Configuracion Servo", "MaxMs:", 0, 1500, 2500, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        dato[NBYTES]=0x06;
    break;
    default:
        return;
    }

    for(int a=0 ;a<indice;a++)
        chk^=dato[a]; //calculamos el checksum
    dato[indice]=chk; //colocamos el checksum en la ultima posicion

    if(QSerialPort1->isWritable()){
        QSerialPort1->write(reinterpret_cast<char *>(dato),dato[NBYTES]+PAYLOAD);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }

    uint16_t valor=dato[NBYTES]+PAYLOAD;
    myDebug->showMessage("***COMANDO NUEVO***");
    myDebug->showMessage("INDICE ** " +QString().number(indice,10) + " **" );
    myDebug->showMessage("NUMERO DE DATOS ** " +QString().number(valor,10) + " **" );
    myDebug->showMessage("CHECKSUM ** " +QString().number(chk,16) + " **" );
    myDebug->showMessage("PC--SERIAL-->MBED ( " + str + " )");

}

void MainWindow::sendSerial(uint8_t *buf, uint8_t length){
    uint8_t tx[24];
    uint8_t cks, i;
    QString strHex;
    _udat w;

    if(!QSerialPort1->isOpen())
        return;

    w.i32 = -1000;

    tx[7] = w.ui8[0];
    tx[8] = w.ui8[1];
    tx[9] = w.ui8[2];
    tx[10] = w.ui8[3];


    tx[0] = 'U';
    tx[1] = 'N';
    tx[2] = 'E';
    tx[3] = 'R';
    tx[4] = length + 1;
    tx[5] = ':';

    memcpy(&tx[6], buf, length);

    cks = 0;
    for (i=0; i<(length+6); i++) {
        cks ^= tx[i];
    }

    tx[i] = cks;

    strHex = "--> 0x";
    for (int i=0; i<length+7; i++) {
        strHex = strHex + QString("%1").arg(tx[i], 2, 16, QChar('0')).toUpper();
    }

    myDebug->showMessage(strHex);

    QSerialPort1->write((char *)tx, length+7);
}

void MainWindow::sendUdp(uint8_t *buf, uint8_t length){
    uint8_t tx[256];
    _udat w;
    unsigned char indice=0, cks=0;


    QString str;
    int puerto=0;
    bool ok;

    if(!QUdpSocket1->isOpen())
        return;


    w.i32 = -1000;

    tx[7] = w.ui8[0];
    tx[8] = w.ui8[1];
    tx[9] = w.ui8[2];
    tx[10] = w.ui8[3];


    tx[0] = 'U';
    tx[1] = 'N';
    tx[2] = 'E';
    tx[3] = 'R';
    tx[4] = length + 1;
    tx[5] = ':';

    memcpy(&tx[6], buf, length);

    cks = 0;
    for (indice=0; indice<(length+6); indice++) {
        cks ^= tx[indice];
    }

    tx[indice] = cks;

    str = "--> 0x";
    for (int i=0; i<length+7; i++) {
        str = str + QString("%1").arg(tx[i], 2, 16, QChar('0')).toUpper();
    }

    puerto=ui->lineEdit_device_port->text().toInt();
    puertoremoto=puerto;

    if(clientAddress.isNull())
        clientAddress.setAddress(ui->lineEdit_device_ip->text());
    if(puertoremoto==0)
        puertoremoto=puerto;
    if(QUdpSocket1->isOpen()){
        QUdpSocket1->writeDatagram(reinterpret_cast<const char *>(tx), (tx[4]+7), clientAddress, puertoremoto);

    }

    for(int i=0; i<=indice; i++){
        if(isalnum(tx[i]))
            str = str + QString("%1").arg(char(tx[i]));
        else
            str = str +"{" + QString("%1").arg(tx[i],2,16,QChar('0')) + "}";
    }
    str=str + clientAddress.toString() + "  " +  QString().number(puertoremoto,10);

    myDebug->showMessage("PC--UDP-->MBED ( " + str + " )");
}

void MainWindow::timeOut(){
    if(rxData.timeOut){
        rxData.timeOut--;
        if(!rxData.timeOut){
            estadoProtocolo=START;
        }
    }
}

void MainWindow::OnUdpRxData(){
    qint64          count=0;
    unsigned char   *incomingBuffer;

    while(QUdpSocket1->hasPendingDatagrams()){
        count = QUdpSocket1->pendingDatagramSize();
        incomingBuffer = new unsigned char[count];
        QUdpSocket1->readDatagram( reinterpret_cast<char *>(incomingBuffer), count, &RemoteAddress, &RemotePort);
    }
    if (count<=0)
        return;

    QString str="";
    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg(char(incomingBuffer[i]));
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    myDebug->showMessage("MBED-->UDP-->PC (" + str + ")");
    QString adress=RemoteAddress.toString();
    myDebug->showMessage(" adr " + adress);

    ui->lineEdit_device_ip->setText(RemoteAddress.toString().right((RemoteAddress.toString().length())-7));
    ui->lineEdit_device_port->setText(QString().number(RemotePort,10));

    for(int i=0;i<count; i++){
        switch (estadoProtocoloUdp) {
        case START:
            if (incomingBuffer[i]=='U'){
                estadoProtocoloUdp=HEADER_1;
                rxDataUdp.cheksum=0;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                estadoProtocoloUdp=HEADER_2;
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                estadoProtocoloUdp=HEADER_3;
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocoloUdp=NBYTES;
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case NBYTES:
            rxDataUdp.nBytes=incomingBuffer[i];
            estadoProtocoloUdp=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                estadoProtocoloUdp=PAYLOAD;
                rxDataUdp.cheksum='U'^'N'^'E'^'R'^ rxDataUdp.nBytes^':';
                rxDataUdp.payLoad[0]=rxDataUdp.nBytes;
                rxDataUdp.index=1;
            }
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case PAYLOAD:
            if (rxDataUdp.nBytes>1){
                rxDataUdp.payLoad[rxDataUdp.index++]=incomingBuffer[i];
                rxDataUdp.cheksum^=incomingBuffer[i];
            }
            rxDataUdp.nBytes--;
            if(rxDataUdp.nBytes==0){
                estadoProtocoloUdp=START;
                if(rxDataUdp.cheksum==incomingBuffer[i]){
                    decodeData(&rxDataUdp.payLoad[0],UDP);
                }else{
                    myDebug->showMessage(" CHK DISTINTO!!!!! ");
                }
            }
            break;

        default:
            estadoProtocoloUdp=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void MainWindow::sendDataUDP(){
    uint8_t cmdId;
    _udat w;
    unsigned char dato[256];
    unsigned char indice=0, chk=0;
    QString str;
    int puerto=0;
    bool ok;

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case SETMOTORTEST://MOTORTEST=0xA1,
        dato[indice++] =SETMOTORTEST;
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor1:", 0, -100, 100, 1, &ok);
        if(!ok)
            return;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor2:", 0, -100, 100, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        dato[NBYTES]= 0x0A;
        break;
    case SETSERVOANGLE://SERVOANGLE=0xA2,
        dato[indice++] =SETSERVOANGLE;
        w.i32 = QInputDialog::getInt(this, "SERVO", "Angulo:", 0, -90, 90, 1, &ok);
        if(!ok)
            return;
        dato[indice++] = w.i8[0];
        dato[NBYTES]= 0x03;
        break;
    case GETALIVE:
    case GETDISTANCE://GETDISTANCE=0xA3,
    case GETSPEED://GETSPEED=0xA4,
    case GETSWITCHES://GETSWITCHES=0xA5
    case GETFIRMWARE:// GETFIRMWARE=0xF1
    case GETANALOGSENSORS://ANALOGSENSORS=0xA0,
    case SETLEDS:
    case SETBLACKCOLOR:
    case SETWHITECOLOR:
    case PATHLENGHT:
        dato[indice++]=cmdId;
        dato[NBYTES]=0x02;
        break;
    case SETSERVOLIMITS:
        dato[indice++]=cmdId;

        w.i32 = QInputDialog::getInt(this, "Configuracion Servo", "MinMs:", 0, 700, 1500, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        w.ui32 = QInputDialog::getInt(this, "Configuracion Servo", "MaxMs:", 0, 1500, 2500, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        dato[NBYTES]=0x06;
        break;
    default:
        return;
    break;
    }

    puerto=ui->lineEdit_device_port->text().toInt();
    puertoremoto=puerto;
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;
    if(clientAddress.isNull())
        clientAddress.setAddress(ui->lineEdit_device_ip->text());
    if(puertoremoto==0)
        puertoremoto=puerto;
    if(QUdpSocket1->isOpen()){
        QUdpSocket1->writeDatagram(reinterpret_cast<const char *>(dato), (dato[4]+7), clientAddress, puertoremoto);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }
    str=str + clientAddress.toString() + "  " +  QString().number(puertoremoto,10);
    myDebug->showMessage("PC--UDP-->MBED ( " + str + " )");
}

void MainWindow::getData(){
    uint8_t cmd, buf[24];
    int n;

    cmd=GETANALOGSENSORS;
    n=1; //bytes de la longitud del
    buf[0] = cmd;
    sendSerial(buf,n);
    sendUdp(buf,n);

    cmd = GETSPEED;
    n=1;
    buf[0] = cmd;
    sendSerial(buf,n);
    sendUdp(buf,n);

    cmd = GETDISTANCE;
    n=1;
    buf[0] = cmd;
    sendSerial(buf,n);
    sendUdp(buf,n);

    cmd=PATHLENGHT;
    n=1; //bytes de la longitud del
    buf[0] = cmd;
    sendSerial(buf,n);
    sendUdp(buf,n);
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event){ //utilizado para mostrar los puestos disponibles
    if(watched == ui->comboBox_PORT) {
        if (event->type() == QEvent::MouseButtonPress) {
            ui->comboBox_PORT->clear();
            QSerialPortInfo SerialPortInfo1;

            for(int i=0;i<SerialPortInfo1.availablePorts().count();i++)
                ui->comboBox_PORT->addItem(SerialPortInfo1.availablePorts().at(i).portName());

            return QMainWindow::eventFilter(watched, event);
        }
        else {
            return false;
        }
    }
    else{
        return QMainWindow::eventFilter(watched, event);
    }
}

void MainWindow::on_pushButton_connectSerial_clicked()
{
    if(QSerialPort1->isOpen()){
        QSerialPort1->close();
        ui->pushButton_sendSerial->setEnabled(false);        
        ui->pushButton_connectSerial->setText("CONNECT");
    }
    else{
        ui->pushButton_sendSerial->setEnabled(true);

        if(ui->comboBox_PORT->currentText() == "")
            return;

        QSerialPort1->setPortName(ui->comboBox_PORT->currentText());
        QSerialPort1->setBaudRate(115200);
        QSerialPort1->setParity(QSerialPort::NoParity);
        QSerialPort1->setDataBits(QSerialPort::Data8);
        QSerialPort1->setStopBits(QSerialPort::OneStop);
        QSerialPort1->setFlowControl(QSerialPort::NoFlowControl);

        if(QSerialPort1->open(QSerialPort::ReadWrite)){
            ui->pushButton_connectSerial->setText("DISCONNECT");
        }
        else
            QMessageBox::information(this, "Serial PORT", "ERROR. Opening PORT");
    }
}

void MainWindow::on_pushButton_sendSerial_clicked()
{

}

void MainWindow::on_pushButton_connectUdp_clicked()
{
    int Port;
    bool ok;

    if(QUdpSocket1->isOpen()){
        QUdpSocket1->close();
        ui->pushButton_sendUdp->setEnabled(false);
        ui->pushButton_connectUdp->setText("CONNECT");
        return;
    }

    Port=ui->lineEdit_local_port->text().toInt(&ok,10);
    if(!ok || Port<=0 || Port>65535){
        QMessageBox::information(this, tr("SERVER PORT"),tr("ERRRO. Number PORT."));
        return;
    }

    try{
        QUdpSocket1->abort();
        QUdpSocket1->bind(Port);
        QUdpSocket1->open(QUdpSocket::ReadWrite);
    }catch(...){
        QMessageBox::information(this, tr("SERVER PORT"),tr("Can't OPEN Port."));
        return;
    }

    ui->pushButton_connectUdp->setText("DISCONNECT");
    ui->pushButton_sendUdp->setEnabled(true);
    if(QUdpSocket1->isOpen()){
        if(clientAddress.isNull())
            clientAddress.setAddress(ui->lineEdit_device_ip->text());
        if(puertoremoto==0)
            puertoremoto=ui->lineEdit_device_port->text().toInt();
        QUdpSocket1->writeDatagram("r", 1, clientAddress, puertoremoto);
    }
}

void MainWindow::on_pushButton_sendUdp_clicked()
{

}

void MainWindow::on_pushButton_actRadar_clicked()
{
    if(ui->pushButton_actRadar->text()=="ACTIVATE"){
        servoAngle = 90; //configuramos el angulo de inicio
        angle = -180;

        ui->pushButton_actRadar->setText("DEACTIVATE");
        servoDir = false;
        firExe = true;

        //movemos el servo a la posicion inicial mediante conexion serial
        uint8_t cmd, buf[24];
        int n;
        cmd = SETSERVOANGLE;
        n = 2;
        buf[0] = cmd;
        buf[1] = servoAngle;
        sendSerial(buf,n);

        //activamos los timers
        timer3->start(100); //radar
        timer4->start(50); //distancia
        timer5->start(100); //velocidad servo

    } else{
        timer3->stop();
        timer4->stop();
        timer5->stop();
        ui->pushButton_actRadar->setText("ACTIVATE");
    }
}

void MainWindow::radar(){
    //variable local
    QPainter paint(QPaintBox1->getCanvas());

    QTime myTime;
    myTime = QTime::currentTime();

    QPen pen;
    QBrush brush; //

    ui->widget->setAttribute(Qt::WA_TranslucentBackground);

    pen.setWidth(2);
    brush.setStyle(Qt::SolidPattern);

    if(firExe){
        firExe=false;
        //pintamos el fondo
        paint.setOpacity(1);
        paint.opacity();
        pen.setColor(QColor::fromRgb(0, 70, 0));
        paint.setPen(pen);
        brush.setColor(QColor::fromRgb(0, 70, 0));
        paint.setBrush(brush);
        paint.resetTransform();
        paint.save();
        paint.translate(-ui->widget->width()/2,-ui->widget->height()/2);
        paint.drawEllipse(0,0,ui->widget->width()*2,ui->widget->height()*2);
        paint.restore();
        paint.save();
    }

    paint.setOpacity(1);
    paint.opacity();

    //circunferencia mas externa
    brush.setStyle(Qt::NoBrush);
    pen.setColor(QColor::fromRgb(0, 220, 0));
    paint.setPen(pen);
    brush.setColor(QColor::fromRgb(0, 220, 0));
    paint.setBrush(brush);
    paint.drawEllipse(0,0,ui->widget->width(),ui->widget->width());

    //circunferencia que le sigue en tamaño
    paint.save();
    paint.translate(ui->widget->width()/8,ui->widget->height()/4);//ui->widget->height(),ui->widget->height());
    paint.drawEllipse(0,0,ui->widget->width()*3/4,ui->widget->width()*3/4);
    paint.restore();
    paint.save();

    //siguente circunferencia
    paint.save();
    paint.translate(ui->widget->width()/4,ui->widget->height()/2);//ui->widget->height(),ui->widget->height());
    paint.drawEllipse(0,0,ui->widget->width()/2,ui->widget->width()/2);
    paint.restore();
    paint.save();

    //circunferencia mas pequeña
    paint.save();
    paint.translate(ui->widget->width()*3/8,ui->widget->height()*3/4);//ui->widget->height(),ui->widget->height());
    paint.drawEllipse(0,0,ui->widget->width()/4,ui->widget->width()/4);
    paint.restore();
    paint.save();

    pen.setColor(QColor::fromRgb(0, 180, 0));
    pen.setWidth(0);
    paint.setPen(pen);

    paint.save();
    paint.translate(ui->widget->width()/2,ui->widget->height());

    for(int i=1;i<=60;i++){
        paint.rotate(30);
        if(i%5==0){
            paint.drawLine(0,0,ui->widget->width(),0);
        }
    }

    paint.restore();
    paint.save();

    //dibujamos las lineas
    pen.setWidth(4);

    pen.setColor(QColor::fromRgb(0, 100, 0));
    paint.setPen(pen);
    paint.setOpacity(0.1);
    paint.opacity();

    paint.save();
    paint.translate(ui->widget->width()/2,ui->widget->height());
    paint.rotate(angle);
    paint.drawLine(0,0,ui->widget->width(),0);
    paint.restore();
    paint.save();

    pen.setColor(QColor::fromRgb(0, 200, 0));
    paint.setPen(pen);
    paint.setOpacity(0.2);
    paint.opacity();

    paint.save();
    paint.translate(ui->widget->width()/2,ui->widget->height());
    paint.rotate(angle);
    paint.drawLine(0,0,distance*10,0);
    paint.restore();
    paint.save();

    QPaintBox1->update();

    //condicion utilizada para pintar una vez el radar al inicio y que no quede en negro el widget
    if(firRadarExe){
        firRadarExe = false;
        timer7->start(100);
    }
}

void MainWindow::carStatus(){
    QPainter paint(QPaintBox2->getCanvas());
    QPen pen;
    QBrush brush;
    QPoint carStructure[8];
    QPoint lfTire[4];
    QPoint rtTire[4];
    QPoint usSensor[9]; //ultrasonic sensor
    QPoint lfIr[4]; //left IR
    QPoint cntIr[4]; //center IR
    QPoint rtIr[4]; //right IR

    pen.setWidth(2);
    brush.setStyle(Qt::SolidPattern);

    //definimos los puntos de la estructura del auto
    carStructure[0].setX(ui->carWidget->width()*2/25);
    carStructure[0].setY(ui->carWidget->height()*8/25);

    carStructure[1].setX(ui->carWidget->width()*8/25);
    carStructure[1].setY(ui->carWidget->height()*7/50);

    carStructure[2].setX(ui->carWidget->width()*17/25);
    carStructure[2].setY(ui->carWidget->height()*7/50);

    carStructure[3].setX(ui->carWidget->width()*23/25);
    carStructure[3].setY(ui->carWidget->height()*8/25);

    carStructure[4].setX(ui->carWidget->width()*17/25);
    carStructure[4].setY(ui->carWidget->height()*8/25);

    carStructure[5].setX(ui->carWidget->width()*17/25);
    carStructure[5].setY(ui->carWidget->height()*41/50);

    carStructure[6].setX(ui->carWidget->width()*8/25);
    carStructure[6].setY(ui->carWidget->height()*41/50);

    carStructure[7].setX(ui->carWidget->width()*8/25);
    carStructure[7].setY(ui->carWidget->height()*8/25);

    //pintamos la estructura
    pen.setColor(Qt::lightGray);
    paint.setPen(pen);
    brush.setColor(Qt::lightGray);
    paint.setBrush(brush);
    paint.save();
    paint.drawPolygon(carStructure,8);

    //definimos los puntos de la rueda izquierda
    lfTire[0].setX(ui->carWidget->width()*2/25);
    lfTire[0].setY(ui->carWidget->height()*1/2);

    lfTire[1].setX(ui->carWidget->width()*3/10);
    lfTire[1].setY(ui->carWidget->height()*1/2);

    lfTire[2].setX(ui->carWidget->width()*3/10);
    lfTire[2].setY(ui->carWidget->height()*41/50);

    lfTire[3].setX(ui->carWidget->width()*2/25);
    lfTire[3].setY(ui->carWidget->height()*41/50);

    //pintamos la rueda izquieda
    pen.setColor(Qt::gray);
    paint.setPen(pen);
    brush.setColor(Qt::gray);
    paint.setBrush(brush);
    paint.save();
    paint.drawPolygon(lfTire,4);

    //definimos los puntos de la rueda derecha
    rtTire[0].setX(ui->carWidget->width()*7/10);
    rtTire[0].setY(ui->carWidget->height()*1/2);

    rtTire[1].setX(ui->carWidget->width()*23/25);
    rtTire[1].setY(ui->carWidget->height()*1/2);

    rtTire[2].setX(ui->carWidget->width()*23/25);
    rtTire[2].setY(ui->carWidget->height()*41/50);

    rtTire[3].setX(ui->carWidget->width()*7/10);
    rtTire[3].setY(ui->carWidget->height()*41/50);

    //pintamos la rueda derecha
    pen.setColor(Qt::gray);
    paint.setPen(pen);
    brush.setColor(Qt::gray);
    paint.setBrush(brush);
    paint.save();
    paint.drawPolygon(rtTire,4);


    //definimos los puntos del IR de la izquierda
    lfIr[0].setX(ui->carWidget->width()*21/50);
    lfIr[0].setY(ui->carWidget->height()*8/25);

    lfIr[1].setX(ui->carWidget->width()*23/50);
    lfIr[1].setY(ui->carWidget->height()*8/25);

    lfIr[2].setX(ui->carWidget->width()*23/50);
    lfIr[2].setY(ui->carWidget->height()*9/25);

    lfIr[3].setX(ui->carWidget->width()*21/50);
    lfIr[3].setY(ui->carWidget->height()*9/25);

    pen.setColor(Qt::white);
    paint.setPen(pen);
    brush.setColor(Qt::white);
    paint.setBrush(brush);
    paint.save();
    paint.drawPolygon(lfIr,4);

    //definimos los puntos del IR de la derecha
    rtIr[0].setX(ui->carWidget->width()*27/50);
    rtIr[0].setY(ui->carWidget->height()*8/25);

    rtIr[1].setX(ui->carWidget->width()*29/50);
    rtIr[1].setY(ui->carWidget->height()*8/25);

    rtIr[2].setX(ui->carWidget->width()*29/50);
    rtIr[2].setY(ui->carWidget->height()*9/25);

    rtIr[3].setX(ui->carWidget->width()*27/50);
    rtIr[3].setY(ui->carWidget->height()*9/25);

    pen.setColor(Qt::white);
    paint.setPen(pen);
    brush.setColor(Qt::white);
    paint.setBrush(brush);
    paint.save();
    paint.drawPolygon(rtIr,4);

    //definimos los puntos del IR del centro
    cntIr[0].setX(ui->carWidget->width()*12/25);
    cntIr[0].setY(ui->carWidget->height()*8/25);

    cntIr[1].setX(ui->carWidget->width()*13/25);
    cntIr[1].setY(ui->carWidget->height()*8/25);

    cntIr[2].setX(ui->carWidget->width()*13/25);
    cntIr[2].setY(ui->carWidget->height()*9/25);

    cntIr[3].setX(ui->carWidget->width()*12/25);
    cntIr[3].setY(ui->carWidget->height()*9/25);

    pen.setColor(Qt::white);
    paint.setPen(pen);
    brush.setColor(Qt::white);
    paint.setBrush(brush);
    paint.save();
    paint.drawPolygon(cntIr,4);

    usSensor[0].setX(0);
    usSensor[0].setY(0);

    usSensor[1].setX(ui->carWidget->width()*-3/25);
    usSensor[1].setY(0);

    usSensor[2].setX(ui->carWidget->width()*-3/25);
    usSensor[2].setY(ui->carWidget->height()*-2/25);

    usSensor[3].setX(ui->carWidget->width()*-1/50);
    usSensor[3].setY(ui->carWidget->height()*-2/25);

    usSensor[4].setX(ui->carWidget->width()*-1/50);
    usSensor[4].setY(ui->carWidget->height()*-1/25);

    usSensor[5].setX(ui->carWidget->width()*1/50);
    usSensor[5].setY(ui->carWidget->height()*-1/25);

    usSensor[6].setX(ui->carWidget->width()*1/50);
    usSensor[6].setY(ui->carWidget->height()*-2/25);

    usSensor[7].setX(ui->carWidget->width()*3/25);
    usSensor[7].setY(ui->carWidget->height()*-2/25);

    usSensor[8].setX(ui->carWidget->width()*3/25);
    usSensor[8].setY(0);

    pen.setColor(Qt::darkGray);
    paint.setPen(pen);
    brush.setColor(Qt::darkGray);
    paint.setBrush(brush);

    paint.save();
    paint.translate(ui->carWidget->width()/2,ui->carWidget->height()*4/25);
    paint.rotate(0);
    paint.drawPolygon(usSensor,9);
    paint.restore();
    paint.save();




    QPaintBox2->update(); //actualizamos los datos actualizados
}

void MainWindow::onTimer4(){
    uint8_t cmd, buf[24];

    int n;
    cmd = GETDISTANCE;
    n = 1;
    buf[0] = cmd;
    sendSerial(buf,n);
}

void MainWindow::onTimer5(){

    uint8_t cmd, buf[24];
    int n;


    if(servoAngle == 90){
        servoDir=false;
    }
    if(servoAngle == -90){
        servoDir=true;
    }

    if(servoDir){
        servoAngle++;
        angle--;
    } else{
        servoAngle--;
        angle++;
    }

    //enviamos el comando con el angulo
    cmd = SETSERVOANGLE;
    n = 2;
    buf[0] = cmd;
    buf[1] = servoAngle;
    sendSerial(buf,n);


}

void MainWindow::onTimer7(){
    //utilizamos este timer para pintar el radar y que no quede en negro el widget
    timer3->stop();
    timer7->stop();
}
