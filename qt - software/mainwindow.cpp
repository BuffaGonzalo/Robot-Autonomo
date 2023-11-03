#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    timer1 = new QTimer(this);
    timer2 = new QTimer(this);

    QSerialPort1=new QSerialPort(this);
    QUdpSocket1 = new QUdpSocket(this);

    ui->comboBox_PORT->installEventFilter(this);

    //connects del puerto serial
    connect(ui->pushButton_sendSerial,&QPushButton::clicked, this, &MainWindow::sendDataSerial);
    connect(QSerialPort1,&QSerialPort::readyRead,this,&MainWindow::dataReceived);

    connect(timer1,&QTimer::timeout,this,&MainWindow::timeOut);
    connect(timer2,&QTimer::timeout,this,&MainWindow::getData);

    //connects de udp
    connect(QUdpSocket1,&QUdpSocket::readyRead,this,&MainWindow::OnUdpRxData);
    connect(ui->pushButton_sendUdp,&QPushButton::clicked,this,&MainWindow::sendDataUDP);

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

    //inicializamos
    estadoProtocolo=START;
    rxData.timeOut=0;

    //desabilitamos los botones con el fin de que no puedan ser presionados si no esta conectado
    ui->pushButton_sendUdp->setEnabled(false);
    ui->pushButton_sendSerial->setEnabled(false);

    timer1->start(100);
    timer2->start(500);

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
    ui->plainTextEdit->appendPlainText("MBED-->SERIAL-->PC (" + str + ")");

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
                    ui->plainTextEdit->appendPlainText("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    ui->plainTextEdit->appendPlainText("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **" );

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
    ui->plainTextEdit->appendPlainText("*(MBED-S->PC)->decodeData (" + str + ")");

    switch (datosRx[1]) {
    case GETANALOGSENSORS://     ANALOGSENSORS=0xA0,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "LEFT IR: " + str;
        ui->plainTextEdit->appendPlainText(strOut);
        ui->label_left_ir_data->setText(str);
        w.ui8[0] = datosRx[4];
        w.ui8[1] = datosRx[5];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "CENTER IR: " + str;
        ui->plainTextEdit->appendPlainText(strOut);
        ui->label_center_ir_data->setText(str);
        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        str =QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "RIGHT IR: " + str;
        ui->plainTextEdit->appendPlainText(strOut);
        ui->label_right_ir_data->setText(str);
        break;
    case SETMOTORTEST://     MOTORTEST=0xA1,
        if(datosRx[2]==0x0D)
            str= "Test Motores ACK";
        ui->plainTextEdit->appendPlainText(str);
        break;
    case SETSERVOANGLE://     SERVOANGLE=0xA2,
        if(datosRx[2]==0x0D)
            str= "Servo moviendose. Esperando posición Final!!!";
                else{
                if(datosRx[2]==0x0A)
                    str= "Servo en posición Final!!!";
            }
        ui->plainTextEdit->appendPlainText(str);
        break;
    case GETDISTANCE://     GETDISTANCE=0xA3,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        str = QString().number(w.ui32/58);
        ui->label_distance_data->setText(str+ "cm");
        ui->plainTextEdit->appendPlainText("DISTANCIA: "+QString().number(w.ui32/58)+ "cm");
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
        ui->plainTextEdit->appendPlainText(str);
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
        ui->plainTextEdit->appendPlainText(str);
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
        ui->plainTextEdit->appendPlainText(str);
        break;
    case GETFIRMWARE://     GETFIRMWARE=0xF1
        str = "FIRMWARE:";
        for(uint8_t a=0;a<(datosRx[0]-1);a++){
            str += (QChar)datosRx[2+a];
        }
        ui->plainTextEdit->appendPlainText(str);

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
        ui->plainTextEdit->appendPlainText(str);
        break;
    default:
        str = str + "Comando DESCONOCIDO!!!!";
        ui->plainTextEdit->appendPlainText(str);
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
        dato[indice++]=cmdId;
        //falta implementar el envío del valor de seteo
        dato[NBYTES]=0x02;
        break;
    default:
        return;
    }
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;

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
    ui->plainTextEdit->appendPlainText("INDICE ** " +QString().number(indice,10) + " **" );
    ui->plainTextEdit->appendPlainText("NUMERO DE DATOS ** " +QString().number(valor,10) + " **" );
    ui->plainTextEdit->appendPlainText("CHECKSUM ** " +QString().number(chk,16) + " **" );
    ui->plainTextEdit->appendPlainText("PC--SERIAL-->MBED ( " + str + " )");

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

    ui->plainTextEdit->appendPlainText(strHex);

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

    ui->plainTextEdit->appendPlainText("PC--UDP-->MBED ( " + str + " )");
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
    ui->plainTextEdit->appendPlainText("MBED-->UDP-->PC (" + str + ")");
    QString adress=RemoteAddress.toString();
    ui->plainTextEdit->appendPlainText(" adr " + adress);
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
                    ui->plainTextEdit->appendPlainText(" CHK DISTINTO!!!!! ");
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
        dato[indice++]=cmdId;
        dato[NBYTES]=0x02;
        break;
    default:
        ;

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
    ui->plainTextEdit->appendPlainText("PC--UDP-->MBED ( " + str + " )");
}

void MainWindow::getData(){
    uint8_t cmd, buf[24];
    int n;

    cmd=GETDISTANCE;
    n = 1;
    buf[0] = cmd;
    sendSerial(buf,n);
    sendUdp(buf,n);

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


void MainWindow::on_pushButton_clean_clicked()
{
    ui->plainTextEdit->clear();
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
