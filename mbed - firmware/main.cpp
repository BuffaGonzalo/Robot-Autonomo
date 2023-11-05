/*! \mainpage Ejercicio Titulo
 * \date 10/09/2023
 * \author Alejandro Rougier
 * \section Ejemplo comunicación USART
 * [Complete aqui con su descripcion]
 *
 * \section desarrollos Observaciones generales
 * [Complete aqui con sus observaciones]
 *
 * \section changelog Registro de cambios
 *
 * |   Fecha    | Descripcion                                    |
 * |:----------:|:-----------------------------------------------|
 * | 30/10/2023 | Creacion del documento                         |
 * | 31/10/2023 | Creacion del documento                         |
 * 
 * 
 * 
 */



/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "util.h"
#include "myDelay.h"
#include "debounce.h"
#include "config.h"
#include "wifi.h"
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/

/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
#define RXBUFSIZE  256
#define TXBUFSIZE  256
#define DEBOUNCE    20
#define HEARBEATIME 100
#define GENERALTIME 10
#define NUMBUTTONS  4
#define DISTANCEINTERVAL    300

#define     FORWARD             2
#define     BACKWARD            1
#define     STOP                0

#define     SERIE               0
#define     WIFI                1


#define RESETFLAGS      flags.bytes 
#define ISCOMAND        flags.bits.bit0
#define SERVOMOVING     flags.bits.bit1
#define SERVODIRECT     flags.bits.bit2
#define MEDIRDISTANCIA  flags.bits.bit3

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/

DigitalOut LED(PC_13);//!< Led de Hearbeta

BusOut leds(PB_6,PB_7,PB_14,PB_15);//!< leds de la placa

BusIn   pulsadores(PA_4,PA_5,PA_6,PA_7);//!< Botonnes de la placa

RawSerial PC(PA_9,PA_10);//!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT

DigitalOut trigger(PB_13);//!< Salida para el trigger del sensor Ultrasonico 

InterruptIn hecho(PB_12); //!<pin de eco del sensor Ultrasonico definido como interrupción 

PwmOut  servo(PA_8);//!< Pin del Servo debe ser PWM para poder modular el ancho del pulso

AnalogIn irLeft(PA_0); //!<Sensor infrarrojo para detección de linea 

AnalogIn irCenter(PA_1);//!<Sensor infrarrojo para detección de linea 

AnalogIn irRight(PA_2);//!<Sensor infrarrojo para detección de linea 

InterruptIn speedLeft(PB_9);//!<Sensor de Horquilla para medir velocidad

InterruptIn speedRight(PB_8);//!<Sensor de Horquilla para medir velocidad

BusOut  dirMLeft(PB_14,PB_15);//!< Pines para determinara la dirección de giro del motor

BusOut  dirMRight(PB_6,PB_7);//!< Pines para determinara la dirección de giro del motor

PwmOut  speedMLeft(PB_1);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo

PwmOut  speedMRight(PB_0);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo




/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Hearbeat, indica el funcionamiento del sistema
 * 
 * @param timeHearbeat Variable para el intervalo de tiempo
 * @param mask Secuencia de encendido/apagado del led de Hearbeat
 */
void hearbeatTask(_delay_t *timeHearbeat, uint16_t mask);

/**
 * @brief Ejecuta las tareas del puerto serie Decodificación/trasnmisión
 * 
 * @param dataRx Estructura de datos de la recepción
 * @param dataTx Estructura de datos de la trasnmisión
 * @param source Identifica la fuente desde donde se enviaron los datos
 */
void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source);


/**
 * @brief Rutina oara medir distancia
 * 
 * @param medicionTime variable que contiene el intervalo de medición de distancia
 * @param triggerTime Variable que contiene el valor de tiempo del Trigger
 */
void distanceTask(_delay_t *medicionTime, int32_t *triggerTime);


/**
 * @brief Rutina para realizar la verificación de mocimiento del servo
 * en caso de que no se mueva envia la respuesta automática a la PC
 * 
 * @param servoTime almacena el tiempo del timer
 * @param intervalServo posee el tiempo del ointervalo para responder en función del ángulo a mover
 */
void servoTask(_delay_t *servoTime, uint32_t *intervalServo);

/**
 * @brief Rutina para medir la velocidad
 * 
 */
void speedTask();

/**
 * @brief Rutina para hacer la medición de los sensores IR
 * 
 */
void irSensorsTask();


/**
 * @brief Recepción de datos por el puerto serie
 * 
 */
void onRxData();

/**
 * @brief Pone el encabezado del protocolo, el ID y la cantidad de bytes a enviar
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param ID Identificación del comando que se envía
 * @param frameLength Longitud de la trama del comando
 * @return uint8_t devuelve el Checksum de los datos agregados al buffer de trasnmisión
 */
uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength);

/**
 * @brief Agrega un byte al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param byte El elemento que se quiere agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putByteOnTx(_sTx    *dataTx, uint8_t byte);

/**
 * @brief Agrega un String al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param str String a agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putStrOntx(_sTx *dataTx, const char *str);

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos);

/**
 * @brief Decodifica la trama recibida
 * 
 * @param dataRx Estructura para la recepción de datos
 */
void decodeHeader(_sRx *dataRx);

/**
 * @brief Decodifica el comando recibido en la transmisión y ejecuita las tareas asociadas a dicho comando
 * 
 * @param dataRx Estructura para la recepción de datos
 * @param dataTx Estructura para la trasnmisión de datos
 */
void decodeCommand(_sRx *dataRx, _sTx *dataTx);


/**
 * @brief Función del Sensor de Horquilla Izquierdo
 * cuenta los pulsos del sensor para medir velocidad luego
 */
void speedCountLeft(void);

/**
 * @brief Función del Sensor de Horquilla Derecho
 * cuenta los pulsos del sensor para medir velocidad luego
 */
void speedCountRight(void);

/**
 * @brief toma el valor inicial del Timer para medir distancia
 * una vez que se detecta el evento RISE del pin de eco
 */
void distanceInitMeasurement(void);

/**
 * @brief Completa la medición una vez que se recibió el pulso de regreso
 * en el pin de eco
 */
void distanceMeasurement(void);

/**
 * @brief Función para realizar la autoconexión de los datos de Wifi
 * 
 */
void autoConnectWifi();

/**
 * @brief envía de manera automática el alive
 * 
 */
void aliveAutoTask(_delay_t *aliveAutoTime);

/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

const char firmware[] = "EX100923v01\n";

volatile _sRx dataRx;

volatile uint32_t countLeftValue, countRightValue;

volatile int32_t  initialValue, finalValue, distanceValue;

uint32_t speedleftValue, speedRightValue;

_sTx dataTx;

wifiData myWifiData;

volatile uint8_t buffRx[RXBUFSIZE];

uint8_t buffTx[TXBUFSIZE];

uint8_t globalIndex, index2;

_uFlag  flags;

_sButton myButton[NUMBUTTONS];

_delay_t    generalTime;

Timer myTimer;

_sSensor irSensor[3];

_sServo miServo;

_uWord myWord;

//_eModes carMode

_sTx wifiTx;

_sRx wifiRx;

uint8_t wifiBuffRx[RXBUFSIZE];

uint8_t wifiBuffTx[TXBUFSIZE];


/* END Global variables ------------------------------------------------------*/


/* Function prototypes user code ----------------------------------------------*/


/**
 * @brief Instanciación de la clase Wifi, le paso como parametros el buffer de recepción, el indice de 
 * escritura para el buffer de recepción y el tamaño del buffer de recepción
 */
Wifi myWifi(wifiBuffRx, &wifiRx.indexW, RXBUFSIZE);


void hearbeatTask(_delay_t *timeHearbeat, uint16_t mask)
{
    static uint8_t sec=0;
    if(delayRead(timeHearbeat)){
        LED = (~mask & (1<<sec));
        sec++;
        sec &= -(sec<16);
    }
}

void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source)
{
    if(dataRx->isComannd){
        dataRx->isComannd=false;
        decodeCommand(dataRx,dataTx);
    }

    if(delayRead(&generalTime)){
        if(dataRx->header){
            dataRx->timeOut--;
        if(!dataRx->timeOut)
            dataRx->header = HEADER_U;
        }
    }

    if(dataRx->indexR!=dataRx->indexW){
        decodeHeader(dataRx);
       /* CODIGO A EFECTOS DE EVALUAR SI FUNCIONA LA RECEPCIÓN , SE DEBE DESCOMENTAR 
       Y COMENTAR LA LINEA decodeHeader(dataRx); 
       while (dataRx->indexR!=dataRx->indexW){
            dataTx->buff[dataTx->indexW++]=dataRx->buff[dataRx->indexR++];
            dataTx->indexW &= dataTx->mask;
            dataRx->indexR &= dataRx->mask;
        } */
    }
        

    if(dataTx->indexR!=dataTx->indexW){
        if(source){
             myWifi.writeWifiData(&dataTx->buff[dataTx->indexR++],1); 
             dataTx->indexR &=dataTx->mask; 
        }else{
            if(PC.writeable()){
                PC.putc(dataTx->buff[dataTx->indexR++]);
                dataTx->indexR &=dataTx->mask;
            }
        }
    }

}


void onRxData()
{
    while(PC.readable()){
        dataRx.buff[dataRx.indexW++]=PC.getc();
        dataRx.indexW &= dataRx.mask;
    }
}

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength)
{
    dataTx->chk = 0;
    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength+1;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= (frameLength+1);
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^ID ^':') ;
    return  dataTx->chk;
}

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte)
{
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}


uint8_t putStrOntx(_sTx *dataTx, const char *str)
{
    globalIndex=0;
    while(str[globalIndex]){
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    return dataTx->chk ;
}

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos){
    uint8_t getByte;
    dataRx->indexData += iniPos;
    dataRx->indexData &=dataRx->mask;
    getByte = dataRx->buff[dataRx->indexData];
    dataRx->indexData += finalPos;
    dataRx->indexData &=dataRx->mask;
    return getByte;
}

void decodeHeader(_sRx *dataRx)
{
    uint8_t auxIndex=dataRx->indexW;
    while(dataRx->indexR != auxIndex){
        switch(dataRx->header)
        {
            case HEADER_U:
                if(dataRx->buff[dataRx->indexR] == 'U'){
                    dataRx->header = HEADER_N;
                    dataRx->timeOut = 5;
                }
            break;
            case HEADER_N:
                if(dataRx->buff[dataRx->indexR] == 'N'){
                    dataRx->header = HEADER_E;
                }else{
                    if(dataRx->buff[dataRx->indexR] != 'U'){
                        dataRx->header = HEADER_U;
                        dataRx->indexR--;
                    }
                }
            break;
            case HEADER_E:
                if(dataRx->buff[dataRx->indexR] == 'E'){
                    dataRx->header = HEADER_R;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case HEADER_R:
                if(dataRx->buff[dataRx->indexR] == 'R'){
                    dataRx->header = NBYTES;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case NBYTES:
                dataRx->nBytes=dataRx->buff[dataRx->indexR];
                dataRx->header = TOKEN;
            break;
            case TOKEN:
                if(dataRx->buff[dataRx->indexR] == ':'){
                    dataRx->header = PAYLOAD;
                    dataRx->indexData = dataRx->indexR+1;
                    dataRx->indexData &= dataRx->mask;
                    dataRx->chk = 0;
                    dataRx->chk ^= ('U' ^'N' ^'E' ^'R' ^dataRx->nBytes ^':') ;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case PAYLOAD:
                dataRx->nBytes--;
                if(dataRx->nBytes>0){
                   dataRx->chk ^= dataRx->buff[dataRx->indexR];
                }else{
                    dataRx->header = HEADER_U;
                    if(dataRx->buff[dataRx->indexR] == dataRx->chk)
                        dataRx->isComannd = true;
                }
            break;
            default:
                dataRx->header = HEADER_U;
            break;
        }
        dataRx->indexR++;
        dataRx->indexR &= dataRx->mask;
    }
}


void decodeCommand(_sRx *dataRx, _sTx *dataTx)
{
    int32_t motorSpeed, auxSpeed;
    int8_t angleSource;
    uint32_t servoPrevio=miServo.currentValue;
    switch(dataRx->buff[dataRx->indexData]){
        case ALIVE:
            putHeaderOnTx(dataTx, ALIVE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case FIRMWARE:
            putHeaderOnTx(dataTx, FIRMWARE, 12);
            putStrOntx(dataTx, firmware);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case LEDSTATUS:
        // NO ESTA CONTEMPLANDO EL ENCENDIDO/APAGADO DE LOS LEDS, SOLO EL ESTADO
            putHeaderOnTx(dataTx, LEDSTATUS, 2);
            putByteOnTx(dataTx, ((~((uint8_t)leds.read()))&0x0F));
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case BUTTONSTATUS:
            putHeaderOnTx(dataTx, BUTTONSTATUS, 2);
            putByteOnTx(dataTx, ((~((uint8_t)pulsadores.read()))&0x0F));
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case ANALOGSENSORS:
            myWord.ui16[0] =  irSensor[0].currentValue;
            putHeaderOnTx(dataTx, ANALOGSENSORS, 7);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            myWord.ui16[0] =  irSensor[1].currentValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            myWord.ui16[0] =  irSensor[2].currentValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] ); 
            putByteOnTx(dataTx, dataTx->chk);       
        break;
        case SETBLACKCOLOR:
        break;
        case SETWHITECOLOR:
        break;
        case MOTORTEST:
            putHeaderOnTx(dataTx, MOTORTEST, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);
            motorSpeed = myWord.i32;
            if(motorSpeed>=0)
                dirMLeft.write(FORWARD);
            else
                dirMLeft.write(BACKWARD);
            auxSpeed=(abs(motorSpeed))*250;
            speedMLeft.pulsewidth_us(auxSpeed);
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);
            motorSpeed = myWord.i32;
            if(motorSpeed>=0)
                dirMRight.write(FORWARD);
            else
                dirMRight.write(BACKWARD);
            auxSpeed=(abs(motorSpeed))*250;
            speedMRight.pulsewidth_us (auxSpeed);
        break;
        case SERVOANGLE:
            putHeaderOnTx(dataTx, SERVOANGLE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
            SERVOMOVING=true;
            angleSource = getByteFromRx(dataRx,1,0);
            if (angleSource<-90 || angleSource>90)
                break;
            else{
                miServo.currentValue = (((angleSource - miServo.Y1) * (miServo.X2-miServo.X1))/(miServo.Y2-miServo.Y1))+miServo.X1;
                if(miServo.currentValue > (uint16_t)miServo.X2)
                    miServo.currentValue=miServo.X2;
                if(miServo.currentValue < (uint16_t)miServo.X1)
                    miServo.currentValue=miServo.X1;
                servo.pulsewidth_us(miServo.currentValue);
            }
            if(miServo.currentValue>servoPrevio){
                miServo.intervalValue=(miServo.currentValue-servoPrevio);
            }else{
                miServo.intervalValue=(servoPrevio-miServo.currentValue);
            }
            miServo.intervalValue=(miServo.intervalValue*1000) /(miServo.X2-miServo.X1);
            if(miServo.intervalValue>1000)
                miServo.intervalValue=1000;
            if(miServo.intervalValue<50)
                miServo.intervalValue=50;
        break;
        case CONFIGSERVO:
        break;
        case GETDISTANCE:
            myWord.ui32 = distanceValue;
            putHeaderOnTx(dataTx, GETDISTANCE, 5);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            putByteOnTx(dataTx, dataTx->chk);  
        break;
        case GETSPEED:
            myWord.ui32 = speedleftValue;
            putHeaderOnTx(dataTx, GETSPEED, 9);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            myWord.ui32 = speedRightValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            putByteOnTx(dataTx, dataTx->chk);           
        break;
        default:
            putHeaderOnTx(dataTx, (_eCmd)dataRx->buff[dataRx->indexData], 2);
            putByteOnTx(dataTx,UNKNOWN );
            putByteOnTx(dataTx, dataTx->chk);
        break;
        
    }


}



void distanceTask(_delay_t *medicionTime, int32_t *triggerTime){

    if(delayRead(medicionTime)){
        MEDIRDISTANCIA=true; 
        trigger.write(false);
    }
        /******************** MEDICIÓN DE DISTANCIA NON-BLOCKING ********************/
    if (MEDIRDISTANCIA){
        if((myTimer.read_us()-*triggerTime)>=10){
            *triggerTime=myTimer.read_us();
            if(trigger.read()){
                trigger.write(false);
                MEDIRDISTANCIA=false;
            }else{
                trigger.write(true);
            }
        }
    }
}

void servoTask(_delay_t *servoTime, uint32_t*intervalServo){
     /******************** RESPUESTA AUTOMATICA DEL SERVO ********************/
    if(SERVOMOVING){
        if(delayRead(servoTime)){
            SERVOMOVING=false;
            putHeaderOnTx(&dataTx,SERVOANGLE,2);
            putByteOnTx(&dataTx,SERVOFINISHMOVE);
            putByteOnTx(&dataTx, dataTx.chk);
        }
    }
}

void speedTask(){
    static int32_t timeSpeed=0;
    #define INTERVAL 1000
    if ((myTimer.read_ms()-timeSpeed)>=INTERVAL){
            timeSpeed=myTimer.read_ms();       
            speedleftValue = countLeftValue;
            countLeftValue=0;
            speedRightValue=countRightValue;
            countRightValue=0;
    } 
}


void irSensorsTask(){
    static int32_t timeSensors=0;
    //static uint8_t index=0;
    #define INTERVALO 10
    if ((myTimer.read_ms()-timeSensors)>=INTERVALO){
        timeSensors=myTimer.read_ms(); 
        irSensor[0].currentValue=irLeft.read_u16();
        irSensor[1].currentValue=irCenter.read_u16();
        irSensor[2].currentValue=irRight.read_u16();
    } 

}


void speedCountLeft(void){
    countLeftValue++;
}

void speedCountRight(void){
    countRightValue++;
}

void distanceInitMeasurement(void){
    initialValue=myTimer.read_us();
}

void distanceMeasurement(void){
    finalValue=myTimer.read_us();
    if (finalValue>=initialValue)
        distanceValue=finalValue-initialValue;
    else
       distanceValue=finalValue-initialValue+0xFFFFFFFF;
}



/**********************************AUTO CONNECT WIF*********************/

void autoConnectWifi(){
    #ifdef AUTOCONNECTWIFI
        memcpy(&myWifiData.cwmode, dataCwmode, sizeof(myWifiData.cwmode));
        memcpy(&myWifiData.cwdhcp,dataCwdhcp, sizeof(myWifiData.cwdhcp) );
        memcpy(&myWifiData.cwjap,dataCwjap, sizeof(myWifiData.cwjap) );
        memcpy(&myWifiData.cipmux,dataCipmux, sizeof(myWifiData.cipmux) );
        memcpy(&myWifiData.cipstart,dataCipstart, sizeof(myWifiData.cipstart) );
        memcpy(&myWifiData.cipmode,dataCipmode, sizeof(myWifiData.cipmode) );
        memcpy(&myWifiData.cipsend,dataCipsend, sizeof(myWifiData.cipsend) );
        myWifi.configWifi(&myWifiData);
    #endif
}


void aliveAutoTask(_delay_t *aliveAutoTime)
{
    if(myWifi.isWifiReady()){
        if(delayRead(aliveAutoTime))
        {
            putHeaderOnTx(&wifiTx, ALIVE, 2);
            putByteOnTx(&wifiTx, ACK );
            putByteOnTx(&wifiTx, wifiTx.chk);
        }
    }
}

/* END Function prototypes user code ------------------------------------------*/




int main()
{

    dataRx.buff = (uint8_t *)buffRx;
    dataRx.indexR = 0;
    dataRx.indexW = 0;
    dataRx.header = HEADER_U;
    dataRx.mask = RXBUFSIZE - 1;

    dataTx.buff = buffTx;
    dataTx.indexR = 0;
    dataTx.indexW = 0;
    dataTx.mask = TXBUFSIZE -1;

    wifiRx.buff = wifiBuffRx;
    wifiRx.indexR = 0;
    wifiRx.indexW = 0;
    wifiRx.header = HEADER_U;
    wifiRx.mask = RXBUFSIZE - 1;

    wifiTx.buff = wifiBuffTx;
    wifiTx.indexR = 0;
    wifiTx.indexW = 0;
    wifiTx.mask = TXBUFSIZE -1;

    RESETFLAGS = 0;

/* Local variables -----------------------------------------------------------*/
    uint16_t    mask = 0x000A;
    _delay_t    hearbeatTime;
    _delay_t    debounceTime;
    _delay_t    medicionTime;
    _delay_t    servoTime;
    _delay_t    aliveAutoTime;
 
    int32_t    triggerTime;
   /* SERVO DEFUALT VALUES*/
    miServo.X2=2500;
    miServo.X1=800;
    miServo.Y2=90;
    miServo.Y1=-90;
    miServo.intervalValue=1000;
    /* FIN SERVO DEFAULT VALUES*/
/* END Local variables -------------------------------------------------------*/


/* User code -----------------------------------------------------------------*/
    PC.baud(115200);
    myTimer.start();
    speedMLeft.period_ms(25);
    speedMRight.period_ms(25);
    servo.period_ms(20);
    trigger.write(false);

   /********** attach de interrupciones ********/
    PC.attach(&onRxData, SerialBase::IrqType::RxIrq);

    speedLeft.rise(&speedCountLeft);

    speedRight.rise(&speedCountRight);

    hecho.rise(&distanceInitMeasurement);

    hecho.fall(&distanceMeasurement);

     /********** FIN - attach de interrupciones ********/

    miServo.currentValue=1500;
    
    servo.pulsewidth_us(miServo.currentValue);

    speedMLeft.pulsewidth_us(0);
    
    speedMRight.pulsewidth_us(0);

    delayConfig(&hearbeatTime, HEARBEATIME);
    delayConfig(&debounceTime, DEBOUNCE);
    delayConfig(&generalTime,GENERALTIME);
    delayConfig(&medicionTime,DISTANCEINTERVAL);
    delayConfig(&servoTime,miServo.intervalValue);
    delayConfig(&aliveAutoTime, ALIVEAUTOINTERVAL);

    startButon(myButton, NUMBUTTONS);
    
    myWifi.initTask();
    
    autoConnectWifi();


    //dirijo el servo a los lados
    servo.pulsewidth_us(800);
    wait_ms(3000);
    servo.pulsewidth_us(2500);
    wait_ms(3000);
    //dirijo el servo al centro
    servo.pulsewidth_us(1500);
    wait_ms(3000);


    while(1){
        myWifi.taskWifi();
        hearbeatTask(&hearbeatTime, mask);
        serialTask((_sRx *)&dataRx,&dataTx, SERIE);
        serialTask(&wifiRx,&wifiTx, WIFI);
        buttonTask(&debounceTime,myButton, pulsadores.read());
        distanceTask(&medicionTime,&triggerTime);
        speedTask();
        irSensorsTask();
        servoTask(&servoTime,&miServo.intervalValue);
        aliveAutoTask(&aliveAutoTime);

        /*
        switch(carMode){
            case IDLE:

            break;
            case MODE1:

            break;
            case MODE2:

            break;
            case MODE3:

            break;
        }
        */
    }

/* END User code -------------------------------------------------------------*/
}









