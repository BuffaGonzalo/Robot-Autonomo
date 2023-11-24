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
#include "config.h"
#include "wifi.h"
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/
/**
 * @brief Tipo de datos de puntero a función, sirve para declarar los distintos callbacks.-
 * 
 */
typedef void(*ptrFunc)(void *param);

//ENUMERACIONES
typedef enum{
    BUTTON_DOWN,
    BUTTON_UP,
    BUTTON_RISING,
    BUTTON_FALLING
}_eButtonState;

/**
 * @brief Enumeracion de los estados de los diferentes estados de los botones, como tengo una configuracion PullDown los coloqué de tal forma que me quede el valor de NOT_PRESSED = 0  y PRESSED = 1
*/
typedef enum{
    PRESSED,
    NOT_PRESSED,
    NO_EVENT
}_eEvent;

//ESTRUCTURAS
typedef struct
{
    _eButtonState   currentState;
    _eEvent         stateInput;
    ptrFunc         callBack;
    uint32_t        timePressed;
    uint32_t        timeDiff;
}_sButton;

/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
#define RXBUFSIZE            256
#define TXBUFSIZE            256
#define DEBOUNCE             40
#define HEARBEATIME          100
#define GENERALTIME          10
#define NUMBUTTONS           1
#define DISTANCEINTERVAL     300
#define MASK                 0x01

#define LOOKTIME             600

//#define RIGHTSERVO         700
//#define LEFTSERVO          2500
#define MIDDLESERVO          1500
#define MINMOVEDISTANCE      9
#define MAXDISTANCE

/*
#define INTERSERVODISTANCE   25
#define MAXSERVODISTANCE     50
#define SERVOANGLE           375
*/

#define MOTORTIME            200
#define SERVOTIME            500

#define WAIT100MS            100
#define WAIT200MS            200
#define WAIT250MS            250
#define WAIT300MS            300
#define WAIT400MS            400
#define WAIT500MS            500
#define WAIT1000MS           1000
#define WAIT2000MS           2000

#define MAXSPEED             12000
#define MEDSPEED             8000
#define MINSPEED             5000
#define TURNSPEED            6000
#define NOSPEED              0
#define SPEEDERROR           1000 //Utilizado para contrarestar la diferencia de velocidades en los motores entre 5500 y 6000

#define INTERVALO            10

#define LIMIT                0x0F // pulsador

#define FORWARD              2
#define BACKWARD             1
#define STOP                 0

#define SERIE                0
#define WIFI                 1

#define DERECHA              0
#define IZQUIERDA            1

#define DODGEDISTANCE        10
#define REFDISTANCE          20


#define RESETFLAGS           flags.bytes 
#define ISCOMAND             flags.bits.bit0
#define SERVOMOVING          flags.bits.bit1
#define SERVODIRECT          flags.bits.bit2
#define MEDIRDISTANCIA       flags.bits.bit3
/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/

//BusOut leds(PB_6,PB_7,PB_14,PB_15);//!< leds de la placa

//BusIn   pulsadores(PA_4,PA_5,PA_6,PA_7);//!< Botonnes de la placa

DigitalOut HEARTBEAT(PC_13);//!< Led de Hearbeat

DigitalIn BUTTON(PA_4);

RawSerial PC(PA_9,PA_10);//!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT

DigitalOut trigger(PB_13);//!< Salida para el trigger del sensor Ultrasonico 

InterruptIn hecho(PB_12); //!<pin de eco del sensor Ultrasonico definido como interrupción 

PwmOut  servo(PA_8);//!< Pin del Servo debe ser PWM para poder modular el ancho del pulso

AnalogIn irLeft(PA_0); //!<Sensor infrarrojo para detección de linea 

AnalogIn irCenter(PA_1);//!<Sensor infrarrojo para detección de linea 

AnalogIn irRight(PA_2);//!<Sensor infrarrojo para detección de linea 

InterruptIn speedLeft(PB_9);//!<Sensor de Horquilla para medir velocidad

InterruptIn speedRight(PB_8);//!<Sensor de Horquilla para medir velocidad

BusOut  dirMRight(PB_14,PB_15);//!< Pines para determinara la dirección de giro del motor

BusOut  dirMLeft(PB_6,PB_7);//!< Pines para determinara la dirección de giro del motor

PwmOut  speedMRight(PB_1);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo

PwmOut  speedMLeft(PB_0);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo


/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

//HEARTBEAT
/**
 * @brief Hearbeat, indica el funcionamiento del sistema
 * 
 * @param timeHeartbeat Variable para el intervalo de tiempo
 * @param mask Secuencia de encendido/apagado del led de Hearbeat
 */
void hearbeatTask(_delay_t *timeHeartbeat, uint8_t index);

//CONEXION SERIAL - WIFI
/**
 * @brief Ejecuta las tareas del puerto serie Decodificación/trasnmisión
 * 
 * @param dataRx Estructura de datos de la recepción
 * @param dataTx Estructura de datos de la trasnmisión
 * @param source Identifica la fuente desde donde se enviaron los datos
 */
void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source);

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
 * @brief Función para realizar la autoconexión de los datos de Wifi
 * 
 */
void autoConnectWifi();

/**
 * @brief Envía de manera automática el alive
 * 
 */
void aliveAutoTask(_delay_t *aliveAutoTime);

//SENSORES

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
 * @brief Función encargada de medir la distancia
*/
void do100ms();

/**
 * @brief Función encargada de calcular el tiempo
*/
void doTimeout();

//BOTONES
/**
 * @brief Función con la cual inicializamos los botones
 * @param _sButton Estructura con los datos del boton
 * @param buttonFunction Puntero a funcion
*/
void startButton(_sButton *button, ptrFunc buttonFunc);

/**
 * @brief Función utilizada para actualizar la MEF de los botones
 * @param _sButton Estructura con los datos
*/
uint8_t updateMefTask(_sButton *button);

void buttonTask(void *param);

//MOVIMIENTO
/**
 * @brief Función encargada de mover hacia adelante el robot
*/
void move(uint32_t leftSpeed, uint32_t rightSpeed, uint8_t leftMotor, uint8_t rightMotor);

void rotate(uint8_t rotationAngle, uint8_t direction);

//MODOS
/**
 * @brief Función utilizada en el modo de seguir linea
*/
void lineFollower();

void dodgeObstacle();

void maintainDistance();

/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

//VALOR FIRMWARE
const char firmware[] = "EX100923v01\n";

//MASCARAS
const uint8_t irMask = 0x01;

uint32_t heartBeatMask[] = {0x55555555, 0x1, 0x2010080, 0x5F, 0x5, 0x28140A00, 0x15F, 0x15, 0x2A150A08, 0x55F}; //IDLE, MODO1, MODO1(1-5), MODO1ON, MODO2, MODO2(1-5), MODO2ON, MODO3, MODO3(1-5), MODO3ON

//BANDERAS
_uFlag  flags;

//COMUNICACION SERIAL - WIFI
_uWord myWord;

_sTx dataTx;

wifiData myWifiData;

volatile _sRx dataRx;

volatile uint8_t buffRx[RXBUFSIZE];

uint8_t buffTx[TXBUFSIZE];

uint8_t globalIndex, index2;

//VARIABLES SENSORES
_sSensor irSensor[3];

_sServo miServo;

_sTx wifiTx;

_sRx wifiRx;

uint8_t wifiBuffRx[RXBUFSIZE];

uint8_t wifiBuffTx[TXBUFSIZE];

volatile uint32_t countLeftValue, countRightValue;

volatile int32_t  initialValue, finalValue, distanceValue;

uint32_t speedleftValue, speedRightValue;

uint16_t whiteValue = 9000;

uint16_t blackValue = 4000;

uint16_t minMsServo = 700;

uint16_t maxMsServo = 2500;

uint8_t irSensorValue = 0;

int32_t timeSpeed=0; //variable utilizada para 

int32_t timeFollowLine = 0; //variable utilizada para realizar la lectura cada 10ms de los datos

int32_t timeToDebounce = 0;

_sButton myButton[NUMBUTTONS];

//VARIABLES DE TIEMPO - TIMEOUTS, TIMERS, TICKERS

_delay_t generalTime;

Timer myTimer;

Timer distanceTimer; //distance timer

Ticker timerGral;

Timeout triggerTimer;

//MODOS DEL AUTO
_eModes carModes;

_eFollowModes followModes;

_eDodgeModes dodgeModes;

//HEARTBEAT
uint8_t heartBeatIndex = 0;

/* END Global variables ------------------------------------------------------*/


/* Function prototypes user code ----------------------------------------------*/

/**
 * @brief Instanciación de la clase Wifi, le paso como parametros el buffer de recepción, el indice de 
 * escritura para el buffer de recepción y el tamaño del buffer de recepción
 */
Wifi myWifi(wifiBuffRx, &wifiRx.indexW, RXBUFSIZE);

//HEARTBEAT
void hearbeatTask(_delay_t *heartBeatTime, uint8_t index)
{
    static uint8_t times=0;
    if(delayRead(heartBeatTime)){
        HEARTBEAT = (~heartBeatMask[index] & (1<<times));
        times++;
        times &= 31; //control de times
    }
}

//CONEXION SERIAL - WIFI
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
        /*
            putHeaderOnTx(dataTx, LEDSTATUS, 2);
            putByteOnTx(dataTx, ((~((uint8_t)leds.read()))&0x0F));
            putByteOnTx(dataTx, dataTx->chk);
        */
        break;
        case BUTTONSTATUS:
        /*
            putHeaderOnTx(dataTx, BUTTONSTATUS, 2);
            putByteOnTx(dataTx, ((~((uint8_t)pulsadores.read()))&0x0F));
            putByteOnTx(dataTx, dataTx->chk);
        */
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
            uint16_t myBlackValue;
            putHeaderOnTx(dataTx, SETBLACKCOLOR, 3); //framelenght = 6(sensores)+1(cabecera)
            
            //obtenemos el mayor valor
            myBlackValue = irSensor[0].currentValue;
            if(myBlackValue < irSensor[1].currentValue)
                myBlackValue = irSensor[1].currentValue;
            if(myBlackValue < irSensor[2].currentValue)
                myBlackValue = irSensor[2].currentValue;

            blackValue = myBlackValue;

            //valor util
            myWord.ui16[0] = myBlackValue;
            putByteOnTx(dataTx, myWord.ui8[0]);
            putByteOnTx(dataTx, myWord.ui8[1]);

            putByteOnTx(dataTx, dataTx->chk); 
        break;
        case SETWHITECOLOR:
            uint16_t myWhiteValue;
            putHeaderOnTx(dataTx, SETWHITECOLOR, 3); //framelenght = 6(sensores)+1(cabecera)
            
            //obtenemos el menor valor
            myWhiteValue = irSensor[0].currentValue;
            if(myWhiteValue > irSensor[1].currentValue)
                myWhiteValue = irSensor[1].currentValue;
            if(myWhiteValue > irSensor[2].currentValue)
                myWhiteValue = irSensor[2].currentValue;

            whiteValue = myWhiteValue;

            //valor util
            myWord.ui16[0] = myWhiteValue;
            putByteOnTx(dataTx, myWord.ui8[0]);
            putByteOnTx(dataTx, myWord.ui8[1]);

            putByteOnTx(dataTx, dataTx->chk); 
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
            speedMLeft.pulsewidth_us(auxSpeed-3000);
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
            //recibimos los datos ingresados
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);

            minMsServo = myWord.ui16[0];
            maxMsServo = myWord.ui16[1];

            putHeaderOnTx(dataTx, CONFIGSERVO, 5); //framelenght = 6(sensores)+1(cabecera)            
            putByteOnTx(dataTx, myWord.ui8[0]);
            putByteOnTx(dataTx, myWord.ui8[1]);
            putByteOnTx(dataTx, myWord.ui8[2]);
            putByteOnTx(dataTx, myWord.ui8[3]);
            putByteOnTx(dataTx, dataTx->chk);  //colocamos el checksum
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
    initialValue=distanceTimer.read_us();
}

void distanceMeasurement(void){
    finalValue=distanceTimer.read_us();
    if (finalValue>=initialValue)
        distanceValue=finalValue-initialValue;
    else
       distanceValue=finalValue-initialValue+0xFFFFFFFF;
}

void doTimeout()
{
    distanceTimer.reset();
    trigger.write(0);   // Prendemos o apagamos la salida
}

void do100ms() 
{
    trigger.write(1);
    triggerTimer.attach_us(&doTimeout,10);
}

//BOTONES
void startButton(_sButton *button, ptrFunc buttonFunc){
    button->currentState = BUTTON_UP;
    button->stateInput = NO_EVENT;
    button->callBack = buttonFunc;
    button->timePressed = 0;
    button->timeDiff = 0;
}

uint8_t updateMefTask(_sButton *button){
    uint8_t action=false;
    
    switch (button->currentState){
        case BUTTON_UP:
            if(button->stateInput==PRESSED)
                button->currentState=BUTTON_FALLING;
        break;
        case BUTTON_FALLING:
            if(button->stateInput==PRESSED){
                button->currentState=BUTTON_DOWN;
                button->timePressed=myTimer.read_ms();
            }else{
                button->currentState=BUTTON_UP;
            }
        break;
        case BUTTON_DOWN:
            if(button->stateInput==NOT_PRESSED)
                button->currentState=BUTTON_RISING;
        break;
        case BUTTON_RISING:
            if(button->stateInput==NOT_PRESSED){
                button->currentState=BUTTON_UP;
                button->timeDiff = myTimer.read_ms() - button->timePressed;
                action=true;
            }else{
                button->currentState=BUTTON_DOWN;
            }
        break;
        default:
            button->currentState=BUTTON_UP;
        break;
    }
    return action;
}

void buttonTask(void *param){
    static uint8_t indice=0;
    uint16_t *maskLed = (uint16_t*)param;
   
    *maskLed &= -(indice!=0);
    *maskLed |= (1<<indice);
    indice +=2;
    indice &=LIMIT;
}

//MOVIMIENTO
void move(uint32_t leftSpeed, uint32_t rightSpeed, uint8_t leftMotor, uint8_t rightMotor){
    //definimos el modo
    dirMLeft.write(leftMotor);
    dirMRight.write(rightMotor);
    
    //asignamos la velocidad
    speedMLeft.pulsewidth_us(leftSpeed);
    speedMRight.pulsewidth_us (rightSpeed + SPEEDERROR);
}

void rotate(uint8_t rotationAngle, uint8_t direction){
    uint8_t x;
    
    switch (direction){
        case 0: //IZQUIERDA
            move(10000,10000,BACKWARD,FORWARD); //velocidad debe de ser 40%
        break;
        case 1: //DERECHA
            move(10000,10000,FORWARD,BACKWARD);
        break;
    }

    x = (((rotationAngle) * 345) / 360) / 3; //cambiar entre 1 y 10 en caso de que no rote bien 
    
    //345 = diametro circunf auto * 3,14
    //5 = cm de movimiento del auto

    if(countRightValue >= x){ //countLeftValue = valor del horquilla
        move(NOSPEED,NOSPEED,STOP,STOP);
    }
}

//MODOS
void lineFollower(){
    static uint8_t irValue = 0;
    static uint8_t lastIrValue;
    bool updValue = false;

    if((myTimer.read_ms()-timeFollowLine)>=INTERVALO){
        timeFollowLine=myTimer.read_ms();
        
        //comprobamos el valor de los sensores
        for(int i=0; i<3;i++){
            if(irSensor[i].currentValue<blackValue){ //si el color es blanco, color > 9000
                irValue = irValue | (irMask << i);
            }
        }

        //actualizamos si hay un valor que es negro 
        if(irValue != 0){
            updValue = true;
            irSensorValue = 0; //acemos 0 el valor para que no se solapen valores de la or bitwise
        } else{
            updValue = false;
        }
        
        for(int i=0; i<3;i++){
            if((irSensor[i].currentValue<blackValue)&&(updValue)){ //si el color es negro y el valor no es 0 actualizamos
                irSensorValue = irSensorValue | (irMask << i);
            } 
        }
        //esperar un tiempo de 500ms o 100ms luego de encontrar la linea para empezar a analizar si encuentro la linea 

        switch(irSensorValue){
            case 1: //sensor de la izquierda
                move(MINSPEED, MEDSPEED, BACKWARD, FORWARD); //aumentamos la velocidad del motor izquierdo
            case 3: 
                move(MINSPEED, MINSPEED, BACKWARD, FORWARD); //aumentamos la velocidad del motor izquierdo
            break;
            case 2: //adelante
                move(MEDSPEED, MEDSPEED, FORWARD, FORWARD);
            break;
            case 4: //sensor de la derecha
                move(MEDSPEED, MINSPEED, FORWARD, BACKWARD);
            case 6: 
                move(MINSPEED, MINSPEED, FORWARD, BACKWARD);
            break;
            case 7:
                move(MEDSPEED, MEDSPEED, FORWARD, FORWARD);
            break;
            default:
                if(lastIrValue == 1 || lastIrValue == 3)
                    move(TURNSPEED, TURNSPEED, BACKWARD, FORWARD);
                else
                    move(TURNSPEED, TURNSPEED, FORWARD, BACKWARD);
            break;
        }

        if((lastIrValue != irSensorValue) && (irSensorValue!=0))
            lastIrValue = irSensorValue;
    }
}

void dodgeObstacle(){
    static int32_t waitTime = myTimer.read_ms();
    //variables del control de distancia deben de ser con signo para que los calculos realizados sean correctos 
    int32_t distance = distanceValue/58; //pasamos la distancia a cm
    static int8_t cont= 0;
    static int8_t irValue = 0;
    static bool rotDir = DERECHA;
    static int8_t Dr = 15; //velocidad referecia

    //DERECHA = 0 - IZQUIERDA = 1
    switch(dodgeModes){
        case FOLLOWLINE:

            if(distance <= DODGEDISTANCE){
                dodgeModes = ROTAR;
                cont = 0;
                move(NOSPEED, NOSPEED, STOP, STOP);
                waitTime = myTimer.read_ms();

                if(rotDir == DERECHA){
                    servo.pulsewidth_us(maxMsServo); //colocamos el servo en la maxima posicion
                } else if (rotDir ==IZQUIERDA){
                    servo.pulsewidth_us(minMsServo);
                } 
            } else{
                lineFollower();

                dodgeModes = FOLLOWLINE;
            }        
        break;
        case ROTAR:
            if((myTimer.read_ms() - waitTime) > WAIT100MS){
                move(NOSPEED, NOSPEED, STOP, STOP);
                if((myTimer.read_ms() - waitTime) > WAIT500MS){
                    waitTime = myTimer.read_ms();
                    cont++;
                }
            } else{
                if(rotDir == DERECHA)
                    move(MEDSPEED,MEDSPEED,FORWARD,BACKWARD); // roto hacia la derecha
                else if( rotDir == IZQUIERDA)   
                    move(MEDSPEED,MEDSPEED,BACKWARD,FORWARD);
            }

            if(cont == 6){ //VALORES 8 O 3
                dodgeModes = ESPERAR;
                waitTime = myTimer.read_ms();
                move(NOSPEED,NOSPEED,STOP,STOP);
            }
        break;
        case ESPERAR:
            if((myTimer.read_ms() - waitTime) - 10000){
                dodgeModes = MOVER;
            }
        break;
        case MOVER:
            //FORMULA DE 1,5 VECES MINSPEED*30)/20
            //MUEVO MIRANDO LA DISTANCIA A LA PARED Y TAMBIEN SI ENCUENTRO LA LINEA

            int32_t rWheelSpeed;
            int32_t lWheelSpeed;

            //calculamos el valor en porcentaje

            if(distance < 40){

                switch(rotDir){
                    case DERECHA:
                        rWheelSpeed = 20 - ((Dr - distance)*2);//distanceCoef;
                    break;
                    case IZQUIERDA:
                        rWheelSpeed = 20 + ((Dr - distance)*2);//distanceCoef;
                    break;
                }

                if(rWheelSpeed < 20) //VALORES EN %
                    rWheelSpeed = 20;
                if(rWheelSpeed > 50)
                    rWheelSpeed = 50;

                switch(rotDir){
                    case DERECHA:
                        lWheelSpeed = 20 + ((Dr - distance)*2);//distanceCoef;
                    break;
                    case IZQUIERDA:
                        lWheelSpeed = 20 - ((Dr - distance)*2);//distanceCoef;
                    break;
                }

                if(lWheelSpeed < 15)
                    lWheelSpeed = 15;
                if(lWheelSpeed > 50)
                    lWheelSpeed = 50;

            } else{ //DISTANCIA MAYOR 40
                if(rotDir == DERECHA){
                    rWheelSpeed = 60;
                    lWheelSpeed = 20;
                } else{
                    rWheelSpeed = 20;
                    lWheelSpeed = 60;
                }
            }

            //calculamos el valor en pulsos
            /*
            25000 ----- 100
            x-----------rWheelSpeed
            */
            //pasamos el porcentaje a pulsos

            rWheelSpeed *= 250;
            lWheelSpeed *= 250;

            move(lWheelSpeed, rWheelSpeed, FORWARD, FORWARD);

            waitTime = myTimer.read_ms();
            dodgeModes = FINDLINE;
            irValue = 0;
        break;
        case FINDLINE:
            for(int i=0; i<3;i++){
                if(irSensor[i].currentValue<blackValue){ //si el color es blanco, color > 9000
                    irValue = irValue | (irMask << i);
                }
            }

            dodgeModes = MOVER;

            if((irValue == 1) || (irValue == 2) || (irValue == 4)){
                dodgeModes = FOLLOWLINE;

                switch(rotDir){
                    case DERECHA:
                        move(MAXSPEED,NOSPEED,FORWARD,STOP);
                    break;
                    case IZQUIERDA:
                        move(NOSPEED,MAXSPEED,STOP,FORWARD);
                    break;
                }

                (rotDir == DERECHA)? (rotDir = IZQUIERDA) : (rotDir = DERECHA);
                servo.pulsewidth_us(MIDDLESERVO);
            }
        break;
    }
}

void maintainDistance(){
    static int32_t savedDistance = 0, savedAngle = 0; 
    static uint32_t servoAngle = minMsServo;
    static uint32_t lookTime; //utilizada para mover el servo actualizando los grados lentamente 
    static uint32_t distanceTime; //utilizado para comprobar que la distancia obtenida es la correcta
    static uint32_t rotateTime;
    static bool isDistance = 0; //utilizada para saber si tuve que leer valor moviendo servo
    int32_t distance = distanceValue/58; //convertimos a cm los datos

    switch(followModes){
        case LOOK:
        
            if(distance<savedDistance){
                savedDistance = distance;
                savedAngle = servoAngle;
            }
            if(myTimer.read_ms() - lookTime > LOOKTIME){
                lookTime = myTimer.read_ms();

                if(servoAngle >= maxMsServo){
                    followModes = ROTATE;
                    isDistance = 1;
                    servoAngle = minMsServo;
                    distanceTime = myTimer.read_ms();                    
                    rotateTime = myTimer.read_ms(); //actualizamos el tiempo aca para que sea mas preciso

                    if(savedAngle > maxMsServo)
                        savedAngle = maxMsServo;
                    //cambiamos los ms a grados
                    savedAngle = ((((savedAngle-miServo.X1) * (miServo.Y2 - miServo.Y1))/(miServo.X2 - miServo.X1)) + miServo.Y1); //convertimos el valor obtenido en pulsos a grados

                    servo.pulsewidth_us(MIDDLESERVO);

                }

                //aumentamos para comprobar antes
                servoAngle += 375;
                servo.pulsewidth_us(servoAngle);
            }

        break;
        case ROTATE:

            if(savedAngle<0){
                //savedAngle *= -1;
                rotate(savedAngle,1); //roto a derecha
            }else if (savedAngle>0){
                rotate(savedAngle,0); //roto a izquierda
            } 
            
            servo.pulsewidth_us(MIDDLESERVO);

            if((myTimer.read_ms() - rotateTime) > 150){ //tiempo de rotacion
                move(NOSPEED,NOSPEED,STOP,STOP);
                servo.pulsewidth_us(MIDDLESERVO);
                if(myTimer.read_ms() - rotateTime > 500){ //esperamos 500 ms de espera para obtener una distancia en caso de rotar
                    rotateTime = myTimer.read_ms();
                    followModes = MOVE;
                }
            }

        break;
        case MOVE:
            //esperamos para realizar una medicion previa en caso de haber tenido que mover el auto para encontrar objeto
            if(distance>11 && distance<50 && (myTimer.read_ms() - distanceTime)>200){
                distanceTime = myTimer.read_ms();
                if(isDistance && (distance > savedDistance-5) && (distance < savedDistance+5)){ //comprobamos si es el objeto previamente encontrado mediante isDistance
                    isDistance = 0;
                    move(MINSPEED,MINSPEED,FORWARD,FORWARD);
                } else if (isDistance && ((distance < savedDistance-5) || (distance > savedDistance+5))){ //si la medida realizada no es 
                    isDistance = 0;
                    servoAngle = minMsServo;
                    followModes = LOOK;
                    lookTime = myTimer.read_ms(); 
                    move(NOSPEED,NOSPEED,STOP,STOP);
                    servo.pulsewidth_us(minMsServo);
                    savedDistance = distance;
                    savedAngle = minMsServo;
                } else { //caso de entrada de inicio
                    if(distance > 25)
                        move(MEDSPEED,MEDSPEED,FORWARD,FORWARD);
                    else
                        move(MINSPEED,MINSPEED,FORWARD,FORWARD);
                }        
            } else if(distance<9){
                move(MINSPEED,MINSPEED,BACKWARD,BACKWARD);
            } else if(distance<=11 && distance>=9){
                move(NOSPEED,NOSPEED,STOP,STOP);
            } else if(distance>=50){
                followModes = LOOK;
                lookTime = myTimer.read_ms(); //simpre actualizar las variables de tiempo antes de entrar al modo para mayor precision
                move(NOSPEED,NOSPEED,STOP,STOP);
                servo.pulsewidth_us(minMsServo); //llevamos el servo al inicio
                savedDistance = distance; //guardamos las distancia inicial
                servoAngle = minMsServo; //guardamos el angulo inicial
                savedAngle = ((((savedAngle-miServo.X1) * (miServo.Y2 - miServo.Y1))/(miServo.X2 - miServo.X1)) + miServo.Y1) - 180; //convertimos el valor de ms a grados
            }
        break;
    }
}

/* END Function prototypes user code ------------------------------------------*/

int main()
{
    //INICIALIZAMOS VARIABLES
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

    //DECLARAMOS VARIABLES
    //uint16_t    mask = 0x000A;
    _delay_t    heartBeatTime;
    _delay_t    debounceTime;
    _delay_t    servoTime;
    _delay_t    aliveAutoTime;

    //int32_t    triggerTime;
    //VALORES DEL SERVO
    miServo.X2=maxMsServo;
    miServo.X1=minMsServo;
    miServo.Y2=90;
    miServo.Y1=-90;
    miServo.intervalValue=MIDDLESERVO;
    //FIN VALORES SERVO

    miServo.currentValue=MIDDLESERVO;
    carModes = IDLE;
    followModes = MOVE;
    dodgeModes = FOLLOWLINE;
    servo.pulsewidth_us(miServo.currentValue);

/* END Local variables -------------------------------------------------------*/


/* User code -----------------------------------------------------------------*/
    PC.baud(115200);
    myTimer.start();
    distanceTimer.start();
    speedMLeft.period_ms(25);
    speedMRight.period_ms(25);
    servo.period_ms(20);
    trigger.write(false);

    //INICIO DE ATTACH INSTRUCCIONES
    PC.attach(&onRxData, SerialBase::IrqType::RxIrq);

    timerGral.attach_us(&do100ms, 100000); 

    //mido flanco ascendente
    speedLeft.rise(&speedCountLeft);

    speedRight.rise(&speedCountRight);

    //mido flanco descendente
    speedLeft.fall(&speedCountLeft);

    speedRight.fall(&speedCountRight);

    hecho.rise(&distanceInitMeasurement);

    hecho.fall(&distanceMeasurement);

    //FIN DE LOS ATTACH INSTRUCCIONES

    //CONFIGURAMOS TEMPORIZADORES
    delayConfig(&heartBeatTime, HEARBEATIME);
    delayConfig(&debounceTime, DEBOUNCE);
    delayConfig(&generalTime,GENERALTIME);
    //delayConfig(&medicionTime,DISTANCEINTERVAL);
    delayConfig(&servoTime,miServo.intervalValue);
    delayConfig(&aliveAutoTime, ALIVEAUTOINTERVAL);

    //INICIALIZAMOS BOTONES
    startButton(&myButton[0], buttonTask);
    
    //CONEXION WIFI
    myWifi.initTask();
    autoConnectWifi();

    //MOVIMIENTOS PREVIOS
    //SERVO A LOS LADOS
    servo.pulsewidth_us(maxMsServo);
    wait_ms(SERVOTIME);
    servo.pulsewidth_us(minMsServo);
    wait_ms(SERVOTIME);
    //SERVO AL CENTRO
    servo.pulsewidth_us(MIDDLESERVO);
    wait_ms(SERVOTIME);

    //MOVEMOS LAS RUEDAS
    move(MINSPEED, MINSPEED, FORWARD, FORWARD);
    wait_ms(MOTORTIME);
    move(MINSPEED, MINSPEED, BACKWARD, BACKWARD);
    wait_ms(MOTORTIME);
    move(NOSPEED, NOSPEED, STOP, STOP);

    while(1){
        //CONEXIONES SERIAL Y WIFI
        myWifi.taskWifi();
        serialTask((_sRx *)&dataRx,&dataTx, SERIE); //serialTask -> conexion serial
        serialTask(&wifiRx,&wifiTx, WIFI); //serialTask -> conexion wifi

        //MEDICIONES DE SENSORES
        speedTask(); //medicion de horquillas
        irSensorsTask();
        servoTask(&servoTime,&miServo.intervalValue);
        aliveAutoTask(&aliveAutoTime);

        //ACTUALIZAMOS EL ANGULO DEL SERVO
        miServo.X2=maxMsServo;
        miServo.X1=minMsServo;

        //ACTUALIZAMOS EL ESTADO DE LOS PULSADORES
        if((myTimer.read_ms()-timeToDebounce)>DEBOUNCE){
            timeToDebounce=myTimer.read_ms();

            for(int i=0; i<NUMBUTTONS; i++){
                if(BUTTON.read())
                    myButton[i].stateInput = PRESSED;
                else
                    myButton[i].stateInput = NOT_PRESSED;
            }
        }

        //MODOS DEL AUTO
        switch(carModes){
            case IDLE:
                if(updateMefTask(myButton) && (myButton[0].timeDiff >= 100) && (myButton[0].timeDiff < 1000)){
                    carModes = PREMODE1;       
                    heartBeatIndex = PREMODE1;             
                }
            break;
            case PREMODE1:
                if(updateMefTask(myButton)){
                    if((myButton[0].timeDiff >= 100) && (myButton[0].timeDiff < 1000)){ //cambia de modo
                        carModes = PREMODE2;
                        heartBeatIndex = PREMODE2;  
                        move(NOSPEED, NOSPEED, STOP, STOP);
                    } else if((myButton[0].timeDiff < 3000)){ //reinicio de modo
                        heartBeatIndex=PREMODE1;
                    }
                }
                if((myTimer.read_ms() - myButton[0].timePressed >= 1000) && (myTimer.read_ms() - myButton[0].timePressed <= 3000)){ //ejecucion de modo
                    heartBeatIndex = EXEMODE1;
                    if(updateMefTask(myButton))
                        carModes = ONMODE1;
                }
                if(myTimer.read_ms() - myButton[0].timePressed > 3000)
                    heartBeatIndex = PREMODE1;
            break;
            case EXEMODE1:
            break;
            case ONMODE1:
                if(updateMefTask(myButton) && myButton[0].timeDiff >= 3000){ //salir ejecucion
                    carModes=PREMODE1;
                    heartBeatIndex=PREMODE1;
                    move(NOSPEED, NOSPEED, STOP, STOP); 
                }
                heartBeatIndex = ONMODE1;
                lineFollower();
            break;
            case PREMODE2:
                if(updateMefTask(myButton)){
                    if((myButton[0].timeDiff >= 100) && (myButton[0].timeDiff < 1000)){ //cambia de modo
                        carModes = PREMODE3; 
                        heartBeatIndex = PREMODE3; 
                        move(NOSPEED, NOSPEED, STOP, STOP);
                    } else if((myButton[0].timeDiff < 3000)){ //reinicio de modo
                        heartBeatIndex = PREMODE2;
                    }
                }
                if((myTimer.read_ms() - myButton[0].timePressed >= 1000) && (myTimer.read_ms() - myButton[0].timePressed <= 3000)){ //ejecucion de modo
                    heartBeatIndex = EXEMODE2;
                    if(updateMefTask(myButton)){
                        carModes = ONMODE2;
                    }
                }
                if(myTimer.read_ms() - myButton[0].timePressed > 3000)
                    heartBeatIndex = PREMODE2;
            break;
            case EXEMODE2:
            break;
            case ONMODE2:
                if(updateMefTask(myButton) && (myButton[0].timeDiff >= 3000)){ //salir ejecucion
                    carModes = PREMODE2;
                    heartBeatIndex = PREMODE2;
                    move(NOSPEED, NOSPEED, STOP, STOP);
                    
                }
                heartBeatIndex = ONMODE2;
                dodgeObstacle();
            break;
            case PREMODE3:
                if(updateMefTask(myButton)){
                    if((myButton[0].timeDiff >= 100) && (myButton[0].timeDiff < 1000)){ //cambia de modo
                        carModes = IDLE;
                        heartBeatIndex = IDLE; 
                        move(NOSPEED, NOSPEED, STOP, STOP);
                    } else if((myButton[0].timeDiff < 3000)){ //reinicio de modo
                        heartBeatIndex=PREMODE3;
                    }
                }
                if((myTimer.read_ms() - myButton[0].timePressed >= 1000) && (myTimer.read_ms() - myButton[0].timePressed <= 3000)){ //ejecucion de modo
                    heartBeatIndex = EXEMODE3;
                    if(updateMefTask(myButton))
                        carModes = ONMODE3;
                }
                if(myTimer.read_ms() - myButton[0].timePressed > 3000)
                    heartBeatIndex = PREMODE3;
            break;
            case EXEMODE3:
            break;
            case ONMODE3:
                if(updateMefTask(myButton) && (myButton[0].timeDiff >= 3000)){ //salir ejecucion
                    carModes=PREMODE3;
                    heartBeatIndex=PREMODE3;
                    move(NOSPEED, NOSPEED, STOP, STOP);
                }
                heartBeatIndex = ONMODE3;
                maintainDistance();
            break;
        }
        //HEARTBEAT 
        hearbeatTask(&heartBeatTime, heartBeatIndex); //ejecutamos la secuencia del heartbeat   
    
        //maintainDistance();

        dodgeObstacle();

        //move(MINSPEED, MINSPEED, FORWARD, FORWARD);

        //lineFollower();

        //rotate(45,1);

    }
/* END User code -------------------------------------------------------------*/
}








