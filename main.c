
//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
//  gráfico (GUI) Qt.
//  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
//  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementara la recepcion de un mensaje
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (ROM_)
#include "driverlib/rom_map.h"   // TIVA: Para usar la opción MAP en las funciones API (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "utils/uartstdioMod.h"  // TIVA: Funciones API UARTSTDIO (printf)
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADC
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "utils/cpu_usage.h"
#include "commands.h"
#include <serial2USBprotocol.h>
#include <usb_dev_serial.h>
#include "usb_messages_table.h"
#include "config.h"
#include "math.h"
#include "event_groups.h"
#include "queue.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"

// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
SemaphoreHandle_t mutexUSB, mutexUART; // Para proteccion del canal USB y el caal UART -terminal-, ya que ahora lo van a usar varias tareas distintas
SemaphoreHandle_t semProducto;
QueueHandle_t colaKits;
EventGroupHandle_t eventGroup;
static QueueHandle_t cola1;
static QueueHandle_t cola2;
static QueueSetHandle_t grupo_colas;
static QueueHandle_t mailbox_freertos;

#define START_BIT (1 << 0)
#define POSICIONES_COLA 5

// ANTONIOOOO
uint32_t resistencia;

uint32_t ui32ADC0Value[2];
volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;
uint32_t envio_mail[2];

void PWM_Timer_Init(void);
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ulLine)
{
    while (1) // Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    {         // Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while (1)
    {
    }
}

void vApplicationTickHook(void)
{
    static uint8_t ui8Count = 0;

    if (++ui8Count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        ui8Count = 0;
    }
    // return;
}
void vApplicationIdleHook(void)
{
    SysCtlSleep();
}

void vApplicationMallocFailedHook(void)
{
    while (1)
        ;
}

//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************
static portTASK_FUNCTION(ProductorTask, pvParameters)
{
    PARAM_TAREA *params = (PARAM_TAREA *)pvParameters;
    PARAM_MENSAJE_PRODUCTO msg;
    xEventGroupWaitBits(eventGroup,
                        START_BIT,
                        pdFALSE,
                        pdFALSE,
                        portMAX_DELAY);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(params->periodo_ms));
        // xSemaphoreGive(semProducto);
        msg.kit_id = (rand() % 100) + 1;
        msg.IDProd = params->IDProd;
        // xQueueSend(colaKits, &msg, portMAX_DELAY);

        /*if (xQueueSend(colaKits, &msg, 0) != pdPASS)
         {
             GPIOPinWrite(GPIO_PORTF_BASE, params->ledPin, params->ledPin);
             vTaskDelete(NULL);
         }
         else
         {
             GPIOPinWrite(GPIO_PORTF_BASE, params->ledPin, 0);
         }*/
        if (params->IDProd == 1)
        {

            vTaskDelay(pdMS_TO_TICKS(2000));
            if (xQueueSend(cola1, &msg, 0) != pdPASS)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, params->ledPin, params->ledPin);
            }
            else
            {
                GPIOPinWrite(GPIO_PORTF_BASE, params->ledPin, 0);
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (xQueueSend(cola2, &msg, 0) != pdPASS)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, params->ledPin, params->ledPin);
            }
            else
            {
                GPIOPinWrite(GPIO_PORTF_BASE, params->ledPin, 0);
            }
        }
        xSemaphoreTake(mutexUART, portMAX_DELAY);
        UARTprintf("Prod %d -> kit %d\n", msg.IDProd, msg.kit_id);
        xSemaphoreGive(mutexUART);
    }
}
static portTASK_FUNCTION(ConsumidorTask, pvParameters)
{
    (void)pvParameters;
    uint32_t contadorProductos = 0;
    PARAM_MENSAJE_PRODUCTO param;
    uint8_t frame[MAX_FRAME_SIZE];
    int32_t size;
    uint32_t kit_id;

    xEventGroupWaitBits(eventGroup,
                        START_BIT,
                        pdFALSE,
                        pdFALSE,
                        portMAX_DELAY);

    for (;;)
    {
        QueueSetMemberHandle_t cola_activa;

        cola_activa = xQueueSelectFromSet(grupo_colas, portMAX_DELAY);

        if (cola_activa == cola1)
        {
            xQueueReceive(cola1, &param, 0);
        }
        else if (cola_activa == cola2)
        {
            xQueueReceive(cola2, &param, 0);
        }

        kit_id = param.kit_id;

        vTaskDelay(pdMS_TO_TICKS(3000));

        contadorProductos++;
        param.totalProductos = contadorProductos;

        xSemaphoreTake(mutexUART, portMAX_DELAY);
        UARTprintf("Producto ensamblado. Prod=%d Kit ID=%d Total=%d\r\n",
                   param.IDProd, kit_id, contadorProductos);
        xSemaphoreGive(mutexUART);

        size = create_frame(frame, MENSAJE_PRODUCTO, &param, sizeof(param), MAX_FRAME_SIZE);

        if (size > 0)
        {
            xSemaphoreTake(mutexUSB, portMAX_DELAY);
            send_frame(frame, size);
            xSemaphoreGive(mutexUSB);
        }
    }
}

//*****************************************************************************
//
// Codigo de tarea que procesa los mensajes recibidos a traves del canal USB
//
//*****************************************************************************
static portTASK_FUNCTION(USBMessageProcessingTask, pvParameters)
{

    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int32_t i32Numdatos;
    uint8_t ui8Message;
    void *ptrtoreceivedparam;
    uint32_t ui32Errors = 0;

    /* The parameters are not used. */
    (void)pvParameters;

    //
    // Mensaje de bienvenida inicial.
    //
    xSemaphoreTake(mutexUART, portMAX_DELAY);
    UARTprintf("\n\nBienvenido a la aplicacion Fabrica Automatizada (curso 2025/26)!\n");
    UARTprintf("\nAutores: XXXXXX y XXXXX ");
    xSemaphoreGive(mutexUART);

    for (;;)
    {
        // Espera hasta que se reciba una trama con datos serializados por el interfaz USB
        i32Numdatos = receive_frame(pui8Frame, MAX_FRAME_SIZE); // Esta funcion es bloqueante
        if (i32Numdatos > 0)
        {                                                                     // Si no hay error, proceso la trama que ha llegado.
            i32Numdatos = destuff_and_check_checksum(pui8Frame, i32Numdatos); // Primero, "destuffing" y comprobación checksum
            if (i32Numdatos < 0)
            {
                // Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                ui32Errors++;
                // Procesamiento del error
            }
            else
            {                                                                                         // El paquete esta bien, luego procedo a tratarlo.
                ui8Message = decode_message_type(pui8Frame);                                          // Obtiene el valor del campo mensaje
                i32Numdatos = get_message_param_pointer(pui8Frame, i32Numdatos, &ptrtoreceivedparam); // Obtiene un puntero al campo de parametros y su tamanio.
                switch (ui8Message)
                {
                case MENSAJE_PING:
                    // A un mensaje de ping se responde con el propio mensaje
                    i32Numdatos = create_frame(pui8Frame, ui8Message, 0, 0, MAX_FRAME_SIZE);
                    if (i32Numdatos >= 0)
                    {
                        xSemaphoreTake(mutexUSB, portMAX_DELAY);
                        send_frame(pui8Frame, i32Numdatos);
                        xSemaphoreGive(mutexUSB);
                    }
                    else
                    {
                        // Error de creacion de trama: determinar el error y abortar operacion
                        ui32Errors++;
                        // Procesamiento del error
                        // Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
                        // tener que copiar y pegar todo en cada operacion de creacion de paquete
                        switch (i32Numdatos)
                        {
                        case PROT_ERROR_NOMEM:
                            // Procesamiento del error NO MEMORY
                            break;
                        case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                            // Procesamiento del error STUFFED_FRAME_TOO_LONG
                            break;
                        case PROT_ERROR_MESSAGE_TOO_LONG:
                            // Procesamiento del error MESSAGE TOO LONG
                            break;
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                            // Procesamiento del error INCORRECT PARAM SIZE
                            break;
                        }
                    }
                    break;
                case MENSAJE_START_BIT:
                {
                    UARTprintf("START recibido -> arrancando sistema\r\n");
                    xEventGroupSetBits(eventGroup, START_BIT);
                    // PWM_Timer_Init();
                    //  ==========================================================
                    // IMPORTANTE:
                    // El sistema NO genera interrupciones del ADC hasta que Qt
                    // envía la orden de START.
                    //
                    // Esto evita que el Timer2 dispare el ADC antes de que el
                    // sistema esté completamente inicializado.
                    // ==========================================================

                    TimerEnable(TIMER2_BASE, TIMER_A);
                    break;
                }
                default:
                {
                    PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                    parametro.message = ui8Message;
                    // El mensaje esta bien pero no esta implementado
                    i32Numdatos = create_frame(pui8Frame, MENSAJE_NO_IMPLEMENTADO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
                    if (i32Numdatos >= 0)
                    {
                        xSemaphoreTake(mutexUSB, portMAX_DELAY);
                        send_frame(pui8Frame, i32Numdatos);
                        xSemaphoreGive(mutexUSB);
                    }
                    break;
                }
                } // switch
            }
        }
        else
        { // if (ui32Numdatos >0)
            // Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
            ui32Errors++;
            // Procesamiento del error
        }
    }
}

void PWM_Timer_Init(void)
{
    uint32_t pwmClock;
    uint32_t period;
    uint32_t duty = 50;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
        ;
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
        ;

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    pwmClock = SysCtlClockGet() / 64;

    period = pwmClock / 10;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (period * duty) / 100);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM0);
}
/*
void ADC_Timer_Init(void)
{
    // Activar periféricos
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
        ;
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
        ;
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
        ;

    // PE3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // Timer a 1 Hz
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SystemClock);

    // Trigger ADC desde timer
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    // Configurar ADC (Secuencia 0)
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);

    // Paso 0: sensor temperatura interno
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_TS);

    // Paso 1: potenciómetro (AIN0) + interrupción
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1,
                             ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 0);

    ADCIntClear(ADC0_BASE, 0);
    ADCIntEnable(ADC0_BASE, 0);

    // Interrupciones
    IntPrioritySet(INT_ADC0SS0, 5 << 5);
    IntEnable(INT_ADC0SS0);

    TimerEnable(TIMER0_BASE, TIMER_A);
}
void ADC0SS0_Handler(void)
{
    uint32_t valores[2];
    PARAM_TEMPERATURA temp;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDataGet(ADC0_BASE, 0, valores);

    // Temperatura ambiente (sensor interno)
    temp.temp_ambiente =
        147.5 - ((75.0 * 3.3 * valores[0]) / 4096.0);

    // Temperatura soldadura (potenciómetro)
    temp.temp_soldadura =
        300.0 + (150.0 * valores[1] / 4095.0);

    // Mailbox (sobrescritura)
    xQueueOverwriteFromISR(mailboxTemp,
                           &temp,
                           &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static portTASK_FUNCTION(TemperaturasTask, pvParameters)
{
    PARAM_TEMPERATURA temp;
    uint8_t frame[MAX_FRAME_SIZE];
    int32_t size;

    for (;;)
    {
        if (xQueueReceive(mailboxTemp, &temp, portMAX_DELAY) == pdPASS)
        {
            // DEBUG UART
            xSemaphoreTake(mutexUART, portMAX_DELAY);
            UARTprintf("Temp -> Amb: %.2f  Sold: %.2f\n",
                       temp.temp_ambiente,
                       temp.temp_soldadura);
            xSemaphoreGive(mutexUART);

            // Enviar a Qt
            size = create_frame(frame,
                                MENSAJE_TEMPERATURA,
                                &temp,
                                sizeof(temp),
                                MAX_FRAME_SIZE);

            if (size > 0)
            {
                xSemaphoreTake(mutexUSB, portMAX_DELAY);
                send_frame(frame, size);
                xSemaphoreGive(mutexUSB);
            }


            float brillo = (temp.temp_soldadura - 300.0) / 150.0;

            if (brillo < 0)
                brillo = 0;
            if (brillo > 1)
                brillo = 1;

            uint32_t color[3];

            color[0] = (uint32_t)(brillo * 255); // Rojo
            color[1] = 0;
            color[2] = 0;

            RGBColorSet(color);
            RGBIntensitySet(1.0f);
        }
    }
}*/

// ANTONIOOO
void ADC0IntHandler(void)
{
    // Lee las muestras obtenidas a un array
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    // en value[0] recurro al valor obtenido de la temperatura, en value[1] el del potenciometro
    //  Calculamos la temperatura media como la media de las muestras de los 4 conversores
    ui32TempAvg = ui32ADC0Value[0];
    resistencia = ui32ADC0Value[1]; // recibo un valor entre 0 y 4095 segun como de girado esté girado el tornillo del potenciometro (un extremo indica el maximo y el otro el minimo)
    // Y lo convertimos a grados centigrados y Farenheit, usando la formula indicada en el Data Sheet
    ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096) / 10;
    ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;
    // el led cambia de brillo
    float intensidad;
    intensidad = (float)ui32ADC0Value[1] / 4095.0f;
    RGBIntensitySet(intensidad);
    RGBEnable(); // Habilita la generacion PWM para el encendido de los LEDs
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    // Limpia el flag de interrupcion del ADC1 y el secuenciador 3 -> siempre despues
    // de leer los datos, porque ADCSequenceDataGet puede no hacerse de forma atomica
    ADCIntClear(ADC0_BASE, 1);
    envio_mail[0] = ui32TempValueC;
    // hago la traslacion a temperatura del valor del potenciometro (0 a 4095 escalado a 300-450)
    envio_mail[1] = 300 + (ui32ADC0Value[1] * 150) / 4095;
    // en envio_mail[0] envio la temperatura en grados
    // en envio_mail[1] envio ya el valor que estará en el intervalo 300-450
    xQueueOverwriteFromISR(mailbox_freertos, envio_mail, &higherPriorityTaskWoken);
    // 3. Forzamos el cambio de contexto si es necesario
    // si la tarea temperatura tiene mayor prioridad salta directamente a ella gracias a este llamada,
    // no deja que lo compruebe automaticamente el programa ya que el programa lo hace cada cierto intervalo de tiempo yy hasta que se produzca ese cambio igual le daria
    // tiempo a ejecutarse algo la tarea anterior
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
void Init_Timer_ADC_RGB(void)
{
    uint32_t ui32Period;

    // =========================
    // TIMER2 (utilizado para disparar el ADC)
    // =========================

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);      // Habilita el TIMER2 (utilizado para el ADC)
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2); // Esto es necesario para que el timer siga funcionando en bajo consumo

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2))
        ; // Espera a que el periférico esté listo

    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // Configura el TIMER2 en modo periódico

    ui32Period = SysCtlClockGet() * 1; // 1 segundo (si pusieses *5 serían 5 segundos cada vez que salta el ADC)

    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period - 1); // Se carga la cuenta del timer

    // IMPORTANTE:
    // El timer NO se habilita aquí (TimerEnable) porque se arranca desde Qt con START_BIT
    // Esto permite control externo del inicio del sistema

    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); // Habilita el timer como disparo del ADC

    // =========================
    // ADC0 CONFIGURACIÓN
    // =========================

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);      // Habilita ADC0
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0); // Permite que el ADC funcione en bajo consumo

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
        ; // Espera a que ADC esté listo

    ADCSequenceDisable(ADC0_BASE, 1); // Importante: deshabilitar secuenciador 1 antes de configurar

    // Configuración del secuenciador:
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); // ADC disparado por TIMER

    // =========================
    // Configuración de los pasos del secuenciador
    // =========================

    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
    // Paso 0: sensor de temperatura interno (ADC_CTL_TS)

    ADCSequenceStepConfigure(ADC0_BASE, 1, 1,
                             ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    // Paso 1: canal 0 (potenciómetro en PE3)
    // ADC_CTL_IE -> genera interrupción
    // ADC_CTL_END -> último paso de la secuencia

    // =========================
    // PIN DEL POTENCIÓMETRO
    // =========================

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Habilita GPIOE

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
        ; // Espera a que esté listo

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // PE3 configurado como entrada ADC

    // =========================
    // CONFIGURACIÓN ADC AVANZADA
    // =========================

    ADCHardwareOversampleConfigure(ADC0_BASE, 64); // Sobremuestreo para mejorar estabilidad

    ADCSequenceEnable(ADC0_BASE, 1); // Habilita secuenciador 1

    ADCIntClear(ADC0_BASE, 1); // Limpia flags de interrupción

    // =========================
    // INTERRUPCIONES ADC
    // =========================

    IntPrioritySet(INT_ADC0SS1, configMAX_SYSCALL_INTERRUPT_PRIORITY); // Prioridad compatible con FreeRTOS

    IntEnable(INT_ADC0SS1); // Habilita interrupción del ADC secuenciador 1

    ADCIntEnable(ADC0_BASE, 1); // Activa interrupciones del ADC

    // =========================
    // CONFIGURACIÓN LED RGB
    // =========================

    uint32_t ui32Color[3];

    ui32Color[RED] = 0x7FFF;   // Rojo: valor medio
    ui32Color[BLUE] = 0x7FFF;  // Azul: valor medio
    ui32Color[GREEN] = 0x0000; // Verde apagado

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0); // RGB usa TIMER0
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1); // RGB usa TIMER1

    RGBInit(0);             // Inicializa el sistema de control de LEDs
    RGBColorSet(ui32Color); // Aplica el color inicial
}
//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************

int main(void)
{

    // Reloj del sistema definido a 40MHz
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Obtiene el reloj del sistema
    g_ui32SystemClock = SysCtlClockGet();

    // Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
    //                                                                         deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ / 10, 3);

    /**                                              Creacion de tareas                                     **/
    // Inicializa el sistema de depuración e interprete de comandos por terminal UART
    if (initCommandLine(256, tskIDLE_PRIORITY + 1) != pdPASS)
    {
        while (1)
            ;
    }

    USBSerialInit(32, 32); // Inicializo el  sistema USB
    //
    // Crea la tarea que gestiona los mensajes USB (definidos en USBMessageProcessingTask)
    //
    if (xTaskCreate(USBMessageProcessingTask, "usbser", 512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
    {
        while (1)
            ;
    }

    //
    // A partir de aqui se crean las tareas de usuario, y los recursos IPC que se vayan a necesitar
    //

    mutexUART = xSemaphoreCreateMutex();
    if (NULL == mutexUART)
        while (1)
            ;

    mutexUSB = xSemaphoreCreateMutex();
    if (NULL == mutexUSB)
        while (1)
            ;

    // Crear semáforo binario
    /*semProducto = xSemaphoreCreateBinary();
    if (semProducto == NULL)
        while (1)
            ;*/

    eventGroup = xEventGroupCreate();
    if (eventGroup == NULL)
        while (1)
            ;

    //  colaKits = xQueueCreate(3, sizeof(PARAM_MENSAJE_PRODUCTO));

    cola1 = xQueueCreate(POSICIONES_COLA, sizeof(PARAM_MENSAJE_PRODUCTO));
    if (NULL == cola1)
        while (1)
            ;

    cola2 = xQueueCreate(POSICIONES_COLA, sizeof(PARAM_MENSAJE_PRODUCTO));
    if (NULL == cola2)
        while (1)
            ;

    grupo_colas = xQueueCreateSet(POSICIONES_COLA * 2);
    if (NULL == grupo_colas)
        while (1)
            ;
    if (xQueueAddToSet(cola1, grupo_colas) != pdPASS)
    {
        while (1)
            ;
    }
    if (xQueueAddToSet(cola2, grupo_colas) != pdPASS)
    {
        while (1)
            ;
    }
    /* if (colaKits == NULL)
     {
         while (1)
             ;
     }*/
    // Crear mailbox
    mailbox_freertos = xQueueCreate(1, sizeof(PARAM_TEMPERATURA));
    if (mailbox_freertos == NULL)
        while (1)
            ;
    /*
          // Crear tarea
          xTaskCreate(TemperaturasTask,
                      "temp",
                      512,
                      NULL,
                      tskIDLE_PRIORITY + 1,
                      NULL);

          // Inicializar RGB
          RGBInit(1);
          RGBEnable();


          ADC_Timer_Init();*/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
        ;
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    //
    static PARAM_TAREA prod1 = {1, 1000, GPIO_PIN_1}; // rojo
    static PARAM_TAREA prod2 = {2, 3000, GPIO_PIN_3}; // verde
    if (xTaskCreate(ProductorTask, "prod1", 512, &prod1, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        while (1)
            ;
    }
    if (xTaskCreate(ProductorTask, "prod2", 512, &prod2, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        while (1)
            ;
    }

    if (xTaskCreate(ConsumidorTask, "cons", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        while (1)
            ;
    }

    // parte led
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    //  ;
    // grupo de colas

    //
    // Pone en marcha el planificador. La llamada NO tiene retorno
    //

    Init_Timer_ADC_RGB();

    vTaskStartScheduler(); // el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

    // De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
    while (1)
    {
        // Si llego aqui es que algo raro ha pasado
    }
}
