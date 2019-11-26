/**
 * carro.cpp       v0.0        08-10-2019 
 *
 * Orientador: Elias Teodoro da Silva Junior
 * Autores: Felipe Moura de Castro e Joao Bruno Costa Cruz,
 * Instituto Federal de Educação, Ciência e Tecnologia do Ceará (IFCE) - Campus Fortaleza
 *
 * @Opensource
 * Este código-fonte pode ser utilizado, copiado, estudado, modificado e redistribuído sem restrições.
 *
 * Copyright (c) 2019, Felipe Moura de Castro e Joao Bruno Costa Cruz.
 *
 */

/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 /**
  *----------------------------------------------------------------------------------------------------------------------
  * Este software foi desenvolvido com o objetivo de executá-lo em uma plataforma com processador
  * ARM-Corte-M, a placa deve ter aproximadamente 20kbytes de memória RAM e aproximadamente 15kbytes
  * de memória FLASH. Mais precisamente, usamos uma placa da STMicroeletronics (NUCLEO_F411RE | NUCLEO_F103).
  * 
  * Este código implementa a ideia de monitoramento de um carro via Rede LoRa (LPWAN - Low Power Wide Area).
  * O monitoramento é feito com informções de periféricos não incluídos no carro, junto de periféricos
  * incluídos no carro (Estes são obtidos via Rede CAN).
  *
  * Após a obtenção desses dados, os mesmo são gravados no cartão a cada 1 segundo (O tempo de gravação pode ser menor)
  * e enviados pela rede a cada 1 minuto. Essa restrição do envio de dados pela rede se dá pelo uso da 
  * tecnologia LoRa, que tem como objetivo ser uma rede para dispositivos de baixo consumo energético.
  *
  * Os periférico utilizados no projeto são:
  *         - MPU6050 [I2C]
  *         - DS1307 RTC (Removido temporariamente - Sua função foi substituída pelo GPS)
  *         - Módulo Cartão SD Card micro
  *         - Antena LoRa: SEMTECH - SX1272MB2DAS Shield
  *         - Modulo GPS: NEO-6M-0-001
  *
  * Para utilizar este software, deve-se utilizar o Mbed OS 5. O programa não serve para ser usado em uma placa
  * que suporta apenas Mbes OS 2.
  *
  * Seria interessante utilizar a IDE Mbed OS Studio, uma vez que ela lhe permite acessar todas as bibliotecas do programa.
  * O compilador online permite o acesso de apenas algumas, logo pode ser que por lá, não seja possível a edição de 
  * algumas variáveis as quais se deseja editar.
  * 
  * A ideia final para o projeto, seria poder utilizar os dados obtidos para definir a qualidade da malha asfáltica.
  *
  * Obs.: Algumas configurações devem ser feitas em outros locais, como por exemplo: "mbed_app.json"
  * No nosso caso, as seguintes configurações foram incluídas:
  *         ..."config": { "lora-radio": { ... "value": "SX1276" }...
  *
  *         ..."target_overrides": { "*": { ... "lora.phy": "AU915", ... "target.features_add": ["STORAGE"], "target.components_add": ["SD"]...
  *
  *          ..."target_overrides": { ...  "NUCLEO_F411RE": {
  *                                         "lora-radio":          "SX1272",
  *                                         "lora-spi-mosi":       "D11",
  *                                         "lora-spi-miso":       "D12",
  *                                         "lora-spi-sclk":       "D13",
  *                                         "lora-cs":             "D10",
  *                                         "lora-reset":          "A0",
  *                                         "lora-dio0":           "D2",
  *                                         "lora-dio1":           "D3",
  *                                         "lora-dio2":           "D4",
  *                                         "lora-dio3":           "D5",
  *                                         "lora-dio4":           "NC",
  *                                         "lora-dio5":           "NC",
  *                                         "lora-rf-switch-ctl1": "NC",
  *                                         "lora-rf-switch-ctl2": "NC",
  *                                         "lora-txctl":          "NC",
  *                                         "lora-rxctl":          "NC",
  *                                         "lora-ant-switch":     "NC",
  *                                         "lora-pwr-amp-ctl":    "NC",
  *                                         "lora-tcxo":           "NC",
  *                                            
  *                                         "target.features_add": ["STORAGE"],
  *                                         "target.components_add": ["SD"],
  *                                         "sd.SPI_MOSI": "PB_15",
  *                                         "sd.SPI_MISO": "PB_14",
  *                                         "sd.SPI_CLK": "PB_13",
  *                                         "sd.SPI_CS": "PB_12"
  *                                     } ...
  * 
  * Mudanças no pinos de comunicação com os módulos talvez sejam necessarias para a comunicação: USART, I2C, SPI, CAN.
  *----------------------------------------------------------------------------------------------------------------------
  */

/**
 *----------------------------------------------------------------------------------------------------------------------
 * INCLUSÃO DE BIBLIOTECAS E DIRETIVAS 'DEFINE'
 *----------------------------------------------------------------------------------------------------------------------
 */

#include "mbed.h"
#include "mbed_events.h"
#include "mbed_trace.h"
#include "LoRaWANInterface.h"
#include "lora_radio_helper.h"
#include "MPU6050.h"
#include "DS1307.h"
#include "PayloadCarro/payloadCarro.h"
#include "BlockDevice.h"
#include "FATFileSystem.h"
#include <stdio.h>
#include <errno.h>
#include "GPS_Carro/GPS_Carro.h"
#include <string.h>

#define TX_INTERVAL         60000

/*
 *----------------------------------------------------------------------------------------------------------------------
 * VARIÁVEIS GLOBAIS, OBJETOS E PROTÓTIPOS DE FUNÇÕES
 *----------------------------------------------------------------------------------------------------------------------
 */

 Serial pc(USBTX, USBRX);

/**
 * Struct para armazenar os dados do GPS
 * Declaração da Thread GPS
 * Declaração do Semaforo para acessar o GPS (Semaforo utilizado para garantir a atomicidade dos dados)
 */
dataGPS dadosDoGPS;
Thread thread_gps;
Semaphore semaforo_acessar_gps (1);

/**
 * Objeto buffer de envio LoRa
 */
PayLoadCarro payloader;

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Objeto para aquisição do GPS
 *----------------------------------------------------------------------------------------------------------------------
 */
Serial gps (PA_15, PB_7); /* TX, RX */

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Objeto para aquisição de: acelerometro, giroscopio e temperatura
 *----------------------------------------------------------------------------------------------------------------------
 */
MPU6050 ark (PB_9, PB_8);

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Objeto para aquisição de: calendario e relogio
 * 
 * Opções de frequência: (hz == 100000) || (hz == 400000) || (hz == 1000000)
 *
 * Obs: A frequencia escolhida deve ser colocada dividida por 100
 *
 * Ex: 100000 hz -> 1000
 *
 * Ordem da pinagem I2C -> SDA / SDL
 *----------------------------------------------------------------------------------------------------------------------
 */
class myI2C2 : public I2C
{
public:
    myI2C2 (PinName sda, PinName scl, int hz, const char *name = NULL) : I2C (sda, scl) { frequency (hz * 100); };
};
myI2C2 gI2c (PC_9, PA_8, 1000); 
RtcDs1307 gRtc ( gI2c );

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Objeto para gravar no cartão micro SD / Thread de gravação no cartão micro SD
 *----------------------------------------------------------------------------------------------------------------------
 */
Thread thread_cartao;

/** 
 *----------------------------------------------------------------------------------------------------------------------
 * Dispositivo de bloco padrão do sistema
 *----------------------------------------------------------------------------------------------------------------------
 */
BlockDevice *bd = BlockDevice::get_default_instance ();
FATFileSystem fs ("fs");

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Endereços da aplicação na rede (The Things Network)
 *
 * @param LORAWAN_DEV_EUI                    Identificador do dispositivo - 8 octetos
 * @param LORAWAN_APP_EUI                    Identificador da aplicação - 8 octetos
 * @param LORAWAN_APP_KEY                    Idenfificador da chave da aplicação - 16 octetos
 *----------------------------------------------------------------------------------------------------------------------
 */
static uint8_t LORAWAN_DEV_EUI[] = { 0x00, 0x2E, 0x92, 0x89, 0xE3, 0xEB, 0xD0, 0xD1 };
static uint8_t LORAWAN_APP_EUI[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x25, 0xB8 };
static uint8_t LORAWAN_APP_KEY[] = { 0xA3, 0xCB, 0x37, 0x09, 0xB1, 0x69, 0xA9, 0x8D, 0x77, 0xE3, 0x6C, 0x6F, 0xDE, 0x7B, 0x67, 0x31 };
//static uint8_t LORAWAN_DEV_EUI[] = { 0x00, 0x4B, 0x39, 0xDE, 0x54, 0xD9, 0x37, 0x56 };
//static uint8_t LORAWAN_APP_EUI[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x20, 0x53 };
//static uint8_t LORAWAN_APP_KEY[] = { 0xB6, 0xCC, 0xE5, 0xE6, 0x70, 0x24, 0xA8, 0x40, 0xE5, 0x9A, 0x51, 0x19, 0x3F, 0xD1, 0x2B, 0x80 };
//static uint8_t LORAWAN_DEV_EUI[] = { 0x00, 0xED, 0x32, 0xB4, 0xF1, 0xD9, 0x33, 0x05 };
//static uint8_t LORAWAN_APP_EUI[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x58, 0x41 };
//static uint8_t LORAWAN_APP_KEY[] = { 0xEC, 0x90, 0x2B, 0x08, 0xC1, 0x25, 0x40, 0x7A, 0x78, 0xD9, 0xF9, 0x1A, 0xB4, 0x9C, 0x47, 0x0F };


/**
 *----------------------------------------------------------------------------------------------------------------------
 * Maninupaladores de evento LoRa
 *
 * A comunicação LoRa gera eventos de entrada e saída que podem ser tratados
 * de acordo com as necessidades do programador / usuário.
 *
 * No programa, nos preocupamos principalmente com o envio da mensagem, e
 * a confirmação de recebimento da mensagem
 *----------------------------------------------------------------------------------------------------------------------
 */
static EventQueue ev_queue;
static void lora_event_handler (lorawan_event_t event);
static LoRaWANInterface lorawan (radio);
static lorawan_app_callbacks_t callbacks;


//------------------------------------------------------------------------------------------------------------------
//-- Protótipos das funções
//------------------------------------------------------------------------------------------------------------------

/**
 * Controle de gravação no arquivo
 */
int controleDeGravacao (char *novoNomeDeArquivo); 

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Thread para gravar no cartão a cada intervalo de tempo predefinido, no nosso caso
 * esse intervalo de tempo é 1 segundo. 
 *
 * A primeira verificação antes do loop infinito é a verificação da existência do arquivo,
 * e essa verificação é feita com a tentativa de abrir o arquivo para leitura. Se o arquivo
 * existir, nada é efeito; caso contrário o arquivo é inicializado na primeira linha com
 * uma string que é definida de acordo com os dados que serão gravados. Um palavra para cada coluna.
 * 
 * No loop infinito, a gravação dos dados no cartão é feita a cada 1 segundo. Sempre há
 * a construção e descontrução do bloco, como a abertura e fechamento do arquivo, para
 * tentar garantir a consistência do armazenamento dos dados.
 *
 * Toda gravação custa x bytes de memória (do cartão micro SD). Onde x é a quantidade de bytes gravados
 *
 * Obs: Cuidado, o tempo de gravação cresce com o tamanho do arquivo, isso se deve a
 * necessidade de se deslocar ao final do arquivo para poder gravar. Por isso é criado
 * um novo arquivo por dia.
 * 
 * Ex: Testes realizados:
 * --Tamanho inicial do arquivo 0 byte:
 *	48ms
 *
 * --Tamanho inicial do arquivo 1k byte:
 *	48ms
 *
 * --Tamanho inicial do arquivo 10k byte:
 *	48ms
 *
 * --Tamanho inicial do arquivo 100k byte:
 *	56ms
 *
 * --Tamanho inicial do arquivo 1M byte:
 *	56ms
 *
 * --Tamanho inicial do arquivo 10M byte:
 *	74ms
 *
 * --Tamanho inicial do arquivo 100M byte:
 *	224ms
 *
 * --Tamanho inicial do arquivo 1G byte:
 *	1750ms
 *
 * No nosso caso, gravamos 41 bytes a cada  1 segundo, o que equivale a 3542400 bytes por dia (3.5 Mbytes)
 * (41 bytes * 86400 segundos). Ou seja, gastamos algo entre 56ms e 74ms ao final do dia.
 *----------------------------------------------------------------------------------------------------------------------
 */
void escrever_no_arquivo ();

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Tenta reenviar a mensagem para o servidor em caso de falha
 *
 * Essa tentativa pode ser realizada após 3 segundos ou 1 minuto
 *----------------------------------------------------------------------------------------------------------------------
 */
static void try_send (int16_t retcode);

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Envia os dados via rede LoRa
 *
 * Primeiro há leitura dos sensores para depois enviar os dados
 *
 * O envio demora cerca de 18 ms
 *----------------------------------------------------------------------------------------------------------------------
 */
static void LoRa_send_message ();

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Recebe mensagens via rede LoRa na geração de eventos pela rede
 *
 * Esses eventos são tratados em: lora_event_handler
 *----------------------------------------------------------------------------------------------------------------------
 */
static void receive_message ();

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Manipula os eventos relacionados a rede LoRa
 *
 * A manipulação pode ser de acordo com as necessidades do programador
 *
 * Esse manipulador é inicializado ao final da main
 *----------------------------------------------------------------------------------------------------------------------
 */
static void lora_event_handler (lorawan_event_t event);

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Adquire constantemente os dados fornecidos por um GPS
 *
 * Os dados adquiridos são armazenados em uma estrutura do tipo: dataGPS
 *
 * Estrutura definida em GPS_Carro.h
 *   
 * Utiliza-se um semaforo na tentativa de manter a atomicidade dos dados
 *----------------------------------------------------------------------------------------------------------------------
 */
void adquirirDadosDoGPS (void);

/**
 * Calibração
 */
DigitalOut ledPouco (PC_8);
DigitalOut ledMedio (PC_6);
DigitalOut ledOkay (PC_5);
void calibracao (void);
Thread thread_calibrador;
Semaphore dadosMPU (1);


//------------------------------------------------------------------------------------------------------------------
//-- FIM - Protótipos das funções
//------------------------------------------------------------------------------------------------------------------


/**
 *----------------------------------------------------------------------------------------------------------------------
 * Na main, há principalmente toda a configuração da aplicação LoRa.
 *
 * Aqui também há a chamada para a Thread que grava no cartão, Thread que faz a aquisição dos dados fornecidos pelo GPS
 * e a chamada para o manipulador de eventos LoRa.
 *
 * O que é configurado do LoRa aqui:
 * Inicialização, Tentativas de confirmação da mensagem, DataRate, 
 * Tipo de conexão (OTTA), Parametros da conexão. 
 *----------------------------------------------------------------------------------------------------------------------
 */
int main (void) {
    //Configurações de comunicação com o modulo do GPS
    gps.baud (9600); //Taxa de comunição serial com o GPS
  
    mbed_trace_init ();
    
    lorawan_status_t retcode;

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 0: Initialize LoRaWAN stack
    //------------------------------------------------------------------------------------------------------------------
    if (lorawan.initialize (&ev_queue) != LORAWAN_STATUS_OK) {
        printf ("LoRa initialization failed! \r\n");
        return -1;
    }

    printf ("Mbed LoRaWANStack initialized \r\n");

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 1: Prepare application callbacks
    //------------------------------------------------------------------------------------------------------------------
    callbacks.events = mbed::callback (lora_event_handler);
    lorawan.add_app_callbacks (&callbacks);

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 2: Set number of retries in case of CONFIRMED messages
    //------------------------------------------------------------------------------------------------------------------
    if (lorawan.set_confirmed_msg_retries(3) != LORAWAN_STATUS_OK) {
        printf ("set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 3: Enable adaptive data rate
    //------------------------------------------------------------------------------------------------------------------
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf ("enable_adaptive_datarate failed! \r\n");
        return -1;
    }
    printf ("Adaptive data  rate (ADR) - Enabled \r\n");

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 4: Configuração dos parametros para conexão (informações sobre a aplicação no TTN)
    //------------------------------------------------------------------------------------------------------------------
    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = LORAWAN_DEV_EUI;
    connect_params.connection_u.otaa.app_eui = LORAWAN_APP_EUI;
    connect_params.connection_u.otaa.app_key = LORAWAN_APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 10;

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 5: Conecta ao servidor indicado pelas chaves
    //------------------------------------------------------------------------------------------------------------------
    retcode = lorawan.connect (connect_params);

    if (retcode == LORAWAN_STATUS_OK || retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
        printf ("Connection - In Progress ...\r\n");
    } else {
        printf ("Connection error, code = %d \r\n", retcode);
        return -1;
    }


    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 6: Inicialização do método de calibração do acelerometro
    //------------------------------------------------------------------------------------------------------------------
    thread_calibrador.start (calibracao);
    wait (2);

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 7: Inicialização do Fluxo de Controle de aquisição dos dados do GPS
    //------------------------------------------------------------------------------------------------------------------
    thread_gps.start (adquirirDadosDoGPS);
    wait (3);

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 8: Inicialização do Fluxo de Controle da gravação no cartão
    //------------------------------------------------------------------------------------------------------------------
    thread_cartao.start (escrever_no_arquivo);

    //------------------------------------------------------------------------------------------------------------------
    //-- PASSO 9: Faz o manipulador de eventos disparar para sempre
    //------------------------------------------------------------------------------------------------------------------    
    ev_queue.dispatch_forever ();

}


/*
 *----------------------------------------------------------------------------------------------------------------------
 * Gravar no cartão micro SD
 *----------------------------------------------------------------------------------------------------------------------
 */
void escrever_no_arquivo () {
    
    //calibracao ();

    //thread_gps.start (adquirirDadosDoGPS);

    //wait (3);

    printf ("Gravando no cartao\r\n");

    /**
     * Perifericos
     *
     * Acelerometro, Sensor de temperatura, Giroscópio, GPS e RTC (Temporariamente não utilizado)
     *
     */    
    float acce[3], temperatura, gyro[3];
    //DateTime dt; Esse era o objeto para o RTC

    //Montagem do sistema em blocos
    int err = fs.mount (bd);
        
    while (err != 0) {
        printf ("reformatando\r\n");
        // Reformata se não conseguir montar o sistema de blocos       
        //fs.reformat (bd);
        err = fs.mount (bd);    
        wait (1);
    }

    mkdir ("fs/dados", 1); //Pasta que contém os arquivos que contém os dados
    mkdir ("fs/controle", 1); //Pasta que contém o arquivo de controle dos arquivos que contém os dados

    //Cria um novo arquivo a cada dia ou a cada vez que o carro for ligado
    
    char novoNomeDeArquivo_2[9], *novoNomeDeArquivo;
    novoNomeDeArquivo = novoNomeDeArquivo_2;

    int verificacaoDoControleDeGravacao = 1;
    do {
        verificacaoDoControleDeGravacao = controleDeGravacao (novoNomeDeArquivo);
    } while (verificacaoDoControleDeGravacao != 0);
    
    printf ("arquivo: %c%c%c%c%c%c%c%c\r\n", novoNomeDeArquivo[0], novoNomeDeArquivo[1],
                                         novoNomeDeArquivo[2], novoNomeDeArquivo[3],
                                         novoNomeDeArquivo[4], novoNomeDeArquivo[5],
                                         novoNomeDeArquivo[6], novoNomeDeArquivo[7]);
    novoNomeDeArquivo[8] = '\0';

    char nomeArquivo[27] = "/fs/dados/";
    char extensao[5] = ".csv";
    strcat(nomeArquivo, novoNomeDeArquivo);
    strcat(nomeArquivo, extensao);    
    printf ("%s\r\n", nomeArquivo);

    /**
     * Verificando a existencia do arquivo   
     * Verifica se o arquivo já existe para poder nomear as colunas
     * Se não existir, cria o arquivo e nomeia as colunas
     */
    FILE *f;
    f = fopen (nomeArquivo, "r");    
    if (!f) {                           
        f = fopen (nomeArquivo, "w");  
        while (!f) {
            printf ("aqui - agora\r\n");
            f = fopen (nomeArquivo, "w");
            wait (2);
        } 
        // Nome das labels
        err = fprintf (f, "Ace 1;Ace 2;Ace 3;Gyro 1;Gyro 2;Gyro 3;Temperatura;Latitude;Longitude;Data;Hora;Velocidade\r\n");
        while (err < 0) {
            
            err = fprintf (f, "Ace 1;Ace 2;Ace 3;Gyro 1;Gyro 2;Gyro 3;Temperatura;Latitude;Longitude;Data;Hora;Velocidade\r\n");
            wait (1);
            printf ("Falha ao gravar. Cartão não encontrado ou memória cheia.\r\n");
        }
        // Fecha o arquivo para liberar qualquer gravação no buffer    
        fclose (f);              
    } else {
        // Fecha o arquivo para liberar qualquer gravação no buffer
        fclose (f);        
    }    


    //LOOP --------------------------------------------------------------------------------
    while (1) {

        // Abrindo o arquivo para gravação   
        f = fopen (nomeArquivo, "a+");    
        if (!f) {
            wait_ms (500);   
            f = fopen (nomeArquivo, "a+"); 
            if (!f) {
                printf ("Erro 1 (Cartao nao encontrado)!\r\nColoque o cartao novamente e reinicie a placa.\r\n");
                fclose (f);
                wait (1);
                continue;        
            }
        }

        // Lendo os dados dos perifericos
        ark.getAccelero (acce);                    
        temperatura = ark.getTemp ();
        ark.getGyro (gyro);
        
        //Escrevendo_no_arquivo
        if (dadosDoGPS.valid == 'A') {
            semaforo_acessar_gps.acquire ();
            err = fprintf (f, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.6lf;%.6lf;%c%c%c%c%c%c;%.0lf;%.4lf\r\n", 
                        acce[0], acce[1], acce[2],
                        gyro[0], gyro[1], gyro[2],
                        temperatura, 
                        dadosDoGPS.latitude, dadosDoGPS.longitude,
                        dadosDoGPS.date[0], dadosDoGPS.date[1], dadosDoGPS.date[2], dadosDoGPS.date[3], dadosDoGPS.date[4], dadosDoGPS.date[5],
                        dadosDoGPS.time,
                        dadosDoGPS.speed);
            semaforo_acessar_gps.release ();
            if (err < 0) {
                wait_ms (500);
                semaforo_acessar_gps.acquire ();        
                err = fprintf (f, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.6lf;%.6lf;%c%c%c%c%c%c;%.0lf;%.4lf\r\n", 
                        acce[0], acce[1], acce[2],
                        gyro[0], gyro[1], gyro[2],
                        temperatura, 
                        dadosDoGPS.latitude, dadosDoGPS.longitude,
                        dadosDoGPS.date[0], dadosDoGPS.date[1], dadosDoGPS.date[2], dadosDoGPS.date[3], dadosDoGPS.date[4], dadosDoGPS.date[5],
                        dadosDoGPS.time,
                        dadosDoGPS.speed);
                semaforo_acessar_gps.release ();
                if (err < 0) {
                    fclose (f);
                    wait (1);
                    printf ("Falha ao gravar. Cartão não encontrado ou memória cheia.\r\n");
                    continue; 
                }       
            }
            semaforo_acessar_gps.acquire ();
            printf ("A1: %.2f; A2: %.2f; A3: %.2f;G1: %.2f; G2: %.2f; G3: %.2f; Temp: %.2f; Lat: %lf; Long: %lf; Vel: %lf; Dat: %c%c%c%c%c%c; Tempo: %.0lf\r\n\n", 
                    acce[0], acce[1], acce[2],
                    gyro[0], gyro[1], gyro[2], 
                    temperatura,
                    dadosDoGPS.latitude, dadosDoGPS.longitude,
                    dadosDoGPS.speed,
                    dadosDoGPS.date[0], dadosDoGPS.date[1], dadosDoGPS.date[2], dadosDoGPS.date[3], dadosDoGPS.date[4], dadosDoGPS.date[5],
                    dadosDoGPS.time);
            semaforo_acessar_gps.release ();    
        } else {
            printf ("GPS conectando...\r\n");
        }

        // Close the file which also flushes any cached writes    
        fclose (f);        
        //Espera por 1000 ms (gravação a cada 1 segundo aproximadamente)
        wait_ms (1000);
    }    
}

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Tenta reenviar a mensagem se necessário
 *----------------------------------------------------------------------------------------------------------------------
 */
static void try_send (int16_t retcode) {
    // Verifica se o ciclo de trabalho foi violoado ou outro erro ocorreu durante o envio
    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf ("send - Duty cycle violation\r\n")
                : printf ("send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            // Tenta novamente em 3 segundos
            ev_queue.call_in (3000, LoRa_send_message);
        }
        else {
            ev_queue.call_in (TX_INTERVAL, LoRa_send_message);
        }
        return;
    }
    return;
}

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Manda mensagem para servidor da rede
 *----------------------------------------------------------------------------------------------------------------------
 */
static void LoRa_send_message () {
        /**
        * Perifericos
        *
        * Acelerometro, Sendor de temperatura, Giroscópio, GPS e RTC (Temporariamente não utilizado)
        *
        * Código de retorno LoRa 
        *
        */ 

        //DateTime dt;
        float acce[3];
        float temperatura;
        int16_t retcode;            
        
        ark.getAccelero (acce);
        temperatura = ark.getTemp ();
        //dt = gRtc.now ();        

        payloader.addAccelerometer (acce[0], acce[1], acce[2]);                
        payloader.addTemperature (temperatura);
        semaforo_acessar_gps.acquire ();
        if (dadosDoGPS.valid == 'A') {
            payloader.addGPS (dadosDoGPS.latitude, dadosDoGPS.longitude, dadosDoGPS.speed);
            payloader.addGPSData (&dadosDoGPS.date[0], dadosDoGPS.time);
            printf ("Latitude: %.2lf / Longitude: %.2lf / Velocidade: %.2lf\r\n", dadosDoGPS.latitude, dadosDoGPS.longitude, dadosDoGPS.speed);
        }            
        semaforo_acessar_gps.release ();

        // Envia os dados para o servidor
        retcode = lorawan.send (15, payloader.dados, payloader.tamanho, MSG_UNCONFIRMED_FLAG);
        
        try_send (retcode);
                       
        // Adiciona o envio da mensagem a pilha de eventos
        ev_queue.call_in (TX_INTERVAL, LoRa_send_message);       
}

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Recebe mensagem do servidor da rede
 *----------------------------------------------------------------------------------------------------------------------
 */
static void receive_message () {
    uint8_t rx_buffer[50] = { 0 };
    int16_t retcode = lorawan.receive (15, rx_buffer,
                                      sizeof (rx_buffer),
                                      MSG_CONFIRMED_FLAG|MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        printf ("receive() - Error code %d \r\n", retcode);
        return;
    }

    printf ("RX Data (%d bytes): ", retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf ("%02x ", rx_buffer[i]);
    }
    printf ("\r\n");
}

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Manipulador de eventos
 *----------------------------------------------------------------------------------------------------------------------
 */
static void lora_event_handler (lorawan_event_t event) {
    switch (event) {
        case CONNECTED:
            printf ("Connection - Successful \r\n");                  
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                LoRa_send_message ();
            } else {
                ev_queue.call_every (TX_INTERVAL, LoRa_send_message);
            }
            break;
        case DISCONNECTED:
            ev_queue.break_dispatch ();
            printf ("Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf ("Message Sent to Network Server \r\n");
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf ("Transmission Error - EventCode = %d \r\n", event);
            break;
        case RX_DONE:
            printf ("Received message from Network Server \r\n");
            receive_message ();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf ("Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf ("OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf ("Uplink required by NS \r\n");
            LoRa_send_message ();
            break;
        default:
            MBED_ASSERT ("Unknown Event");
    }
}

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Adquire dados do GPS
 *----------------------------------------------------------------------------------------------------------------------
 */
void adquirirDadosDoGPS (void) {
    char c;
    char cDataBuffer[200];
    while (true) {
        if (gps.readable ()) {
            if (gps.getc () == '$') { // Espera um $ (Identifica o inicio de um mensagem)
                for (int i = 0; i < sizeof (cDataBuffer); i++) {
                    c = gps.getc();
                    if (c == '\r' ) {
                        semaforo_acessar_gps.acquire ();
                        parse (cDataBuffer, i, &dadosDoGPS);
                        semaforo_acessar_gps.release ();
                        i = sizeof (cDataBuffer);                        
                    } else {
                        cDataBuffer[i] = c;
                    }                
                }
            } 
        }
    }
    return;
}

void calibracao (void) {
    //printf ("Calibrando...\r\n\n");
    float acceCalib[3];
    bool controle = true;

    while (true) {
        ark.getAccelero (acceCalib);

        if (acceCalib[2] <= 7.5) {
            ledPouco = 1; // LED is OFF
            ledMedio = 0; // LED is OFF
            ledOkay = 0; // LED is ON            
        } else if (acceCalib[2] > 7.9 && acceCalib[2] < 10.7) {
            ledPouco = 0; // LED is ON
            ledMedio = 1; // LED is OFF
            ledOkay = 0; // LED is OFF 
        } else if (acceCalib[2] >= 10.8) {
            ledPouco = 0; // LED is OFF
            ledMedio = 0; // LED is ON
            ledOkay = 1; // LED is OFF
        }
        wait_ms (700);
    }  
    return;
}

/**
 *----------------------------------------------------------------------------------------------------------------------
 * Controla os arquivos do SD
 *----------------------------------------------------------------------------------------------------------------------
 */
int controleDeGravacao (char *novoNomeDeArquivo) {

    FILE *arq;
    arq = fopen ("/fs/controle/controle.txt","r");
    if (arq == NULL) {
        arq = fopen ("/fs/controle/controle.txt","w");
        if (arq == NULL) {            
            printf ("Erro!\n");
            wait (1);
            return 1;
        } 
        fclose (arq);
        arq = fopen ("/fs/controle/controle.txt","r");       
    }
    
    char ajuste[2];
    char texto[9];
    int tamanho;
    int err = 0;

    fseek (arq, 0, SEEK_END);
    tamanho = ftell (arq);
    fseek (arq, (tamanho - 10), SEEK_SET);
    fgets (texto, 9, arq);
    fclose (arq);
    printf ("\r\nLido: %s \r\n", texto);

    ajuste[0] = texto[6]; ajuste[1] = texto[7];

    arq = fopen ("/fs/controle/controle.txt","a+");

    semaforo_acessar_gps.acquire ();
    if (dadosDoGPS.date[4] == texto[0] && dadosDoGPS.date[5] == texto[1] && dadosDoGPS.date[2] == texto[2] &&
        dadosDoGPS.date[3] == texto[3] && dadosDoGPS.date[0] == texto[4] && dadosDoGPS.date[1] == texto[5] ) {
        
        semaforo_acessar_gps.release ();

        printf ("Mesma data!\r\n");

        if (texto[7] == 'Z') {
            if (texto[6] == 'Z') {
                printf ("\r\nErro 2! (Limite de arquivos atingido)\r\n");
                printf ("Salve os arquivos, e remova-os do cartao.\r\n");
                printf ("Feito isso, coloque o cartao e reinicie a placa.\r\n\n");
                while (1);
            } else {
                ajuste[1] = 'A';
            ajuste[0] += 1;
            }
        } else {
            ajuste[1] += 1;
        }
        fseek (arq, 0, SEEK_END);

        err = fprintf(arq, "%d%d%d%d%d%d%c%c\r\n",
                (texto[0] - 48), (texto[1] - 48), (texto[2] - 48),
                (texto[3] - 48), (texto[4] - 48), (texto[5] - 48),
                ajuste[0], ajuste[1]);
        if (err < 0) {
            fclose (arq);
            printf ("Falha ao gravar. Cartão não encontrado ou memória cheia.\r\n");
            return 1;
        }
        
          novoNomeDeArquivo[0] = texto[0] ; novoNomeDeArquivo[1] = texto[1] ;
          novoNomeDeArquivo[2] = texto[2] ; novoNomeDeArquivo[3] = texto[3] ;
          novoNomeDeArquivo[4] = texto[4] ; novoNomeDeArquivo[5] = texto[5] ;
          novoNomeDeArquivo[6] = ajuste[0]; novoNomeDeArquivo[7] = ajuste[1];

    } else {
        semaforo_acessar_gps.release ();
        fseek (arq, 0, SEEK_END);
        printf ("Data diferente!\r\n");

        semaforo_acessar_gps.acquire ();
        fprintf (arq, "%d%d%d%d%d%d%c%c\r\n", 
                 dadosDoGPS.date[4] - 48,  dadosDoGPS.date[5] - 48,
                 dadosDoGPS.date[2] - 48,  dadosDoGPS.date[3] - 48,
                 dadosDoGPS.date[0] - 48,  dadosDoGPS.date[1] - 48,
                 'A', 'A');
        semaforo_acessar_gps.release ();

        if (err < 0) {
            fclose (arq);
            printf ("Falha ao gravar. Cartão não encontrado ou memória cheia.\r\n");
            return 1;
        }
        
        semaforo_acessar_gps.acquire ();
        novoNomeDeArquivo[0] = dadosDoGPS.date[4] ; novoNomeDeArquivo[1] = dadosDoGPS.date[5] ;
        novoNomeDeArquivo[2] = dadosDoGPS.date[2] ; novoNomeDeArquivo[3] = dadosDoGPS.date[3] ;
        novoNomeDeArquivo[4] = dadosDoGPS.date[0] ; novoNomeDeArquivo[5] = dadosDoGPS.date[1] ;
        novoNomeDeArquivo[6] = 'A'; novoNomeDeArquivo[7] = 'A';
        semaforo_acessar_gps.release ();         
    }
    
    fclose (arq);

    return 0;
}